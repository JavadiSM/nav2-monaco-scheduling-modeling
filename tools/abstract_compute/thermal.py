"""An RC network advanced exactly over constant-power intervals."""
from collections import OrderedDict
import numpy as np
import math
from .primitives import nonnegative_float


class ThermalModel:
    def __init__(self, processor):
        self.processor = processor
        self.core_ids = tuple(sorted(processor.cores))
        self.time_s = 0.
        cores = [processor.cores[n] for n in self.core_ids]
        capacity = np.array([c.thermal_capacitance_j_per_k for c in cores])
        ambient = np.array([c.ambient_conductance_w_per_k for c in cores])
        if not np.all(np.isfinite(capacity)) or np.any(capacity <= 0):
            raise ValueError('Invalid thermal capacitance')
        if not np.all(np.isfinite(ambient)) or np.any(ambient <= 0):
            raise ValueError('Invalid conductance to ambient')
        coupling = np.zeros((len(cores), len(cores)))
        for row, a in enumerate(self.core_ids):
            for column in range(row + 1, len(cores)):
                b = self.core_ids[column]
                value = processor.coupling_conductance_w_per_k(a, b)
                if not np.isfinite(value) or value <= 0:
                    raise ValueError('Invalid inter-core conductance')
                coupling[row, column] = coupling[column, row] = value
        self.A = np.diag(capacity)
        self.G = ambient
        self.B = np.diag(ambient + coupling.sum(axis=1)) - coupling
        self.capacitance_matrix_j_per_k = self.A
        self.ambient_conductance_vector_w_per_k = self.G
        self.conductance_matrix_w_per_k = self.B
        # A similarity transform gives a symmetric eigensystem for the RC decay.
        self._sqrt_capacity = np.sqrt(capacity)
        transformed = self.B / np.outer(self._sqrt_capacity, self._sqrt_capacity)
        self._decay_rates, self._basis = np.linalg.eigh(transformed)
        if np.any(self._decay_rates <= 0):
            raise ValueError('RC network must be stable')
        self.thermal_state_matrix_per_s = -self.B / capacity[:, None]
        self._transitions = OrderedDict()
        self._equilibrium_cache = OrderedDict()

    def _vector(self, values, fallback):
        count = len(self.core_ids)
        result = np.array(fallback, dtype=float, copy=True)
        supplied = np.asarray(values, dtype=float).reshape(-1)[:count]
        mask = np.isfinite(supplied)
        indices = np.flatnonzero(mask)
        result[indices] = supplied[indices]
        return result

    def temperature_vector_c(self):
        return np.array([self.processor.cores[n].temperature_c for n in self.core_ids])

    def steady_temperature_c(self, power_vector_w):
        power = np.maximum(self._vector(power_vector_w, np.zeros(len(self.core_ids))), 0.)
        ambient = self.processor.thermal_spec.ambient_temperature_c
        key=tuple(power)
        if key not in self._equilibrium_cache:
            self._equilibrium_cache[key]=ambient+np.linalg.solve(self.B,power)
            if len(self._equilibrium_cache)>128:self._equilibrium_cache.popitem(last=False)
        return self._equilibrium_cache[key].copy()

    def transition_matrix(self, duration_s):
        elapsed = nonnegative_float(duration_s)
        key = round(elapsed, 15)
        if key in self._transitions:
            self._transitions.move_to_end(key)
            return self._transitions[key]
        weighted_basis = self._basis * np.exp(-elapsed * self._decay_rates)
        symmetric_decay = weighted_basis @ self._basis.T
        transition = symmetric_decay * self._sqrt_capacity[None, :] / self._sqrt_capacity[:, None]
        self._transitions[key] = transition
        if len(self._transitions) > 256:
            self._transitions.popitem(last=False)
        return transition

    def evolve_temperature_c(self, temperature_c, power_vector_w, duration_s):
        initial = self._vector(temperature_c, self.temperature_vector_c())
        equilibrium = self.steady_temperature_c(power_vector_w)
        return equilibrium + self.transition_matrix(duration_s) @ (initial - equilibrium)

    def evolve_single_core_c(self, temperature_c, power_w, duration_s):
        """Scalar form of the same exact RC transition for a one-core device."""
        if len(self.core_ids)!=1:raise ValueError('Scalar evolution requires one core')
        key=round(duration_s,15)
        if not hasattr(self,'_scalar_decay'):self._scalar_decay={}
        if key not in self._scalar_decay:self._scalar_decay[key]=math.exp(-float(self.B[0,0])/float(self.A[0,0])*duration_s)
        equilibrium=self.processor.thermal_spec.ambient_temperature_c+power_w/float(self.B[0,0])
        return equilibrium+self._scalar_decay[key]*(temperature_c-equilibrium)

    def validate_idle_recovery(self):
        power = [self.processor.cores[n].core_type.cooling_power_w for n in self.core_ids]
        equilibrium = self.steady_temperature_c(power)
        limit = np.array([self.processor.cores[n].balance_temperature_c for n in self.core_ids])
        if np.any(equilibrium >= limit - 1e-9):
            raise ValueError('Cooling cannot bring every core below Tbalance')
        return equilibrium
