"""Configurable heterogeneous processors with compute-time-only metrics."""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import random
from pathlib import Path
from typing import Optional

from .primitives import OperatingPoint, ExecutionLane, finite_float, make_operating_points, probability


@dataclass(frozen=True)
class CoreType:
    name: str
    family: str
    performance_eta: float
    dvfs_levels: tuple[OperatingPoint, ...]
    default_dvfs_level_id: int
    reference_power_intercept_w: float
    reference_power_activity_slope_w: float
    leakage_power_ref_w: float
    leakage_gamma_per_k: float
    idle_power_w: float
    cooling_power_w: float
    dvfs_power_scaling_mode: str
    max_voltage_v: float
    thermal_capacitance_range_j_per_k: tuple[float, float]
    ambient_resistance_range_k_per_w: tuple[float, float]

    @property
    def max_frequency_mhz(self):
        return max(v.frequency_mhz for v in self.dvfs_levels)

    def reference_active_power_w(self, activity_factor):
        return self.reference_power_intercept_w + self.reference_power_activity_slope_w * probability(activity_factor, .5)

    @classmethod
    def from_dict(cls, data):
        levels = make_operating_points(data['frequency_range_mhz'], data['voltage_range_v'], data['l'])
        eta = float(data['eta'])
        if not 0 < eta < float('inf'):
            raise ValueError('eta must be finite and positive')
        power = data['power']
        thermal = data['thermal']
        return cls(data['name'], data['family'], eta, levels, levels[-1].level_id,
                   power['reference_intercept_w'], power['reference_activity_slope_w'],
                   power['leakage_ref_w'], power['leakage_gamma_per_k'], power['idle_w'],
                   power['cooling_power_w'], power['dvfs_scaling_mode'], data['max_voltage_v'],
                   tuple(thermal['capacitance_range_j_per_k']),
                   tuple(thermal['ambient_resistance_range_k_per_w']))

    @property
    def dvfs_by_id(self):
        return {int(level.level_id): level for level in self.dvfs_levels}

    def level(self, level_id: Optional[int] = None):
        resolved = self.default_dvfs_level_id if level_id is None else int(level_id)
        return self.dvfs_by_id.get(resolved, self.dvfs_by_id[int(self.default_dvfs_level_id)])


@dataclass
class Core:
    core_id: int
    core_type: CoreType
    processor_id: int
    core_name: str
    coordinate_xy: tuple[float, float]
    current_dvfs_level_id: Optional[int] = None
    lane: ExecutionLane = field(default_factory=ExecutionLane)
    thermal_capacitance_j_per_k: float = .1
    ambient_resistance_k_per_w: float = 5.
    reference_temperature_c: float = 35.
    initial_temperature_c: float = 55.
    temperature_c: float = 55.
    max_temperature_c: float = 55.6
    balance_temperature_c: float = 55.45
    thermal_forced_idle: bool = False

    @property
    def ambient_conductance_w_per_k(self):
        return 1.0 / self.ambient_resistance_k_per_w

    def __post_init__(self):
        if self.current_dvfs_level_id is None:
            self.current_dvfs_level_id = self.core_type.default_dvfs_level_id

    def dvfs_level(self, level_id=None):
        return self.core_type.level(self.current_dvfs_level_id if level_id is None else level_id)

    def execution_metrics(self, computational_demand_mcycles, dvfs_level_id=None):
        demand = max(0.0, finite_float(computational_demand_mcycles, default=0.0))
        level = self.dvfs_level(dvfs_level_id)
        frequency_mhz = float(level.frequency_mhz)
        time_s = 0.0 if demand == 0.0 else demand / (float(self.core_type.performance_eta) * frequency_mhz)
        return {
            'time_s': time_s,
            'frequency_mhz': frequency_mhz,
            'nominal_frequency_mhz': float(level.frequency_mhz),
            'voltage_v': level.voltage_v,
            'dvfs_level_id': int(level.level_id),
            'performance_eta': float(self.core_type.performance_eta),
            'core_type': self.core_type.name,
        }

    def preview(self, demand_mcycles, ready_s, dvfs_level_id=None):
        metrics = self.execution_metrics(demand_mcycles, dvfs_level_id)
        start, finish = self.lane.earliest_slot(metrics['time_s'], ready_s)
        return {'EST': start, 'CT': finish, **metrics}

    def reserve(self, job_id, demand_mcycles, ready_s, dvfs_level_id=None):
        result = self.preview(demand_mcycles, ready_s, dvfs_level_id)
        self.lane.reserve(job_id, result['EST'], result['CT'],
                          {'core_id': self.core_id, 'dvfs_level_id': result['dvfs_level_id']})
        return result


@dataclass(frozen=True)
class ThermalSpec:
    ambient_temperature_c: float
    reference_temperature_c: float
    max_temperature_c: float
    balance_temperature_c: float
    thermal_epoch_ms: float
    coupling_base_conductance_range_w_per_k: tuple[float, float]
    coupling_distance_attenuation: float
    control_epoch_s: float

    @property
    def thermal_epoch_s(self):
        return self.thermal_epoch_ms / 1000.

    def __post_init__(self):
        if self.balance_temperature_c >= self.max_temperature_c:
            raise ValueError('Tbalance must be strictly below Tmax')
        if not (0 < self.control_epoch_s <= self.thermal_epoch_s):
            raise ValueError('Control epoch must be positive and at most the thermal epoch')


@dataclass
class Processor:
    processor_id: int
    device_class: str
    cores: dict[int, Core]
    thermal_spec: ThermalSpec
    coupling_base_conductance_w_per_k: float

    def coupling_conductance_w_per_k(self, core_a, core_b):
        if core_a == core_b:
            raise ValueError('Self coupling belongs to the matrix diagonal')
        a, b = self.cores[core_a].coordinate_xy, self.cores[core_b].coordinate_xy
        distance = abs(a[0] - b[0]) + abs(a[1] - b[1])
        if distance <= 0:
            raise ValueError('Distinct cores require distinct coordinates')
        return self.coupling_base_conductance_w_per_k * math.exp(
            -self.thermal_spec.coupling_distance_attenuation * (distance - 1.))

    def resource_options(self, include_all_dvfs_levels=False):
        if include_all_dvfs_levels:
            return [(cid, level.level_id) for cid, c in sorted(self.cores.items())
                    for level in c.core_type.dvfs_levels]
        return [(cid, c.core_type.default_dvfs_level_id) for cid, c in sorted(self.cores.items())]

    def reset(self):
        for core in self.cores.values():
            core.lane.reset()
            core.current_dvfs_level_id = core.core_type.default_dvfs_level_id
            core.temperature_c = core.initial_temperature_c
            core.thermal_forced_idle = False


def load_platform(path=None, *, device_classes=('vehicle', 'server')):
    """Build independent endpoint templates; this does not create simulated cars."""
    path = Path(path) if path is not None else Path(__file__).resolve().parents[2] / 'config/abstract_compute.json'
    data = json.loads(path.read_text())
    types = {name: CoreType.from_dict(cfg) for name, cfg in data['core_types'].items()}
    devices = {}
    spec = ThermalSpec(**data['thermal'])
    rng = random.Random(data['physical_seed'])
    for device_id, name in enumerate(device_classes):
        cfg = data['device_classes'][name]
        counts = [cfg['lp_cores'], cfg['hp_cores']]
        if any(isinstance(n, bool) or not isinstance(n, int) or n < 0 for n in counts) or sum(counts) == 0:
            raise ValueError('Core counts must be nonnegative integers, with at least one core')
        cores = {}
        for row, (kind, prefix, count) in enumerate(zip(('A7', 'A15'), ('LP', 'HP'), counts)):
            for col in range(count):
                cid = len(cores)
                ctype = types[kind]
                cores[cid] = Core(cid, ctype, device_id, f'{prefix}{col + 1}', (float(col), float(row)),
                                  thermal_capacitance_j_per_k=rng.uniform(*ctype.thermal_capacitance_range_j_per_k),
                                  ambient_resistance_k_per_w=rng.uniform(*ctype.ambient_resistance_range_k_per_w),
                                  reference_temperature_c=spec.reference_temperature_c,
                                  initial_temperature_c=spec.ambient_temperature_c,
                                  temperature_c=spec.ambient_temperature_c,
                                  max_temperature_c=spec.max_temperature_c,
                                  balance_temperature_c=spec.balance_temperature_c)
        devices[device_id] = Processor(device_id, name, cores, spec, rng.uniform(*spec.coupling_base_conductance_range_w_per_k))
    return devices
