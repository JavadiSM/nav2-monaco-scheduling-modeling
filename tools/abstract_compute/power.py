"""Per-core active power and the two inactive operating states."""
import math
from .primitives import finite_float, probability


class PowerModel:
    def leakage_power_w(self, core, temperature_c=None, reference_temperature_c=None):
        spec = core.core_type
        temperature = finite_float(core.temperature_c if temperature_c is None else temperature_c,
                                   core.reference_temperature_c)
        reference = core.reference_temperature_c if reference_temperature_c is None else reference_temperature_c
        return spec.leakage_power_ref_w * math.exp(spec.leakage_gamma_per_k * (temperature - reference))

    def average_power_w(self, core, activity_factor, dvfs_level_id=None,
                        temperature_c=None, reference_temperature_c=None):
        spec, point = core.core_type, core.dvfs_level(dvfs_level_id)
        fraction = point.frequency_mhz / spec.max_frequency_mhz
        if spec.dvfs_power_scaling_mode == 'voltage_frequency':
            scale = fraction * (point.voltage_v / spec.max_voltage_v) ** 2
        elif spec.dvfs_power_scaling_mode == 'frequency_cubic':
            scale = fraction ** 3
        else:
            raise ValueError('Unsupported dynamic-power scaling mode')
        reference_power = spec.reference_power_intercept_w + spec.reference_power_activity_slope_w * probability(activity_factor, .5)
        switching_power = max(reference_power - spec.leakage_power_ref_w, 0.) * scale
        return switching_power + self.leakage_power_w(core, temperature_c, reference_temperature_c)

    def runtime_power_w(self, core, *, is_active, activity_factor=None,
                        dvfs_level_id=None, temperature_c=None, reference_temperature_c=None):
        if core.thermal_forced_idle:
            return core.core_type.cooling_power_w
        if not is_active:
            return core.core_type.idle_power_w
        return self.average_power_w(core, .5 if activity_factor is None else activity_factor,
                                    dvfs_level_id, temperature_c, reference_temperature_c)
