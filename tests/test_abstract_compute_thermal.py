"""Power equations, coupled RC physics and device-wide thermal recovery."""
from dataclasses import replace
import math
import json
import tempfile
from pathlib import Path
import unittest
import numpy as np

from tools.abstract_compute import Job, load_platform
from tools.abstract_compute.power import PowerModel
from tools.abstract_compute.thermal import ThermalModel
from tools.abstract_compute.thermal_scheduler import ThermalScheduler


class ThermalTests(unittest.TestCase):
    def hot_platform(self):
        devices=load_platform()
        for device in devices.values():
            device.thermal_spec=replace(device.thermal_spec,ambient_temperature_c=55.,max_temperature_c=55.6,balance_temperature_c=55.45)
            for core in device.cores.values():
                core.initial_temperature_c=core.temperature_c=55.
                core.max_temperature_c=55.6;core.balance_temperature_c=55.45
        return devices

    def test_seeded_physical_realizations_stay_in_ranges(self):
        a, b = load_platform(), load_platform()
        for did, device in a.items():
            for cid, core in device.cores.items():
                self.assertEqual(core.thermal_capacitance_j_per_k, b[did].cores[cid].thermal_capacitance_j_per_k)
                self.assertEqual(core.ambient_resistance_k_per_w, b[did].cores[cid].ambient_resistance_k_per_w)
                low, high = core.core_type.thermal_capacitance_range_j_per_k
                self.assertTrue(low <= core.thermal_capacitance_j_per_k <= high)
                low, high = core.core_type.ambient_resistance_range_k_per_w
                self.assertTrue(low <= core.ambient_resistance_k_per_w <= high)

    def test_initial_temperature_is_independent_of_ambient_and_reset(self):
        data=json.loads((Path(__file__).resolve().parents[1]/'config/abstract_compute.json').read_text())
        data['thermal']['ambient_temperature_c']=40.
        data['thermal']['initial_temperature_c']=55.
        with tempfile.TemporaryDirectory() as folder:
            config=Path(folder)/'hardware.json';config.write_text(json.dumps(data))
            device=load_platform(config)[0]
        self.assertEqual(device.thermal_spec.ambient_temperature_c,40.)
        self.assertEqual([c.temperature_c for c in device.cores.values()],[55.,55.])
        model=ThermalModel(device)
        power=[c.core_type.idle_power_w for c in device.cores.values()]
        relaxed=model.evolve_temperature_c([55.,55.],power,.01)
        self.assertTrue(np.all(relaxed<55.))
        for core,value in zip(device.cores.values(),relaxed):core.temperature_c=float(value)
        device.reset()
        self.assertEqual([c.temperature_c for c in device.cores.values()],[55.,55.])

    def test_active_power_voltage_frequency_and_temperature(self):
        cores = load_platform()[0].cores
        power = PowerModel()
        for core, expected in ((cores[0], .47), (cores[1], 1.79)):
            self.assertAlmostEqual(power.average_power_w(core, .8, 4, temperature_c=35), expected)
            low = power.average_power_w(core, .8, 0, temperature_c=35)
            self.assertLess(low, expected)
            self.assertGreater(power.leakage_power_w(core, 55), power.leakage_power_w(core, 35))

    def test_ordinary_idle_and_cooling_power_are_distinct(self):
        cores = load_platform()[0].cores
        power = PowerModel()
        for core, idle, cooling in ((cores[0], .05, .005), (cores[1], .15, .015)):
            self.assertEqual(power.runtime_power_w(core, is_active=False), idle)
            core.thermal_forced_idle = True
            self.assertEqual(power.runtime_power_w(core, is_active=False), cooling)

    def test_rc_matrices_and_analytic_single_core_solution(self):
        device = load_platform()[0]
        core = device.cores[0]
        device.cores = {0: core}
        model = ThermalModel(device)
        start, p, dt = 56., .2, .3
        equilibrium = device.thermal_spec.ambient_temperature_c + p * core.ambient_resistance_k_per_w
        expected = equilibrium + (start - equilibrium) * math.exp(-dt / (core.ambient_resistance_k_per_w * core.thermal_capacitance_j_per_k))
        self.assertAlmostEqual(model.evolve_temperature_c([start], [p], dt)[0], expected, places=11)
        for device in load_platform().values():
            model = ThermalModel(device)
            np.testing.assert_allclose(model.B, model.B.T, atol=1e-12)
            np.testing.assert_allclose(model.B.sum(axis=1), model.G, atol=1e-12)
            self.assertTrue(np.all(np.linalg.eigvalsh(model.B) > 0))

    def test_current_cooling_can_reach_balance_and_bad_power_is_rejected(self):
        for device in self.hot_platform().values():
            eq = ThermalModel(device).validate_idle_recovery()
            self.assertTrue(np.all(eq < 55.45))
            for core in device.cores.values():
                core.core_type = replace(core.core_type, cooling_power_w=.1)
            with self.assertRaises(ValueError):
                ThermalModel(device).validate_idle_recovery()

    def test_entire_device_pauses_and_resumes_without_losing_work(self):
        runtime = ThermalScheduler(self.hot_platform())
        results = runtime.run([Job(1, 3600, 0, 1), Job(2, 3600, 0, 0)])
        starts = [e for e in runtime.cooling_events if e['kind'] == 'cooling_start' and e['device_id'] == 0]
        ends = [e for e in runtime.cooling_events if e['kind'] == 'cooling_end' and e['device_id'] == 0]
        self.assertGreater(len(starts), 0)
        self.assertEqual(len(starts), len(ends))
        for start, end in zip(starts, ends):
            self.assertEqual(start['affected_core_ids'], [0, 1])
            self.assertLess(start['time_s'], end['time_s'])
            self.assertTrue(all(t <= 55.45 + 1e-9 for t in end['temperature_c'].values()))
            for segment in runtime.work_segments:
                if segment['device_id'] == 0:
                    self.assertTrue(segment['end_s'] <= start['time_s'] + 1e-12 or segment['start_s'] >= end['time_s'] - 1e-12)
        self.assertAlmostEqual(results[0]['active_time_s'], 1., places=10)
        self.assertAlmostEqual(results[1]['active_time_s'], 2.25, places=10)
        for item in results:
            self.assertAlmostEqual(item['executed_mcycles'], item['demand_mcycles'], places=7)
            self.assertGreater(item['finish_s'] - item['start_s'], item['metrics']['time_s'])
        for sample in runtime.samples:
            if sample['cooling'] and sample['device_id'] == 0:
                for cid, watts in sample['power_w'].items():
                    expected = .005 if cid == 0 else .015
                    self.assertEqual(watts, expected)
        for event in runtime.cooling_events:
            self.assertEqual(round(event['time_s'] * 1e9) % 5_000_000, 0)

    def test_other_device_keeps_executing_during_cooling(self):
        runtime = ThermalScheduler(self.hot_platform())
        results = runtime.run([Job(1, 3600, 0, 1), Job(2, 80, 1, 0, dvfs_level_id=0)])
        self.assertAlmostEqual(results[1]['finish_s'], .1, places=10)
        first = runtime.cooling_events[:2]
        self.assertLess(first[0]['time_s'], results[1]['finish_s'])
        self.assertGreater(first[1]['time_s'], results[1]['finish_s'])

    def test_high_thermal_limits_recover_pure_compute_timing(self):
        devices = load_platform()
        for device in devices.values():
            for c in device.cores.values():
                c.max_temperature_c = 100.
                c.balance_temperature_c = 90.
        runtime = ThermalScheduler(devices)
        results = runtime.run([Job(1, 1600, 0, 0), Job(2, 1800, 0, 1)])
        self.assertEqual(runtime.cooling_events, [])
        self.assertAlmostEqual(results[0]['finish_s'], 1., places=10)
        self.assertAlmostEqual(results[1]['finish_s'], .5, places=10)

    def test_zero_work_and_empty_trace_complete(self):
        self.assertEqual(ThermalScheduler(load_platform()).run([]), [])
        r = ThermalScheduler(load_platform()).run([Job(1, 0, 0, 0)])
        self.assertEqual(r[0]['finish_s'], 0)


if __name__ == '__main__':
    unittest.main()
