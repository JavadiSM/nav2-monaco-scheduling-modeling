"""Analytic scheduling cases and virtual CPU resource invariants."""
import json
import math
from pathlib import Path
import random
import tempfile
import unittest

from tools.abstract_compute import Job, VirtualScheduler, load_platform
from tools.abstract_compute.primitives import ExecutionLane

from platform_fixtures import load_platform, LiveEngine


class ComputeTests(unittest.TestCase):
    def test_operating_points_and_endpoint_counts(self):
        devices = load_platform()
        self.assertEqual([len(d.cores) for d in devices.values()], [2, 4])
        expected = {
            'Cortex-A7': ([800, 1000, 1200, 1400, 1600], [.9, .95, 1, 1.05, 1.1]),
            'Cortex-A15': ([1000, 1250, 1500, 1750, 2000], [1.1, 1.15, 1.2, 1.25, 1.3]),
        }
        for device in devices.values():
            for core in device.cores.values():
                fs, vs = expected[core.core_type.name]
                self.assertEqual([x.frequency_mhz for x in core.core_type.dvfs_levels], fs)
                for a, b in zip([x.voltage_v for x in core.core_type.dvfs_levels], vs):
                    self.assertAlmostEqual(a, b)
                self.assertEqual(core.dvfs_level().level_id, 4)
        self.assertEqual([len(d.resource_options(True)) for d in devices.values()], [10, 20])

    def test_analytic_execution_units(self):
        d = load_platform()[0]
        self.assertEqual(d.cores[0].execution_metrics(1600, 4)['time_s'], 1)
        self.assertEqual(d.cores[1].execution_metrics(3600, 4)['time_s'], 1)
        self.assertEqual(d.cores[1].execution_metrics(1800, 0)['time_s'], 1)
        self.assertEqual(d.cores[0].execution_metrics(0)['time_s'], 0)
        self.assertEqual(d.cores[0].execution_metrics(1600, 999)['dvfs_level_id'], 4)

    def test_preview_is_pure_and_gap_reservations_serialize(self):
        core = load_platform()[0].cores[0]
        later = core.reserve(1, 1600, 2, 4)
        self.assertEqual((later['EST'], later['CT']), (2, 3))
        snapshot = [x.copy() for x in core.lane.timeline]
        preview = core.preview(800, 0, 4)
        self.assertEqual(core.lane.timeline, snapshot)
        self.assertEqual((preview['EST'], preview['CT']), (0, .5))
        gap = core.reserve(2, 800, .25, 4)
        self.assertEqual((gap['EST'], gap['CT']), (.25, .75))
        overlap = core.reserve(3, 3200, 0, 4)
        self.assertEqual((overlap['EST'], overlap['CT']), (3, 5))

    def test_ready_hi_precedes_earlier_lo(self):
        runtime = VirtualScheduler(load_platform())
        results = runtime.run([Job(1, 1600, 0, 0, rank=0), Job(2, 800, 0, 0, rank=9, criticality=1)])
        self.assertEqual([e['job_id'] for e in runtime.events if e['kind'] == 'start'], [2, 1])
        self.assertEqual(results[0]['start_s'], .5)

    def test_unready_hi_does_not_block_or_preempt(self):
        runtime = VirtualScheduler(load_platform())
        results = runtime.run([Job(1, 1600, 0, 0, rank=5), Job(2, 800, 0, 0, ready_s=.25, rank=0, criticality=1)])
        self.assertEqual([(x['start_s'], x['finish_s']) for x in results], [(0, 1), (1, 1.5)])

    def test_same_class_uses_rank_then_job_id(self):
        runtime = VirtualScheduler(load_platform())
        runtime.run([Job(5, 160, 0, 0, rank=2), Job(4, 160, 0, 0, rank=1), Job(3, 160, 0, 0, rank=1)])
        self.assertEqual([e['job_id'] for e in runtime.events if e['kind'] == 'start'], [3, 4, 5])

    def test_cores_and_devices_execute_in_parallel(self):
        results = VirtualScheduler(load_platform()).run([Job(1, 1600, 0, 0), Job(2, 3600, 0, 1), Job(3, 1600, 1, 0)])
        self.assertEqual([(x['start_s'], x['finish_s']) for x in results], [(0, 1)] * 3)

    def test_completion_and_simultaneous_arrivals_are_visible_before_dispatch(self):
        runtime = VirtualScheduler(load_platform())
        r = runtime.run([Job(1, 1600, 0, 0), Job(2, 1600, 0, 0, rank=1), Job(3, 800, 0, 0, release_s=1, criticality=1)])
        self.assertEqual([e['job_id'] for e in runtime.events if e['kind'] == 'start'], [1, 3, 2])
        self.assertEqual(r[2]['start_s'], 1)

    def test_no_default_level_leaks_between_jobs_or_runs(self):
        devices = load_platform()
        r = VirtualScheduler(devices).run([Job(1, 800, 0, 0, dvfs_level_id=0), Job(2, 1600, 0, 0, rank=1)])
        self.assertEqual([(x['metrics']['dvfs_level_id'], x['metrics']['time_s']) for x in r], [(0, 1), (4, 1)])
        r = VirtualScheduler(devices).run([Job(3, 1600, 0, 0)])
        self.assertEqual(r[0]['finish_s'], 1)
        self.assertEqual(len(devices[0].cores[0].lane.timeline), 1)

    def test_zero_work_and_empty_trace_terminate(self):
        self.assertEqual(VirtualScheduler(load_platform()).run([]), [])
        r = VirtualScheduler(load_platform()).run([Job(1, 0, 0, 0), Job(2, 0, 0, 0)])
        self.assertEqual([x['finish_s'] for x in r], [0, 0])

    def test_invalid_jobs_fail_before_dispatch(self):
        for value in (-1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                Job(1, value, 0, 0)
        with self.assertRaises(ValueError):
            Job(1, 1, 0, 0, criticality=2)
        with self.assertRaises(ValueError):
            VirtualScheduler(load_platform()).run([Job(1, 1, 99, 0)])
        with self.assertRaises(ValueError):
            VirtualScheduler(load_platform()).run([Job(1, 1, 0, 0)] * 2)

    def test_configurable_counts_and_level_grids(self):
        p = Path(__file__).resolve().parents[1] / 'config/abstract_compute.json'
        config = json.loads(p.read_text())
        config['device_classes']['vehicle'] = {'lp_cores': 0, 'hp_cores': 3}
        config['core_types']['A15']['l'] = 7
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp) / 'hardware.json'
            f.write_text(json.dumps(config))
            d = load_platform(f, device_classes=('vehicle',))[0]
            self.assertEqual(len(d.cores), 3)
            self.assertTrue(all(c.core_type.name == 'Cortex-A15' for c in d.cores.values()))
            self.assertEqual(len(d.resource_options(True)), 21)

    def test_random_release_traces_have_no_overlap_or_early_results(self):
        rng = random.SystemRandom()
        for _ in range(40):
            devices = load_platform()
            device_choices = [rng.randrange(2) for _ in range(100)]
            jobs = [Job(i, rng.uniform(0, 500), device_choices[i], rng.randrange(len(devices[device_choices[i]].cores)),
                        release_s=rng.randrange(20) / 10, ready_s=rng.randrange(20) / 10,
                        rank=rng.randrange(100), criticality=rng.randrange(2), dvfs_level_id=rng.randrange(5))
                    for i in range(100)]
            r = VirtualScheduler(devices).run(jobs)
            for item in r:
                self.assertGreaterEqual(item['start_s'], max(item['release_s'], item['ready_s']))
                self.assertAlmostEqual(item['finish_s'] - item['start_s'], item['metrics']['time_s'])
            for device in devices.values():
                for core in device.cores.values():
                    for a, b in zip(core.lane.timeline, core.lane.timeline[1:]):
                        self.assertLessEqual(a['finish'], b['start'])


if __name__ == '__main__':
    unittest.main()
