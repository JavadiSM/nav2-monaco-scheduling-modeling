import json
import random
import unittest
from pathlib import Path
from tools.abstract_compute import load_platform, GraphJob, DependencyFIFOScheduler, load_tasks, instantiate_graph, periodic_jobs

ROOT = Path(__file__).resolve().parents[1]

class TaskGraphTests(unittest.TestCase):
    def run_jobs(self, jobs, *, thermal_headroom_c=None, **kwargs):
        devices=load_platform()
        if thermal_headroom_c is not None:
            for device in devices.values():
                ambient=device.thermal_spec.ambient_temperature_c
                for core in device.cores.values():
                    core.max_temperature_c=ambient+thermal_headroom_c
                    core.balance_temperature_c=ambient+.75*thermal_headroom_c
        s = DependencyFIFOScheduler(devices)
        return s, {j['job_id']: j for j in s.run_graph(jobs, **kwargs)}

    def test_child_and_join_wait_for_all_parents(self):
        _, r = self.run_jobs([GraphJob(2,'child',1,parents=(0,1)), GraphJob(0,'parent',8), GraphJob(1,'parent',36)])
        self.assertGreaterEqual(r[2]['start_s'],max(r[0]['finish_s'],r[1]['finish_s']))
        self.assertIsNotNone(r[2]['core_id'])

    def test_blocked_child_never_reserves_a_core(self):
        _,r=self.run_jobs([GraphJob(0,'blocked',1,parents=(2,)),GraphJob(1,'independent',1),GraphJob(2,'parent',36)])
        self.assertEqual(r[1]['start_s'],0)
        self.assertEqual(r[2]['start_s'],0)

    def test_fifo_uses_readiness_and_does_not_preempt(self):
        _,r=self.run_jobs([GraphJob(0,'a',16),GraphJob(1,'b',72),GraphJob(2,'child',1,parents=(0,)),GraphJob(3,'arrival',1,.001)])
        self.assertLess(r[3]['start_s'],r[2]['start_s'])
        self.assertEqual(r[0]['start_s'],0)
        self.assertAlmostEqual(r[0]['finish_s'],.01)

    def test_oldest_idle_core_not_fastest_finish_prediction(self):
        _,r=self.run_jobs([GraphJob(0,'a',1.6),GraphJob(1,'b',7.2),GraphJob(2,'later',36,.01)])
        self.assertEqual(r[2]['core_id'],0)
        self.assertAlmostEqual(r[2]['metrics']['time_s'],.0225)

    def test_invalid_graphs_rejected_before_execution(self):
        for jobs in ([GraphJob(0,'x',1,parents=(3,))], [GraphJob(0,'x',1,parents=(1,)),GraphJob(1,'y',1,parents=(0,))], [GraphJob(0,'x',1),GraphJob(0,'y',1)]):
            with self.assertRaises(ValueError):self.run_jobs(jobs)

    def test_zero_duration_chain_and_empty(self):
        _,r=self.run_jobs([GraphJob(0,'a',0),GraphJob(1,'b',0,parents=(0,)),GraphJob(2,'c',0,parents=(1,))])
        self.assertTrue(all(v['status']=='completed' for v in r.values()))
        self.assertEqual(self.run_jobs([])[1],{})

    def test_thermal_pause_blocks_child_until_actual_finish(self):
        s,r=self.run_jobs([GraphJob(0,'hot',3600),GraphJob(1,'child',.1,parents=(0,))],max_time_s=60,thermal_headroom_c=.6)
        self.assertTrue(s.cooling_events)
        self.assertGreater(r[0]['finish_s'],r[0]['metrics']['time_s'])
        self.assertGreaterEqual(r[1]['start_s'],r[0]['finish_s'])
        self.assertAlmostEqual(r[0]['executed_mcycles'],3600,places=6)
        self.assertTrue(all(e['device_id']==0 for e in s.events if 'device_id' in e))

    def test_random_dags_no_early_child_no_core_overlap(self):
        rng=random.Random(4242)
        for trial in range(10):
            jobs=[GraphJob(i,'x',rng.uniform(.01,2),rng.random()*.03,tuple(j for j in range(i) if rng.random()<.05)) for i in range(50)]
            s,r=self.run_jobs(jobs,max_time_s=20)
            for x in r.values():
                self.assertGreaterEqual(x['start_s']+1e-12,x['release_s'])
                for p in x['parents']:self.assertGreaterEqual(x['start_s']+1e-12,r[p]['finish_s'])
            for cid in (0,1):
                lane=sorted((x for x in r.values() if x['core_id']==cid),key=lambda x:x['start_s'])
                for a,b in zip(lane,lane[1:]):self.assertLessEqual(a['finish_s'],b['start_s']+1e-12)

    def test_measured_graph_runs_with_all_primary_wcet_budgets(self):
        tasks=load_tasks(); graph=json.loads((ROOT/'docs/evidence/extracted-observed-job-dag.json').read_text())
        jobs=instantiate_graph(tasks,graph)
        _,r=self.run_jobs(jobs)
        self.assertEqual(len(r),15)
        self.assertEqual(len([j for j in jobs if not j.parents]),9)

    def test_work_units_rates_and_target_period_cycles(self):
        t=load_tasks()['control_iteration']; cores=load_platform()[0].cores
        self.assertAlmostEqual(t.work_mcycles,10.0682)
        self.assertAlmostEqual(t.activation_rate_hz,20)
        self.assertAlmostEqual(t.execution_s(cores[0]),.006292625)
        self.assertAlmostEqual(t.execution_s(cores[1]),.002796722222222222)
        self.assertAlmostEqual(t.period_cycles(cores[0]),80000000)
        self.assertAlmostEqual(t.execution_cycles(cores[1]),t.work_mcycles*1e6/1.8)

    def test_coverage_boundary_and_zero_cost(self):
        from tools.abstract_compute.communication import reachable_endpoints, communication_cost_s
        endpoints=[dict(id='e',x=3,y=4)]
        self.assertEqual(reachable_endpoints((0,0),endpoints),['e'])
        self.assertEqual(reachable_endpoints((-.001,0),endpoints),[])
        self.assertEqual(communication_cost_s(),0.)

    def test_periodic_releases_preserve_seconds_and_aperiodic_has_no_T(self):
        tasks=load_tasks(); t=tasks['control_iteration']
        self.assertEqual([j.release_s for j in periodic_jobs(t,.11)],[0,.05,.1])
        self.assertIsNone(tasks['amcl_scan_callback'].activation_rate_hz)
        with self.assertRaises(ValueError):periodic_jobs(tasks['amcl_scan_callback'],1)

if __name__=='__main__':unittest.main()
