import math
import unittest
from tools.abstract_compute.task_model import load_tasks
from tools.live_bridge.engine import LiveEngine
import test_live_bridge as legacy_tests

from platform_fixtures import load_platform, LiveEngine


class DualBudgetTests(unittest.TestCase):
    def test_every_current_task_is_HI_with_mean_and_max(self):
        import json
        from tools.abstract_compute.task_model import ROOT
        source=json.loads((ROOT/'docs/evidence/task-characterization-summary.json').read_text())
        tasks=load_tasks(dual=True)
        self.assertEqual(len(tasks),11)
        for name,t in tasks.items():
            m=source['tasks'][name]['metrics']['cpu_inclusive_ms']
            self.assertEqual(t.criticality,'HI')
            self.assertEqual(t.budget_lo_s,m['mean']/1000.)
            self.assertEqual(t.budget_hi_s,m['max']/1000.)
            self.assertLessEqual(t.budget_lo_s,t.budget_hi_s)

    def test_missing_measurement_cannot_stage_or_dispatch(self):
        e=LiveEngine();j=e.arrive(0,'control_iteration')
        with self.assertRaises(RuntimeError):e.stage(j)
        for _ in range(10):e.advance()
        self.assertIsNone(e.jobs[j].start_tick)

    def test_comparison_is_unrounded_CPU_not_wall_or_target_time(self):
        from dataclasses import replace
        t=replace(load_tasks(dual=True)['control_iteration'],budget_lo_s=.001,budget_hi_s=.004)
        self.assertEqual(t.select_budget(1_000_000),('LO',.001,False))
        self.assertEqual(t.select_budget(1_000_001),('HI',.004,False))
        self.assertEqual(t.select_budget(4_000_001),('HI',.004,True))
        with self.assertRaises(ValueError):t.select_budget(-1)

    def test_selected_budget_converts_and_rounds_on_both_cores(self):
        for mode in ('LO','HI'):
            e=LiveEngine();spec=e.tasks['control_iteration']
            cpu=0 if mode=='LO' else math.floor(spec.budget_lo_s*1e9)+1
            ids=[e.arrive(0,'control_iteration') for _ in range(2)]
            for j in ids:e.stage(j,cpu)
            self.assertEqual({e.jobs[j].core_id for j in ids},{0,1})
            for j in ids:
                job=e.jobs[j];core=e.devices[0].cores[job.core_id]
                duration=spec.reference_work(job.selected_budget_s)/(core.dvfs_level().frequency_mhz*core.core_type.performance_eta)
                self.assertEqual(job.budget_mode,mode)
                job.expected_ticks=math.ceil(duration/e.dt-1e-9)
            for _ in range(max(e.jobs[j].expected_ticks for j in ids)):e.advance()
            for j in ids:self.assertEqual(e.jobs[j].budget_end_tick,e.jobs[j].expected_ticks)

    def test_HI_total_replaces_LO_instead_of_adding_it_twice(self):
        e=LiveEngine();j=e.arrive(0,'control_iteration');spec=e.tasks['control_iteration']
        e.stage(j,math.floor(spec.budget_lo_s*1e9)+1)
        self.assertEqual(e.jobs[j].remaining,spec.reference_work(spec.budget_hi_s))

    def test_dependencies_and_cooling_hold_outputs(self):
        e=LiveEngine();parent=e.arrive(0,'planning_request');child=e.arrive(0,'controller_path_install',[parent])
        e.stage(child,0);e.stage(parent,10**9)
        self.assertTrue(e.jobs[parent].observed_max_exceeded)
        self.assertIsNone(e.jobs[child].start_tick)
        while not e.deliverable(parent):e.advance()
        e.cooling[0]=True
        self.assertFalse(e.deliverable(parent))
        with self.assertRaises(RuntimeError):e.finish(parent)
        e.cooling[0]=False;e.finish(parent)
        self.assertEqual(e.jobs[child].start_tick,e.jobs[parent].finish_tick)


class SealedProtocolTests(unittest.TestCase):
    def test_output_is_rejected_before_seal_and_selected_completion(self):
        from tools.live_bridge.broker import ENTER,STAGE,PUBLISH,END,TRIGGER
        b,c=legacy_tests.LiveProtocolTests().fixture();b.engine=LiveEngine()
        b.packet(c,(ENTER,0,0,0,0,0,0));j=b.clients[c]['job']
        with self.assertRaises(RuntimeError):b.packet(c,(STAGE,j,0,0,0,0,0))
        with self.assertRaises(RuntimeError):b.packet(c,(PUBLISH,j,123,100,200,0,0))
        b.packet(c,(TRIGGER,j,456,0,0,0,0))
        self.assertEqual(b.engine.jobs[j].status,'shadow')
        cpu=math.floor(b.engine.tasks['control_iteration'].budget_lo_s*1e9)+1
        b.packet(c,(STAGE,j,cpu,1,0,0,0));b.release()
        self.assertNotEqual(b.engine.jobs[j].status,'completed')
        while not b.engine.deliverable(j):b.engine.advance()
        b.release();self.assertEqual(b.engine.jobs[j].budget_mode,'HI')
        b.packet(c,(PUBLISH,j,123,100,200,0,0))
        with self.assertRaises(RuntimeError):b.packet(c,(END,j,cpu+1,0,0,0,0))
        b.packet(c,(END,j,cpu,0,0,0,0))

    def test_cached_action_result_waits_for_complete_output_commit(self):
        from tools.live_bridge.broker import RESULT_GATE,PUBLISH
        b,c=legacy_tests.LiveProtocolTests().fixture();b.engine=LiveEngine()
        j=b.engine.arrive(0,'planning_request');b.engine.stage(j,0)
        while not b.engine.deliverable(j):b.engine.advance()
        b.engine.finish(j)
        b.packet(c,(RESULT_GATE,j,0,0,0,0,0));b.release()
        self.assertFalse(c.replies)
        b.engine.jobs[j].outputs_committed=True;b.release()
        self.assertTrue(c.replies);self.assertTrue(b.clients[c]['busy'])
        b.packet(c,(PUBLISH,j,0,0,0,0,0));self.assertFalse(b.clients[c]['busy'])

    def test_DDS_take_before_publish_registration_keeps_exact_parent(self):
        from tools.live_bridge.broker import PUBLISH_BEGIN,PUBLISH,LOOKUP,PACKET
        b,c=legacy_tests.LiveProtocolTests().fixture();b.engine=LiveEngine()
        j=b.engine.arrive(0,'control_iteration');b.engine.stage(j,0)
        while not b.engine.deliverable(j):b.engine.advance()
        b.engine.finish(j)
        sender=type(c)();b.clients[sender]={'job':j,'busy':True,'wait':None}
        b.packet(sender,(PUBLISH_BEGIN,j,123,100,0,0,0))
        b.packet(c,(LOOKUP,123,0,150,0,0,0));b.release()
        self.assertFalse(c.replies)
        b.packet(sender,(PUBLISH,j,123,100,200,0,0));b.release()
        self.assertEqual(PACKET.unpack(c.replies[-1])[1],j)
        self.assertEqual(b.stats['lookup_misses'],0)


if __name__=='__main__':unittest.main()
