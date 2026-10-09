import unittest
from dataclasses import asdict,replace
from tools.live_bridge.engine import LiveEngine
from tools.live_bridge.dvfs import configured_selector,validate_job_execution


class LiveDVFSTests(unittest.TestCase):
    def fixture(self,selector=None,lo=.02,hi=.04):
        e=LiveEngine(dvfs_selector=selector)
        e.tasks['control_iteration']=replace(e.tasks['control_iteration'],budget_lo_s=lo,budget_hi_s=hi)
        return e

    def finish_budget(self,e,j):
        for _ in range(10000):
            if e.deliverable(j):return
            e.advance()
        self.fail('Budget did not complete')

    def check_accounting(self,e,j):
        mask=[False]*e.tick;opened=None
        for event in e.events:
            if event['kind']=='cooling_start':opened=event['tick']
            elif event['kind']=='cooling_end':mask[opened:event['tick']]=[True]*(event['tick']-opened);opened=None
        if opened is not None:mask[opened:]=[True]*(e.tick-opened)
        row=asdict(e.jobs[j]);core=e.devices[0].cores[row['core_id']]
        self.assertEqual(validate_job_execution(row,e.tasks[row['task']],core,e.dt,mask,e.tick),[])
        return row,core,mask

    def test_maximum_default_and_no_carryover_to_next_job(self):
        e=self.fixture();e.devices[0].cores[0].current_dvfs_level_id=0
        j=e.arrive(0,'control_iteration');e.stage(j,0);self.finish_budget(e,j)
        self.assertEqual({s['level_id'] for s in e.jobs[j].execution_segments},{4})
        self.assertEqual(e.jobs[j].budget_end_tick,13)
        self.check_accounting(e,j)

    def test_scheduler_changes_point_during_one_job_without_resetting_work(self):
        def select(e,j,core):
            elapsed=e.tick-j.start_tick
            return 0 if elapsed<3 else (2 if elapsed<5 else None)
        e=self.fixture(select);j=e.arrive(0,'control_iteration');e.stage(j,0)
        self.finish_budget(e,j)
        spans=e.jobs[j].execution_segments
        self.assertEqual([(s['start_tick'],s['end_tick'],s['level_id']) for s in spans],[(0,3,0),(3,5,2),(5,15,4)])
        self.assertAlmostEqual(sum(s['work_mcycles'] for s in spans),20.)
        self.assertIsNone(e.jobs[j].overrun_tick)
        self.check_accounting(e,j)

    def test_HI_remainder_forces_max_at_first_LO_crossing_boundary(self):
        e=self.fixture(lambda e,j,c:0,lo=.008,hi=.02)
        j=e.arrive(0,'control_iteration');e.stage(j,8_000_001)
        self.finish_budget(e,j)
        self.assertEqual(e.jobs[j].overrun_tick,10)
        self.assertEqual(e.jobs[j].budget_end_tick,18)
        self.assertEqual([(s['level_id'],s['end_tick']) for s in e.jobs[j].execution_segments],[(0,10),(4,18)])
        self.assertAlmostEqual(e.jobs[j].consumed_work_mcycles,20.)
        self.assertEqual(len([v for v in e.events if v['kind']=='job_overrun']),1)
        self.check_accounting(e,j)
        e.finish(j);next_j=e.arrive(0,'control_iteration');e.stage(next_j,0);e.advance()
        self.assertEqual(e.jobs[next_j].execution_segments[0]['level_id'],0)

    def test_noninteger_crossing_uses_next_one_ms_boundary(self):
        e=self.fixture(lambda e,j,c:0,lo=.0081,hi=.02)
        j=e.arrive(0,'control_iteration');e.stage(j,8_100_001);self.finish_budget(e,j)
        self.assertEqual(e.jobs[j].overrun_tick,11)
        self.assertEqual(e.jobs[j].budget_end_tick,18)
        self.check_accounting(e,j)

    def test_power_uses_selected_voltage_frequency_on_both_core_types(self):
        e=self.fixture(lambda e,j,c:0)
        ids=[e.arrive(0,'control_iteration') for _ in range(2)]
        for j in ids:e.stage(j,0)
        expected=[]
        for c in e.devices[0].cores.values():expected.append(e.power.runtime_power_w(c,is_active=True,dvfs_level_id=0))
        e.advance()
        self.assertEqual(e.samples[0]['power_w'],expected)
        self.assertEqual([e.jobs[j].execution_segments[0]['frequency_mhz'] for j in ids],[800.,1000.])
        self.assertEqual([e.jobs[j].execution_segments[0]['voltage_v'] for j in ids],[.9,1.1])

    def test_cooling_freezes_work_and_child_waits_for_released_parent(self):
        e=self.fixture(lambda e,j,c:0,lo=.008,hi=.02)
        parent=e.arrive(0,'control_iteration');child=e.arrive(0,'controller_path_install',[parent])
        e.stage(child,0);e.stage(parent,8_000_001);e.advance()
        for c in e.devices[0].cores.values():c.temperature_c=c.max_temperature_c+.1
        e.advance();self.assertTrue(e.cooling[0]);before=e.jobs[parent].remaining
        e.advance();self.assertEqual(e.jobs[parent].remaining,before)
        self.assertEqual(e.samples[0]['power_w'][0],e.power.average_power_w(e.devices[0].cores[0],.5,0,temperature_c=45.))
        for c in e.devices[0].cores.values():self.assertEqual(e.power.runtime_power_w(c,is_active=True),c.core_type.cooling_power_w)
        self.finish_budget(e,parent);self.assertIsNone(e.jobs[child].start_tick)
        self.check_accounting(e,parent);e.finish(parent)
        self.assertEqual(e.jobs[child].start_tick,e.jobs[parent].finish_tick)

    def test_invalid_policy_level_and_configuration_are_rejected(self):
        for level in (-1,5,True,1.5):
            e=self.fixture(lambda e,j,c,level=level:level);j=e.arrive(0,'control_iteration');e.stage(j,0)
            with self.assertRaises(ValueError):e.advance()
        for dvfs in ({'policy':'other'},{'policy':'maximum','switch_latency_s':.001},{'decision_epoch_s':.01}):
            with self.assertRaises(ValueError):configured_selector({'step_ns':1_000_000,'dvfs':dvfs})

    def test_independent_validator_detects_early_finish_and_wrong_point(self):
        e=self.fixture(lambda e,j,c:0);j=e.arrive(0,'control_iteration');e.stage(j,0);self.finish_budget(e,j)
        row,core,mask=self.check_accounting(e,j)
        row['execution_segments'][0]['voltage_v']=1.234
        self.assertTrue(validate_job_execution(row,e.tasks[row['task']],core,e.dt,mask,e.tick))
        row=asdict(e.jobs[j]);row['budget_end_tick']-=1
        self.assertTrue(validate_job_execution(row,e.tasks[row['task']],core,e.dt,mask,e.tick))


if __name__=='__main__':unittest.main()
