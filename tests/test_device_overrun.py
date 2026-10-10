"""System-level overrun reactions affect every core of one device only."""
from dataclasses import replace,asdict
import unittest
from platform_fixtures import LiveEngine
from tools.live_bridge.dvfs import validate_job_execution

RULE='device_maximum_until_overruns_complete'

class DeviceOverrunTests(unittest.TestCase):
    def make(self,vehicles=1):
        e=LiveEngine(vehicles,dvfs_selector=lambda e,j,c:2,overrun_policy=RULE)
        e.tasks['control_iteration']=replace(e.tasks['control_iteration'],budget_lo_s=.008,budget_hi_s=.04)
        e.tasks['planning_request']=replace(e.tasks['planning_request'],budget_lo_s=.1,budget_hi_s=.2)
        return e

    def test_other_core_boosts_without_own_overrun_and_other_device_does_not(self):
        e=self.make(2)
        a=e.arrive(0,'control_iteration');b=e.arrive(0,'planning_request');c=e.arrive(1,'planning_request')
        e.stage(a,8_000_001);e.stage(b,0);e.stage(c,0)
        for _ in range(8):e.advance()
        self.assertEqual(e.jobs[a].overrun_tick,7)
        self.assertIsNone(e.jobs[b].overrun_tick)
        self.assertEqual(e.jobs[b].execution_segments[-1]['level_id'],4)
        self.assertEqual(e.jobs[c].execution_segments[-1]['level_id'],2)
        self.assertEqual(next(v for v in e.events if v['kind']=='device_boost_start')['core_ids'],[0,1])
        while not e.deliverable(a):e.advance()
        e.finish(a);self.assertFalse(e.overrun_jobs[0]);e.advance()
        self.assertEqual(e.jobs[b].execution_segments[-1]['level_id'],2)
        mask=[False]*(e.tick+1)
        for event in e.events:
            if event['kind']=='device_boost_start':start=event['tick']
            if event['kind']=='device_boost_end':mask[start:event['tick']]=[True]*(event['tick']-start)
        row=asdict(e.jobs[b]);core=e.devices[0].cores[row['core_id']]
        self.assertEqual(validate_job_execution(row,e.tasks[row['task']],core,e.dt,[False]*(e.tick+1),e.tick,overrun_policy=RULE,device_boost_mask=mask),[])

    def test_last_active_overrun_controls_device_recovery(self):
        e=self.make();e.tasks['planning_request']=replace(e.tasks['planning_request'],budget_lo_s=.003,budget_hi_s=.1)
        a=e.arrive(0,'control_iteration');b=e.arrive(0,'planning_request')
        e.stage(a,8_000_001);e.stage(b,3_000_001)
        while not e.deliverable(a):e.advance()
        self.assertEqual(e.overrun_jobs[0],{a,b})
        e.finish(a);self.assertEqual(e.overrun_jobs[0],{b})
        e.advance();self.assertEqual(e.jobs[b].execution_segments[-1]['level_id'],4)
        while not e.deliverable(b):e.advance()
        e.finish(b);self.assertFalse(e.overrun_jobs[0])
        self.assertEqual(sum(v['kind']=='device_boost_start' for v in e.events),1)
        self.assertEqual(sum(v['kind']=='device_boost_end' for v in e.events),1)

if __name__=='__main__':unittest.main()
