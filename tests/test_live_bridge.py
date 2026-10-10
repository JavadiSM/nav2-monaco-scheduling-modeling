import math,unittest
from tools.live_bridge.engine import LiveEngine
from platform_fixtures import load_platform, LiveEngine


class LiveBridgeTests(unittest.TestCase):
    def test_only_observed_entries_create_jobs(self):
        m=LiveEngine(1,step_ns=1_000_000,dual_budget=False)
        for _ in range(100):m.advance()
        self.assertFalse(m.jobs)
        with self.assertRaises(ValueError):LiveEngine(1,3_000_000,dual_budget=False)

    def test_q95_budget_and_one_ms_quantization(self):
        m=LiveEngine(1,step_ns=1_000_000,dual_budget=False);j=m.arrive(0,'control_iteration');m.stage(j)
        c=m.devices[0].cores[0];spec=m.tasks['control_iteration'];ticks=math.ceil(spec.execution_s(c)/m.dt-1e-9)
        for _ in range(ticks-1):m.advance();self.assertFalse(m.deliverable(j))
        m.advance();self.assertTrue(m.deliverable(j));self.assertEqual(m.tick,ticks)

    def test_ten_ms_quantizes_budget_and_waits_for_parent(self):
        m=LiveEngine(step_ns=10_000_000,dual_budget=False);self.assertEqual(m.step_ns,10_000_000)
        parent=m.arrive(0,'planning_request');child=m.arrive(0,'controller_path_install',[parent])
        m.stage(child);m.stage(parent)
        ticks=math.ceil(m.tasks['planning_request'].execution_s(m.devices[0].cores[0])/m.dt-1e-9)
        self.assertEqual(ticks,4)
        for _ in range(ticks-1):
            m.advance();self.assertFalse(m.deliverable(parent));self.assertIsNone(m.jobs[child].start_tick)
        m.advance();self.assertTrue(m.deliverable(parent));self.assertIsNone(m.jobs[child].start_tick)
        m.finish(parent);self.assertEqual(m.jobs[child].start_tick,ticks)
        m.advance();self.assertTrue(m.deliverable(child));self.assertEqual(m.tick,ticks+1)

    def test_real_parent_completion_blocks_child(self):
        m=LiveEngine(1,step_ns=1_000_000,dual_budget=False);p=m.arrive(0,'planning_request');c=m.arrive(0,'controller_path_install',[p]);m.stage(c);m.stage(p)
        self.assertIsNone(m.jobs[c].start_tick)
        while not m.deliverable(p):m.advance()
        self.assertIsNone(m.jobs[c].start_tick)
        m.finish(p);self.assertEqual(m.jobs[c].start_tick,m.jobs[p].finish_tick)

    def test_fifo_and_oldest_idle_not_fastest_core(self):
        m=LiveEngine(1,step_ns=1_000_000,dual_budget=False);a=m.arrive(0,'planning_request');b=m.arrive(0,'bt_tick');m.stage(a);m.stage(b)
        self.assertEqual((m.jobs[a].core_id,m.jobs[b].core_id),(0,1))
        while not m.deliverable(b):m.advance()
        m.finish(b)
        while not m.deliverable(a):m.advance()
        m.finish(a);c=m.arrive(0,'bt_tick');m.stage(c)
        self.assertEqual(m.jobs[c].core_id,1)

    def test_devices_are_independent_and_cross_device_edges_rejected(self):
        m=LiveEngine(4,step_ns=1_000_000,dual_budget=False);a=m.arrive(0,'planning_request');b=m.arrive(1,'bt_tick');m.stage(a);m.stage(b)
        self.assertEqual(m.jobs[a].start_tick,m.jobs[b].start_tick)
        with self.assertRaises(ValueError):m.add_parent(b,a)
        with self.assertRaises(ValueError):m.add_parent(a,a)

    def test_device_cooling_holds_both_cores_and_recovers(self):
        m=LiveEngine(1,step_ns=1_000_000,dual_budget=False);a=m.arrive(0,'planning_request');b=m.arrive(0,'control_iteration');m.stage(a);m.stage(b)
        for c in m.devices[0].cores.values():c.temperature_c=c.max_temperature_c+.1
        m.advance();self.assertTrue(m.cooling[0]);remaining=[m.jobs[j].remaining for j in (a,b)]
        m.advance();self.assertEqual(remaining,[m.jobs[j].remaining for j in (a,b)])
        for c in m.devices[0].cores.values():self.assertTrue(c.thermal_forced_idle)
        for _ in range(100000):
            m.advance()
            if not m.cooling[0]:break
        self.assertFalse(m.cooling[0]);self.assertTrue(all(c.temperature_c<=c.balance_temperature_c+1e-9 for c in m.devices[0].cores.values()))
        self.assertTrue(any(e['kind']=='cooling_end' for e in m.events))

    def test_cycle_and_dependency_discovered_too_late_fail(self):
        m=LiveEngine(1,step_ns=1_000_000,dual_budget=False);a=m.arrive(0,'bt_tick');b=m.arrive(0,'bt_tick',[a])
        with self.assertRaises(RuntimeError):m.add_parent(a,b)
        m.stage(a)
        while not m.deliverable(a):m.advance()
        m.finish(a);m.stage(b)
        while not m.deliverable(b):m.advance()
        m.finish(b)
        with self.assertRaises(RuntimeError):m.add_parent(a,b)


class LiveProtocolTests(unittest.TestCase):
    def fixture(self):
        import collections,io
        from tools.live_bridge.broker import Broker
        class Client:
            def __init__(self):self.replies=[]
            def send(self,value):self.replies.append(value);return len(value)
            def fileno(self):return 10
        b=Broker.__new__(Broker);b.engine=LiveEngine(1,step_ns=1_000_000,dual_budget=False);b.clients={};b.producers={};b.expected_commands=[0];b.stats=collections.defaultdict(int);b.rows=io.StringIO();c=Client();b.clients[c]={'job':None,'busy':False,'wait':None};return b,c
    def test_stage_waits_for_budget_and_DDS_identity_is_time_bound(self):
        from tools.live_bridge.broker import ENTER,STAGE,END,PUBLISH,LOOKUP,PACKET
        b,c=self.fixture();b.packet(c,(ENTER,0,0,0,0,0,0));jid=b.clients[c]['job'];b.packet(c,(STAGE,jid,0,0,0,0,0))
        self.assertIsNone(b.engine.jobs[jid].start_tick);b.engine.dispatch()
        while not b.engine.deliverable(jid):b.engine.advance()
        b.release();self.assertEqual(b.engine.jobs[jid].status,'completed');b.packet(c,(END,jid,1000,0,0,0,0))
        b.packet(c,(PUBLISH,jid,123,100,200,0,0));b.packet(c,(LOOKUP,123,0,150,0,0,0));self.assertEqual(PACKET.unpack(c.replies[-1])[1],jid)
        b.packet(c,(LOOKUP,123,0,250,0,0,0));self.assertEqual(PACKET.unpack(c.replies[-1])[1],0)
        b.packet(c,(LOOKUP,123,1,150,0,0,0));self.assertEqual(PACKET.unpack(c.replies[-1])[1],0)
    def test_mutex_wait_after_completion_keeps_publication_on_same_tick(self):
        from tools.live_bridge.broker import ENTER,STAGE,WAIT,RESUME,PUBLISH,END
        b,c=self.fixture();b.packet(c,(ENTER,0,0,0,0,0,0));jid=b.clients[c]['job'];b.packet(c,(STAGE,jid,0,0,0,0,0));b.engine.dispatch()
        while not b.engine.deliverable(jid):b.engine.advance()
        b.release();tick=b.engine.tick;self.assertTrue(b.clients[c]['busy'])
        b.packet(c,(WAIT,0,0,0,0,0,0));self.assertTrue(b.clients[c]['busy'])
        b.packet(c,(RESUME,0,0,0,0,0,0));b.packet(c,(PUBLISH,jid,123,100,200,0,0))
        publication=b.engine.events[-1];self.assertEqual(publication['kind'],'publication');self.assertEqual(publication['tick'],tick)
        b.packet(c,(END,jid,1000,0,0,0,0));self.assertFalse(b.clients[c]['busy'])
    def test_computation_mutex_wait_still_allows_modeled_work_to_advance(self):
        from tools.live_bridge.broker import ENTER,WAIT,RESUME
        b,c=self.fixture();b.packet(c,(ENTER,0,0,0,0,0,0));jid=b.clients[c]['job']
        self.assertNotEqual(b.engine.jobs[jid].status,'completed');b.packet(c,(WAIT,0,0,0,0,0,0));self.assertFalse(b.clients[c]['busy'])
        b.packet(c,(RESUME,0,0,0,0,0,0));self.assertTrue(b.clients[c]['busy'])
    def test_distance_cutoff_waits_only_for_released_output_commits(self):
        b,c=self.fixture();b.config={'settle_quiet_host_s':.002};b.last_activity=1.
        j=b.engine.arrive(0,'bt_tick');b.engine.stage(j)
        while not b.engine.deliverable(j):b.engine.advance()
        b.engine.finish(j)
        self.assertFalse(b.cutoff_quiescent(2.))
        b.engine.jobs[j].outputs_committed=True
        self.assertTrue(b.cutoff_quiescent(2.))
        waiting=b.engine.arrive(0,'planning_request');b.engine.stage(waiting)
        self.assertTrue(b.cutoff_quiescent(2.))
        b.clients[c]['busy']=True;self.assertFalse(b.cutoff_quiescent(2.))
        b.clients[c]['busy']=False;self.assertFalse(b.cutoff_quiescent(1.001))

    def test_unclassified_actuation_is_held_during_device_cooling(self):
        from tools.live_bridge.broker import DEVICE_GATE,PUBLISH
        b,c=self.fixture();b.engine.cooling[0]=True;b.packet(c,(DEVICE_GATE,0,0,0,0,0,0));b.release();self.assertFalse(c.replies);self.assertFalse(b.engine.jobs)
        b.engine.cooling[0]=False;b.release();self.assertEqual(len(c.replies),1);self.assertTrue(b.clients[c]['busy'])
        b.packet(c,(PUBLISH,0,123,100,200,1,0));self.assertFalse(b.clients[c]['busy']);self.assertEqual(b.expected_commands,[123])

class ActualReadinessTests(unittest.TestCase):
    def test_shadow_wait_does_not_backdate_FIFO_readiness(self):
        m=LiveEngine(1,step_ns=1_000_000,dual_budget=False);a=m.arrive(0,'control_iteration')
        for _ in range(5):m.advance()
        m.stage(a);self.assertEqual(m.jobs[a].staged_tick,5);self.assertEqual(m.jobs[a].ready_tick,5)


class CommonStartTests(unittest.TestCase):
    def test_four_actual_goals_and_first_jobs_share_one_tick(self):
        import tempfile,time
        from pathlib import Path
        b,c=LiveProtocolTests().fixture();b.engine=LiveEngine(4,step_ns=1_000_000,dual_budget=False);b.engine.tick=6000;b.bootstrap=6000;b.armed=False;b.seconds=12
        b.start_ready={0,1,2};b.start_acked=set();b.origin_tick=None;b.dispatch_started=False;b.clock=bytearray(32)
        with tempfile.TemporaryDirectory() as folder:
            b.out=Path(folder);(b.out/'nav2-ready').touch();b.start_barrier();self.assertFalse(b.armed)
            b.start_ready.add(3);b.start_barrier();self.assertEqual(b.origin_tick,6000)
            jobs=[b.engine.arrive(d,'bt_tick') for d in range(4)];b.start_acked=set(range(4))
            for j in jobs[:3]:b.engine.stage(j,dispatch=False)
            b.start_barrier();self.assertFalse(b.dispatch_started)
            b.engine.stage(jobs[3],dispatch=False);b.start_barrier();self.assertTrue(b.dispatch_started)
            b.engine.dispatch();self.assertEqual({b.engine.jobs[j].start_tick for j in jobs},{6000})
            b.clients={}
            for j in jobs:
                client=type(c)();b.clients[client]={'job':j,'busy':False,'wait':('stage',j)}
            b.engine.advance();b.release()
            self.assertEqual({b.engine.jobs[j].finish_tick for j in jobs},{6001})
            from tools.live_bridge.broker import PACKET
            self.assertEqual({PACKET.unpack(cl.replies[-1])[2] for cl in b.clients},{6001000000})

if __name__=='__main__':unittest.main()
