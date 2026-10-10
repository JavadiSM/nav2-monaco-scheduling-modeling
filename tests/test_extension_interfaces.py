import unittest
from tools.live_bridge.engine import LiveEngine
from tools.live_bridge.policies import Assignment,ReadyFIFOOldestIdle
from tools.abstract_compute.communication import CommunicationModel,communication_cost_s
from tools.abstract_compute.hardware import load_platform


class ReverseFastest(ReadyFIFOOldestIdle):
    name='test_reverse_ready_fastest_free'
    def assignments(self,*,engine,device_id,ready_jobs,free_cores,**kwargs):
        jobs=sorted(ready_jobs,key=lambda j:j.job_id,reverse=True)
        cores=sorted(free_cores,key=lambda c:c.core_type.max_frequency_mhz*c.core_type.performance_eta,reverse=True)
        return [Assignment(j.job_id,c.core_id) for j,c in zip(jobs,cores)]
    def dvfs_level(self,**kwargs):return 1


class ExtensionInterfaceTests(unittest.TestCase):
    def test_custom_policy_replaces_FIFO_and_core_order_and_DVFS(self):
        from platform_fixtures import LiveEngine as MulticoreEngine
        e=MulticoreEngine(scheduling_policy=ReverseFastest())
        ids=[e.arrive(0,'control_iteration') for _ in range(3)]
        for j in ids:e.stage(j,0,dispatch=False)
        e.dispatch()
        self.assertEqual((e.jobs[ids[2]].core_id,e.jobs[ids[1]].core_id),(1,0))
        self.assertIsNone(e.jobs[ids[0]].core_id);e.advance()
        for j in ids[1:]:self.assertEqual(e.jobs[j].execution_segments[0]['level_id'],1)

    def test_policy_cannot_start_blocked_child_or_assign_one_core_twice(self):
        class Invalid(ReadyFIFOOldestIdle):
            def assignments(self,**kwargs):return self.decisions
        policy=Invalid();policy.decisions=[];e=LiveEngine(scheduling_policy=policy)
        parent=e.arrive(0,'control_iteration');child=e.arrive(0,'bt_tick',[parent])
        e.stage(child,0,dispatch=False);e.stage(parent,0,dispatch=False)
        policy.decisions=[Assignment(child,0)]
        with self.assertRaises(RuntimeError):e.dispatch()
        other=e.arrive(0,'bt_tick');e.stage(other,0,dispatch=False)
        policy.decisions=[Assignment(parent,0),Assignment(other,0)]
        with self.assertRaises(RuntimeError):e.dispatch()
        self.assertTrue(all(j.start_tick is None for j in e.jobs.values()))

    def test_default_policy_still_maximum_and_FIFO(self):
        e=LiveEngine();j=e.arrive(0,'bt_tick');e.stage(j,0);e.advance()
        self.assertEqual(e.scheduling_policy.name,ReadyFIFOOldestIdle.name)
        self.assertEqual(e.jobs[j].execution_segments[0]['level_id'],4)

    def test_request_coverage_only_and_cost_function_used_in_both_directions(self):
        calls=[]
        def cost(**kwargs):calls.append(kwargs);return .2 if kwargs['direction']=='request' else .3
        model=CommunicationModel(cost_function=cost);endpoint={'id':'edge_test','x':3.,'y':4.}
        request=model.send_request(position_xy=(0.,0.),endpoint=endpoint,send_time_s=1.,payload_bytes=1024)
        self.assertEqual(request.arrives_at_s,1.2)
        result=model.send_result(request,send_time_s=5.,position_xy=(999.,999.),payload_bytes=2048)
        self.assertEqual(result.arrives_at_s,5.3)
        self.assertEqual([r['direction'] for r in calls],['request','result'])
        self.assertEqual(calls[1]['payload_bytes'],2048)
        with self.assertRaises(ConnectionError):model.send_request(position_xy=(-.001,0.),endpoint=endpoint,send_time_s=1.)
        with self.assertRaises(ValueError):model.send_result(request,send_time_s=1.1)
        self.assertEqual(communication_cost_s(any_future_parameter='x'),0.)

    def test_zero_cost_and_invalid_custom_delays(self):
        endpoint={'id':'edge_test','x':0.,'y':0.}
        model=CommunicationModel();request=model.send_request(position_xy=(0,0),endpoint=endpoint,send_time_s=2.)
        self.assertEqual((request.delay_s,request.arrives_at_s),(0.,2.))
        self.assertEqual(model.send_result(request,send_time_s=3.).delay_s,0.)
        for delay in (-1,float('inf'),float('nan')):
            model=CommunicationModel(cost_function=lambda **kwargs:delay)
            with self.assertRaises(ValueError):model.send_request(position_xy=(0,0),endpoint=endpoint,send_time_s=2.)

    def test_device_class_specific_thermal_parameters(self):
        devices=load_platform()
        for did,cores,tmax,balance in ((0,1,46.2,45.6),(1,1,46.5,46.0)):
            d=devices[did];self.assertEqual(len(d.cores),cores)
            self.assertEqual((d.thermal_spec.max_temperature_c,d.thermal_spec.balance_temperature_c),(tmax,balance))
            for c in d.cores.values():self.assertEqual((c.max_temperature_c,c.balance_temperature_c,c.initial_temperature_c),(tmax,balance,45.))



class RoadsideLayoutTests(unittest.TestCase):
    def test_consecutive_RSU_spacing_and_resources(self):
        import json,math
        from pathlib import Path
        from tools.abstract_compute.edge_resources import load_edge_resources
        root=Path(__file__).resolve().parents[1];scene=json.loads((root/'scenarios/monaco/scenario.json').read_text())
        endpoints=scene['servers'];ordered=sorted(endpoints,key=lambda e:e['route_s_m'])
        self.assertEqual(len({e['id'] for e in endpoints}),len(endpoints))
        self.assertEqual(ordered[0]['placement_role'],'start');self.assertEqual(ordered[-1]['placement_role'],'finish')
        self.assertEqual(ordered[0]['route_s_m'],scene['start']['s']);self.assertEqual(ordered[-1]['route_s_m'],scene['finish']['s'])
        for a,b in zip(ordered,ordered[1:]):self.assertLessEqual(math.dist([a['x'],a['y']],[b['x'],b['y']]),10.)
        resources=load_edge_resources(scene)
        self.assertEqual({r.processor.processor_id for r in resources.values()},set(range(1,len(endpoints)+1)))
        for endpoint in endpoints:
            r=resources[endpoint['id']];self.assertEqual(r.position_xy,(endpoint['x'],endpoint['y']))
            self.assertEqual([c.core_type.family for c in r.processor.cores.values()],['ARM_HP'])
            self.assertTrue(all(c.max_temperature_c==46.5 and c.balance_temperature_c==46.0 for c in r.processor.cores.values()))

if __name__=='__main__':unittest.main()
