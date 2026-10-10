import math,unittest
from dataclasses import replace
from tools.live_bridge.engine import LiveEngine
from tools.live_bridge.policies import FIFOShortestFinish
from tools.live_bridge.placement import DevicePlacement
from tools.live_bridge.deadlines import generate_deadlines,validate_deadlines
from tools.abstract_compute.communication import task_upload_cost_s,edge_data_cost_s

SCENE={'start':{'x':0.,'y':0.},'servers':[{'id':'a','x':2.,'y':0.},{'id':'b','x':8.,'y':0.}]}

def engine(policy='offload',**kwargs):
    return LiveEngine(edge_config={'placement':policy},scene=SCENE,scheduling_policy=FIFOShortestFinish(),**kwargs)

def finish(e,j):
    for _ in range(10000):
        if e.deliverable(j):e.finish(j);return
        e.advance()
    raise AssertionError('No modeled completion')

class EdgePlacementTests(unittest.TestCase):
    def test_delay_boundaries_and_extensible_size_inputs(self):
        for distance,expected in [(0,0),(.999,0),(1,.003),(1.999,.003),(2,.004),(2.999,.004),(3,.005),(5,.005),(5.001,.010)]:
            self.assertEqual(task_upload_cost_s(distance_m=distance,task_size_bytes=12,custom='x'),expected)
            self.assertEqual(edge_data_cost_s(distance_m=distance,edge_size_bytes=10,custom='y'),expected)
        for invalid in (-1,float('inf'),float('nan')):
            with self.assertRaises(ValueError):task_upload_cost_s(distance_m=invalid)

    def test_nearest_and_fallback_and_random_scope(self):
        p=DevicePlacement('offload');endpoints={1:(2,0),2:(4,0)}
        self.assertEqual(p.choose(owner_device=0,position_xy=(0,0),endpoints=endpoints)[0],1)
        self.assertEqual(p.choose(owner_device=0,position_xy=(10,0),endpoints=endpoints)[0],0)
        a=DevicePlacement('random')
        values=[a.choose(owner_device=0,position_xy=(0,0),endpoints={1:(2,0),2:(8,0)})[0] for _ in range(30)]
        self.assertTrue(set(values)<={0,1})

    def test_calibration_uses_measured_work_without_updating_reference_budgets(self):
        e=engine('local',calibration_actual_work=True);spec=e.tasks['bt_tick'];before=(spec.budget_lo_s,spec.budget_hi_s)
        j=e.arrive(0,'bt_tick');e.stage(j,100000)
        self.assertEqual(e.jobs[j].budget_mode,'CALIBRATION')
        self.assertAlmostEqual(e.jobs[j].selected_budget_s,.0001)
        self.assertEqual((spec.budget_lo_s,spec.budget_hi_s),before)
        finish(e,j);self.assertIsNone(e.jobs[j].overrun_tick)

    def test_scalar_RC_matches_matrix_model_over_repeated_power_changes(self):
        e=engine();m=e.thermal[0];scalar=matrix=45.
        for n in range(3000):
            power=(.005,.05,1.5,3.)[(n//37)%4]
            scalar=m.evolve_single_core_c(scalar,power,.001)
            matrix=float(m.evolve_temperature_c([matrix],[power],.001)[0])
            self.assertAlmostEqual(scalar,matrix,places=11)

    def test_greedy_uses_temperature_only_with_inclusive_coverage(self):
        p=DevicePlacement('greedy');points={1:(5.,0.),2:(5.001,0.)}
        chosen,covered,candidates=p.choose(owner_device=0,position_xy=(0.,0.),endpoints=points,device_temperatures_c={0:46.,1:45.5,2:40.},queue_lengths={1:100})
        self.assertEqual(chosen,1);self.assertEqual(covered,[1])
        self.assertEqual(p.choose(owner_device=0,position_xy=(20.,0.),endpoints=points,device_temperatures_c={0:46.,1:45.,2:40.})[0],0)
        self.assertEqual(p.choose(owner_device=0,position_xy=(0.,0.),endpoints=points,device_temperatures_c={0:45.,1:45.,2:40.})[0],0)

    def test_greedy_records_actual_temperature_snapshot(self):
        e=engine('greedy');e.devices[0].cores[0].temperature_c=46.
        j=e.arrive(0,'bt_tick')
        self.assertEqual(e.jobs[j].device,1)
        event=next(v for v in e.events if v['kind']=='placement')
        self.assertEqual(event['device_temperatures_c'][0],46.)

    def test_edge_API_defaults_to_shared_shortest_finish_FIFO_policy(self):
        e=LiveEngine(edge_config={'placement':'offload'},scene=SCENE)
        self.assertEqual(e.scheduling_policy.name,FIFOShortestFinish.name)

    def test_upload_blocks_dispatch_at_exact_boundary(self):
        e=engine();j=e.arrive(0,'bt_tick');e.stage(j,0)
        self.assertEqual((e.jobs[j].device,e.jobs[j].owner_device),(1,0))
        self.assertEqual(e.jobs[j].upload_arrival_tick,4)
        for _ in range(3):e.advance();self.assertIsNone(e.jobs[j].start_tick)
        e.advance();self.assertEqual(e.jobs[j].start_tick,4)

    def test_cross_RSU_child_waits_for_parent_and_edge_arrival(self):
        e=engine();p=e.arrive(0,'planning_request');e.stage(p,0)
        e.update_position(0,(8.,0.),e.tick)
        c=e.arrive(0,'controller_path_install',[p]);e.stage(c,0)
        self.assertEqual((e.jobs[p].device,e.jobs[c].device),(1,2))
        finish(e,p);parent_end=e.tick;edge=e.jobs[c].edge_transfers[p]
        self.assertEqual((edge['delay_s'],edge['sent_tick']),(.010,parent_end))
        for _ in range(9):e.advance();self.assertIsNone(e.jobs[c].start_tick)
        e.advance();self.assertEqual(e.jobs[c].start_tick,parent_end+10)
        self.assertEqual(e.jobs[c].ready_tick,parent_end+10)

    def test_finished_parent_data_cannot_be_sent_before_child_is_known(self):
        e=engine();p=e.arrive(0,'planning_request');e.stage(p,0);finish(e,p)
        for _ in range(5):e.advance()
        e.update_position(0,(8,0),e.tick);c=e.arrive(0,'bt_tick',[p]);e.stage(c,0)
        self.assertEqual(e.jobs[c].edge_transfers[p]['sent_tick'],e.jobs[c].release_tick)
        self.assertIsNone(e.jobs[c].start_tick)

    def test_queue_uses_predicted_finish_and_FIFO_not_idle_age(self):
        from platform_fixtures import LiveEngine as MulticoreEngine
        e=MulticoreEngine(edge_config={'placement':'local'},scene=SCENE,scheduling_policy=FIFOShortestFinish());ids=[e.arrive(0,'planning_request') for _ in range(5)]
        for j in ids:e.stage(j,10**9,dispatch=False)
        e.dispatch();assign=[r for r in e.events if r['kind']=='core_queue_assignment']
        self.assertEqual([r['job_id'] for r in assign],ids)
        self.assertEqual(assign[0]['core_id'],1)
        for row in assign:
            expected=min(row['candidate_finish_ticks'],key=lambda c:(row['candidate_finish_ticks'][c],c))
            self.assertEqual(row['core_id'],expected)
        starts=[]
        for _ in range(10000):
            for j in ids:
                if e.deliverable(j):e.finish(j)
            if all(e.jobs[j].finish_tick is not None for j in ids):break
            e.advance()
        for cid in (0,1):
            queued=[r['job_id'] for r in assign if r['core_id']==cid]
            actual=[r['job_id'] for r in e.events if r['kind']=='start' and r['core_id']==cid]
            self.assertEqual(queued,actual)

    def test_deadlines_use_HI_fastest_A15_and_keep_period_seconds(self):
        e=engine();a=generate_deadlines(e.tasks)
        self.assertEqual(validate_deadlines(a,e.tasks,e.devices),[])
        for name,row in a['tasks'].items():
            spec=e.tasks[name]
            self.assertEqual(row['T_i_s'],spec.period_s)
            self.assertAlmostEqual(row['W_HI_mcycles'],spec.reference_work(spec.budget_hi_s))
            self.assertAlmostEqual(row['ideal_A15_max_HI_s'],row['W_HI_mcycles']/3600.)
            self.assertTrue(1.1<=row['uniform_factor']<=1.3)
            self.assertAlmostEqual(row['D_i_s'],row['ideal_A15_max_HI_s']*row['uniform_factor'])

    def test_deadline_audit_rejects_LO_based_or_thermally_adjusted_values(self):
        e=engine();table=generate_deadlines(e.tasks)
        table['tasks']['control_iteration']['D_i_s']*=2
        self.assertTrue(validate_deadlines(table,e.tasks,e.devices))

    def test_production_single_core_types_and_medium_HI_execution(self):
        from tools.live_bridge.dvfs import configured_selector,validate_job_execution
        from dataclasses import asdict
        cfg={'step_ns':1_000_000,'dvfs':{'policy':'fixed','level_id':2,'overrun_policy':'continue_selected_point'}}
        for policy,kind,freq in [('local','Cortex-A7',1200.),('offload','Cortex-A15',1500.)]:
            e=engine(policy,dvfs_selector=configured_selector(cfg),overrun_policy='continue_selected_point')
            self.assertEqual([c.core_type.name for c in e.devices[0].cores.values()],['Cortex-A7'])
            self.assertEqual([c.core_type.name for c in e.devices[1].cores.values()],['Cortex-A15'])
            j=e.arrive(0,'control_iteration');e.stage(j,10**9)
            while not e.deliverable(j):e.advance()
            job=e.jobs[j];core=e.devices[job.device].cores[0]
            self.assertEqual(core.core_type.name,kind)
            self.assertIsNotNone(job.overrun_tick)
            self.assertEqual({s['frequency_mhz'] for s in job.execution_segments},{freq})
            self.assertEqual(validate_job_execution(asdict(job),e.tasks[job.task],core,e.dt,[False]*e.tick,e.tick,overrun_policy='continue_selected_point'),[])

    def test_random_child_may_choose_other_covered_RSU(self):
        class Choices:
            def __init__(self):self.values=iter([1,2])
            def choice(self,candidates):
                selected=next(self.values);assert selected in candidates;return selected
        scene={'start':{'x':0.,'y':0.},'servers':[{'id':'a','x':2.,'y':0.},{'id':'b','x':-2.,'y':0.}]}
        e=LiveEngine(edge_config={'placement':'random'},scene=scene)
        e.placement.rng=Choices()
        p=e.arrive(0,'planning_request');e.stage(p,0)
        c=e.arrive(0,'controller_path_install',[p]);e.stage(c,0)
        self.assertEqual((e.jobs[p].device,e.jobs[c].device),(1,2))
        finish(e,p)
        self.assertEqual(e.jobs[c].edge_transfers[p]['delay_s'],.005)
        self.assertIsNone(e.jobs[c].start_tick)
        for _ in range(5):e.advance()
        self.assertEqual(e.jobs[c].start_tick,e.jobs[p].finish_tick+5)

    def test_cross_device_transfer_uses_send_time_vehicle_position(self):
        calls=[]
        def edge_cost(**kwargs):calls.append(kwargs);return .007
        e=engine('local',data_cost=edge_cost);parent=e.arrive(0,'planning_request');e.stage(parent,0)
        e.placement.name='offload';child=e.arrive(0,'bt_tick',[parent]);e.stage(child,0)
        e.update_position(0,(-3.,0.),e.tick);finish(e,parent)
        row=e.jobs[child].edge_transfers[parent]
        self.assertEqual(row['source_xy'],(-3.,0.));self.assertEqual(row['distance_m'],5.)
        self.assertEqual(row['arrival_tick'],e.jobs[parent].finish_tick+7)
        self.assertEqual(calls[0]['parent_id'],parent)

    def test_remote_device_cooling_blocks_output_and_child_dispatch(self):
        e=engine();parent=e.arrive(0,'bt_tick');e.stage(parent,0)
        for _ in range(100):
            if e.deliverable(parent):break
            e.advance()
        self.assertTrue(e.deliverable(parent));e.cooling[e.jobs[parent].device]=True
        self.assertFalse(e.deliverable(parent))
        with self.assertRaises(RuntimeError):e.finish(parent)
        child=e.arrive(0,'bt_tick',[parent]);e.stage(child,0)
        self.assertIsNone(e.jobs[child].start_tick)
        e.cooling[e.jobs[parent].device]=False;e.finish(parent)
        for _ in range(4):e.advance()
        self.assertIsNotNone(e.jobs[child].start_tick)

    def test_explicit_hardware_realization_roundtrip(self):
        import json,tempfile
        from pathlib import Path
        from tools.abstract_compute.hardware import load_platform,physical_realization
        source=Path(__file__).resolve().parents[1]/'config/abstract_compute.json'
        devices=load_platform();config=json.loads(source.read_text());config['realized_physical_parameters']=physical_realization(devices)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'hardware.json';path.write_text(json.dumps(config));other=load_platform(path)
            deadlines=generate_deadlines(engine().tasks,path)
            self.assertEqual(deadlines['deadline_reference']['frequency_mhz'],2000.)
        self.assertEqual(physical_realization(devices),physical_realization(other))

    def test_remote_publications_keep_vehicle_DDS_owner(self):
        from tests.test_live_bridge import LiveProtocolTests
        from tools.live_bridge.broker import PUBLISH_BEGIN,PUBLISH,LOOKUP,PACKET
        b,c=LiveProtocolTests().fixture();b.engine=engine();j=b.engine.arrive(0,'bt_tick');b.engine.stage(j,0);finish(b.engine,j)
        b.packet(c,(PUBLISH_BEGIN,j,123,100,0,0,0));b.packet(c,(PUBLISH,j,123,100,200,1,0))
        b.packet(c,(LOOKUP,123,0,150,0,0,0));self.assertEqual(PACKET.unpack(c.replies[-1])[1],j)
        self.assertEqual(b.expected_commands,[123])

if __name__=='__main__':unittest.main()
