#!/usr/bin/env python3
"""Run an isolated, audited live Nav2 placement/FIFO lockstep trial."""
import argparse,json,os,signal,subprocess,sys,time,re,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tools.live_bridge.broker import runtime_paths
def main():
    p=argparse.ArgumentParser();p.add_argument('--placement',choices=('local','offload','random','greedy'));p.add_argument('--deadline-file',type=Path);p.add_argument('--task-parameters',type=Path);p.add_argument('--calibration',action='store_true');p.add_argument('--full-course',action='store_true');p.add_argument('--until',type=float,help='Absolute UTC Unix cutoff, including startup.');p.add_argument('--hardware-config',type=Path);p.add_argument('--random-scope',choices=('covered','all'),default='covered');p.add_argument('--dvfs-level',type=int,choices=range(5),help='Use a fixed paired operating point with the configured overrun rule; omitted uses configured DVFS.');p.add_argument('--seconds',type=float,default=30);p.add_argument('--distance',type=float,help='Stop after this actual active-interval odometry distance (m), with --seconds as the maximum.');p.add_argument('--output',type=Path);p.add_argument('--gui',action='store_true');p.add_argument('--views',action='store_true');p.add_argument('--record',action='store_true');a=p.parse_args()
    if a.distance is not None and a.distance<=0:p.error('--distance must be positive')
    out=(a.output or ROOT/'artifacts/live-bridge'/time.strftime('%Y%m%d-%H%M%S')).resolve()
    if out.exists() and any(out.iterdir()):raise SystemExit('Choose an empty output directory to preserve earlier evidence')
    out.mkdir(parents=True,exist_ok=True)
    ps=subprocess.check_output(['ps','-eo','args'],text=True)
    if any(('gz sim -s' in line or 'gz sim -r -s' in line or 'ros2 launch' in line) and 'ps -eo' not in line for line in ps.splitlines()):raise SystemExit('A simulation is already active')
    config=json.loads((ROOT/'config/live_bridge.json').read_text())
    config['calibration_actual_work']=a.calibration
    if a.placement is not None:
        config.update(edge_enabled=True,placement=a.placement,vehicle_color={'local':'red','offload':'green','random':'blue','greedy':'amber'}[a.placement],policy='ready_FIFO_shortest_finish_queue_nonpreemptive',coverage_radius_m=5.,random_scope=a.random_scope,deadlines=json.loads((a.deadline_file or ROOT/config['deadline_file']).read_text()))
    if config.get('edge_enabled') and 'deadlines' not in config:config['deadlines']=json.loads((a.deadline_file or ROOT/config['deadline_file']).read_text())
    if a.dvfs_level is not None:config['dvfs']={**config.get('dvfs',{}),'policy':'fixed','level_id':a.dvfs_level}
    (out/'trial-config.json').write_text(json.dumps(config,indent=2)+'\n');count=config['vehicle_count']
    (out/'hardware-config.json').write_text((a.hardware_config or ROOT/'config/abstract_compute.json').read_text())
    (out/'task-parameters.json').write_text((a.task_parameters or ROOT/'docs/evidence/dual-budget-parameters.json').read_text())
    if (ROOT/'build/live-bridge/build.json').exists():
        (out/'adapter-build.json').write_text((ROOT/'build/live-bridge/build.json').read_text())
    sources=['tools/live_bridge/nav2_adapter.cpp','tools/live_bridge/algorithm_clock.cpp','tools/live_bridge/tf_audit.cpp','tools/live_bridge/dvfs.py','scripts/observe_live_bridge.py','scenarios/monaco/scenario.json','scenarios/monaco/world.sdf','scenarios/monaco/nav2_params.yaml','scenarios/monaco/navigate_through_poses.xml','tools/live_bridge/deadlines.py','tools/abstract_compute/task_model.py','scripts/run_live_bridge.py','scripts/drive_monaco.py','tools/live_bridge/engine.py','tools/live_bridge/broker.py','tools/live_bridge/policies.py','tools/live_bridge/placement.py','tools/live_bridge/stepper.cpp','tools/abstract_compute/communication.py','tools/abstract_compute/hardware.py','tools/abstract_compute/thermal.py','tools/abstract_compute/power.py','launch/monaco_bridge.launch.py']
    (out/'source-hashes.json').write_text(json.dumps({name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources},indent=2)+'\n')
    owned=[];handles=[];result={'output':str(out),'requested_sim_s':a.seconds,'requested_distance_m':a.distance,'full_course':a.full_course,'wall_cutoff_unix_s':a.until,'scheduler_coupled_to_robot':True,'completed':False}
    def start(cmd,name):
        log=(out/(name+'.log')).open('w');handles.append(log);proc=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);owned.append(proc);return proc
    def terminate_trial(signum,frame):
        raise KeyboardInterrupt(f'Trial interrupted by signal {signum}')
    signal.signal(signal.SIGTERM,terminate_trial)
    try:
        # The broker creates the clock/socket before launch, but Gazebo must exist before its first step.
        sockpath,clock=runtime_paths(out);clock.write_bytes(bytes(32))
        sim=start(['env','NAV2_LIVE_CONFIG_PATH='+str(out/'trial-config.json'),'ros2','launch',str(ROOT/'launch/monaco_bridge.launch.py'),'gui:='+str(a.gui and not(a.views or a.record)).lower(),'socket:='+sockpath,'clock:='+str(clock)],'launch')
        ready=False
        for _ in range(120):
            if a.until and time.time()>=a.until:raise RuntimeError('Authorized wall-clock cutoff reached')
            if sim.poll() is not None:raise RuntimeError('Launch exited before world control was available')
            services=subprocess.run(['gz','service','-l'],capture_output=True,text=True,timeout=10).stdout
            if '/world/monaco/control' in services:ready=True;break
            time.sleep(.5)
        if not ready:raise RuntimeError('Gazebo world-control service did not appear')
        start([sys.executable,str(ROOT/'scripts/observe_live_bridge.py'),'--output',str(out/'ros-observations.csv'),'--ready-file',str(out/'nav2-ready'),'--vehicles',str(count)],'observer')
        if a.views or a.record:(out/'views-required').touch()
        broker=start([sys.executable,'-m','tools.live_bridge.broker','--output',str(out),'--seconds',str(a.seconds)],'broker')
        presentation=start([sys.executable,str(ROOT/'scripts/show_live_bridge.py'),'--trial',str(out)]+(['--record'] if a.record else [])+(['--defer-encoding'] if a.record and a.full_course else []),'presentation') if a.views or a.record else None
        missions=[];sent=False;deadline=time.monotonic()+max(180,a.seconds*30)
        while broker.poll() is None:
            if sim.poll() is not None:raise RuntimeError('Simulation launch exited')
            if presentation and presentation.poll() is not None:raise RuntimeError('Three-view presentation exited before simulation')
            if re.search(r'\[ERROR\].*component_container_isolated.*process has died',(out/'launch.log').read_text()):raise RuntimeError('A Nav2 container exited before the trial limit')
            progress=out/'progress.json'
            try:sim_s=json.loads(progress.read_text())['sim_s']
            except (FileNotFoundError,json.JSONDecodeError):sim_s=0
            if sim_s>=1 and not sent:
                for color in ('red','blue','white','green')[:count]:
                    cmd=[sys.executable,str(ROOT/'scripts/drive_monaco.py'),'--timeout','7200','--robot-name','racecar' if color=='red' else 'racecar_'+color,'--output',str(out/(color+'-mission.json')),'--bridge-ready-file',str(out/'armed')]+(['--stop-bridge-at-finish'] if a.full_course else [])
                    if count>1:cmd+=['--fleet-color',color]
                    if color!='red':cmd+=['--namespace',color]
                    missions.append(start(cmd,color+'-mission'))
                sent=True
            if not (out/'armed').exists() and any(m.poll() is not None for m in missions):raise RuntimeError('A mission failed during initialization; inspect its report')
            if a.distance is not None and (out/'armed').exists() and not (out/'stop-requested.json').exists():
                try:distance=json.loads((out/'distance-progress.json').read_text())
                except (FileNotFoundError,json.JSONDecodeError):distance={}
                if distance.get('distance_m',{}).get('0',0)>=a.distance:
                    request={'reason':'requested odometry distance reached','target_m':a.distance,'observation':distance}
                    tmp=out/'stop-requested.tmp';tmp.write_text(json.dumps(request)+'\n');tmp.replace(out/'stop-requested.json')
            if a.until and time.time()>=a.until:raise RuntimeError('Authorized wall-clock cutoff reached')
            if a.full_course and sent and any(m.poll() is not None and m.returncode for m in missions):raise RuntimeError('Full-course navigation failed; original mission report retained')
            if time.monotonic()>deadline:raise RuntimeError('Trial exceeded bounded host timeout')
            time.sleep(.5)
        if broker.returncode:raise RuntimeError('Bridge rejected the trial; inspect broker.log')
        if not (out/'armed').exists():raise RuntimeError('Nav2 did not fully activate; inspect nav2-ready.json')
        result['simulation_completed']=True
        if presentation and presentation.wait(timeout=180):raise RuntimeError('Three-view presentation failed')
        result['completed']=True
        result['stop_reason']=json.loads((out/'trace.json').read_text()).get('stop_reason','simulation-time limit')
        result['end_of_trial_monotonic_ns']=time.monotonic_ns()
    except BaseException as ex:result['error']=str(ex);raise
    finally:
        for proc in reversed(owned):
            try:os.killpg(proc.pid,signal.SIGINT)
            except ProcessLookupError:pass
        time.sleep(2)
        for sig in (signal.SIGTERM,signal.SIGKILL):
            for proc in reversed(owned):
                try:os.killpg(proc.pid,sig)
                except ProcessLookupError:pass
            time.sleep(1)
        for proc in owned:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:pass
        for log in handles:log.close()
        outcomes={}
        for color in ('red','blue','white','green')[:count]:
            report=out/(color+'-mission.json')
            if report.exists():
                r=json.loads(report.read_text());outcomes[color]={k:r.get(k) for k in ('passed','action_status','nav2_error_code','error','ordered_targets_passed','odometry_distance_m','completion_sim_s','full_course')}
        result['navigation_outcomes']=outcomes
        if a.full_course and not all(r.get('passed') for r in outcomes.values()):result['completed']=False
        result['navigation_failed']=any(r.get('error') not in (None,'Interrupted.') for r in outcomes.values())
        (out/'run.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
