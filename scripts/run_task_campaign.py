#!/usr/bin/env python3
"""Repeated unchanged missions, immutable per-run evidence, strict wall-clock cutoff."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from zoneinfo import ZoneInfo

ROOT=Path(__file__).resolve().parents[1]
KEEP_RUNNING=True
def stop(*_):
    global KEEP_RUNNING
    KEEP_RUNNING=False
def write_json(path,obj):
    temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(obj,indent=2)+'\n');temp.replace(path)
def shutdown(proc,seconds=12):
    if proc is None:return {'forced':False,'returncode':None}
    forced=False
    try:
        os.killpg(proc.pid,signal.SIGINT)
    except ProcessLookupError:pass
    try:proc.wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        forced=True
        try:os.killpg(proc.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            proc.wait(timeout=3)
    # Some launch children can outlive their parent. Signal only our owned group.
    try:os.killpg(proc.pid,signal.SIGTERM)
    except ProcessLookupError:pass
    return {'forced':forced,'returncode':proc.returncode}
def spawn(cmd,log,env):
    return subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
def proc_snapshot():
    text=subprocess.run(['ps','-eo','pid,ppid,comm,args'],capture_output=True,text=True).stdout
    return text
def main():
    p=argparse.ArgumentParser();p.add_argument('--until',default='2026-10-09T09:00:00+03:30');p.add_argument('--max-runs',type=int,default=0);p.add_argument('--pilot',action='store_true');p.add_argument('--gui',action='store_true');p.add_argument('--campaign-id',required=True);p.add_argument('--probe',type=Path,default=ROOT/'build/task-probe/libnav2_task_probe.so');args=p.parse_args()
    deadline=datetime.fromisoformat(args.until).timestamp()
    if deadline<=time.time():raise SystemExit('Cutoff already passed; no simulation started.')
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    campaign=ROOT/'artifacts/task-profiling'/args.campaign_id;campaign.mkdir(parents=True,exist_ok=False)
    probe=args.probe.resolve()
    evidence={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in ['scenarios/monaco/nav2_params.yaml','scenarios/monaco/world.sdf','scenarios/monaco/racecar.sdf','scenarios/monaco/navigate_through_poses.xml','tools/profiling/nav2_task_probe.cpp']}
    manifest={'campaign_id':args.campaign_id,'started_iso':datetime.now(ZoneInfo('Asia/Tehran')).isoformat(),'cutoff_iso':args.until,'cutoff_epoch_s':deadline,'gui':args.gui,'profile_binary_sha256':hashlib.sha256(probe.read_bytes()).hexdigest(),'configuration_sha256':evidence,'runs':[],'status':'running','scope':'same unchanged full mission; host CPU-time profiling; no new scheduling or deadline assignment'}
    write_json(campaign/'manifest.json',manifest)
    write_json(campaign/'build-metadata.json',json.loads((probe.parent/'build-metadata.json').read_text()))
    import shutil
    shutil.copy2(probe,campaign/'libnav2_task_probe.so')
    build_meta=json.loads((probe.parent/'build-metadata.json').read_text())
    source_arg=build_meta['command'][build_meta['command'].index('-ldl')-1]
    shutil.copy2(source_arg,campaign/'probe-source.cpp')
    probe=campaign/'libnav2_task_probe.so'
    (campaign/'host.txt').write_text(subprocess.run(['bash','-c','uname -a\nlscpu\nfree -h'],capture_output=True,text=True).stdout)
    print(str(campaign),flush=True)
    while KEEP_RUNNING and time.time()<deadline and (not args.max_runs or len(manifest['runs'])<args.max_runs):
        # Reserve at least a small startup window; do not launch a fresh lap at cutoff.
        if deadline-time.time()<45:break
        k=len(manifest['runs'])+1;run=campaign/f'run-{k:03d}';run.mkdir()
        raw=run/'raw';raw.mkdir()
        start=time.time();meta={'run':k,'started_epoch_s':start,'started_iso':datetime.now(ZoneInfo('Asia/Tehran')).isoformat(),'gui':args.gui,'full_course':True,'status':'starting'}
        env=dict(os.environ);env['NAV2_PROFILE_DIR']=str(raw);env['LD_PRELOAD']=str(probe)
        launcher=observer=mission=None
        logs=[]
        try:
            launchlog=(run/'launch.log').open('w');logs.append(launchlog)
            launcher=spawn(['bash','scripts/launch_monaco.sh',f'gui:={str(args.gui).lower()}'],launchlog,env)
            meta['launch_pid']=launcher.pid
            # Probe is applied to Nav2 and bridge; independent observer and mission are uninstrumented.
            plain=dict(os.environ);plain.pop('LD_PRELOAD',None);plain.pop('NAV2_PROFILE_DIR',None)
            oblog=(run/'observer.log').open('w');logs.append(oblog)
            observer=spawn(['bash','-c','source scripts/environment.sh\nexec python3 scripts/observe_task_run.py --output "$1"','observer',str(run/'observer.jsonl')],oblog,plain)
            mlog=(run/'mission.log').open('w');logs.append(mlog)
            mission=spawn(['bash','-c','source scripts/environment.sh\nexec python3 scripts/drive_monaco.py --timeout "$1" --output "$2"','mission',str(min(1500,max(1,deadline-time.time()-25))),str(run/'mission.json')],mlog,plain)
            meta.update(observer_pid=observer.pid,mission_pid=mission.pid,status='running')
            write_json(run/'metadata.json',meta)
            (run/'processes-start.txt').write_text(proc_snapshot())
            last_sample=0
            runtime_snapshot_done=False
            with (run/'host-samples.jsonl').open('w') as samples:
                while KEEP_RUNNING and time.time()<deadline-20 and mission.poll() is None and launcher.poll() is None:
                    if not runtime_snapshot_done and time.time()-start>35:
                        (run/'processes-active.txt').write_text(proc_snapshot())
                        runtime_snapshot_done=True
                    if time.time()-last_sample>5:
                        samples.write(json.dumps({'epoch_s':time.time(),'mono_ns':time.monotonic_ns(),'loadavg':os.getloadavg(),'meminfo':Path('/proc/meminfo').read_text()})+'\n');samples.flush();last_sample=time.time()
                    time.sleep(.5)
            if time.time()>=deadline-20:meta['stop_reason']='cutoff safety margin (stop before 09:00)' 
            elif not KEEP_RUNNING:meta['stop_reason']='external stop'
            elif launcher.poll() is not None:meta['stop_reason']='launch exited'
            else:meta['stop_reason']='mission finished'
        except Exception as e:meta['error']=repr(e)
        finally:
            # Stop simulation FIRST at cutoff; allow evidence flushing after it stops.
            meta['simulation_shutdown']=shutdown(launcher,seconds=0 if time.time()>=deadline else 12)
            meta['mission_shutdown']=shutdown(mission,seconds=6)
            meta['observer_shutdown']=shutdown(observer,seconds=5)
            for f in logs:f.close()
            meta['ended_epoch_s']=time.time();meta['elapsed_wall_s']=meta['ended_epoch_s']-start
            meta['mission_result']=json.loads((run/'mission.json').read_text()) if (run/'mission.json').exists() else None
            # Avoid duplicating trajectory samples in metadata.
            if meta['mission_result']:
                meta['mission_result']={a:b for a,b in meta['mission_result'].items() if a not in ['velocity_samples','pose_samples','localized_trace','target_events']}
            meta['passed']=bool(meta['mission_result'] and meta['mission_result'].get('passed'))
            meta['status']='passed' if meta['passed'] else 'failed_or_truncated'
            meta['raw_files']=[{'path':str(f.relative_to(run)),'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in raw.glob('*.csv')]
            write_json(run/'metadata.json',meta)
            manifest['runs'].append({'run':k,'path':run.name,'passed':meta['passed'],'stop_reason':meta.get('stop_reason'),'elapsed_wall_s':meta['elapsed_wall_s'],'ended_epoch_s':meta['ended_epoch_s']})
            write_json(campaign/'manifest.json',manifest)
            print(json.dumps(manifest['runs'][-1]),flush=True)
        if args.pilot:break
        # A crash is evidence, not permission to run hundreds of empty restarts.
        if not meta['passed'] and meta['elapsed_wall_s']<60:
            manifest['status']='needs_attention';break
        time.sleep(2)
    if manifest['status']=='running':manifest['status']='finished'
    manifest['ended_iso']=datetime.now(ZoneInfo('Asia/Tehran')).isoformat()
    manifest['attempts']=len(manifest['runs']);manifest['successful_runs']=sum(x['passed'] for x in manifest['runs'])
    write_json(campaign/'manifest.json',manifest)
    print('Campaign stopped; final manifest saved.',flush=True)
if __name__=='__main__':main()
