#!/usr/bin/env python3
"""Calibrate once, then run balanced full-course policy blocks before a hard cutoff."""
import argparse,hashlib,json,os,shutil,signal,subprocess,sys,time
from datetime import datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.abstract_compute.hardware import load_platform,physical_realization
from scripts.extract_full_course_parameters import extract
STOP=False

def save(path,obj):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(obj,indent=2)+'\n');temp.replace(path)
def stopped(*_):
    global STOP;STOP=True

def active_identity(pid,start_ticks,command):
    if not pid or start_ticks is None:return False
    folder=Path('/proc')/str(pid)
    try:
        fields=(folder/'stat').read_text().rsplit(') ',1)[1].split()
        if fields[0]=='Z' or int(fields[19])!=start_ticks:return False
        actual=(folder/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    except FileNotFoundError:return False
    if command not in actual:raise ValueError('Process identity matched but its command did not')
    return True

def assert_no_active_campaign(control):
    old=control.get('runner',{})
    if active_identity(old.get('pid'),old.get('start_ticks'),'scripts/run_full_course_comparison.py'):
        raise ValueError('The previous campaign runner is still active')
    if active_identity(control.get('active_trial_pid'),control.get('active_trial_start_ticks'),'scripts/run_live_bridge.py'):
        raise ValueError('The previous trial is still active; let it finish before resuming')

def has_valid_completion(campaign,row):
    if row.get('status')!='complete':return False
    trial=campaign/row['trial']
    try:
        run=json.loads((trial/'run.json').read_text())
        audit=json.loads((trial/'validation.json').read_text())
    except (FileNotFoundError,json.JSONDecodeError):return False
    native=run.get('navigation_outcomes',{}).get('red',{})
    if not audit.get('passed') or not native.get('passed') or native.get('ordered_targets_passed')!=20 or not run.get('simulation_completed'):return False
    if row.get('exit_code')==0 and run.get('completed'):return True
    for key,kind in (('postprocessing_recovery','recording_postprocessing'),('manager_recovery','runner_interruption')):
        recovery=row.get(key)
        if not recovery:continue
        try:record=json.loads((trial/recovery).read_text())
        except (FileNotFoundError,json.JSONDecodeError):return False
        if record.get('kind')!=kind or not record.get('validated_full_mission'):return False
        if kind=='runner_interruption' and not run.get('completed'):return False
        for name,digest in record.get('original_evidence_sha256',{}).items():
            if hashlib.sha256((trial/name).read_bytes()).hexdigest()!=digest:return False
        return bool(record.get('original_evidence_sha256'))
    return False

def main():
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);p.add_argument('--resume',action='store_true');p.add_argument('--until',default='2026-10-10T08:00:00+03:30');a=p.parse_args();out=a.campaign.resolve();out.mkdir(parents=True,exist_ok=True)
    if any(shutil.which(name) is None for name in ('gz','ros2')):raise SystemExit('Source scripts/environment.sh before running the campaign')
    cutoff=datetime.fromisoformat(a.until).timestamp();signal.signal(signal.SIGTERM,stopped);signal.signal(signal.SIGINT,stopped)
    control_path=out/'control.json';control=json.loads(control_path.read_text())
    manifest_path=out/'manifest.json'
    if manifest_path.exists() and not a.resume:raise SystemExit('Existing campaign evidence must be resumed explicitly, not overwritten')
    if a.resume:
        if not manifest_path.exists():raise SystemExit('No existing campaign to resume')
        assert_no_active_campaign(control)
        manifest=json.loads(manifest_path.read_text())
        if manifest['cutoff_iso']!=a.until:raise SystemExit('Resume must preserve the original cutoff')
        hashes=json.loads((out/'frozen-input-hashes.json').read_text())
        if not all(hashlib.sha256((out/name).read_bytes()).hexdigest()==digest for name,digest in hashes.items()):raise SystemExit('Frozen inputs changed')
        manifest.setdefault('resume_history',[]).append(dict(unix_s=time.time(),previous_status=manifest['status'],reason='Continue retained trials and frozen inputs'))
        manifest['status']='running'
    else:
        manifest={'status':'running','cutoff_iso':a.until,'policy_order':['local','random','offload','greedy'],'calibration_laps':3,'replicates_per_policy':3,'trials':[],'started_unix_s':time.time(),'randomness':'system entropy; realized inputs retained'}
        scene=json.loads((ROOT/'scenarios/monaco/scenario.json').read_text());hardware=json.loads((ROOT/'config/abstract_compute.json').read_text())
        hardware['realized_physical_parameters']=physical_realization(load_platform(device_classes=('vehicle',)+('server',)*len(scene['servers'])))
        save(out/'hardware.json',hardware)
        (out/'calibration-input-parameters.json').write_text((ROOT/'docs/evidence/dual-budget-parameters.json').read_text());(out/'calibration-deadlines.json').write_text((ROOT/'config/task_deadlines.json').read_text())
    control['runner']={'pid':os.getpid(),'start_ticks':int(Path('/proc/self/stat').read_text().split()[21]),'command':'scripts/run_full_course_comparison.py'}
    save(manifest_path,manifest);save(control_path,control)
    schedule=[('calibration',i,'local') for i in range(1,4)]+[('comparison',i,p) for i in range(1,4) for p in manifest['policy_order']]
    try:
        for phase,replicate,policy in schedule:
            previous=[r for r in manifest['trials'] if (r['phase'],r['replicate'],r['policy'])==(phase,replicate,policy)]
            if any(has_valid_completion(out,r) for r in previous):continue
            if any(r['status']=='complete' for r in previous):raise ValueError('Previously completed evidence failed validation')
            if STOP or time.time()>=cutoff-40:break
            if phase=='comparison' and not (out/'frozen-task-parameters.json').exists():
                extract(out);control['phase']='parameters_frozen';save(control_path,control)
            if phase=='comparison':
                while not (out/'comparisons-ready').exists() and not STOP and time.time()<cutoff-40:time.sleep(1)
            if STOP or time.time()>=cutoff-40:break
            success=False
            first_attempt=1+max([r['attempt'] for r in previous] or [0])
            for attempt in range(first_attempt,first_attempt+3):
                if STOP or time.time()>=cutoff-40:break
                name=f'{phase}-{replicate:02d}-{policy}-attempt-{attempt:02d}';trial=out/name
                row=dict(phase=phase,replicate=replicate,policy=policy,attempt=attempt,trial=name,status='running',started_unix_s=time.time());manifest['trials'].append(row);save(manifest_path,manifest)
                params=out/('calibration-input-parameters.json' if phase=='calibration' else 'frozen-task-parameters.json');deadlines=out/('calibration-deadlines.json' if phase=='calibration' else 'deadlines.json')
                cmd=[sys.executable,str(ROOT/'scripts/run_live_bridge.py'),'--full-course','--placement',policy,'--seconds','1200','--task-parameters',str(params),'--deadline-file',str(deadlines),'--hardware-config',str(out/'hardware.json'),'--until',str(cutoff-25),'--output',str(trial)]
                if phase=='calibration':cmd+=['--calibration']
                if phase=='comparison' and replicate==1:cmd+=['--record']
                with (out/(name+'-console.log')).open('w') as log:
                    process=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    control.update(phase=phase,current_trial=name,active_trial_pid=process.pid,active_trial_start_ticks=int(Path(f'/proc/{process.pid}/stat').read_text().split()[21]));save(control_path,control)
                    while process.poll() is None:
                        if STOP or time.time()>=cutoff-25:
                            process.send_signal(signal.SIGTERM)
                            try:process.wait(timeout=18)
                            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL)
                            break
                        time.sleep(1)
                    row.update(exit_code=process.wait(),ended_unix_s=time.time())
                if (trial/'trace.json').exists():
                    with (out/(name+'-analysis.log')).open('w') as log:validation=subprocess.run([sys.executable,str(ROOT/'scripts/analyze_placement_trial.py'),str(trial)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
                    row['validation_exit_code']=validation.returncode
                    if (trial/'metrics.json').exists():row['metrics']=json.loads((trial/'metrics.json').read_text())
                run=json.loads((trial/'run.json').read_text()) if (trial/'run.json').exists() else {}
                success=bool(row['exit_code']==0 and row.get('validation_exit_code')==0 and run.get('navigation_outcomes',{}).get('red',{}).get('passed'))
                row['status']='complete' if success else 'failed_or_partial';save(manifest_path,manifest);print(json.dumps({k:row[k] for k in ('phase','replicate','policy','attempt','status')}),flush=True)
                if success:break
            if not success:
                manifest['status']='cutoff' if STOP or time.time()>=cutoff-40 else 'attention_required';break
        else:manifest['status']='complete'
        if manifest['status']=='running':manifest['status']='cutoff'
    except BaseException as ex:manifest.update(status='attention_required',error=repr(ex));raise
    finally:
        manifest['finished_unix_s']=time.time();save(manifest_path,manifest);control.update(phase=manifest['status'],active_trial_pid=None);save(control_path,control)
if __name__=='__main__':main()
