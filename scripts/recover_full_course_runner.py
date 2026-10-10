#!/usr/bin/env python3
"""Seal an independently completed trial after its campaign manager exits."""
import argparse,fcntl,hashlib,json,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.run_full_course_comparison import assert_no_active_campaign,has_valid_completion,save

def recover(campaign):
    campaign=Path(campaign).resolve()
    with (campaign/'runner-recovery.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        control=json.loads((campaign/'control.json').read_text());assert_no_active_campaign(control)
        manifest_path=campaign/'manifest.json';original=manifest_path.read_bytes();manifest=json.loads(original)
        rows=[r for r in manifest['trials'] if r['trial']==control['current_trial'] and r['status']=='running']
        if len(rows)!=1:raise ValueError('Exactly one retained running trial is required')
        row=rows[0];trial=campaign/row['trial'];run=json.loads((trial/'run.json').read_text())
        if not (trial/'trace.json').exists():raise ValueError('The trial has not sealed its trace')
        # Refuse a concurrent analyzer for this trial before taking ownership of analysis.
        for folder in Path('/proc').iterdir():
            if not folder.name.isdigit():continue
            try:args=(folder/'cmdline').read_bytes().split(b'\0')
            except (FileNotFoundError,PermissionError):continue
            if len(args)>2 and args[1].endswith(b'analyze_placement_trial.py') and str(trial).encode() in args:
                raise ValueError('A trial analyzer is already active')
        with (campaign/(row['trial']+'-runner-recovery-analysis.log')).open('w') as log:
            analysis=subprocess.run(['nice','-n','15',sys.executable,str(ROOT/'scripts/analyze_placement_trial.py'),str(trial)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        audit=json.loads((trial/'validation.json').read_text());metrics=json.loads((trial/'metrics.json').read_text())
        native=run.get('navigation_outcomes',{}).get('red',{})
        valid=bool(analysis.returncode==0 and audit.get('passed') and run.get('completed') and run.get('simulation_completed') and native.get('passed') and native.get('ordered_targets_passed')==20)
        evidence=['run.json','trace.json','red-mission.json','validation.json']
        evidence.extend(n for n in ['presentation.json'] if (trial/n).exists())
        record=dict(kind='runner_interruption',validated_full_mission=valid,recorded_unix_s=time.time(),
                    previous_runner=control.get('runner'),previous_trial_pid=control.get('active_trial_pid'),previous_trial_start_ticks=control.get('active_trial_start_ticks'),
                    process_exit_code=None,process_exit_status='Unavailable: trial continued independently after manager exit',
                    original_evidence_sha256={n:hashlib.sha256((trial/n).read_bytes()).hexdigest() for n in evidence})
        recovery=trial/'runner-recovery.json'
        if recovery.exists():raise ValueError('A recovery record already exists')
        backup=campaign/('manifest-before-runner-recovery-'+str(time.time_ns())+'.json');backup.write_bytes(original)
        save(recovery,record)
        row.update(original_status=row['status'],exit_code=None,ended_unix_s=time.time(),validation_exit_code=analysis.returncode,metrics=metrics,manager_recovery=recovery.name,status='complete' if valid else 'failed_or_partial')
        if valid and not has_valid_completion(campaign,row):raise ValueError('Recovered completion failed its independent evidence gate')
        manifest.update(status='attention_required',interruption_reason='Campaign manager exited while its retained trial continued; original trial evidence sealed',finished_unix_s=time.time())
        save(manifest_path,manifest);control.update(phase='attention_required',active_trial_pid=None);save(campaign/'control.json',control)
        print(json.dumps(dict(trial=row['trial'],status=row['status'],exit_code=None,metrics=metrics),indent=2))
        return valid
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args();sys.exit(0 if recover(a.campaign) else 1)
