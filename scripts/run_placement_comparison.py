#!/usr/bin/env python3
"""Run three serial 50 m live policies against one saved hardware/deadline draw."""
import argparse,json,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

def main():
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args();out=a.campaign.resolve()
    out.mkdir(parents=True,exist_ok=True)
    if (out/'manifest.json').exists():raise SystemExit('Use a new campaign directory; existing evidence is retained')
    from tools.abstract_compute.hardware import load_platform,physical_realization
    if not (out/'deadlines.json').exists():(out/'deadlines.json').write_text((ROOT/'config/task_deadlines.json').read_text())
    if not (out/'hardware.json').exists():
        scene=json.loads((ROOT/'scenarios/monaco/scenario.json').read_text());hardware=json.loads((ROOT/'config/abstract_compute.json').read_text())
        hardware['realized_physical_parameters']=physical_realization(load_platform(device_classes=('vehicle',)+('server',)*len(scene['servers'])))
        (out/'hardware.json').write_text(json.dumps(hardware,indent=2)+'\n')
    manifest={'status':'running','target_distance_m':50.,'replicates_per_policy':1,'randomness':'system entropy; realized inputs retained','started_unix_s':time.time(),'trials':[]}
    target=out/'manifest.json'
    for policy in ('local','offload','random'):
        trial=out/(policy+'-01')
        if trial.exists():raise SystemExit('Do not overwrite a previously attempted trial')
        row={'policy':policy,'trial':trial.name,'status':'running'};manifest['trials'].append(row);target.write_text(json.dumps(manifest,indent=2)+'\n')
        cmd=[sys.executable,str(ROOT/'scripts/run_live_bridge.py'),'--placement',policy,'--seconds','600','--distance','50','--deadline-file',str(out/'deadlines.json'),'--hardware-config',str(out/'hardware.json'),'--output',str(trial)]
        with (out/(policy+'-runner.log')).open('w') as log:result=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        row['exit_code']=result.returncode
        if (trial/'trace.json').exists():
            with (out/(policy+'-analysis.log')).open('w') as log:check=subprocess.run([sys.executable,str(ROOT/'scripts/analyze_placement_trial.py'),str(trial)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            row['validation_exit_code']=check.returncode
            if (trial/'metrics.json').exists():row['metrics']=json.loads((trial/'metrics.json').read_text())
        row['status']='complete' if result.returncode==0 and row.get('validation_exit_code')==0 and row.get('metrics',{}).get('target_reached') else 'failed'
        target.write_text(json.dumps(manifest,indent=2)+'\n')
        if row['status']=='failed':manifest['status']='attention_required';target.write_text(json.dumps(manifest,indent=2)+'\n');raise SystemExit(1)
    manifest['status']='complete';manifest['finished_unix_s']=time.time();target.write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))
if __name__=='__main__':main()
