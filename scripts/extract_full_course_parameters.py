#!/usr/bin/env python3
"""Freeze pooled empirical CPU budgets from three audited full-course calibration laps."""
import argparse,csv,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.abstract_compute.task_model import load_tasks
from tools.live_bridge.deadlines import generate_deadlines

def write_csv(path,rows):
    if not rows:return
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def extract(campaign):
    campaign=Path(campaign);manifest=json.loads((campaign/'manifest.json').read_text())
    runs=[r for r in manifest['trials'] if r['phase']=='calibration' and r['status']=='complete']
    if len(runs)!=3:raise ValueError('Three validated full-course calibration laps are required')
    original=json.loads((campaign/'calibration-input-parameters.json').read_text());table={**original,'source':'three full-course calibration laps','successful_laps':3,'cpu_clock':'HOST_THREAD_CPU','calibration_runs':[r['trial'] for r in runs],'tasks':{}}
    samples=[];summaries=[];inputs={};by_task={name:[] for name in original['tasks']};intervals={name:[] for name in by_task}
    for row in runs:
        trial=campaign/row['trial'];path=trial/'trace.json';t=json.loads(path.read_text());inputs[row['trial']]=hashlib.sha256(path.read_bytes()).hexdigest()
        for name in by_task:
            jobs=sorted((j for j in t['jobs'] if j['task']==name),key=lambda j:j['release_tick'])
            values=[j['measured_cpu_ns']/1e9 for j in jobs if j['measured_cpu_ns'] is not None]
            by_task[name].extend(values);iat=np.diff([j['release_tick']*t['step_s'] for j in jobs]);intervals[name].extend(iat.tolist())
            summaries.append(dict(run=row['trial'],task=name,jobs=len(jobs),CPU_samples=len(values),C_mean_s=float(np.mean(values)) if values else None,C_max_s=max(values) if values else None,C_p95_s=float(np.quantile(values,.95)) if values else None,observed_rate_per_s=len(jobs)/t['race_seconds'],observed_mean_inter_arrival_s=float(np.mean(iat)) if len(iat) else None))
            for j in jobs:samples.append(dict(run=row['trial'],task=name,job_id=j['job_id'],release_sim_s=j['release_tick']*t['step_s'],actual_CPU_s=None if j['measured_cpu_ns'] is None else j['measured_cpu_ns']/1e9,finished=j['finish_tick'] is not None,outputs_committed=j['outputs_committed']))
    for name,old in original['tasks'].items():
        values=by_task[name]
        if not values:raise ValueError('No actual CPU samples for '+name)
        lo=float(np.mean(values));hi=max(values);iat=intervals[name];period=old['T_nominal_s']
        table['tasks'][name]={**old,'C_LO_s':lo,'C_HI_s':hi,'sample_count':len(values),'historical_Q95_s':float(np.quantile(values,.95)),'observed_mean_inter_entry_s':float(np.mean(iat)) if iat else None,'observed_min_inter_entry_s':min(iat) if iat else None,'nominal_rate_hz':None if period is None else 1/period,'observed_mean_rate_hz':None if not iat or not np.mean(iat) else 1/float(np.mean(iat)),'C_min_s':min(values),'C_median_s':float(np.median(values)),'C_p99_s':float(np.quantile(values,.99)),'inter_arrival_clock':'SIM_ACTUAL_CALLBACK_ARRIVAL','observed_max_inter_entry_s':max(iat) if iat else None,'observed_std_inter_entry_s':float(np.std(iat,ddof=1)) if len(iat)>1 else None,'C_std_s':float(np.std(values,ddof=1)) if len(values)>1 else 0.}
        for label,c in [('LO',lo),('HI',hi)]:
            work=c*1000.;table['tasks'][name].update({f'W_{label}_mcycles':work,f'A7_C_{label}_s':work/1600.,f'A15_C_{label}_s':work/3600.,f'A7_mid_C_{label}_s':work/1200.,f'A15_mid_C_{label}_s':work/2700.})
    table['source_trace_sha256']=inputs;table.pop('source_sha256',None)
    path=campaign/'frozen-task-parameters.json';path.write_text(json.dumps(table,indent=2)+'\n')
    deadlines=generate_deadlines(load_tasks(path,dual=True),campaign/'hardware.json');(campaign/'deadlines.json').write_text(json.dumps(deadlines,indent=2)+'\n')
    write_csv(campaign/'calibration-samples.csv',samples);write_csv(campaign/'calibration-per-run.csv',summaries)
    write_csv(campaign/'frozen-task-parameters.csv',[dict(task=name,**{k:v for k,v in row.items() if k!='task'}) for name,row in table['tasks'].items()])
    (campaign/'frozen-input-hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [path,campaign/'deadlines.json',campaign/'hardware.json']},indent=2)+'\n')
    return table
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args();t=extract(a.campaign);print(json.dumps({'full_laps':t['successful_laps'],'CPU_samples':sum(r['sample_count'] for r in t['tasks'].values())}))
