#!/usr/bin/env python3
"""Analyze completed ROS/Nav2 trials; never launch or control a simulation."""
import argparse
from array import array
from collections import Counter, defaultdict
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
from zoneinfo import ZoneInfo
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
PRIMARY={
 'control_iteration':('P*','20 Hz / 50 ms','host WallRate','Active FollowPath loop; nominal periodic, jitter/skip sensitive'),
 'local_costmap_update':('P*','5 Hz / 200 ms','host WallRate','Dedicated cyclic map update; publication is a separate activity'),
 'global_costmap_update':('P*','1 Hz / 1000 ms','host WallRate','Dedicated cyclic map update'),
 'velocity_smoothing_tick':('P*','20 Hz / 50 ms','host wall timer','Periodic callback reading a held command; can suppress publication'),
 'bt_tick':('P*','100 Hz / 10 ms','host WallRate','Active mission orchestration tick; action workers run independently'),
 'planning_request':('A','1 Hz rate limiter; not T=1 s','host rate limiter + action','Aperiodic action job; computation and BT limiter state affect arrivals'),
 'amcl_scan_callback':('A','LiDAR source 5 Hz; no callback T guarantee','scan + TF filter','Message-triggered job; sensor generation is periodic, delivery may burst'),
 'mppi_noise_generation':('A','Triggered; no configured T','condition-variable wakeup','Helper job; readiness flag can coalesce triggers; separate thread'),
 'velocity_command_callback':('A','Message-triggered; no configured T','command arrival','Input callback stores a command for later timer ticks'),
 'collision_check':('A','Message-triggered; no configured T','smoothed command arrival','Checks current scan/TF cache and forwards or modifies command'),
 'controller_path_install':('A','Action/state update; no configured T','FollowPath goal/preemption','Installs the current path; not a /plan subscription'),
}
CHILD={
 'planning_segment':'Sequential computational subjob of a planning action; not an independent periodic task',
 'mppi_optimize':'Nested computational subjob of one control iteration; inclusive control C already contains it',
 'amcl_filter_update':'Conditional nested subjob; heavy filter work is not performed by every scan callback',
 'amcl_particle_publication':'Nested publication subjob; aux records actual particle count',
 'velocity_command_stamped_callback':'Internal forwarding subjob for the unstamped input callback in this configuration',
 'collision_check_stamped':'Internal forwarding subjob for the unstamped collision callback in this configuration',
}

def write_json(path,data):
 path.parent.mkdir(parents=True,exist_ok=True)
 tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');tmp.replace(path)
def stat(values):
 a=np.asarray(values,dtype=float)
 if not len(a):return {'n':0,'mean':None,'std':None,'min':None,'max':None,'p50':None,'p95':None,'p99':None}
 return {'n':len(a),'mean':float(a.mean()),'std':float(a.std(ddof=1)) if len(a)>1 else 0.,'min':float(a.min()),'max':float(a.max()),'p50':float(np.percentile(a,50)),'p95':float(np.percentile(a,95)),'p99':float(np.percentile(a,99))}

def merged_stats(items):
 from task_trace_analysis import Moments
 total=Moments()
 for item in items:
  if not item or not item.get('n'):continue
  one=Moments();one.n=item['n'];one.mean=item['mean'];one.m2=(item.get('std') or 0)**2*max(0,one.n-1);one.low=item['min'];one.high=item['max'];total.merge(one)
 return total.result()

def main():
 from task_trace_analysis import analyze
 p=argparse.ArgumentParser();p.add_argument('--final',action='store_true');p.add_argument('--campaign',action='append');p.add_argument('--main-cohort',default='full-v2-20261009');p.add_argument('--reportlab-path');p.add_argument('--no-report',action='store_true');args=p.parse_args()
 cutoff=datetime.fromisoformat('2026-10-09T09:00:00+03:30').timestamp()
 if args.final and time.time()<cutoff:raise SystemExit('Final report is forbidden before the user cutoff. Use a preliminary report.')
 base=ROOT/'artifacts/task-profiling'
 campaigns=[base/x for x in args.campaign] if args.campaign else sorted(d for d in base.iterdir() if d.is_dir() and (d/'manifest.json').exists())
 manifests=[json.loads((d/'manifest.json').read_text()) for d in campaigns]
 if args.final and any(m.get('status')=='running' for m in manifests):raise SystemExit('A campaign has not sealed its final attempt. Wait for simulator shutdown and evidence flushing; final counts must include that attempt.')
 main_cohort=args.main_cohort if any(d.name==args.main_cohort for d in campaigns) else campaigns[-1].name
 pooled=defaultdict(lambda:defaultdict(lambda:array('d')));topicpool=defaultdict(lambda:defaultdict(lambda:array('d')))
 trials=[];attempts=[];all_summaries=[];features=[];example=None
 for campaign,manifest in zip(campaigns,manifests):
  for entry in manifest.get('runs',[]):
   run=campaign/entry['path'];attempts.append({'campaign':campaign.name,**entry,'metadata':str((run/'metadata.json').relative_to(ROOT))})
   if not (run/'metadata.json').exists() or not (run/'mission.json').exists():continue
   print('Analyze/cache: '+campaign.name+'/'+run.name,flush=True)
   summary,buckets,topics,spans,events=analyze(run,stat,PRIMARY)
   all_summaries.append(summary)
   if campaign.name!=main_cohort or not summary['passed']:continue
   trials.append(summary);features.extend(summary.get('feature_points',[]))
   for k,metrics in buckets.items():
    for metric,v in metrics.items():pooled[k][metric].extend(v)
   for topic,metrics in topics.items():
    for metric,v in metrics.items():topicpool[topic][metric].extend(v)
   if example is None:example=(summary,spans,events)
 tasks={}
 for k,metrics in sorted(pooled.items()):
  means=[t['tasks'][k]['metrics']['cpu_inclusive_ms']['mean'] for t in trials if k in t['tasks']]
  tasks[k]={'class':PRIMARY[k][0] if k in PRIMARY else 'nested/worker','metrics':{m:stat(v) for m,v in metrics.items()},'run_mean_cpu_ms':stat(means),'runs_with_samples':len(means),'exceptions':sum(t['tasks'].get(k,{}).get('exceptions',0) for t in trials)}
 callback_roles=sorted({c['role'] for t in trials for c in t.get('callbacks',[])})
 callbacks=[]
 for role in callback_roles:
  rows=[c for t in trials for c in t.get('callbacks',[]) if c['role']==role]
  metrics=sorted({m for c in rows for m in c['metrics']})
  node,kind,source=role.split(' | ',2)
  interpretation='A: message-triggered' if kind=='subscription' else 'A: request-triggered' if kind=='service' else 'P while armed; dynamic timer lifetimes are event-driven' if kind=='timer' else 'Unresolved; no assumed activation contract'
  callbacks.append({'role':role,'node':node,'kind':kind,'source':source,'activation_model':interpretation,'configured_periods_ms':sorted({p for c in rows for p in c.get('configured_periods_ms',[])}),'clock_types':sorted({p for c in rows for p in c.get('clock_types',[])}),'binding_generations':sum(c.get('binding_count',0) for c in rows),'metrics':{m:merged_stats([c['metrics'].get(m) for c in rows]) for m in metrics}})
 cohorts={}
 for campaign in campaigns:
  group=[s for s in all_summaries if s['campaign']==campaign.name and s['passed']]
  kinds=sorted({k for s in group for k in s['tasks']})
  cohorts[campaign.name]={'successful_laps':len(group),'task_statistics':{k:{'cpu_inclusive_ms':merged_stats([s['tasks'].get(k,{}).get('metrics',{}).get('cpu_inclusive_ms') for s in group]),'mean_of_run_C_means_ms':stat([s['tasks'][k]['metrics']['cpu_inclusive_ms']['mean'] for s in group if k in s['tasks']])} for k in kinds}}
 data={'status':'FINAL' if args.final else 'PRELIMINARY','generated_iso':datetime.now(ZoneInfo('Asia/Tehran')).isoformat(),'simulation_cutoff_iso':'2026-10-09T09:00:00+03:30','notation_reference':'Buttazzo (2011), Section 2.2 pp. 26-29 / PDF pp. 43-46; Section 4.1 pp. 79-82 / PDF pp. 96-99','main_measurement_cohort':main_cohort,'attempts':len(attempts),'successful_analyzed_laps':len(trials),'successful_laps_all_cohorts':sum(s['passed'] for s in all_summaries),'failed_or_truncated_attempts':sum(not e['passed'] for e in attempts),'all_attempts':attempts,'campaign_manifests':manifests,'cohorts':cohorts,'tasks':tasks,'topics':{t:{m:stat(v) for m,v in ms.items()} for t,ms in topicpool.items()},'callbacks':callbacks,'trials':trials,'retained_partial_trials':[s for s in all_summaries if not s['passed']],'feature_points':features,'DDS_source_to_receive_ms':{},'steady_timer_due_metrics':{},'mutex_acquisition_elapsed_ms':{},'measurement_scope':'Measured thread CPU time and monotonic elapsed time; inclusive/exclusive costs and whole-loop costs use distinct granularities. Do not add nested costs. No certified WCET, guaranteed sporadic minimum or OS-ready timestamp. Cached process /clock stamps are not exact job simulator time. Different instrumentation cohorts are never pooled.'}
 for field in ['DDS_source_to_receive_ms','mutex_acquisition_elapsed_ms']:
  for key in sorted({k for t in trials for k in t.get(field,{})}):data[field][key]=merged_stats([t.get(field,{}).get(key) for t in trials])
 for role in sorted({r for t in trials for r in t.get('steady_timer_due_metrics',{})}):
  ms=sorted({m for t in trials for m in t.get('steady_timer_due_metrics',{}).get(role,{})})
  data['steady_timer_due_metrics'][role]={m:merged_stats([t.get('steady_timer_due_metrics',{}).get(role,{}).get(m) for t in trials]) for m in ms}
 evidence=ROOT/'docs/evidence';write_json(evidence/'task-characterization-summary.json',data)
 fields=['task','class','configured_activation','clock','interpretation','trials','jobs','C_cpu_mean_ms','C_cpu_std_ms','C_cpu_min_ms','C_cpu_p50_ms','C_cpu_p95_ms','C_cpu_p99_ms','C_cpu_observed_max_ms','elapsed_mean_ms','inter_entry_wall_mean_ms','inter_entry_wall_min_ms','inter_entry_cached_sim_mean_ms','full_iteration_cpu_mean_ms','mean_of_run_C_means_ms','std_of_run_C_means_ms','deadline','WCET','sporadic_minimum_guaranteed']
 with (evidence/'task-characterization-table.csv').open('w',newline='') as out:
  writer=csv.DictWriter(out,fieldnames=fields);writer.writeheader()
  for k,definition in PRIMARY.items():
   t=tasks.get(k,{});m=t.get('metrics',{});c=m.get('cpu_inclusive_ms',stat([]));r=t.get('run_mean_cpu_ms',stat([]))
   writer.writerow({'task':k,'class':definition[0],'configured_activation':definition[1],'clock':definition[2],'interpretation':definition[3],'trials':t.get('runs_with_samples',0),'jobs':c['n'],'C_cpu_mean_ms':c['mean'],'C_cpu_std_ms':c['std'],'C_cpu_min_ms':c['min'],'C_cpu_p50_ms':c['p50'],'C_cpu_p95_ms':c['p95'],'C_cpu_p99_ms':c['p99'],'C_cpu_observed_max_ms':c['max'],'elapsed_mean_ms':m.get('elapsed_wall_ms',{}).get('mean'),'inter_entry_wall_mean_ms':m.get('inter_entry_wall_ms',{}).get('mean'),'inter_entry_wall_min_ms':m.get('inter_entry_wall_ms',{}).get('min'),'inter_entry_cached_sim_mean_ms':m.get('inter_entry_cached_sim_ms',{}).get('mean'),'full_iteration_cpu_mean_ms':m.get('full_iteration_cpu_ms',{}).get('mean'),'mean_of_run_C_means_ms':r['mean'],'std_of_run_C_means_ms':r['std'],'deadline':'UNASSIGNED','WCET':'NOT CERTIFIED','sporadic_minimum_guaranteed':'NOT ESTABLISHED'})
 with (evidence/'task-characterization-per-run.csv').open('w',newline='') as out:
  writer=csv.DictWriter(out,fieldnames=['campaign','run','passed','task','metric','n','mean','std','min','max','p50','p95','p99']);writer.writeheader()
  for s in all_summaries:
   for k,t in s['tasks'].items():
    for metric,values in t['metrics'].items():writer.writerow({'campaign':s['campaign'],'run':s['run'],'passed':s['passed'],'task':k,'metric':metric,**{f:values.get(f) for f in ['n','mean','std','min','max','p50','p95','p99']}})
 with (evidence/'callback-characterization-table.csv').open('w',newline='') as out:
  writer=csv.DictWriter(out,fieldnames=['node','kind','source','activation_model','configured_periods_ms','clock_types','binding_generations','samples','C_cpu_mean_ms','C_cpu_std_ms','C_cpu_min_ms','C_cpu_observed_max_ms','elapsed_mean_ms']);writer.writeheader()
  for c in callbacks:
   cpu=c['metrics'].get('cpu_inclusive_ms',{});writer.writerow({**{f:c[f] for f in ['node','kind','source','activation_model','configured_periods_ms','clock_types','binding_generations']},'samples':cpu.get('n',0),'C_cpu_mean_ms':cpu.get('mean'),'C_cpu_std_ms':cpu.get('std'),'C_cpu_min_ms':cpu.get('min'),'C_cpu_observed_max_ms':cpu.get('max'),'elapsed_mean_ms':c['metrics'].get('elapsed_wall_ms',{}).get('mean')})
 if not args.no_report:
  from render_task_model_report import render
  render(data,pooled,example,reportlab_path=args.reportlab_path)
 print(json.dumps({'status':data['status'],'attempts':data['attempts'],'main_successes':len(trials),'all_successes':data['successful_laps_all_cohorts'],'failed_or_truncated':data['failed_or_truncated_attempts'],'summary':str(evidence/'task-characterization-summary.json')},indent=2))
if __name__=='__main__':main()
