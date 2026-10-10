#!/usr/bin/env python3
"""Validate the applied live schedule and draw its measured execution evidence."""
import argparse,collections,csv,hashlib,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.abstract_compute.task_model import load_tasks,TaskSpec
from tools.abstract_compute.hardware import load_platform
from tools.live_bridge.engine import TASK_NAMES
from tools.live_bridge.dvfs import validate_job_execution
from tools.live_bridge.policies import ReadyFIFOOldestIdle

def main():
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--output',type=Path,default=ROOT/'docs/figures/live-bridge');args=p.parse_args()
    trial=args.trial.resolve();out=args.output;out.mkdir(parents=True,exist_ok=True)
    t=json.loads((trial/'trace.json').read_text());jobs={j['job_id']:j for j in t['jobs']};dt=t['step_s'];step_ns=round(dt*1e9);end=round(t['sim_seconds']/dt)
    if t.get('schema',1)>=4:
        from scripts.analyze_placement_trial import analyze
        stats,edge_errors=analyze(trial);out.mkdir(parents=True,exist_ok=True)
        (out/'validation.json').write_text(json.dumps(dict(passed=not edge_errors,errors=edge_errors,metrics=stats),indent=2)+'\n')
        print(json.dumps(stats,indent=2));return bool(edge_errors)
    dual=t.get('schema',1)>=2
    fifo=t.get('scheduling_policy_id',ReadyFIFOOldestIdle.name)==ReadyFIFOOldestIdle.name
    piecewise=t.get('execution_accounting')=='piecewise_DVFS_equivalent_work'
    tasks=({name:TaskSpec(**row) for name,row in t['task_parameters'].items()} if dual and t.get('task_parameters') else load_tasks(trial/'task-parameters.json' if (trial/'task-parameters.json').exists() else None,dual=dual));devices=load_platform(trial/'hardware-config.json' if (trial/'hardware-config.json').exists() else None,device_classes=('vehicle',)*len(t['hardware']));errors=[];intervals={d:[] for d in devices};opened={}
    for e in t['events']:
        if e['kind']=='cooling_start':opened[e['device_id']]=e['tick']
        elif e['kind']=='cooling_end':intervals[e['device_id']].append((opened.pop(e['device_id']),e['tick']))
    for d,a in opened.items():intervals[d].append((a,end))
    mask={d:np.zeros(end,dtype=int) for d in devices}
    for d,rows in intervals.items():
        for a,b in rows:mask[d][a:b]=1
    prefix={d:np.r_[0,np.cumsum(v)] for d,v in mask.items()};deps=budgets=0;occupancy=collections.defaultdict(list);edge_types=collections.Counter();path_bindings=goal_bindings=readiness_checks=0
    for j in jobs.values():
        a,b,d,c=j['start_tick'],j['budget_end_tick'],j['device'],j['core_id']
        if a is None:continue
        readiness_checks+=1
        required=max([j['release_tick'],j['staged_tick']]+[jobs[p]['finish_tick'] if jobs[p]['finish_tick'] is not None else end for p in j['parents']])
        if j['ready_tick']!=required or a<required:errors.append('Readiness mismatch '+str(j['job_id']))
        parent_tasks={jobs[p]['task'] for p in j['parents']}
        if j['task']=='controller_path_install':
            path_bindings+=1
            if 'planning_request' not in parent_tasks:errors.append('Path without selected planner parent')
        if j['task']=='planning_request':
            goal_bindings+=1
            if 'bt_tick' not in parent_tasks:errors.append('Planner without selected BT goal parent')
        for pid in j['parents']:
            deps+=1;edge_types[jobs[pid]['task']+' -> '+j['task']]+=1
            if jobs[pid]['finish_tick'] is None or jobs[pid]['finish_tick']>a:errors.append('Early child '+str(j['job_id']))
        occupancy[d,c].append((a,j['finish_tick'] or end,j['job_id']))
        if piecewise:
            errors.extend(validate_job_execution(j,tasks[j['task']],devices[d].cores[c],dt,mask[d],end))
            budgets+=b is not None
        elif b is not None:
            budgets+=1;actual=b-a-int(prefix[d][b]-prefix[d][a]);spec=tasks[j['task']];core=devices[d].cores[c];duration=(spec.reference_work(j['selected_budget_s'])/(core.dvfs_level().frequency_mhz*core.core_type.performance_eta)) if dual else spec.execution_s(core);expected=math.ceil(duration/dt-1e-9)
            if actual!=expected:errors.append(f'Budget {j["job_id"]}: {actual} != {expected}')
        f=j['finish_tick']
        if f is not None and f<end and mask[d][f]:errors.append('Completion while cooling')
    if dual:
        for j in jobs.values():
            if j['staged_tick'] is None:continue
            if j['measured_cpu_ns'] is None:errors.append('Unmeasured staged job')
            else:
                mode,budget,exceeded=tasks[j['task']].select_budget(j['measured_cpu_ns'])
                if (j['budget_mode'],j['selected_budget_s'],j['observed_max_exceeded'])!=(mode,budget,exceeded):errors.append('Wrong actual-CPU budget selection')
            if j['criticality']!='HI':errors.append('Unexpected LO task')
            if j.get('outputs_committed') and j['real_cpu_ns']!=j['measured_cpu_ns']:errors.append('CPU measurement changed after seal')
        sealed={e['job_id']:e for e in t['events'] if e['kind']=='actual_computation_sealed'}
        publications=[e for e in t['events'] if e['kind']=='publication']
        for e in t['events']:
            if e['kind']=='input_binding' and e['source_ns']:
                matches=[v for v in publications if v['device_id']==e['device_id'] and v['signature']==e['signature'] and v['source_begin_ns']<=e['source_ns']<=v['source_end_ns']]
                if len(matches)==1 and matches[0]['job_id']!=e['parent_id']:errors.append('Lost exact DDS producer binding')
            if e['kind']=='publication' and e.get('job_id'):
                j=jobs[e['job_id']];seal=sealed.get(j['job_id'])
                if not seal or seal['tick']>e['tick']:errors.append('Output before complete actual measurement')
                if j['finish_tick']!=e['tick']:errors.append('Output not delivered at selected modeled finish')
    for key,rows in occupancy.items():
        rows.sort()
        if any(a[1]>b[0] for a,b in zip(rows,rows[1:])):errors.append('Core overlap '+str(key))
    ready={};idle={(d,c):0 for d,device in devices.items() for c in device.cores};busy={};assignments=0
    for e in t['events']:
        kind=e['kind'];jid=e.get('job_id')
        if kind=='ready':ready[jid]=(e['eligible_tick'],jobs[jid]['release_tick'],jid)
        elif kind=='start':
            j=jobs[jid];d,c=j['device'],j['core_id'];eligible=[k for k in ready if jobs[k]['device']==d];free=[n for n in devices[d].cores if (d,n) not in busy]
            if jid not in eligible:errors.append('Ineligible assignment '+str(jid))
            if fifo and (not eligible or min(eligible,key=ready.get)!=jid):errors.append('FIFO mismatch '+str(jid))
            if c not in free:errors.append('Occupied-core assignment '+str(jid))
            if fifo and (not free or min(free,key=lambda n:(idle[d,n],n))!=c):errors.append('Idle-core mismatch '+str(jid))
            ready.pop(jid,None);busy[d,c]=jid;assignments+=1
        elif kind=='finish':
            key=(e['device_id'],e['core_id']);busy.pop(key,None);idle[key]=e['tick']
        elif kind=='publication':
            if jid and (jobs[jid]['finish_tick'] is None or jobs[jid]['finish_tick']>e['tick']):errors.append('Early output')
            d=e.get('device_id',jobs[jid]['device'] if jid else -1)
            if d in mask and e['tick']<end and mask[d][e['tick']]:errors.append('Output while cooling')
    clock_rows=0
    with (trial/'clock-pairs.csv').open() as f:
        for r in csv.DictReader(f):
            clock_rows+=1
            if int(r['tick'])!=clock_rows or int(r['physics_ns'])!=clock_rows*step_ns or r['physics_ns']!=r['hardware_ns'] or int(r['real_callbacks_busy']):errors.append('Clock mismatch')
    if clock_rows!=end:errors.append('Missing clock pairs')
    if t['failure']:errors.append(t['failure'])
    origin=t.get('origin_tick');common_start={}
    if origin is not None:
        accepted=[e for e in t['events'] if e['kind']=='mission_accepted']
        first={d:min(j['start_tick'] for j in jobs.values() if j['device']==d and j['start_tick'] is not None) for d in devices}
        if {e['device_id'] for e in accepted}!=set(devices) or any(e['tick']!=origin for e in accepted):errors.append('Goals not accepted at common start')
        if set(first.values())!={origin}:errors.append('Initial hardware dispatch not simultaneous')
        common_start={'origin_sim_s':origin*dt,'race_s':t['race_seconds'],'goal_acceptance_ticks':[e['tick'] for e in accepted],'first_core_start_ticks':first}
    completed=collections.Counter(j['task'] for j in jobs.values() if j['finish_tick'] is not None)
    if set(completed)!=set(TASK_NAMES):errors.append('Missing completed task class')
    bindings=collections.Counter((e['method'],bool(e['parent_id'])) for e in t['events'] if e['kind']=='input_binding')
    odom={d:[] for d in devices}
    with (trial/'ros-observations.csv').open() as f:
        for r in csv.DictReader(f):
            if r['topic']=='odom':odom[int(r['device'])].append((int(r['sim_ns'])/1e9,float(r['x']),float(r['y'])))
    moving={d:sum(a[0]<b[0] and math.dist(a[1:],b[1:])>1e-4 and any(x*dt<=a[0] and b[0]<=y*dt for x,y in intervals[d]) for a,b in zip(rows,rows[1:])) for d,rows in odom.items()}
    live_thermal=[]
    live_file=trial/'live-events.jsonl'
    if live_file.exists():
        with live_file.open() as stream:
            for line in stream:
                row=json.loads(line)
                if row.get('kind')=='thermal':
                    row.pop('kind');live_thermal.append(row)
    if live_thermal and live_thermal!=t['thermal']:errors.append('Live per-core thermal samples differ from the applied model')
    summary={'trial':str(trial.relative_to(ROOT)),'step_s':dt,'sim_s':t['sim_seconds'],'host_s':t['host_seconds'],'jobs':len(jobs),'modeled_completions':sum(completed.values()),'completed_counts':dict(completed),'clock_pairs_verified':clock_rows,'dependency_edges_verified':deps,'dependency_types':dict(edge_types),'readiness_times_verified':readiness_checks,'selected_path_parents_verified':path_bindings,'selected_BT_goal_parents_verified':goal_bindings,'budgets_verified':budgets,'scheduling_policy_id':t.get('scheduling_policy_id',ReadyFIFOOldestIdle.name),'core_assignments_verified':assignments,'FIFO_assignments_verified':assignments if fifo else 0,'validation_errors':errors,'cooling_entries':sum(e['kind']=='cooling_start' for e in t['events']),'cooling_exits':sum(e['kind']=='cooling_end' for e in t['events']),'exact_DDS_source_bindings':bindings['DDS_source_interval',True],'payload_or_UUID_bindings':bindings['equivalent_payload_or_goal_UUID',True],'unresolved_or_external_inputs':sum(n for (method,found),n in bindings.items() if not found),'moving_odometry_segments_during_cooling':moving,'trace_sha256':hashlib.sha256((trial/'trace.json').read_bytes()).hexdigest(),'outcome':'Intentional finite trial; no full-lap completion is claimed.'}
    summary['common_start']=common_start
    if dual:
        summary['budget_policy']=t['config']['budget_policy']
        summary['selected_budget_counts']=dict(collections.Counter(j['budget_mode'] for j in jobs.values() if j['budget_mode']))
        summary['observed_max_exceeded_jobs']=sum(j['observed_max_exceeded'] for j in jobs.values())
        summary['sealed_computations']=len(sealed)
        summary['outputs_at_exact_selected_finish']=sum(e['kind']=='publication' and bool(e.get('job_id')) for e in t['events'])
    if piecewise:
        summary['execution_accounting']=t['execution_accounting']
        summary['DVFS_jobs_verified']=sum(j['start_tick'] is not None for j in jobs.values())
        summary['DVFS_change_events']=sum(e['kind']=='dvfs_change' for e in t['events'])
        summary['job_overrun_events']=sum(e['kind']=='job_overrun' for e in t['events'])
        summary['active_ticks_by_point']={f'{d}:{c}':dict(collections.Counter({level:sum(s['end_tick']-s['start_tick'] for j in jobs.values() if j['device']==d and j['core_id']==c for s in j['execution_segments'] if s['level_id']==level) for level in devices[d].cores[c].core_type.dvfs_by_id})) for d,c in occupancy}
    summary['thermal_configuration']=t.get('hardware_configuration',{}).get('thermal',devices[0].thermal_spec.__dict__)
    summary['thermal_guard_period_s']=dt
    summary['live_thermal_samples_verified']=len(live_thermal) if live_thermal==t['thermal'] else 0
    summary['stop_reason']=t.get('stop_reason','simulation-time limit')
    summary['cooling_time_s']={d:sum(b-a for a,b in spans)*dt for d,spans in intervals.items()}
    summary['cooling_fraction']={d:value/t['race_seconds'] for d,value in summary['cooling_time_s'].items()}
    summary['peak_sampled_temperature_c']={d:[max(r['temperature_c'][c] for r in t['thermal'] if r['device_id']==d) for c in (0,1)] for d in devices}
    summary['last_sampled_temperature_c']={d:next(r['temperature_c'] for r in reversed(t['thermal']) if r['device_id']==d) for d in devices}
    summary['vehicles']={}
    for color in ('red','blue','white','green')[:len(devices)]:
        r=json.loads((trial/(color+'-mission.json')).read_text());summary['vehicles'][color]={k:r.get(k) for k in ('passed','action_status','nav2_error_code','odometry_distance_m','ordered_targets_passed','navigation_recoveries','scheduler','error')}
    summary['navigation_failed']=any(r.get('error') not in (None,'Interrupted.') for r in summary['vehicles'].values())
    if summary['navigation_failed']:summary['outcome']='Bridge invariant validation and navigation outcome are separate: Nav2 aborted the mission; see vehicles and the retained launch log.'
    (out/'validation.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (out/'task-summary.csv').open('w',newline='') as f:
        w=csv.writer(f,lineterminator="\n");w.writerow(['task','jobs','completed','C_ref_HI_max_s' if dual else 'C_ref_Q95_s','T_metadata_s','mean_release_to_finish_s','mean_actual_thread_CPU_s','C_ref_LO_mean_s','criticality','LO_budget_jobs','HI_budget_jobs'])
        for name in TASK_NAMES:
            rows=[j for j in jobs.values() if j['task']==name];done=[j for j in rows if j['finish_tick'] is not None];cpu=[j['real_cpu_ns']/1e9 for j in done if j['real_cpu_ns']]
            w.writerow([name,len(rows),len(done),tasks[name].budget_s,tasks[name].period_s,np.mean([(j['finish_tick']-j['release_tick'])*dt for j in done]),np.mean(cpu) if cpu else '',tasks[name].budget_lo_s,tasks[name].criticality,sum(j.get('budget_mode')=='LO' for j in rows),sum(j.get('budget_mode')=='HI' for j in rows)])
    palette={name:plt.get_cmap('tab20')(i) for i,name in enumerate(TASK_NAMES)};first=min(j['start_tick'] for j in jobs.values() if j['task']=='control_iteration' and j['start_tick'] is not None)*dt;first=origin*dt if origin is not None else first;last=first+2.
    fig,ax=plt.subplots(figsize=(14,5.5),layout='constrained')
    for d,rows in intervals.items():
        for a,b in rows:
            if b*dt>first and a*dt<last:ax.broken_barh([(max(a*dt,first),min(b*dt,last)-max(a*dt,first))],(2*d-.4,1.8),facecolors='#f5d2d2',alpha=.7)
    for j in jobs.values():
        if j['start_tick'] is None:continue
        spans=[(j['start_tick'],j['budget_end_tick'] or end)];d=j['device']
        for a,b in intervals[d]:spans=[(x,y) for lo,hi in spans for x,y in ((lo,min(a,hi)),(max(lo,b),hi)) if y>x]
        for a,b in spans:
            left=max(a*dt,first);width=min(b*dt,last)-left
            if width>0:ax.broken_barh([(left,width)],(2*d+j['core_id']-.28,.56),facecolors=palette[j['task']])
    ax.set(xlim=(first,last),ylim=(-.6,2*len(devices)-.3),xlabel='Simulation / hardware time (s)',title='Applied live schedule — '+t.get('scheduling_policy_id','ready FIFO / oldest idle core'));ax.set_yticks(range(2*len(devices)),[f'{car} / {core}' for car in ('Red','Blue','White','Green')[:len(devices)] for core in ('A7','A15')]);ax.grid(axis='x',alpha=.2)
    handles=[Patch(facecolor=palette[n],label=tasks[n].task_id+': '+n.replace('_',' ')) for n in TASK_NAMES]+[Patch(facecolor='#f5d2d2',label='Whole-device cooling')]
    if origin is not None:
        from matplotlib.ticker import FuncFormatter
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x,pos:f'{x-origin*dt:.2f}'));ax.set_xlabel('Time since common start (s)')
    ax.set_xticks(np.linspace(first,last,11));ax.invert_yaxis()
    ax.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,-.13),ncol=3,fontsize=8,frameon=False);fig.savefig(out/'applied-gantt.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,6),layout='constrained',sharex=True);colors=('#c9252d','#2374c5','#888888','#25974b')
    for d in devices:
        rows=[r for r in t['thermal'] if r['device_id']==d];time=[(r['tick']-(origin or 0))*dt for r in rows]
        for c,style in ((0,'--'),(1,'-')):axes[0].plot(time,[r['temperature_c'][c] for r in rows],style,color=colors[d],lw=.7,label=f'{("Red","Blue","White","Green")[d]} / {("A7","A15")[c]}')
        axes[1].plot(time,[sum(r['power_w']) for r in rows],color=colors[d],lw=.7)
    for value,label in ((devices[0].thermal_spec.max_temperature_c,'Tmax'),(devices[0].thermal_spec.balance_temperature_c,'Tbalance'),(devices[0].thermal_spec.ambient_temperature_c,'Ambient')):axes[0].axhline(value,color='#555',ls=':',label=label)
    axes[0].set(ylabel='Modeled temperature (°C)',title='Modeled thermal guards and instantaneous device power');axes[0].legend(ncol=5,fontsize=7);axes[1].set(xlabel='Time since common start (s)' if origin is not None else 'Simulation / hardware time (s)',ylabel='Modeled power (W)');fig.savefig(out/'thermal.png',dpi=150);plt.close(fig)
    points=[r for rows in odom.values() for r in rows] or [(0.,0.,0.)]
    bounds=dict(xlim=(min(r[1] for r in points)-.25,max(r[1] for r in points)+.25),ylim=(min(r[2] for r in points)-.25,max(r[2] for r in points)+.25))
    fig,axes=plt.subplots(1,len(devices),figsize=(10,4),layout='constrained',squeeze=False)
    for d,(rows,ax) in enumerate(zip(odom.values(),axes.flat)):
        if rows:ax.plot([r[1] for r in rows],[r[2] for r in rows],color=colors[d])
        ax.set(aspect='equal',**bounds,xlabel='Odometry x (m)',ylabel='Odometry y (m)',title=('Red','Blue','White','Green')[d]);ax.grid(alpha=.2)
    fig.suptitle('Actual Gazebo movement — separate per-vehicle odometry frames')
    fig.savefig(out/'motion.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in summary.items() if k not in ('completed_counts','vehicles')},indent=2));return bool(errors)
if __name__=='__main__':raise SystemExit(main())
