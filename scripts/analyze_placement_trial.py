#!/usr/bin/env python3
"""Independently audit a live placement trace and retain per-job metric evidence."""
import argparse,collections,csv,json,math,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.abstract_compute.hardware import load_platform
from tools.abstract_compute.task_model import TaskSpec
from tools.abstract_compute.communication import distance_delay_s
from tools.live_bridge.dvfs import validate_job_execution
from tools.live_bridge.deadlines import validate_deadlines


def write_csv(path,rows):
    rows=list(rows)
    if not rows:return
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for row in rows for k in row)),lineterminator="\n");w.writeheader();w.writerows(rows)


def analyze(trial):
    trial=Path(trial);t=json.loads((trial/'trace.json').read_text());dt=t['step_s'];end=round(t['sim_seconds']/dt);errors=[]
    jobs={j['job_id']:j for j in t['jobs']};devices=load_platform(trial/'hardware-config.json',device_classes=tuple(t['device_classes']))
    tasks={k:TaskSpec(**v) for k,v in t['task_parameters'].items()};endpoint={int(k):v for k,v in t['endpoint_positions'].items()}
    deadline_table=t['config'].get('deadlines',{})
    if deadline_table.get('schema_version')==2:errors.extend(validate_deadlines(deadline_table,tasks,devices))
    masks={d:np.zeros(end+1,dtype=bool) for d in devices};opened={};cooling=collections.Counter()
    for event in t['events']:
        if event['kind']=='cooling_start':opened[event['device_id']]=event['tick'];cooling[event['device_id']]+=1
        elif event['kind']=='cooling_end':
            d=event['device_id'];a=opened.pop(d);masks[d][a:event['tick']]=True
    for d,a in opened.items():masks[d][a:]=True
    boosts={d:np.zeros(end+1,dtype=bool) for d in devices};expected_boosts={d:np.zeros(end+1,dtype=bool) for d in devices};boost_open={}
    for event in t['events']:
        if event['kind']=='device_boost_start':
            d=event['device_id']
            if d in boost_open:errors.append('Nested device boost start')
            boost_open[d]=event['tick']
            if event['core_ids']!=list(devices[d].cores):errors.append('Boost did not cover every device core')
        elif event['kind']=='device_boost_end':
            d=event['device_id'];a=boost_open.pop(d,None)
            if a is None:errors.append('Unmatched device boost end')
            else:boosts[d][a:event['tick']]=True
    for d,a in boost_open.items():boosts[d][a:]=True
    if t['config'].get('dvfs',{}).get('overrun_policy')=='device_maximum_until_overruns_complete':
        for j in jobs.values():
            if j['overrun_tick'] is not None:expected_boosts[j['device']][j['overrun_tick']:end+1 if j['finish_tick'] is None else j['finish_tick']]=True
        if any(not np.array_equal(boosts[d],expected_boosts[d]) for d in devices):errors.append('Device boost lifetime differs from active overruns')
    transfers={r['transfer_id']:r for r in t['transfers']};edge={};uploads={}
    placements={e['job_id']:e for e in t['events'] if e['kind']=='placement'}
    for r in transfers.values():
        source,target=r['source_device'],r['target_device'];distance=0. if source==target else math.dist(r['source_xy'],r['target_xy'])
        if not math.isclose(distance,r['distance_m'],abs_tol=1e-9):errors.append('Transmission distance mismatch')
        delay=distance_delay_s(distance)
        if not math.isclose(delay,r['delay_s'],abs_tol=1e-12) or r['arrival_tick']!=r['sent_tick']+math.ceil(delay/dt-1e-9):errors.append('Transmission delay mismatch')
        j=jobs[r['job_id']]
        if r['transfer_kind']=='task_upload':
            uploads[j['job_id']]=r
            if source!=j['owner_device'] or target!=j['device'] or r['sent_tick']!=j['release_tick']:errors.append('Incorrect task upload')
        else:
            parent=jobs[r['parent_id']];edge[j['job_id'],parent['job_id']]=r
            if target!=j['device'] or source!=parent['device'] or parent['finish_tick'] is None or r['sent_tick']<max(parent['finish_tick'],j['release_tick']):errors.append('Early/incorrect parent transfer')
    occupancy=collections.defaultdict(list);rows=[];completed=meet=missed=not_due=overruns=0;responses=[]
    for j in jobs.values():
        jid=j['job_id'];spec=tasks[j['task']];d=j['device'];pos=j['placement_position_xy']
        distances={x:math.dist(pos,p) for x,p in endpoint.items()};covered=sorted(x for x,v in distances.items() if v<=5.)
        policy=t['placement_policy'];expected=j['owner_device']
        if policy=='offload' and covered:expected=min(covered,key=lambda x:(distances[x],x))
        if policy=='greedy':
            temperatures={int(k):v for k,v in placements[jid]['device_temperatures_c'].items()}
            expected=min([j['owner_device']]+covered,key=lambda x:(temperatures[x],x))
        if policy!='random' and d!=expected:errors.append('Incorrect device placement')
        if policy=='random' and t['config']['random_scope']=='covered' and d not in [j['owner_device']]+covered:errors.append('Random selection outside candidate set')
        if j['placement_position_tick']!=j['release_tick']:errors.append('Stale placement position')
        upload=uploads.get(jid)
        if not upload or upload['arrival_tick']!=j['upload_arrival_tick']:errors.append('Missing task upload')
        if j['measured_cpu_ns'] is not None:
            expected_budget=('CALIBRATION',j['measured_cpu_ns']/1e9,False) if t['config'].get('calibration_actual_work') else spec.select_budget(j['measured_cpu_ns'])
            if (j['budget_mode'],j['selected_budget_s'],j['observed_max_exceeded'])!=expected_budget:errors.append('Incorrect sealed-CPU budget classification')
            if not math.isclose(j['selected_work_mcycles'],spec.reference_work(j['selected_budget_s']),abs_tol=1e-10):errors.append('Incorrect normalized selected work')
        start=j['start_tick'];finish=j['finish_tick'];response=None
        deadline=j['relative_deadline_s']
        if deadline!=deadline_table['tasks'][j['task']]['D_i_s']:errors.append('Job deadline differs from the shared table')
        deadline_tick=j['release_tick']+deadline/dt
        status='pending_not_due'
        if finish is not None:
            completed+=1;response=(finish-j['release_tick'])*dt;responses.append(response)
            if finish<=deadline_tick+1e-9:meet+=1;status='met'
            else:missed+=1;status='missed'
        elif end>=deadline_tick-1e-9:missed+=1;status='missed_unfinished'
        else:not_due+=1
        if j['overrun_tick'] is not None:overruns+=1
        if start is not None:
            arrivals=[j['release_tick'],j['staged_tick'],j['upload_arrival_tick']]
            for parent_id in j['parents']:
                transfer=edge.get((jid,parent_id));parent=jobs[parent_id]
                if transfer is None or parent['finish_tick'] is None:errors.append('Child without complete parent/transfer');continue
                arrivals += [parent['finish_tick'],transfer['arrival_tick']]
            if start<max(arrivals) or j['ready_tick']!=max(arrivals):errors.append('Early child/upload/readiness '+str(jid))
            errors.extend(validate_job_execution(j,spec,devices[d].cores[j['core_id']],dt,masks[d],end,overrun_policy=t['config'].get('dvfs',{}).get('overrun_policy','remaining_work_at_maximum_point'),device_boost_mask=boosts[d]))
            parent_types={jobs[p]['task'] for p in j['parents']}
            if j['task']=='controller_path_install' and 'planning_request' not in parent_types:errors.append('Installed path without selected planner dependency')
            if j['task']=='planning_request' and 'bt_tick' not in parent_types:errors.append('Planner without selected BT goal dependency')
            if t['config'].get('dvfs',{}).get('policy')=='maximum' and any(span['level_id']!=devices[d].cores[j['core_id']].core_type.default_dvfs_level_id for span in j['execution_segments']):errors.append('Baseline executed below maximum DVFS')
            dvfs=t['config'].get('dvfs',{})
            if dvfs.get('policy')=='fixed':
                for span in j['execution_segments']:
                    forced=dvfs.get('overrun_policy')=='remaining_work_at_maximum_point' and j['overrun_tick'] is not None and span['start_tick']>=j['overrun_tick']
                    if dvfs.get('overrun_policy')=='device_maximum_until_overruns_complete':
                        forced=bool(boosts[d][span['start_tick']])
                        if dvfs['level_id']!=devices[d].cores[j['core_id']].core_type.default_dvfs_level_id and any(boosts[d][span['start_tick']:span['end_tick']]!=forced):errors.append('Segment crosses device boost boundary')
                    expected_level=devices[d].cores[j['core_id']].core_type.default_dvfs_level_id if forced else dvfs['level_id']
                    if span['level_id']!=expected_level:errors.append('Incorrect fixed operating point')
            occupancy[d,j['core_id']].append((start,end if finish is None else finish,jid))
            if finish is not None and masks[d][finish]:errors.append('Result released during execution-device cooling')
        rows.append(dict(job_id=jid,task=j['task'],owner_device=j['owner_device'],execution_device=d,core_id=j['core_id'],release_s=j['release_tick']*dt,staged_s=None if j['staged_tick'] is None else j['staged_tick']*dt,upload_arrival_s=j['upload_arrival_tick']*dt,ready_s=None if j['ready_tick'] is None else j['ready_tick']*dt,start_s=None if start is None else start*dt,finish_s=None if finish is None else finish*dt,C_LO_s=spec.budget_lo_s,C_HI_s=spec.budget_hi_s,selected_budget_s=j['selected_budget_s'],actual_cpu_s=None if j['measured_cpu_ns'] is None else j['measured_cpu_ns']/1e9,budget_mode=j['budget_mode'],overrun=j['overrun_tick'] is not None,D_i_s=deadline,response_s=response,deadline_status=status,parents=json.dumps(j['parents']),position_x_m=pos[0],position_y_m=pos[1],position_tick=j['placement_position_tick']))
    for spans in occupancy.values():
        spans.sort()
        if any(a[1]>b[0] for a,b in zip(spans,spans[1:])):errors.append('Overlapping core reservations')
    sealed={e['job_id']:e for e in t['events'] if e['kind']=='actual_computation_sealed'};queues=collections.defaultdict(list);queue_order=[];pending={};busy={};boosting=set()
    for e in t['events']:
        kind=e['kind'];jid=e.get('job_id')
        fifo=t.get('scheduling_policy_id')=='ready_FIFO_shortest_finish_queue_nonpreemptive'
        if kind=='device_boost_start':boosting.add(e['device_id'])
        elif kind=='device_boost_end':boosting.discard(e['device_id'])
        elif kind=='ready':pending[jid]=(e['eligible_tick'],jobs[jid]['release_tick'],jid)
        elif kind=='core_queue_assignment' and fifo:
            j=jobs[jid]
            eligible=[k for k in pending if jobs[k]['device']==j['device']]
            if not eligible or min(eligible,key=pending.get)!=jid:errors.append('Eligible-job FIFO mapping violation')
            pending.pop(jid,None)
            scores={int(k):v for k,v in e['candidate_finish_ticks'].items()}
            def service(job_id,core,*,running=False):
                job=jobs[job_id];remaining=job['selected_work_mcycles']
                if running:
                    for span in job['execution_segments']:
                        ticks=max(0,min(span['end_tick'],e['tick'])-span['start_tick'])
                        remaining-=min(span['work_mcycles'],ticks*span['frequency_mhz']*core.core_type.performance_eta*dt)
                dvfs=t['config'].get('dvfs',{});level=core.core_type.default_dvfs_level_id
                if dvfs.get('policy')=='fixed':
                    forced=dvfs.get('overrun_policy')=='remaining_work_at_maximum_point' and job['overrun_tick'] is not None and e['tick']>=job['overrun_tick']
                    if dvfs.get('overrun_policy')=='device_maximum_until_overruns_complete':forced=job['device'] in boosting
                    if not forced:level=dvfs['level_id']
                rate=core.core_type.dvfs_by_id[level].frequency_mhz*core.core_type.performance_eta
                return max(0,math.ceil(max(0.,remaining)/(rate*dt)-1e-9))
            expected_scores={}
            for cid,core in devices[j['device']].cores.items():
                key=j['device'],cid;tail=e['tick']
                if key in busy:tail+=service(busy[key],core,running=True)
                tail+=sum(service(k,core) for k in queues[key]);tail+=service(jid,core)
                expected_scores[cid]=tail
            if scores!=expected_scores:errors.append('Incorrect predicted core queue finish')
            if e['core_id']!=min(expected_scores,key=lambda c:(expected_scores[c],c)):errors.append('Not the shortest predicted finish queue')
            if j['ready_tick'] is None or j['ready_tick']>e['tick']:errors.append('Queued before eligibility')
            queues[e['device_id'],e['core_id']].append(jid)
            queue_order.append((e['tick'],e['device_id'],j['ready_tick'],j['release_tick'],jid))
        elif kind=='start' and fifo:
            q=queues[e['device_id'],e['core_id']]
            if not q or q.pop(0)!=jid:errors.append('Core queue FIFO violation')
            busy[e['device_id'],e['core_id']]=jid
        elif kind=='finish':busy.pop((e['device_id'],e['core_id']),None)
        elif kind=='publication' and jid:
            j=jobs[jid]
            if e['device_id']!=j['owner_device']:errors.append('DDS ownership changed with placement')
            if j['finish_tick']!=e['tick'] or jid not in sealed or sealed[jid]['tick']>e['tick']:errors.append('Output before/after modeled release gate')
    groups=collections.defaultdict(list)
    for tick,d,*key in queue_order:groups[tick,d].append(tuple(key))
    publications=collections.defaultdict(list)
    for event in t['events']:
        if event['kind']=='publication':publications[event['device_id'],event['signature']].append(event)
    bindings=matched=unresolved=0
    for event in t['events']:
        if event['kind']!='input_binding':continue
        bindings+=1;unresolved+=not bool(event['parent_id'])
        if not event['source_ns']:continue
        found=[v for v in publications[event['device_id'],event['signature']] if v['source_begin_ns']<=event['source_ns']<=v['source_end_ns']]
        if len(found)==1:
            matched+=1
            if found[0]['job_id']!=event['parent_id']:errors.append('Lost exact DDS producer binding')
    clock_rows=0
    with (trial/'clock-pairs.csv').open() as f:
        for row in csv.DictReader(f):
            clock_rows+=1
            if int(row['tick'])!=clock_rows or int(row['physics_ns'])!=clock_rows*round(dt*1e9) or row['physics_ns']!=row['hardware_ns'] or int(row['real_callbacks_busy']):errors.append('Clock mismatch')
    if clock_rows!=end:errors.append('Incomplete clock evidence')
    if t['failure']:errors.append(t['failure'])
    distance_file=trial/'distance-progress.json';distance=json.loads(distance_file.read_text()).get('distance_m',{}).get('0',0.) if distance_file.exists() else 0.
    run=json.loads((trial/'run.json').read_text()) if (trial/'run.json').exists() else {}
    checkpoint_count=run.get('navigation_outcomes',{}).get('red',{}).get('ordered_targets_passed')
    stats=dict(ordered_targets_passed=checkpoint_count,trial=trial.name,placement=policy,colour={'local':'red','offload':'green','random':'blue','greedy':'amber'}[policy],distance_m=distance,target_reached=bool(run.get('navigation_outcomes',{}).get('red',{}).get('passed')) if run.get('full_course') else distance>=50.,race_seconds=t['race_seconds'],host_seconds=t['host_seconds'],jobs=len(jobs),completed=completed,unfinished=len(jobs)-completed,overrun_count=overruns,overruns_per_1000_jobs=1000*overruns/len(jobs) if jobs else None,deadline_met=meet,deadline_missed=missed,deadline_pending_not_due=not_due,deadline_meet_rate=meet/(meet+missed) if meet+missed else None,average_response_s=float(np.mean(responses)) if responses else None,response_std_s=float(np.std(responses,ddof=1)) if len(responses)>1 else None,response_p95_s=float(np.quantile(responses,.95)) if responses else None,thermal_cooling_entries_total=sum(cooling.values()),thermal_violations_per_device=sum(cooling.values())/len(devices),devices=len(devices),devices_used=len({j['device'] for j in jobs.values()}),completed_outputs_committed=sum(j['finish_tick'] is not None and j['outputs_committed'] for j in jobs.values()),offloaded_jobs=sum(j['device']!=j['owner_device'] for j in jobs.values()),nonzero_task_uploads=sum(r['transfer_kind']=='task_upload' and r['delay_s']>0 for r in transfers.values()),nonzero_parent_transfers=sum(r['transfer_kind']=='edge_data' and r['delay_s']>0 for r in transfers.values()),validation_errors=len(errors),input_bindings=bindings,exact_DDS_bindings_checked=matched,unresolved_input_bindings=unresolved)
    pending_commits=[j['job_id'] for j in jobs.values() if j['finish_tick'] is not None and not j['outputs_committed']]
    stats['completed_outputs_pending_commit']=len(pending_commits)
    mission=json.loads((trial/'red-mission.json').read_text()) if (trial/'red-mission.json').exists() else {}
    stats.update(full_course=run.get('full_course',False),completion_sim_s=mission.get('completion_sim_s'),navigation_recoveries=mission.get('navigation_recoveries'),observed_max_exceeded=sum(j['observed_max_exceeded'] for j in jobs.values()))
    if pending_commits:errors.append('Modeled completed outputs missing commit at cutoff: '+str(pending_commits))
    mode_metrics={}
    for mode in ('LO','HI'):
        chosen=[r for r in rows if r['budget_mode']==mode];due=[r for r in chosen if r['deadline_status']!='pending_not_due'];met=sum(r['deadline_status']=='met' for r in due)
        mode_metrics[mode]=dict(jobs=len(chosen),deadline_evaluable=len(due),deadline_met=met,deadline_meet_rate=met/len(due) if due else None)
    stats['selected_budget_modes']=mode_metrics
    if deadline_table.get('schema_version')==2 and t['config'].get('dvfs',{}).get('policy')=='fixed' and t['config']['dvfs'].get('level_id')==2 and t['config']['dvfs'].get('overrun_policy')=='continue_selected_point' and mode_metrics['HI']['deadline_met']:
        errors.append('HI service unexpectedly met the fastest-A15 deadline')
    stats['validation_errors']=len(errors)
    per_task=[]
    for name in sorted(tasks):
        selected=[r for r in rows if r['task']==name];done=[r for r in selected if r['response_s'] is not None];due=[r for r in selected if r['deadline_status']!='pending_not_due']
        per_task.append(dict(task=name,jobs=len(selected),completed=len(done),overrun_count=sum(r['overrun'] for r in selected),deadline_evaluable=len(due),deadline_met=sum(r['deadline_status']=='met' for r in due),deadline_meet_rate=sum(r['deadline_status']=='met' for r in due)/len(due) if due else None,average_response_s=float(np.mean([r['response_s'] for r in done])) if done else None))
    write_csv(trial/'task-metrics.csv',per_task)
    write_csv(trial/'jobs.csv',rows);write_csv(trial/'transfers.csv',transfers.values());write_csv(trial/'cooling-by-device.csv',[dict(device_id=d,device_class=devices[d].device_class,core_types=';'.join(c.core_type.name for c in devices[d].cores.values()),cooling_entries=cooling[d],cooling_duration_s=float(np.sum(masks[d][:-1]))*dt) for d in devices])
    (trial/'metrics.json').write_text(json.dumps(stats,indent=2)+'\n');(trial/'validation.json').write_text(json.dumps(dict(passed=not errors,errors=errors,checks=['work integral','thermal pauses','exact output gates','device admission','ground-truth pose age','parent data arrivals','FIFO core queues','shortest predicted finish','physics/hardware clocks']),indent=2)+'\n')
    return stats,errors

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args();stats,errors=analyze(a.trial);print(json.dumps(stats,indent=2));raise SystemExit(bool(errors))
