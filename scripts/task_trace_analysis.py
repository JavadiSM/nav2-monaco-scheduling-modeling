"""Streaming analysis of kernel spans plus framework callbacks, without flattening millions of callbacks into memory."""
from collections import Counter, defaultdict
from bisect import bisect_right
import csv
import gzip
import json
from pathlib import Path
import numpy as np

class Moments:
    def __init__(self):self.n=0;self.mean=0.;self.m2=0.;self.low=float('inf');self.high=float('-inf')
    def add(self,x):
        self.n+=1;d=x-self.mean;self.mean+=d/self.n;self.m2+=d*(x-self.mean);self.low=min(self.low,x);self.high=max(self.high,x)
    def merge(self,b):
        if not b.n:return
        if not self.n:self.n,self.mean,self.m2,self.low,self.high=b.n,b.mean,b.m2,b.low,b.high;return
        n=self.n+b.n;d=b.mean-self.mean;self.m2+=b.m2+d*d*self.n*b.n/n;self.mean+=d*b.n/n;self.n=n;self.low=min(self.low,b.low);self.high=max(self.high,b.high)
    def result(self):
        return {'n':self.n,'mean':self.mean if self.n else None,'std':(max(0,self.m2/(self.n-1)))**.5 if self.n>1 else 0. if self.n else None,'min':self.low if self.n else None,'max':self.high if self.n else None,'p50':None,'p95':None,'p99':None,'quantiles':'not computed for streaming callback groups; complete per-callback raw spans are retained'}
def ptr(v):
    if isinstance(v,int):return v
    try:return int(v,16) if v.startswith('0x') else 0
    except (TypeError,ValueError,AttributeError):return 0
def load_rows(path):
    opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'rt') as f:yield from csv.DictReader(f)
def cache_signature(run):
    meta=run/'metadata.json'
    return (meta.stat().st_size,meta.stat().st_mtime_ns)

def analyze(run,stat,primary,schema=8):
    meta=json.loads((run/'metadata.json').read_text());mission=json.loads((run/'mission.json').read_text()) if (run/'mission.json').exists() else {}
    start=mission.get('goal_send_mono_ns',0);end=mission.get('action_result_mono_ns',mission.get('end_mono_ns',2**63-1))
    summary_path=run/'summary.json';cache=run/'analysis-cache.npz';graph_path=run/'trace-graph-evidence.json'
    signature=list(cache_signature(run))
    if summary_path.exists() and cache.exists() and graph_path.exists():
        result=json.loads(summary_path.read_text())
        if result.get('schema')==schema and result.get('cache_signature')==signature:
            with np.load(cache,allow_pickle=False) as saved:
                buckets=defaultdict(dict);topics=defaultdict(dict)
                for key in saved.files:
                    kind,metric=key.split('::',1)
                    target=topics if kind.startswith('topic=') else buckets
                    target[kind[6:] if kind.startswith('topic=') else kind][metric]=saved[key].copy()
            evidence=json.loads(graph_path.read_text())
            return result,buckets,topics,evidence['spans'],evidence['events']
    buckets=defaultdict(lambda:defaultdict(list));children=defaultdict(float);child_kernel_parents=set();spans=[];events=[];metadata={};all_spans=active_count=message_count=bad=0
    cb_stats=defaultdict(lambda:defaultdict(Moments));cb_objects={};cb_prev={};exceptions=Counter();actor_rows=defaultdict(list)
    dds=defaultdict(list);timer_last={};timer_pairs=[];cb_interval_regressions=0;unbound_callbacks=0
    graph_low=start+3_000_000_000;graph_high=start+8_000_000_000
    fields=['pid','tid','id','parent','kind','object','event','active_mission','wall_start_ns','wall_end_ns','sim_start_ns','sim_end_ns','input_stamp_ns','aux','cpu_inclusive_ms','cpu_exclusive_ms','elapsed_wall_ms','off_cpu_or_wait_ms']
    # Derived sample CSV contains algorithm/worker spans. ALL framework samples remain in raw CSVs.
    output=run/'job-samples.csv'
    with output.open('w',newline='') as out:
        writer=csv.DictWriter(out,fieldnames=fields);writer.writeheader()
        rawfiles=sorted((run/'raw').glob('*.csv'))+sorted((run/'raw').glob('*.csv.gz'))
        for f in rawfiles:
            for raw in load_rows(f):
                try:
                    event=raw['event'];kind=raw['kind']
                    # Only numerics needed by this row are converted.
                    r={**raw}
                    for k in ['id','parent','pid','tid','wall_start_ns','wall_end_ns','cpu_start_ns','cpu_end_ns','sim_start_ns','sim_end_ns','stamp_ns','signature','aux']:r[k]=int(raw[k])
                    for k in ['rmw_received_epoch_ns','rmw_source_epoch_ns','publish_epoch_start_ns','publish_epoch_end_ns']:
                        if k in r:r[k]=int(r[k])
                    isactive=bool(start and start<=r['wall_start_ns']<end)
                    if event=='metadata':metadata[(r['pid'],r['id'])]=r;continue
                    if not event.startswith('span'):
                        message_count+=event in ['publish','take','rmw_publish']
                        if event=='take' and isactive:
                            received=r.get('rmw_received_epoch_ns',-1);source=r.get('rmw_source_epoch_ns',-1)
                            if received>0 and source>0:dds[r['topic']].append((received-source)/1e6)
                        if event=='timer_due':timer_last[(r['pid'],r['tid'])]=r
                        # Shared-state and timing events are small; message graph keeps a finite observation window.
                        if event not in ['publish','take','rmw_publish'] or graph_low<=r['wall_start_ns']<=graph_high:events.append(r)
                        continue
                    all_spans+=1
                    if isactive:active_count+=1
                    c=(r['cpu_end_ns']-r['cpu_start_ns'])/1e6;e=(r['wall_end_ns']-r['wall_start_ns'])/1e6
                    direct=children.pop((r['pid'],r['id']),0.);exclusive=max(0,c-direct)
                    if r['parent']:children[(r['pid'],r['parent'])]+=max(0,c)
                    if kind=='ros_callback':
                        if isactive:
                            if not r['aux']:unbound_callbacks+=1
                            identity=(r['pid'],r['aux'] if r['aux'] else -ptr(r['object']))
                            cb_objects[identity]=r['object']
                            for m,v in [('cpu_inclusive_ms',c),('cpu_exclusive_ms',exclusive),('elapsed_wall_ms',e),('off_cpu_or_wait_ms',max(0,e-c))]:cb_stats[identity][m].add(v)
                            if identity in cb_prev:
                                delta=(r['wall_start_ns']-cb_prev[identity])/1e6
                                if delta>=0:cb_stats[identity]['inter_entry_wall_ms'].add(delta)
                                else:cb_interval_regressions+=1
                            cb_prev[identity]=r['wall_start_ns']
                            due=timer_last.pop((r['pid'],r['tid']),None)
                            binding=metadata.get((r['pid'],r['aux']))
                            if due and binding and binding['kind']=='timer_callback' and ptr(binding['object'])==ptr(due['object']):
                                timer_pairs.append((identity,due,r))
                        if (r['pid'],r['id']) in child_kernel_parents:spans.append(r)
                        continue
                    spans.append(r)
                    if r['parent']:child_kernel_parents.add((r['pid'],r['parent']))
                    writer.writerow(dict(pid=r['pid'],tid=r['tid'],id=r['id'],parent=r['parent'],kind=kind,object=r['object'],event=event,active_mission=int(isactive),wall_start_ns=r['wall_start_ns'],wall_end_ns=r['wall_end_ns'],sim_start_ns=r['sim_start_ns'],sim_end_ns=r['sim_end_ns'],input_stamp_ns=r['stamp_ns'],aux=r['aux'],cpu_inclusive_ms=c,cpu_exclusive_ms=exclusive,elapsed_wall_ms=e,off_cpu_or_wait_ms=max(0,e-c)))
                    if isactive:
                        for m,v in [('cpu_inclusive_ms',c),('cpu_exclusive_ms',exclusive),('elapsed_wall_ms',e),('off_cpu_or_wait_ms',max(0,e-c))]:buckets[kind][m].append(v)
                        actor_rows[(kind,r['pid'],r['object'])].append(r)
                        if event!='span':exceptions[kind]+=1
                except (ValueError,KeyError,TypeError):bad+=1
    # Complete algorithm spans allow chronological grouping independent of per-thread flush order.
    for (kind,pid,obj),rows in actor_rows.items():
        rows.sort(key=lambda r:r['wall_start_ns'])
        for a,b in zip(rows,rows[1:]):
            buckets[kind]['inter_entry_wall_ms'].append((b['wall_start_ns']-a['wall_start_ns'])/1e6)
            if a['sim_start_ns']>=0 and b['sim_start_ns']>=0:buckets[kind]['inter_entry_cached_sim_ms'].append((b['sim_start_ns']-a['sim_start_ns'])/1e6)
        buckets[kind]['observed_phase_relative_goal_send_ms'].append((rows[0]['wall_start_ns']-start)/1e6)
    # Metadata is joined by the binding generation recorded at callback start.
    by_obj=defaultdict(list)
    cpp_links=defaultdict(list)
    for r in metadata.values():by_obj[(r['pid'],r['kind'],ptr(r['object']))].append(r)
    for r in metadata.values():
        if r['kind']=='subscription_cpp':cpp_links[(r['pid'],r['signature'])].append(r)
    for values in by_obj.values():values.sort(key=lambda r:r['wall_start_ns'])
    for values in cpp_links.values():values.sort(key=lambda r:r['wall_start_ns'])
    times={k:[r['wall_start_ns'] for r in rows] for k,rows in by_obj.items()}
    def latest(pid,kind,obj,at=2**63-1):
        key=(pid,kind,obj);rows=by_obj.get(key,[]);i=bisect_right(times.get(key,[]),at)-1
        return rows[i] if i>=0 else None
    def first_after(pid,kind,obj,at):
        key=(pid,kind,obj);rows=by_obj.get(key,[]);i=bisect_right(times.get(key,[]),at)
        return rows[i] if i<len(rows) else None
    def node_name(pid,obj,at):
        r=latest(pid,'node_name',obj,at);return '/'+r['topic'].strip('/') if r else 'unresolved_node'
    callback_rows=[];groups=defaultdict(lambda:defaultdict(Moments));resolved={};group_meta=defaultdict(lambda:{'periods':set(),'clocks':set(),'bindings':0})
    for identity,metrics in cb_stats.items():
        pid,generation=identity;binding=metadata.get((pid,generation)) if generation>0 else None
        kind='unresolved_callback';node='unresolved_node';source='unresolved';period=None;clock_type=None
        if binding:
            at=binding['wall_start_ns'];handle=ptr(binding['object'])
            if binding['kind']=='subscription_callback':
                candidates=[r for r in cpp_links.get((pid,handle),[]) if r['wall_start_ns']<=at]
                link=max(candidates,key=lambda r:r['wall_start_ns']) if candidates else None
                info=latest(pid,'subscription_node_topic',ptr(link['object']),at) if link else None
                if info:kind='subscription';source=info['topic'];node=node_name(pid,info['signature'],at)
            elif binding['kind']=='timer_callback':
                info=first_after(pid,'timer_node',handle,at);p=latest(pid,'timer_clock',handle,at)
                next_binding=first_after(pid,'timer_callback',handle,at)
                if info and next_binding and info['wall_start_ns']>=next_binding['wall_start_ns']:info=None
                if info is None:info=latest(pid,'timer_node',handle,at)
                kind='timer';source='timer';node=node_name(pid,info['signature'],info['wall_start_ns']) if info else node
                if p:period=p['stamp_ns']/1e6;clock_type=p['aux']
            elif binding['kind']=='service_callback':
                info=latest(pid,'service_node_name',handle,at);kind='service'
                if info:source=info['topic'];node=node_name(pid,info['signature'],at)
        role=f'{node} | {kind} | {source}'
        resolved[identity]=(role,kind,node,period,clock_type)
        gm=group_meta[(pid,role)];gm['bindings']+=1
        if period is not None:gm['periods'].add(period)
        if clock_type is not None:gm['clocks'].add(clock_type)
        for m,v in metrics.items():groups[(pid,role)][m].merge(v)
        callback_rows.append({'pid':pid,'binding_generation':generation,'callback_object':cb_objects[identity],'role':role,'kind':kind,'source':source,'node':node,'configured_timer_period_ms':period,'timer_clock_type':clock_type,'metrics':{m:v.result() for m,v in metrics.items()}})
    callbacks=[{'pid':pid,'role':role,'configured_periods_ms':sorted(group_meta[(pid,role)]['periods']),'clock_types':sorted(group_meta[(pid,role)]['clocks']),'binding_count':group_meta[(pid,role)]['bindings'],'metrics':{m:v.result() for m,v in metrics.items()}} for (pid,role),metrics in sorted(groups.items())]
    timer_metrics=defaultdict(lambda:defaultdict(list));timer_sequences=defaultdict(list)
    kernel_parents={(r['pid'],r['parent']):r['kind'] for r in spans if r['kind']=='velocity_smoothing_tick'}
    for identity,due,callback in timer_pairs:
        role,kind,node,period,clock_type=resolved.get(identity,('unresolved','unresolved','unresolved',None,None))
        if clock_type!=3:continue
        release=due['stamp_ns']
        keys=[role+f' | period_ms={period}']
        kernel=kernel_parents.get((callback['pid'],callback['id']))
        if kernel:keys.append(kernel+f' | period_ms={period}')
        for key in keys:
            timer_metrics[key]['expected_due_to_callback_start_ms'].append((callback['wall_start_ns']-release)/1e6)
            timer_metrics[key]['expected_due_to_callback_finish_ms'].append((callback['wall_end_ns']-release)/1e6)
            timer_metrics[key]['expected_due_to_timer_call_ms'].append((due['aux']-release)/1e6)
            timer_sequences[(key,identity)].append((callback['wall_start_ns'],(callback['wall_start_ns']-release)/1e6,(callback['wall_end_ns']-release)/1e6))
    for (key,identity),seq in timer_sequences.items():
        seq.sort()
        for a,b in zip(seq,seq[1:]):
            timer_metrics[key]['relative_start_jitter_difference_ms'].append(abs(b[1]-a[1]))
            timer_metrics[key]['relative_finish_jitter_difference_ms'].append(abs(b[2]-a[2]))
        if seq:
            timer_metrics[key]['absolute_start_jitter_range_ms_per_binding'].append(max(x[1] for x in seq)-min(x[1] for x in seq))
            timer_metrics[key]['absolute_finish_jitter_range_ms_per_binding'].append(max(x[2] for x in seq)-min(x[2] for x in seq))
    # Loop-level work between two Rate::sleep calls includes the work outside the chosen kernel.
    tid_roles=defaultdict(set)
    for r in spans:
        if r['kind'] in ['control_iteration','local_costmap_update','global_costmap_update','bt_tick']:tid_roles[(r['pid'],r['tid'])].add(r['kind'])
    loops=defaultdict(list)
    for r in spans:
        if r['kind']=='rate_sleep':loops[(r['pid'],r['tid'],r['object'],r['stamp_ns'])].append(r)
    loop_stats={}
    for (pid,tid,obj,period),rows in loops.items():
        roles=tid_roles[(pid,tid)];role=next(iter(roles)) if len(roles)==1 else 'unresolved_loop'
        expected={'control_iteration':50_000_000,'local_costmap_update':200_000_000,'global_costmap_update':1_000_000_000,'bt_tick':10_000_000}.get(role)
        if expected!=period:continue
        rows.sort(key=lambda r:r['wall_start_ns']);values=defaultdict(list)
        for a,b in zip(rows,rows[1:]):
            if start<=a['wall_end_ns']<end and start<=b['wall_start_ns']<end:
                values['full_iteration_cpu_ms'].append((b['cpu_start_ns']-a['cpu_end_ns'])/1e6)
                values['full_iteration_elapsed_wall_ms'].append((b['wall_start_ns']-a['wall_end_ns'])/1e6)
                values['inter_wakeup_wall_ms'].append((b['wall_end_ns']-a['wall_end_ns'])/1e6)
        loop_stats[role]={'samples':len(values['full_iteration_cpu_ms']),'period_config_ms':period/1e6,'sleep_returned_false_count':sum(not r['aux'] for r in rows if start<=r['wall_start_ns']<end),'metrics':{m:stat(v) for m,v in values.items()},'release_semantics':'Sleep return is observed software continuation, not OS readiness or ideal periodic due time.'}
        for m,v in values.items():buckets[role][m].extend(v)
    active=[r for r in spans if r['kind']!='ros_callback' and start<=r['wall_start_ns']<end]
    tasks={k:{'metrics':{m:stat(v) for m,v in ms.items()},'samples':len(ms['cpu_inclusive_ms']),'exceptions':exceptions[k],'actors':[{'pid':pid,'object':obj,'tids':sorted({r['tid'] for r in rows})} for (kind,pid,obj),rows in actor_rows.items() if kind==k]} for k,ms in sorted(buckets.items())}
    by_parent=defaultdict(list)
    for r in active:by_parent[(r['pid'],r['parent'])].append(r)
    plans=[r for r in active if r['kind']=='planning_request']
    features={'planning_segments_per_request':stat([sum(c['kind']=='planning_segment' for c in by_parent[(p['pid'],p['id'])]) for p in plans]),'published_particle_counts':stat([r['aux'] for r in active if r['kind']=='amcl_particle_publication']),'installed_path_pose_counts':stat([r['aux'] for r in active if r['kind']=='controller_path_install']),'filter_updates':sum(r['kind']=='amcl_filter_update' for r in active),'scan_callbacks':sum(r['kind']=='amcl_scan_callback' for r in active)}
    feature_points=[]
    for p in plans:feature_points.append({'task':'planner','feature':sum(c['kind']=='planning_segment' for c in by_parent[(p['pid'],p['id'])]),'cpu_ms':(p['cpu_end_ns']-p['cpu_start_ns'])/1e6})
    for a in active:
        if a['kind']=='amcl_scan_callback':
            counts=[c['aux'] for c in by_parent[(a['pid'],a['id'])] if c['kind']=='amcl_particle_publication']
            if counts:feature_points.append({'task':'amcl','feature':counts[-1],'cpu_ms':(a['cpu_end_ns']-a['cpu_start_ns'])/1e6})
    # Source intervals and observer receipt intervals are retained separately.
    topic_values=defaultdict(lambda:defaultdict(list));prev={};topic_counts=Counter();clock_first=clock_last=None
    observer=run/'observer.jsonl'
    if observer.exists():
        for line in observer.open():
            try:
                r=json.loads(line)
                if not start<=r['receipt_mono_ns']<end:continue
                t=r['topic'];topic_counts[t]+=1
                if t in prev:
                    a=prev[t];topic_values[t]['observer_inter_receipt_wall_ms'].append((r['receipt_mono_ns']-a['receipt_mono_ns'])/1e6)
                    if 'source_stamp_ns' in r and 'source_stamp_ns' in a:topic_values[t]['source_stamp_interval_sim_ms'].append((r['source_stamp_ns']-a['source_stamp_ns'])/1e6)
                prev[t]=r
                if t=='/clock':
                    if clock_first is None:clock_first=r
                    clock_last=r
            except (KeyError,ValueError):bad+=1
    topics={t:{'samples':topic_counts[t],'metrics':{m:stat(v) for m,v in ms.items()}} for t,ms in topic_values.items()}
    if clock_first and clock_last:
        dw=clock_last['receipt_mono_ns']-clock_first['receipt_mono_ns'];ds=clock_last['source_stamp_ns']-clock_first['source_stamp_ns'];topics['/clock']['observed_real_time_factor']=ds/dw if dw else None
    # Middleware delay covers the full active mission, not just the graph window.
    locks=defaultdict(list)
    for r in events:
        if r['event']=='mutex_acquire' and start<=r['wall_start_ns']<end:locks[r['kind']].append((r['wall_end_ns']-r['wall_start_ns'])/1e6)
    thread_policy=[r for r in metadata.values() if r['kind']=='thread_os_policy']
    # Keep a small graph window, plus every referenced writer and ancestor needed to substantiate it.
    keep={(r['pid'],r['id']) for r in spans if graph_low<=r['wall_start_ns']<=graph_high and r['kind']!='rate_sleep'}
    by_id={(r['pid'],r['id']):r for r in spans}
    graph_events=[r for r in events if graph_low<=r['wall_start_ns']<=graph_high]
    for r in graph_events:
        if r['parent']:keep.add((r['pid'],r['parent']))
        if r['event'] in ['state_read','trigger_read'] and r['signature']:keep.add((r['pid'],r['signature']))
    for _ in range(8):
        for key in list(keep):
            r=by_id.get(key)
            if r and r['parent']:keep.add((r['pid'],r['parent']))
    graph_spans=[r for r in spans if (r['pid'],r['id']) in keep]
    graph_events +=[r for r in events if r['event'] in ['state_commit','trigger_commit'] and (r['pid'],r['parent']) in keep and r not in graph_events]
    duration=(end-start)/1e9 if start and end<2**63-1 else None
    lookup={(r['pid'],r['id']):r for r in spans};named_cpu_s=0.
    for r in active:
        if r['kind'] not in primary:continue
        key=(r['pid'],r['parent']);nested=False
        for _ in range(32):
            parent=lookup.get(key)
            if not parent:break
            if parent['kind'] in primary:nested=True;break
            key=(parent['pid'],parent['parent'])
        if not nested:named_cpu_s+=(r['cpu_end_ns']-r['cpu_start_ns'])/1e9
    bindings_path=run/'callback-binding-details.json'
    bindings_path.write_text(json.dumps(callback_rows,separators=(',',':'))+'\n')
    result={'schema':schema,'cache_signature':signature,'campaign':run.parent.name,'run':run.name,'passed':meta.get('passed',False),'profile_complete_shutdown':not meta.get('simulation_shutdown',{}).get('forced',True),'active_start_mono_ns':start,'active_end_mono_ns':end,'active_duration_wall_s':duration,'all_recorded_spans':all_spans,'active_spans':active_count,'message_events':message_count,'malformed_rows':bad,'tasks':tasks,'topics':topics,'features':features,'feature_points':feature_points,'callbacks':callbacks,'callback_binding_details_file':str(bindings_path),'unbound_callback_count':unbound_callbacks,'callback_interval_regressions':cb_interval_regressions,'loop_stats':loop_stats,'thread_os_policy':thread_policy,'mutex_acquisition_elapsed_ms':{k:stat(v) for k,v in locks.items()},'DDS_source_to_receive_ms':{k:stat(v) for k,v in dds.items()},'steady_timer_due_metrics':{k:{m:stat(v) for m,v in ms.items()} for k,ms in timer_metrics.items()},'graph_window_mono_ns':[graph_low,graph_high],'named_work_cpu_s':named_cpu_s,'named_work_average_core_demand':named_cpu_s/duration if duration else None}
    summary_path.write_text(json.dumps(result,indent=2)+'\n')
    saved={k+'::'+m:np.asarray(v,dtype=float) for k,ms in buckets.items() for m,v in ms.items()}
    # Source timing arrays also belong to the per-run cache.
    for k,ms in topic_values.items():
        for m,v in ms.items():saved['topic='+k+'::'+m]=np.asarray(v,dtype=float)
    np.savez(cache,**saved)
    graph_path.write_text(json.dumps({'spans':graph_spans,'events':graph_events,'metadata':[r for r in metadata.values() if r['kind'] in ['node_name','thread_os_policy','clock_anchor']]},indent=2)+'\n')
    return result,buckets,topic_values,graph_spans,graph_events
