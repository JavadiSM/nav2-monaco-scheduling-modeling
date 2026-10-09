"""English empirical real-time characterization; static figures and a portable PDF."""
from collections import defaultdict
import csv
import html
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
NAMES={'control_iteration':'Control / MPPI','local_costmap_update':'Local map','global_costmap_update':'Global map','velocity_smoothing_tick':'Velocity timer','bt_tick':'BT tick','planning_request':'Planning action','amcl_scan_callback':'AMCL scan','mppi_noise_generation':'Noise helper','velocity_command_callback':'Command input','collision_check':'Collision callback','controller_path_install':'Path install'}
SUBJOBS={'planning_segment':'planning_request','layered_costmap_update':'local_costmap_update or global_costmap_update (keep actors distinct)','mppi_velocity_commands':'control_iteration','mppi_optimize':'mppi_velocity_commands','mppi_noised_trajectories':'mppi_optimize','mppi_noise_copy':'mppi_noised_trajectories','mppi_noise_trigger':'mppi_noised_trajectories','mppi_critic_scoring':'mppi_optimize','mppi_control_sequence_update':'mppi_optimize','mppi_noise_reset':'initialization / reset context; see recorded parent','amcl_filter_update':'amcl_scan_callback (conditional branch)','amcl_particle_publication':'amcl_scan_callback','velocity_command_stamped_callback':'velocity_command_callback','collision_check_stamped':'collision_check','rate_sleep':'cyclic loop wait; not an independent computational task'}
def fmt(x,d=3):return 'unknown' if x is None else f'{x:.{d}f}'
def table(headers,rows):
 return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join('---' for _ in headers)+' |']+['| '+' | '.join(str(v).replace('|',' / ').replace('\n',' ') for v in row)+' |' for row in rows])+'\n'
def savefig(fig,figdir,name):
 fig.savefig(figdir/(name+'.png'),dpi=160,bbox_inches='tight');fig.savefig(figdir/(name+'.svg'),bbox_inches='tight');plt.close(fig)
def graph(figdir,name,body,rankdir='TB'):
 source='digraph G { graph [rankdir='+rankdir+'; bgcolor="white"; pad="0.25"; nodesep="0.30"; ranksep="0.45"]; node [shape=box; style="rounded,filled"; fillcolor="#edf3fb"; fontname="DejaVu Sans"; fontsize=11]; edge [fontname="DejaVu Sans"; fontsize=9; color="#4d6681"];\n'+body+'\n}'
 (figdir/(name+'.dot')).write_text(source)
 for extension in ['png','svg']:subprocess.run(['dot','-T'+extension,str(figdir/(name+'.dot')),'-o',str(figdir/(name+'.'+extension))],check=True)
def q(s):return json.dumps(str(s))

def observed_phase_graph(example,figdir):
 if not example:return None
 summary,spans,events=example;byid={(s['pid'],s['id']):s for s in spans}
 controls=sorted([s for s in spans if s['kind']=='control_iteration'],key=lambda x:x['wall_start_ns'])
 low,high=summary['graph_window_mono_ns'];controls=[s for s in controls if low<=s['wall_start_ns']<high]
 if len(controls)<2:return None
 selected=controls[1:3] if len(controls)>2 else controls[:2];roots={(s['pid'],s['id']) for s in selected}
 def inside(e,ids):
  key=(e['pid'],e['parent'])
  for _ in range(20):
   if key in ids:return True
   span=byid.get(key)
   if not span:return False
   key=(span['pid'],span['parent'])
  return False
 chosen=[e for e in events if e['event'] in ['state_read','trigger_commit','rmw_publish'] and inside(e,roots) and (e['event']!='rmw_publish' or e['topic']=='/cmd_vel_nav')]
 writer_ids={(e['pid'],e['signature']) for e in chosen if e['event']=='state_read' and e['signature']}
 for key in writer_ids:
  if key in byid:roots.add(key)
 chosen +=[e for e in events if e['event'] in ['state_commit'] and (e['pid'],e['parent']) in writer_ids]
 triggers={(e['pid'],e['parent']) for e in chosen if e['event']=='trigger_commit'}
 noise_reads=[e for e in events if e['event']=='trigger_read' and (e['pid'],e['signature']) in triggers]
 for e in noise_reads:
  if (e['pid'],e['parent']) in byid:roots.add((e['pid'],e['parent']))
 chosen+=noise_reads
 chosen +=[e for e in events if e['event']=='state_commit' and (e['pid'],e['parent']) in roots and e not in chosen]
 message_links=[]
 for publication in list(chosen):
  if publication['event']!='rmw_publish':continue
  takes=[e for e in events if e['event']=='take' and e['topic']==publication['topic'] and e.get('rmw_source_epoch_ns',-1)>0 and e.get('rmw_source_epoch_ns')==publication.get('rmw_source_epoch_ns') and e['signature']==publication['signature']]
  if len(takes)!=1:continue
  take=takes[0]
  callbacks=[s for s in spans if s['kind']=='velocity_command_callback' and s['pid']==take['pid'] and s.get('message_address')==take.get('message_address') and s['signature']==take['signature'] and take['wall_start_ns']<=s['wall_start_ns']<take['wall_start_ns']+50_000_000]
  if len(callbacks)!=1:continue
  callback=callbacks[0];roots.add((callback['pid'],callback['id']))
  if take not in chosen:chosen.append(take)
  message_links.append((publication,take,callback))
 nodes={};ordered=defaultdict(list);origin=min([s['wall_start_ns'] for key,s in byid.items() if key in roots]+[e['wall_start_ns'] for e in chosen])
 body=[];edges=[]
 for key in sorted(roots):
  s=byid.get(key)
  if not s:continue
  for phase,t,c in [('begin',s['wall_start_ns'],s['cpu_start_ns']),('end',s['wall_end_ns'],s['cpu_end_ns'])]:
   name=f's{s["pid"]}_{s["id"]}_{phase}';label=f'{s["kind"]}\n{phase} #{s["id"]}\nt = {(t-origin)/1e6:.3f} ms'
   body.append(name+' [label='+q(label)+'; fillcolor="'+('#e6f3e9' if 'noise' in s['kind'] else '#edf3fb')+'"];');nodes[name]={'event':'scope_'+phase,'scope_id':s['id'],'pid':s['pid'],'tid':s['tid'],'wall_ns':t,'cpu_ns':c};ordered[(s['pid'],s['tid'])].append((t,name,c))
 for e in chosen:
  name=f'e{e["pid"]}_{e["id"]}';label=f'{e["event"]}: {e["kind"]}\nversion {e["aux"]} / event #{e["id"]}\nt = {(e["wall_start_ns"]-origin)/1e6:.3f} ms'
  if e['event'] in ['rmw_publish','take']:label=f'{"publish" if e["event"]=="rmw_publish" else "take"} {e["topic"]}\nevent #{e["id"]}\nt = {(e["wall_start_ns"]-origin)/1e6:.3f} ms'
  body.append(name+' [label='+q(label)+'; fillcolor="#fff2d9"];');nodes[name]={**e,'wall_ns':e['wall_start_ns'],'cpu_ns':e['cpu_start_ns']};ordered[(e['pid'],e['tid'])].append((e['wall_start_ns'],name,e['cpu_start_ns']))
 for rows in ordered.values():
  rows.sort(key=lambda x:(x[0],0 if x[1].endswith('_begin') else 2 if x[1].endswith('_end') else 1,x[1]))
  for a,b in zip(rows,rows[1:]):
   body.append(f'{a[1]} -> {b[1]} [label='+q(f'CPU between markers: {(b[2]-a[2])/1e6:.3f} ms')+'];');edges.append({'from':a[1],'to':b[1],'type':'observed_same_thread_order','cpu_between_ms':(b[2]-a[2])/1e6})
 for read in chosen:
  if read['event'] not in ['state_read','trigger_read'] or not read['signature']:continue
  candidates=[e for e in chosen if e['pid']==read['pid'] and e['parent']==read['signature'] and e['event']==('state_commit' if read['event']=='state_read' else 'trigger_commit') and ((e['kind']==read['kind'] and e['aux']==read['aux']) if read['event']=='state_read' else e['object']==read['object'])]
  if len(candidates)!=1:continue
  writer=candidates[0];a=f'e{writer["pid"]}_{writer["id"]}';b=f'e{read["pid"]}_{read["id"]}'
  if writer['wall_start_ns']>read['wall_start_ns']:continue
  dependency='recorded version dependency' if read['event']=='state_read' else 'recorded trigger dependency'
  body.append(f'{a} -> {b} [color="#c05b2b"; penwidth=2; label='+q(dependency)+'];');edges.append({'from':a,'to':b,'type':'recorded_version_dependency' if read['event']=='state_read' else 'recorded_trigger_dependency','version':read['aux'] if read['event']=='state_read' else None,'resource':read['kind'],'referenced_scope_id':read['signature']})
 for publication,take,callback in message_links:
  a=f'e{publication["pid"]}_{publication["id"]}';b=f'e{take["pid"]}_{take["id"]}';c=f's{callback["pid"]}_{callback["id"]}_begin'
  if publication['wall_start_ns']>take['wall_start_ns']:continue
  body.append(f'{a} -> {b} [color="#7c50a3"; penwidth=2; label="unique DDS source stamp + payload"];');body.append(f'{b} -> {c} [color="#7c50a3"; penwidth=2; label="unique observed input match"];')
  edges.append({'from':a,'to':b,'type':'matched_message_dependency','topic':take['topic'],'rmw_source_epoch_ns':take['rmw_source_epoch_ns'],'payload_signature':take['signature']})
  edges.append({'from':b,'to':c,'type':'matched_callback_input','message_address':take['message_address'],'selection_window_ns':50_000_000})
 # Validate the finite extracted event graph with Kahn's algorithm.
 degrees={k:0 for k in nodes};adj=defaultdict(list)
 for e in edges:degrees[e['to']]+=1;adj[e['from']].append(e['to'])
 queue=[k for k,v in degrees.items() if not v];visited=[]
 while queue:
  v=queue.pop();visited.append(v)
  for child in adj[v]:
   degrees[child]-=1
   if degrees[child]==0:queue.append(child)
 if len(visited)!=len(nodes):raise ValueError('Extracted causal event graph contains a cycle; do not publish it as a DAG.')
 graph(figdir,'observed-phase-dag','\n'.join(body))
 evidence={'campaign':summary['campaign'],'run':summary['run'],'origin_mono_ns':origin,'nodes':nodes,'edges':edges,'acyclic':True,'scope':'Observed finite phase/event graph. Same-thread order is the observed schedule, not a policy-independent precedence requirement. No unobserved TF-version edges are invented.'}
 (ROOT/'docs/evidence/observed-phase-dag.json').write_text(json.dumps(evidence,indent=2)+'\n');return evidence

def make_figures(data,pooled,example,figdir):
 plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.2})
 graph(figdir,'runtime-architecture',r'''world [label="Gazebo dynamics + sensors\nSIM clock / 3 ms physics step"; fillcolor="#e6f3e9"];
 scan [label="LiDAR source\n5 Hz / 200 ms SIM"];
 odom [label="Odometry source\n30 Hz / 33.333 ms SIM"];
 amcl [label="AMCL scan + TF gate\nA; conditional filter update"];
 tf [label="TF state\nmap -> odom / odom -> base"];
 lm [label="Local grid update\n5 Hz / 200 ms HOST"];
 gm [label="Global grid update\n1 Hz / 1 s HOST"];
 bt [label="BT tick\n100 Hz / 10 ms HOST"];
 plan [label="NavFn planning action\nA; 1 Hz BT limiter"];
 path [label="FollowPath path install\nA; action result/goal"];
 ctrl [label="Control + MPPI\n20 Hz / 50 ms HOST"];
 noise [label="Noise helper thread\nA; trigger/coalescing"];
 input [label="Command input callback\nA; cache command"];
 smooth [label="Velocity timer\n20 Hz / 50 ms HOST"];
 collision [label="Collision callback\nA; latest scan + TF"];
 viz [label="RViz visualization\n/plan / maps / trajectories"; fillcolor="#eeeeee"];
 world -> scan; world -> odom; scan -> amcl; amcl -> tf; odom -> tf;
 scan -> lm; scan -> gm; scan -> collision; tf -> lm; tf -> gm; tf -> plan; tf -> ctrl; tf -> collision;
 odom -> ctrl; odom -> bt; bt -> plan [label="action request"]; gm -> plan [label="shared grid + mutex"];
 plan -> path [label="action path; not /plan"]; path -> ctrl; lm -> ctrl [label="shared grid + mutex"];
 ctrl -> noise [label="trigger"]; noise -> ctrl [label="previous committed noise"];
 ctrl -> input [label="/cmd_vel_nav"]; input -> smooth [label="held sample; no 1:1 guarantee"];
 smooth -> collision [label="/cmd_vel_smoothed"]; collision -> world [label="/cmd_vel via bridge"];
 plan -> viz [label="/plan"]; lm -> viz; gm -> viz; ctrl -> viz;
 ''')
 evidence=observed_phase_graph(example,figdir)
 graph(figdir,'indexed-job-model',r'''scan [label="S_j: scan generation"]; gate [label="TF eligibility gate\nexact readiness not measured"];
 loc [label="A_j: AMCL conditional update"]; map [label="L_m: local map commit\nversion m"];
 copy [label="C_k: read noise version n"]; calc [label="C_k: trajectories + critics\ncontrol-sequence update"];
 trigger [label="C_k: noise-ready commit"]; nextnoise [label="N_(n+1): generation + commit\nseparate thread"];
 pub [label="C_k: publish command"]; copy2 [label="C_(k+1): read committed noise"];
 store [label="U_l: command reception + store"]; tick [label="V_h: timer reads held sample"];
 check [label="X_h: collision check + publish"]; nextscan [label="S_(j+1): future world sample"];
 scan -> gate; gate -> loc; gate -> map; map -> calc;
 copy -> trigger; trigger -> nextnoise; trigger -> calc; copy -> calc; calc -> pub;
 nextnoise -> copy2; pub -> store; store -> tick; tick -> check; check -> nextscan;
 ''')
 keys=[k for k in NAMES if len(pooled.get(k,{}).get('cpu_inclusive_ms',[]))]
 if keys:
  fig,ax=plt.subplots(figsize=(11,5.5));arrays=[np.asarray(pooled[k]['cpu_inclusive_ms']) for k in keys]
  ax.boxplot(arrays,vert=False,labels=[NAMES[k] for k in keys],showfliers=False);ax.set_xscale('log');ax.set_xlabel('Measured inclusive thread CPU demand (ms), log scale');ax.set_title(data['status']+' — main cohort CPU distributions (whiskers exclude displayed outliers)');savefig(fig,figdir,'cpu-distributions')
  fig,ax=plt.subplots(figsize=(10,5));x=np.arange(len(keys));cpu=[data['tasks'][k]['metrics']['cpu_inclusive_ms']['mean'] for k in keys];elapsed=[data['tasks'][k]['metrics']['elapsed_wall_ms']['mean'] for k in keys]
  ax.bar(x-.18,cpu,.36,label='Thread CPU');ax.bar(x+.18,elapsed,.36,label='Elapsed including waiting/preemption');ax.set_yscale('log');ax.set_xticks(x,[NAMES[k] for k in keys],rotation=35,ha='right');ax.set_ylabel('Mean milliseconds, log scale');ax.legend();ax.set_title('CPU demand and elapsed duration are different parameters');savefig(fig,figdir,'cpu-vs-elapsed')
  fig,axes=plt.subplots(2,2,figsize=(11,7))
  for ax,k in zip(axes.flat,['control_iteration','local_costmap_update','planning_request','amcl_scan_callback']):
   vals=np.asarray(pooled.get(k,{}).get('inter_entry_wall_ms',[]));vals=vals[np.isfinite(vals)]
   if len(vals):ax.hist(vals,bins=60,color='#4475a4');ax.set_title(NAMES[k]);ax.set_xlabel('Observed inter-entry interval HOST (ms)');ax.set_ylabel('Samples')
  fig.suptitle('Inter-entry statistics do not by themselves establish a release contract');fig.tight_layout();savefig(fig,figdir,'activation-intervals')
  fig,ax=plt.subplots(figsize=(10,4.7))
  for k in ['control_iteration','planning_request','amcl_scan_callback','mppi_noise_generation']:
   points=[(i+1,t['tasks'][k]['metrics']['cpu_inclusive_ms']['mean']) for i,t in enumerate(data['trials']) if k in t['tasks']]
   if points:ax.plot(*zip(*points),marker='o',label=NAMES[k])
  ax.set_xlabel('Completed main-cohort lap');ax.set_ylabel('Mean measured CPU demand (ms)');ax.set_title('Per-lap means; sample counts are recorded separately');ax.legend();savefig(fig,figdir,'per-run-means')
  fig,axes=plt.subplots(1,2,figsize=(10,4))
  for ax,task,label in zip(axes,['planner','amcl'],['Remaining path segments','Published particle count']):
   points=[p for p in data['feature_points'] if p['task']==task]
   if points:ax.scatter([p['feature'] for p in points],[p['cpu_ms'] for p in points],s=8,alpha=.25)
   ax.set_xlabel(label);ax.set_ylabel('Parent measured CPU demand (ms)');ax.set_title(task.upper()+' — observed workload dependence')
  fig.tight_layout();savefig(fig,figdir,'workload-features')
 # A single actual planning action gives a useful sequential subjob DAG.
 if example:
  summary,spans,_=example;plans=sorted([s for s in spans if s['kind']=='planning_request'],key=lambda r:r['wall_start_ns'])
  if plans:
   p=plans[0];segments=sorted([s for s in spans if s['kind']=='planning_segment' and s['pid']==p['pid'] and s['parent']==p['id']],key=lambda r:r['wall_start_ns'])
   body=['request [label='+q(f'Planning request #{p["id"]}')+'];'];last='request'
   for first in range(0,len(segments),5):
    group=segments[first:first+5];name='seg'+str(first)
    label=f'Sequential segments {first+1}–{first+len(group)}\n'+'\n'.join(f'{first+i+1}: event #{s["id"]}; C={(s["cpu_end_ns"]-s["cpu_start_ns"])/1e6:.3f} ms; {s["aux"]} poses' for i,s in enumerate(group))
    body.append(name+' [label='+q(label)+'];');body.append(last+' -> '+name+';');last=name
   body+=['result [label="Concatenate path / action result"];',last+' -> result;'];graph(figdir,'planning-subjob-dag','\n'.join(body))
 return evidence

def render(data,pooled,example,reportlab_path=None):
 from analyze_task_campaign import PRIMARY
 figdir=ROOT/'docs/figures/task-model';figdir.mkdir(parents=True,exist_ok=True)
 observed=make_figures(data,pooled,example,figdir)
 sections=[]
 def add(title,text):sections.append((title,text))
 add('Scope and evidence',f'''This is the **{data['status']}** characterization generated at {data['generated_iso']}. Authorized simulation cutoff: {data['simulation_cutoff_iso']}. Counts include only sealed attempts at generation time; an active unfinished run is not treated as a completed lap.

The unchanged Monaco-style scenario has one moving vehicle, 19 parked visual edge vehicles, and 20 ordered navigation targets. The parked vehicles currently execute no offloaded tasks. Nav2 performs real perception, planning, control, smoothing and collision checks. No custom scheduling algorithm, mixed-criticality mode or processor emulation is active. The experiment measures the current task set before deciding an abstraction.

There are {data['attempts']} recorded attempts, {data['successful_laps_all_cohorts']} successful laps across all instrumentation cohorts, and {data['failed_or_truncated_attempts']} failed/truncated attempts. The main comparable measurement cohort is **{data['main_measurement_cohort']}**, with {data['successful_analyzed_laps']} successful analyzed laps. Supporting cohorts are reported separately because their probes differ. Each attempt retains its configuration hashes, frozen probe source/binary, outcome, raw spans, message events and host samples.

Notation follows the supplied Buttazzo (2011), Section 2.2 (printed pp. 26–29; PDF pp. 43–46) and Section 4.1 (printed pp. 79–82; PDF pp. 96–99). Local trace measurements, installed-version source and scenario configuration determine this system's facts. The book supplies definitions, not execution-cost values.''')
 add('Classical task and job notation',r'''A task is a stream of jobs. Let $J_{i,k}$ denote the k-th job of task $\tau_i$. Its logical release, execution start and finish are $r_{i,k}$, $s_{i,k}$ and $f_{i,k}$. The relative deadline is $D_i=d_{i,k}-r_{i,k}$ and response time is $R_{i,k}=f_{i,k}-r_{i,k}$. All deadlines remain unassigned here.

For a strictly periodic stream, $r_{i,k}=\phi_i+(k-1)T_i$. Buttazzo denotes its k-th job by $\tau_{i,k}$, and an aperiodic job by $J_i$; this report uses $J_{i,k}$ as a generic indexed job when discussing all streams together. A sporadic stream requires the guaranteed constraint $r_{i,k+1}-r_{i,k}\geq T_i^{\min}$. An aperiodic stream has no such established deterministic arrival constraint. A measured average or observed minimum inter-entry interval does not establish a sporadic guarantee. A subscription can consume data from a periodic sensor and still have variable, delayed or bursty callback activations.

P* below means an intended periodic implementation whose actual release/continuation can experience jitter, overruns, skips, deactivation and state-dependent work. P* is a report annotation, not a new classical task category. A means aperiodic under the currently established contract. No listed message/action stream is promoted to sporadic without a separately verified lower-bound contract.

For empirical abstraction use $\widehat{\tau}_i=(\mathrm{type}_i,\theta_i,\overline{C}_i,T_i^0\ \mathrm{or}\ \overline{I}_i,D_i=\bot,\mathrm{resources}_i,\mathrm{dependencies}_i)$. Here $\theta_i$ is the workload feature vector. The measured mean $\overline{C}_i$ is a descriptive processor-demand parameter; it is not a certified worst-case execution time. $\bot$ denotes a quantity that is not assigned or established. Hard, firm and soft describe consequences of missing a deadline; neither those classes nor mixed-criticality levels are inferred from a task's name or a successful lap.''')
 rows=[]
 for k,definition in PRIMARY.items():
  t=data['tasks'].get(k,{});ms=t.get('metrics',{});c=ms.get('cpu_inclusive_ms',{});interval=ms.get('inter_entry_wall_ms',{})
  rows.append([k,definition[0],definition[1],definition[2],c.get('n',0),fmt(c.get('mean')),fmt(interval.get('mean'))])
 add('Task classification and mean parameters',table(['Measured work unit','Class','Native activation setting','Clock / trigger','Jobs','Mean CPU C (ms)','Mean inter-entry HOST (ms)'],rows)+'''
The empirical inter-entry column is a start-to-start statistic. For P* it describes the observed implementation; the configured period remains available separately. For A it describes arrivals/executions and is not a declared period. The exported CSV also contains CPU standard deviation, min, median, p95, p99, observed max, elapsed duration, observed minimum interval, run counts, and the mean/std of run means. Each run has its own sample counts and distributions.

Classification details: the planning action is issued by the BT; the 1 Hz RateController limits successful replanning behavior but can tick a RUNNING child and can be reset/preempted. It is not a strict independent 1-second periodic task. AMCL's scan callback is gated by TF; heavy particle-filter work is conditional on motion/initialization. The noise helper waits on a condition variable with a boolean readiness flag, so triggers may coalesce. Command callbacks store a held sample for the smoother's timer. Collision checks are activated by smoothed commands and consume cached scan/TF data. Path installation follows action state changes, rather than subscribing to /plan.''')
 add('Native parameters, units and clocks',table(['Parameter / source','Native value','Meaning / measured counterpart'],[
 ['Controller','20 Hz = 0.050 s HOST','Control iteration; full-loop work is also measured'],
 ['Local map','5 Hz = 0.200 s HOST; publish 2 Hz','Compute updates and map publication are separate'],
 ['Global map','1 Hz = 1.000 s HOST','Map update loop'],
 ['Velocity smoothing','20 Hz = 0.050 s HOST','Wall timer; exact expected-call stamps observed'],
 ['BT loop','10 ms = 0.010 s HOST','Tree tick while a navigation mission is active'],
 ['Planning limiter','1 Hz nominal limiter','Not a certified sporadic minimum or strict T'],
 ['expected_planner_frequency','20 Hz','50 ms elapsed warning threshold; not planner release rate'],
 ['LiDAR generation','5 Hz = 0.200 s SIM','Header stamp differences and HOST receipt intervals measured'],
 ['Odometry generation','30 Hz = 1/30 s SIM','Header stamp differences and HOST receipt intervals measured'],
 ['IMU / depth sensors','200 Hz / 5 Hz SIM','IMU bridged but unused by this Nav2 path; depth unbridged here'],
 ['Physics step','0.003 s SIM','Simulation integration quantization, not CPU task period'],
 ['MPPI','2000 samples × 56 steps × 1 iteration','model_dt = 0.050 s is a prediction grid, not measured C or task T'],
 ['AMCL','500–2000 particles; max_beams 120','Actual published particle counts and conditional filter jobs measured'],
 ['Map sizes','Local 60 × 60; global 1009 × 446 cells','0.05 m resolution; scene data, not processor-time units'],
 ['Timeouts','0.3 s controller map wait; 1 s planner map wait','Runtime guards; not assigned task deadlines'],
 ['Velocity / collision staleness','1 s / 1 s; progress allowance 10 s','Runtime checks; not D_i'],
 ])+r'''
Converting frequency to seconds is valid: $T^0=1/f^0$. It does not change the clock domain. HOST denotes monotonic host wall time, CPU denotes accumulated running time on one host thread, SIM denotes Gazebo /clock or a simulation-stamped sensor header. Nav2's scheduling loops here use host time even though use_sim_time is enabled for other operations. A simulator-time interval does not equal the same host-time interval when the real-time factor varies.

The process-level cached /clock values in span rows are the latest delivered clock, not the exact simulator time sampled by that function. Concurrent delivery can yield cache regressions. Original cached values are retained; source header stamps and the independent /clock observer are used for actual source interval analysis. CPU/elapsed values in the report use HOST/CPU clocks and are not silently divided by a single real-time factor.''')
 source_rows=[]
 for topic,ms in data['topics'].items():
  a=ms.get('source_stamp_interval_sim_ms',{});b=ms.get('observer_inter_receipt_wall_ms',{})
  source_rows.append([topic,a.get('n',0),fmt(a.get('mean')),fmt(a.get('std')),fmt(b.get('mean'))])
 add('Source generation and observed message intervals',table(['Topic','Header interval samples','Mean SIM interval (ms)','SIM interval std (ms)','Mean observer HOST receipt interval (ms)'],source_rows)+'''
Only messages with an actual source header or /clock value contribute to the SIM interval statistic. Unstamped Twist commands have no invented header time. Observer receipt is the observer's delivery time and is not the application's release time. Scan scan_time and time_increment are zero in the original simulator messages; those zero metadata fields do not mean the sensor period is zero.

In the initial retained runs, odometry header intervals were 36 ms SIM, despite a native 30 Hz / 33.333 ms setting. This is consistent with a source that rounds its publish opportunity upward to the 3 ms physics grid: ceil(33.333/3) × 3 = 36 ms. That causal explanation is an inference from the setting and measured grid, not a substitute for the original configuration. The exported statistics retain the actual source interval for every run, so the final model can use the measured mean without hiding this discrepancy.''')
 add('Execution-cost boundaries and measurement equations',r'''For a measured span on thread $\ell$, let $u_\ell(w)$ be CLOCK_THREAD_CPUTIME_ID at host monotonic time w. The measurements are $\widehat{c}_{i,k}=u_\ell(w_f)-u_\ell(w_s)$ and $\widehat{\Delta w}_{i,k}=w_f-w_s$. Their residual $\widehat{Q}_{i,k}=\widehat{\Delta w}_{i,k}-\widehat{c}_{i,k}$ combines blocking, preemption and other non-running intervals; it is not a pure blocking bound. E remains reserved for Buttazzo's tardiness, and H for hyperperiod.

Inclusive parent cost contains its synchronous children. Exclusive cost subtracts directly nested child CPU intervals: $\widehat{c}^{\mathrm{excl}}_v=\widehat{c}^{\mathrm{incl}}_v-\sum_{h\in\mathrm{children}(v)}\widehat{c}^{\mathrm{incl}}_h$. Do not add both a parent and its children to a workload total. The noise producer runs on a separate thread; its CPU demand is measured separately, while overlap means its elapsed duration must not simply be added to the control response time.

Two granularities are provided. Kernel/function spans measure the named work units. Whole-loop CPU is measured between the return of Rate::sleep for the previous iteration and the next call to Rate::sleep. It includes iteration work outside the named kernel. The first body without a preceding sleep anchor is omitted from this derived measurement, while its raw span remains. Sleep return is an observed software continuation, not exact OS readiness or an ideal release timestamp.

The exported control/MPPI subtasks include velocity-command computation, noise copying/trigger/reset, noised-trajectory generation, critic scoring and control-sequence update. Planning segments are sequential subjobs within one action. AMCL filter and particle-publication spans are nested conditional subjobs. Stamped command forwarding is a nested call in this configuration, not an additional independent job stream.''')
 cost_rows=[]
 for k in NAMES:
  m=data['tasks'].get(k,{}).get('metrics',{});c=m.get('cpu_inclusive_ms',{});e=m.get('elapsed_wall_ms',{});loop=m.get('full_iteration_cpu_ms',{})
  cost_rows.append([k,fmt(c.get('mean')),fmt(c.get('std')),fmt(c.get('p95')),fmt(c.get('max')),fmt(e.get('mean')),fmt(loop.get('mean'))])
 add('CPU, elapsed and whole-loop statistics',table(['Work unit','Mean C (ms)','C std','C p95','Observed C max','Mean elapsed (ms)','Mean full-loop C (ms)'],cost_rows)+'''
![Measured CPU distributions](figures/task-model/cpu-distributions.png)

![CPU versus elapsed time](figures/task-model/cpu-vs-elapsed.png)

![Activation intervals](figures/task-model/activation-intervals.png)

![Mean cost per run](figures/task-model/per-run-means.png)

![Observed workload dependence](figures/task-model/workload-features.png)''')
 subjob_rows=[]
 for k in sorted(data['tasks']):
  if k in NAMES:continue
  c=data['tasks'][k]['metrics'].get('cpu_inclusive_ms',{})
  subjob_rows.append([k,SUBJOBS.get(k,'Recorded nested phase; see parent IDs'),c.get('n',0),fmt(c.get('mean')),fmt(c.get('std'))])
 add('Subjob classification and measured costs',table(['Measured subjob','Parent / activation context','Samples','Mean inclusive CPU ms','CPU std ms'],subjob_rows)+'''\nThese synchronous subjobs inherit their parent's activation model; they are not additional independent periodic or sporadic task streams. Layered-map costs are pooled here only as a scope inventory; use the distinct local/global parent streams for modeling different grid sizes. Rate::sleep is a waiting interval whose small CPU overhead is measured, not an independent computational job. Exact parent IDs remain in the original trace and derived job-samples.csv. The control/noise branch is asynchronous only for the separately listed producer thread.''')
 with (ROOT/'docs/evidence/task-subjob-table.csv').open('w',newline='') as out:
  writer=csv.writer(out);writer.writerow(['subjob','parent_activation_context','samples','mean_inclusive_CPU_ms','sample_CPU_std_ms']);writer.writerows(subjob_rows)
 add('Release, jitter, communication and locking',r'''The velocity smoother's wall timer exposes expected_call_time and actual_call_time. For an identified steady-clock timer callback, expected_call_time provides an observed timer due point $\rho_k$. Therefore $\widehat W^{\mathrm{due}}_k=s_k-\rho_k$ and $\widehat R^{\mathrm{due}}_k=f_k-\rho_k$ are reported. This due point is not a kernel scheduler-ready event. W denotes due-to-start delay, not Buttazzo's deadline lateness L. The same interpretation is not applied to ROS-clock timers or loop jobs without an exact due stamp.

Fast DDS source_timestamp and received_timestamp expose middleware source-to-receipt delay $\widehat\delta^{\mathrm{DDS}}=t_{\mathrm{received}}-t_{\mathrm{source}}$ on this single host. RMW publish events and take events retain source timestamps and payload signatures; unique matching keys can substantiate selected message dependencies. Duplicate matches remain ambiguous. Middleware receipt, rcl_take, callback start and TF eligibility are different stages. A received scan can wait for transforms before the heavy AMCL callback.

Selected map and MPPI mutex acquisitions record call duration. This acquisition elapsed time includes synchronization and host preemption. It does not isolate a deterministic blocking term B_i or certify a priority-inheritance protocol. Map/noise versions are recorded while the associated mutex is held. The complete raw event stream retains version commits/reads and thread CPU/wall stamps.''')
 transport_rows=[[topic,v['n'],fmt(v['mean']),fmt(v['std']),fmt(v['min']),fmt(v['max'])] for topic,v in data['DDS_source_to_receive_ms'].items()]
 add('Measured middleware delay',table(['Topic','Takes with valid DDS stamps','Mean ms','Std ms','Observed min ms','Observed max ms'],transport_rows)+'\nNegative intervals, if present, remain visible and require clock/timestamp diagnosis; they are not clipped into a fabricated physical delay.')
 timer_rows=[]
 for role,ms in data['steady_timer_due_metrics'].items():
  if role.startswith('velocity_smoothing_tick |'):
   v=ms.get('expected_due_to_callback_start_ms',{});r=ms.get('expected_due_to_callback_finish_ms',{});timer_rows.append([role,v.get('n',0),fmt(v.get('mean')),fmt(v.get('max')),fmt(r.get('mean'))])
 add('Identified steady-timer timing',table(['Callback role','Samples','Mean due-to-start ms','Observed max due-to-start ms','Mean due-to-finish ms'],timer_rows)+r'''
Other timer roles remain in the JSON evidence. Ephemeral TF-wait polling timers are classified as periodic while armed inside an event-driven transaction, rather than a permanent independent periodic task. Bond heartbeat timers are distinct from the smoother's 50 ms work; the identified work-unit series uses its actual enclosing callback.

Using $W_k=s_k-\rho_k$ and $F_k=f_k-\rho_k$, the measured timer equivalents of Buttazzo's jitter definitions are $\widehat{RRJ}=\max_k|W_k-W_{k-1}|$, $\widehat{ARJ}=\max_k W_k-\min_k W_k$, $\widehat{RFJ}=\max_k|F_k-F_{k-1}|$ and $\widehat{AFJ}=\max_k F_k-\min_k F_k$. They are computed within each binding lifetime/run; the JSON retains difference statistics and per-binding ranges. The maximum observed difference is the empirical relative jitter. These are finite-observation timer-due statistics, not guaranteed OS-release jitter bounds.
''')
 add('Framework callback inventory',f'''The main cohort contains {len(data['callbacks'])} identified callback role groups. The separate callback-characterization-table.csv contains every observed role, node, trigger/source, configured timer periods, clock types, binding generation count and mean/std/min/max CPU cost. Subscription callbacks are modeled as A under the current reception contract; services are A; persistent timers are P while enabled. Dynamic TF polling belongs to an event-driven transaction. Action worker spans are measured separately from the framework callback that initially accepts the goal.

Callback objects and timer handles can be destroyed and reused. The full-v2 trace records the binding generation at callback start and joins metadata at that lifetime, avoiding last-pointer-wins attribution. Unresolved callbacks remain explicitly unresolved. Middleware /clock, TF, lifecycle, action-support, visualization and bridge callbacks are infrastructure work; treating only the eleven algorithmic units as the complete host workload would omit this demand.''')
 add('Architecture, finite DAGs and phase precedence',r'''The static runtime architecture contains physical feedback, asynchronous state and a noise producer/consumer cycle. It is not itself a single finite DAG. Unrolling a finite observation horizon gives $G_H=(V_H,E_{\mathrm{program}}\cup E_{\mathrm{data}}\cup E_{\mathrm{state}})$, with job/phase instances and versioned dependencies.

![Runtime architecture; includes feedback](figures/task-model/runtime-architecture.png)

An important observation is that an output can be committed before its enclosing callback finishes. A map mutex can be released before updateMap's trailing footprint work ends. Control can trigger the helper and then continue its own computation. Publishing a command can make it visible before computeAndPublishVelocity returns. A whole callback modeled as an atomic node can therefore impose false full-completion precedence. Use read/compute/commit phases, or explicitly represent early output availability and parallel helper work.

![Indexed structural model](figures/task-model/indexed-job-model.png)

The indexed graph is an explanatory structural template; its scan/TF edges are not claimed as exact measured version matches. The separately extracted observed graph uses actual selected event IDs, timestamps and recorded local-map/noise versions. The extractor verifies acyclicity. Same-thread order records this run's resource ordering; it is not automatically a policy-independent application dependency.

![Observed finite phase/event DAG](figures/task-model/observed-phase-dag.png)

![Actual planning action subjob DAG](figures/task-model/planning-subjob-dag.png)

For a latest-value consumer, a version edge points from the actual recorded commit to the corresponding read. Initial values committed before the horizon act as roots. The MPPI readiness flag may coalesce trigger jobs; a noise generation is not invented for each trigger. /plan is a visualization output; the controller's path dependency uses the planning action result and FollowPath goal/path installation. /amcl_pose is an observation output; navigation obtains localization through TF. Exact TF/cache-read provenance remains unresolved without more instrumentation.

Global-map epochs need an additional qualification. NavFn clears the starting cell before acquiring its map-copy mutex, then copies the grid under that mutex. The probe logs a conservative planning read/write epoch at unlock; this does not observe every byte mutation or prove complete global-grid provenance. Local-map and noise locked versions used in the illustrated control graph have a narrower recorded scope. Source: [NavFn 1.3.13 makePlan and clearRobotCell](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_navfn_planner/src/navfn_planner.cpp).''')
 if observed:add('Observed DAG evidence',f"The extracted example is {observed['campaign']}/{observed['run']}: {len(observed['nodes'])} phase/event vertices and {len(observed['edges'])} edges; acyclicity verified. Full event IDs and raw timestamps are in docs/evidence/observed-phase-dag.json. Its scope and dependency types are recorded alongside the graph.")
 add('Current resources and scheduling', r'''The processor is the user's laptop: Intel Core i7-13620H, 16 WSL-visible logical CPUs, approximately 15 GiB RAM and 4 GiB swap. These are shared host resources, not simulated per-edge CPUs. WSL2, host workload, software rendering, Gazebo, RViz, middleware and instrumentation compete for them. No target A15/A7 costs are assigned in this characterization.

ROS 2 Jazzy uses the installed Nav2 1.3.13 and rclcpp 28.1.22. The components use isolated single-thread callback executors, with separate action workers, costmap update threads and the MPPI noise thread. All observed scheduling policy, nice value and affinity snapshots are exported per measured thread. Normal SCHED_OTHER execution does not implement RMS or EDF job priorities; ROS callback-group/executor order and Linux thread scheduling are separate mechanisms. Configuration numbers called frequency/timeout do not provide fixed job priorities or execution budgets.

The nominal HOST periodic rates have $H^0=\mathrm{lcm}(10,50,200,1000)\ \mathrm{ms}=1000\ \mathrm{ms}$. This is a nominal hyperperiod of selected rates, not repetition of the entire feedback system. The per-run empirical core demand is $\widehat U^{\mathrm{named}}=\sum\widehat{c}^{\mathrm{nonoverlap}}_{i,k}/\Delta w_{\mathrm{mission}}$, summing the named primary work once and the separate noise helper. It excludes omitted infrastructure/Gazebo CPU and is not the classical WCET utilization bound $\sum_i C_i/T_i$ or a schedulability guarantee.''')
 policies=defaultdict(int)
 for t in data['trials']:
  for r in t.get('thread_os_policy',[]):policies[(r['aux'],r['stamp_ns'],r['signature'])]+=1
 add('Observed thread scheduling snapshots',table(['Linux policy code','Nice','Affinity bitmask','Thread snapshots across runs'],[[a,b,hex(c),n] for (a,b,c),n in sorted(policies.items())])+'\nPolicy code 0 denotes SCHED_OTHER. Repeated snapshots belong to distinct run/thread lifetimes; they are not a count of physical CPU cores.')
 add('Known, measured and still unestablished parameters',table(['Parameter','Existing fact / measurement','What it can support'],[
 ['T_i^0 / rate','Known for cyclic loops and source sensors; units/clock explicit','Nominal activation contract; measured intervals retained separately'],
 ['C_i,k / mean C_i','Measured CPU per kernel, subjob and complete loop where anchored','Empirical cost model; workload-feature conditioning'],
 ['Arrival/inter-entry distribution','Measured per task and callback role','Trace replay / descriptive aperiodic arrival model'],
 ['Phase','Observed first entry relative to goal send','Run-specific phase; not exact formal release phase'],
 ['r_i,k','Timer expected-call point available for identified steady timers','Exact observed timer-due analysis only; other readiness remains unknown'],
 ['D_i / d_i,k','Intentionally unassigned','User will choose deadlines later'],
 ['WCET','Observed maximum and quantiles retained; no certified bound','Finite tests cannot prove a worst-case guarantee'],
 ['Sporadic T_min','Observed minimum retained; no general guaranteed bound','Do not substitute average/minimum for a specification'],
 ['Blocking B_i','Mutex acquisition elapsed recorded; no pure/preemption split','Empirical contention description, not schedulability blocking bound'],
 ['J_i / release jitter','Due-to-start for selected steady timers; inter-entry variability elsewhere','Do not label interval variance as guaranteed release jitter'],
 ['Priority','OS policy/nice/affinity and ROS execution structure observed','No per-task fixed-priority assignment active'],
 ['Communication','Actual DDS source/receive delay and message events retained','Current-host transport evidence; no edge network-rate model active'],
 ['State dependencies','Instrumented map/noise version matches plus message event evidence','Finite phase DAG; exact TF/other cache versions not fully covered'],
 ['Hard/firm/soft and mixed criticality','Not specified by this scenario','Requires consequence/assurance requirements, not inferred from runtime'],
 ['CPU cycles','No hardware cycle-counter demand measurement','Use CPU seconds; do not invent cycles from GHz'],
 ['Gazebo plugin/render tasks','Host process/system samples, not all internal per-job spans','Outside the Nav2 work-unit cost inventory'],
 ]))
 add('Classical schedulability assumptions',table(['Buttazzo Section 4.1 assumption','Current system'],[
 ['A1: periodic instances at a constant rate','Nominal loop/timer periods plus event-driven streams; jitter/skip/deactivation matter'],
 ['A2: a common WCET C_i for every instance','Not established; empirical CPU distributions and workload-feature variation are available'],
 ['A3: common relative deadline D_i = T_i','Not adopted; deadlines intentionally deferred'],
 ['A4: independent tasks; no precedence/resource constraints','False: maps, held commands, TF, sequential segments and stateful control'],
 ['A5: no task self-suspension','Not valid generally: mutex, condition-variable, map/TF and action waits'],
 ['A6: release immediately on arrival','Not established: delivery, executor dispatch and TF eligibility are separate stages'],
 ['A7: zero kernel overhead','Not adopted: real OS dispatch, synchronization and probe overhead exist'],
 ])+'''\nThese departures explain why a bare independent (C_i,T_i,D_i) table is insufficient. The measured table, activation contracts, phases, resources and data dependencies together describe the current workload. An empirical DAG simulator can be built from them later; this report does not claim an already valid hard schedulability proof.''')
 add('Repeated-trial protocol and statistics',r'''Every trial restarts the same world and navigation launch, sends the unchanged full ordered mission, records outcome and raw data, then shuts down its owned process groups. Instrumentation source/binary is frozen per cohort. No overlapping campaign simulators are allowed. A safety margin stops simulation before 09:00; after the cutoff only analysis/reporting is allowed.

For observed CPU values $c_{i,k}$, the pooled mean is $\overline{C}_i=(1/N_i)\sum_k c_{i,k}$. Sample standard deviation uses denominator $N_i-1$. Quantiles in the algorithmic task tables use NumPy's default linear percentile convention. The mean of run means is also reported: $\overline{C}_i^{\mathrm{runs}}=(1/M_i)\sum_m\overline{C}_{i,m}$. It weights laps equally and can differ from the pooled mean when job counts differ. Jobs within a lap are dependent, so no independent-sample confidence claim is made.

All failed/truncated attempts remain in the attempt manifest and raw evidence; their partial statistics remain separate from the completed-lap main means. A forced shutdown can leave an unfinished span or buffered tail unrecorded. Missing samples are not imputed. Framework callback groups use streaming moments (n, mean, sample std, min, max); their quantiles are explicitly not computed, but every original callback span remains available for later exact statistics. Supporting instrumentation cohorts are kept separate.''')
 attempt_rows=[]
 for e in data['all_attempts']:attempt_rows.append([e['campaign'],e['path'],'success' if e['passed'] else 'failed/truncated',fmt(e.get('elapsed_wall_s'),2),e.get('stop_reason','')])
 add('Per-attempt ledger',table(['Cohort','Run','Outcome','Attempt wall seconds','Stop reason'],attempt_rows))
 add('Measurement perturbation and completeness', '''The preload probe adds timestamping, hashing, buffer writes, framework hook work and selective lock instrumentation. A basic-probe no-op harness retained three enabled and three disabled batches of 100,000 calls; roughly 1.9 microseconds CPU per enabled call versus 0.6 microseconds disabled. This is a small-call harness for the earlier basic probe, not a certified full-v2 overhead correction. No fixed value is subtracted from real jobs. Full-v2 also traces many middleware/TF callbacks, and shared host load is preserved in host-samples.jsonl. Main-cohort measurements are therefore explicitly instrumented measurements of this host and configuration.

Version tracing covers the instrumented map update/plan and noise paths. Current successful missions report their recovery count; an external map clear/reset would need a separate version boundary. Mutex ownership epochs handle recursive locking once at the outer unlock. Temporary timer/callback pointer reuse is resolved by binding generations in full-v2. Unbound callbacks, malformed rows and forced-shutdown flags remain in each run summary. Data files retain raw nanoseconds and original payload/header metadata for independent reanalysis.''')
 add('Artifact index and reproducibility', '''The editable mathematical report is docs/task-model.en.md; the portable version is docs/task-model.en.pdf. Scientific figures are available as PNG, SVG and editable Graphviz DOT in docs/figures/task-model/. The primary classification/statistics CSV is docs/evidence/task-characterization-table.csv; per-run statistics are task-characterization-per-run.csv; all framework role groups are callback-characterization-table.csv. The complete machine-readable report is task-characterization-summary.json; observed phase-DAG evidence is observed-phase-dag.json.

Original evidence lives under artifacts/task-profiling/<cohort>/run-NNN/: metadata.json, mission.json, raw/events-PID.csv, observer.jsonl, host-samples.jsonl, process snapshots, logs, job-samples.csv, summary.json, callback-binding-details.json and analysis-cache.npz. The derived job CSV contains algorithm/worker spans; every framework callback is preserved in the original raw CSV. Original CSVs were retained; replacement/deletion for compression was not executed.

Analyze sealed runs without launching simulation: `python3 scripts/analyze_task_campaign.py`. After the user cutoff, generate the final report with `python3 scripts/analyze_task_campaign.py --final`. The final flag is rejected before the cutoff. `--no-report` analyzes and exports statistics only. Cache keys include analysis schema and sealed metadata signature. No simulation is started by the analyzer.

The original pre-measurement inventory remains docs/evidence/task-abstraction-inventory.json; its null costs describe that earlier inspection, not the newly measured evidence. Scenario facts are in nav2_params.yaml, racecar.sdf, world.sdf and navigate_through_poses.xml. The installed-version source study is docs/task-abstraction-study.fa.md.

References: Buttazzo, Hard Real-Time Computing Systems, third edition (2011), supplied local PDF. Nav2 1.3.13 source: [controller](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_controller/src/controller_server.cpp), [costmap](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_costmap_2d/src/costmap_2d_ros.cpp), [planner](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_planner/src/planner_server.cpp), [AMCL](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_amcl/src/amcl_node.cpp), [MPPI noise](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_mppi_controller/src/noise_generator.cpp), and [RateController](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_behavior_tree/plugins/decorator/rate_controller.cpp).''')
 md='# '+data['status']+' — Empirical Real-Time Task Characterization\n\n'+ '\n\n'.join('## '+title+'\n\n'+text for title,text in sections)+'\n'
 (ROOT/'docs/task-model.en.md').write_text(md)
 if reportlab_path:sys.path.append(reportlab_path)
 else:sys.path.append('/mnt/c/Users/Asus/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/Lib/site-packages')
 make_pdf(sections,data,figdir)

def make_pdf(sections,data,figdir):
 from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,Image,KeepTogether,PageBreak
 from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
 from reportlab.lib import colors
 from reportlab.lib.enums import TA_LEFT
 from reportlab.pdfbase import pdfmetrics
 from reportlab.pdfbase.ttfonts import TTFont
 import re
 from PIL import Image as PILImage
 for name,file in [('DejaVu','DejaVuSans.ttf'),('DejaVuBold','DejaVuSans-Bold.ttf')]:
  pdfmetrics.registerFont(TTFont(name,'/usr/share/fonts/truetype/dejavu/'+file))
 pdfmetrics.registerFontFamily('DejaVu',normal='DejaVu',bold='DejaVuBold',italic='DejaVu',boldItalic='DejaVuBold')
 styles=getSampleStyleSheet();styles.add(ParagraphStyle(name='BodyRT',fontName='DejaVu',fontSize=8.7,leading=13,autoLeading='max',spaceAfter=7));styles.add(ParagraphStyle(name='SmallRT',fontName='DejaVu',fontSize=6.5,leading=9));styles.add(ParagraphStyle(name='HeadingRT',fontName='DejaVuBold',fontSize=14,leading=19,spaceBefore=13,spaceAfter=9));styles.add(ParagraphStyle(name='TitleRT',fontName='DejaVuBold',fontSize=19,leading=25,spaceAfter=15))
 story=[Paragraph(data['status']+' — Empirical Real-Time Task Characterization',styles['TitleRT']),Paragraph(html.escape(data['generated_iso']),styles['BodyRT'])]
 equation_counter=0
 def para(text):
  # Equations are typeset as actual mathtext images, rather than discarded in PDF conversion.
  nonlocal equation_counter
  pieces=re.split(r'(\$[^$]+\$)',text)
  if len(pieces)==1:
   cleaned=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',r'\1',text);cleaned=html.escape(cleaned);cleaned=re.sub(r'\*\*(.*?)\*\*',r'<b>\1</b>',cleaned);cleaned=re.sub(r'`([^`]+)`',r'\1',cleaned);story.append(Paragraph(cleaned,styles['BodyRT']));return
  # Keep mathematical notation readable in place using ReportLab's inline image support.
  assembled=''
  for part in pieces:
   if part.startswith('$') and part.endswith('$'):
    equation_counter+=1;p=figdir/f'equation-{equation_counter:03d}.png'
    fig=plt.figure(figsize=(8,.6));fig.text(0,0.1,part,fontsize=12);fig.savefig(p,dpi=180,bbox_inches='tight',pad_inches=.02,transparent=False);plt.close(fig)
    with PILImage.open(p) as im:w,h=im.size
    # Preserve glyph size instead of shrinking every tall summation to a fixed height.
    hpt=h*72/180*(9/12);wpt=w*72/180*(9/12)
    if wpt>480:wpt=480;hpt=h/w*wpt
    assembled+=f'<img src="{html.escape(str(p),quote=True)}" width="{wpt:.2f}" height="{hpt:.2f}" valign="middle"/>'
   else:assembled+=html.escape(part).replace('**','').replace('`','')
  story.append(Paragraph(assembled,styles['BodyRT']))
 for title,text in sections:
  story.append(Paragraph(html.escape(title),styles['HeadingRT']))
  blocks=text.split('\n\n')
  for block in blocks:
   if not block.strip():continue
   if block.startswith('| '):
    lines=[line for line in block.splitlines() if line.startswith('|')];rows=[[cell.strip() for cell in line.strip('|').split('|')] for line in lines if not set(line.replace('|','').replace(' ',''))<=set('-')]
    if rows:
     count=len(rows[0]);widths=[500/count]*count
     # Give long work-unit/source labels more room.
     if count>4:widths=[115]+[(500-115)/(count-1)]*(count-1)
     cells=[[Paragraph(html.escape(cell),styles['SmallRT']) for cell in row] for row in rows]
     t=Table(cells,colWidths=widths,repeatRows=1,hAlign='LEFT');t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#edf3fb')),('GRID',(0,0),(-1,-1),.25,colors.HexColor('#ccd5df')),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),4),('RIGHTPADDING',(0,0),(-1,-1),4),('TOPPADDING',(0,0),(-1,-1),4),('BOTTOMPADDING',(0,0),(-1,-1),4)]));story+=[t,Spacer(1,9)]
    continue
   image=re.match(r'!\[([^\]]*)\]\(([^)]+)\)',block)
   if image:
    p=ROOT/'docs'/image.group(2)
    if p.exists():
     with PILImage.open(p) as im:w,h=im.size
     scale=min(500/w,620/h);story+=[Image(str(p),width=w*scale,height=h*scale),Paragraph(html.escape(image.group(1)),styles['SmallRT']),Spacer(1,10)]
    continue
   para(block)
 def footer(canvas,doc):
  canvas.saveState();canvas.setFont('DejaVu',7);canvas.drawString(40,23,data['status']+' | Nav2 1.3.13 | instrumented host measurements');canvas.drawRightString(555,23,str(doc.page));canvas.restoreState()
 pdf=SimpleDocTemplate(str(ROOT/'docs/task-model.en.pdf'),pagesize=(595.28,841.89),rightMargin=42,leftMargin=42,topMargin=42,bottomMargin=38,title='Nav2 empirical real-time task characterization',author='ROS2 project')
 pdf.build(story,onFirstPage=footer,onLaterPages=footer)
