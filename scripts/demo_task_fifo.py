#!/usr/bin/env python3
"""Replay a measured dependency window with assumed WCET on local virtual cores."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tools.abstract_compute import load_platform, load_tasks, instantiate_graph, DependencyFIFOScheduler

SHORT={'control_iteration':'Control','local_costmap_update':'Local map','global_costmap_update':'Global map',
       'velocity_smoothing_tick':'Smooth timer','bt_tick':'BT tick','planning_request':'Planner',
       'amcl_scan_callback':'AMCL scan','mppi_noise_generation':'Noise','velocity_command_callback':'Command',
       'collision_check':'Collision','controller_path_install':'Path install'}

def main():
    tasks=load_tasks(); devices=load_platform(); cores=devices[0].cores
    evidence=ROOT/'docs/evidence'; figures=ROOT/'docs/figures/task-fifo'; figures.mkdir(parents=True,exist_ok=True)
    rows=[]
    for name,t in tasks.items():
        row=dict(task=name,task_id=t.task_id,activation=t.activation,C_reference_s=t.budget_s,
                 C_reference_ms=t.budget_s*1000,T_nominal_s=t.period_s,T_nominal_ms=None if t.period_s is None else t.period_s*1000,
                 activation_rate_hz=t.activation_rate_hz,observed_mean_interval_s=t.observed_interval_s,
                 observed_mean_interval_ms=t.observed_interval_s*1000,work_mcycles=t.work_mcycles,
                 reference_frequency_mhz=t.reference_frequency_mhz,reference_eta=t.reference_eta)
        for kind,cid in [('A7',0),('A15',1)]:
            c=cores[cid]
            row.update({f'{kind}_C_s':t.execution_s(c),f'{kind}_execution_cycles':t.execution_cycles(c),
                        f'{kind}_period_cycles':t.period_cycles(c)})
        rows.append(row)
    with (evidence/'modeled-task-parameters.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,lineterminator='\n',fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    (evidence/'modeled-task-parameters.json').write_text(json.dumps(rows,indent=2)+'\n')
    graph=json.loads((evidence/'extracted-observed-job-dag.json').read_text())
    trace=ROOT/'artifacts/task-profiling/full-v2-20261009/run-001/trace-graph-evidence.json'
    release_config=ROOT/'config/task_fifo_window.json'
    if not release_config.exists():
        trace_data=json.loads(trace.read_text())
        spans={(x['pid'],x['id']):x for x in trace_data['spans']}
        starts={name:spans[(node['pid'],node['scope_id'])]['wall_start_ns'] for name,node in graph['nodes'].items()}
        origin=min(starts.values())
        releases={name:(time-origin)/1e9 for name,time in starts.items()}
        release_config.write_text(json.dumps(dict(source='run-001 recorded callback entry offsets',
            source_trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(), releases_s=releases),indent=2)+'\n')
    release_inputs=json.loads(release_config.read_text())
    releases=release_inputs['releases_s']
    jobs=instantiate_graph(tasks,graph,releases=releases)
    scheduler=DependencyFIFOScheduler(devices)
    result=scheduler.run_graph(jobs,sample_period_s=.001)
    byid={x['job_id']:x for x in result}; names=list(graph['nodes'])
    for x in result:
        node=graph['nodes'][names[x['job_id']]]
        x['label']=tasks[x['task']].task_id+'['+node['index']+']'
        x['source_vertex']=names[x['job_id']]
    summary=dict(policy='ready_FIFO_oldest_idle_core_nonpreemptive',mode='offline_local_virtual_compute',
        robot_motion_coupled=False,source='run-001 selected recorded dependency window',
        release_semantics='Measured callback entry offsets replayed as modeled release inputs; not exact OS readiness.',
        work_reference='1000 MHz eta=1 virtual normalization; not calibrated laptop cycle counts',
        graph_semantics='Explicit selected versions; atomic completion delivery imposed on measured phase edges.',
        task_count=len(tasks),jobs=len(result),edges=len(graph['edges']),
        standalone_vertices=graph['standalone_vertices'],makespan_s=max(x['finish_s'] for x in result),
        cooling_events=scheduler.cooling_events,result=result,events=scheduler.events,
        source_trace_sha256=release_inputs['source_trace_sha256'])
    (evidence/'task-fifo-demo.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (evidence/'task-fifo-jobs.csv').open('w',newline='') as f:
        fields=['job_id','label','task','release_s','eligible_s','start_s','finish_s','core_id','demand_mcycles','parents']
        w=csv.DictWriter(f,lineterminator='\n',fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(result)
    palette=plt.get_cmap('tab20'); colours={name:palette(i) for i,name in enumerate(tasks)}
    fig,ax=plt.subplots(figsize=(16,5.8));fig.patch.set_facecolor('#f6f8fc');ax.set_facecolor('#fff')
    for x in result:
        y=x['core_id'];start=x['start_s'];end=x['finish_s'];colour=colours[x['task']]
        ax.broken_barh([(start,end-start)],(y-.22,.44),facecolors=[colour],edgecolor='#263449',linewidth=.7)
        ax.plot([x['release_s'],x['release_s']],[y-.32,y-.25],color=colour,lw=1.2)
        ax.annotate(x['label'],((start+end)/2,y),xytext=(0,27 if x['job_id']%2 else -38),
                    textcoords='offset points',ha='center',fontsize=8,color='#17283b',
                    arrowprops=dict(arrowstyle='-',color=colour,lw=.7))
    for child in result:
        for p in child['parents']:
            parent=byid[p]
            ax.add_patch(FancyArrowPatch((parent['finish_s'],parent['core_id']+.24),
              (child['start_s'],child['core_id']+.24),arrowstyle='-|>',mutation_scale=8,
              connectionstyle='arc3,rad=-.20',color='#35465c',alpha=.5,lw=.8))
    ax.set_yticks([0,1],['Vehicle A7 | 1600 MHz','Vehicle A15 | 2000 MHz']);ax.set_ylim(-.7,1.8)
    ax.set_xlabel('Virtual time (s)');ax.grid(axis='x',alpha=.18);ax.set_axisbelow(True)
    ax.set_title('Local FIFO scheduling — measured arrivals, selected dependencies, Q95 budgets',loc='left',fontweight='bold',pad=24)
    fig.text(.17,.02,'Bars = execution; short ticks = release; arrows = selected producer → consumer. Thermal model enabled. Offline replay; no ROS result gating.',fontsize=9)
    fig.tight_layout(rect=[0,.05,1,1]);fig.savefig(figures/'local-fifo-gantt.png',dpi=180);fig.savefig(figures/'local-fifo-gantt.svg');plt.close(fig)
    fig,ax=plt.subplots(figsize=(13,5.1));fig.patch.set_facecolor('#f6f8fc')
    lower,upper=.062,.096
    visible=[x for x in result if x['start_s']>=lower and x['finish_s']<=upper]
    positions={'velocity_smoothing_tick':(.0640,1.65),'collision_check':(.0690,-.65),
               'control_iteration':(.0838,1.65),'velocity_command_callback':(.0935,1.65),
               'mppi_noise_generation':(.088,-.65)}
    for x in visible:
        start,end,y=x['start_s'],x['finish_s'],x['core_id'];colour=colours[x['task']]
        ax.broken_barh([(start,end-start)],(y-.19,.38),facecolors=[colour],edgecolors='#23334a',linewidth=.9)
        ax.plot([x['release_s'],x['release_s']],[y-.31,y-.24],color=colour,lw=1.5)
        pos=positions[x['task']]
        ax.annotate(x['label']+'  '+SHORT[x['task']],((start+end)/2,y),xytext=pos,
            ha='center',fontsize=10,fontweight='bold',arrowprops=dict(arrowstyle='-',color=colour,lw=1.))
        for parent_id in x['parents']:
            parent=byid[parent_id]
            if parent in visible:
                ax.add_patch(FancyArrowPatch((parent['finish_s'],parent['core_id']+.21),
                    (start,y+.21),arrowstyle='-|>',mutation_scale=12,connectionstyle='arc3,rad=-.2',
                    color='#273b53',lw=1.3))
    ax.set_xlim(lower,upper);ax.set_ylim(-1,2.05)
    ax.set_yticks([0,1],['Vehicle A7 | 1600 MHz','Vehicle A15 | 2000 MHz'])
    ax.set_xlabel('Virtual time (s)');ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
    ax.set_title('FIFO detail | Control → next noise and command',loc='left',fontweight='bold',pad=16)
    fig.text(.18,.025,'Smooth timer and Collision have no selected edge in this window. Map/old-noise parents completed earlier.\nAll jobs use Q95 assumed WCET. Short coloured ticks show arrivals; the noise job waits for Control completion.',fontsize=9)
    fig.tight_layout(rect=[0,.09,1,1]);fig.savefig(figures/'local-fifo-detail.png',dpi=180);fig.savefig(figures/'local-fifo-detail.svg');plt.close(fig)
    # Graphviz lays out only genuine selected edges; isolated representatives stay isolated.
    lines=['digraph G {','graph [rankdir=LR, bgcolor="#f6f8fc", pad="0.35", nodesep="0.35", ranksep="0.6", label="Selected job DAG | atomic modeled completion | all time labels in seconds", labelloc=t, fontsize=19];',
           'node [shape=box, style="rounded,filled", fontname="DejaVu Sans", fontsize=10, fillcolor="white", color="#66758b"];']
    for j in jobs:
        t=tasks[j.task];x=byid[j.job_id];label=f'{x["label"]}  {SHORT[j.task]}\\nCref={t.budget_s:.7g} s'
        if t.period_s is not None:label+=f' | T={t.period_s:g} s'
        else:label+=' | aperiodic'
        label+=f'\\nA7: {t.execution_s(cores[0]):.7g} s | A15: {t.execution_s(cores[1]):.7g} s'
        lines.append(f'n{j.job_id} [label="{label}"];')
        for p in j.parents:lines.append(f'n{p} -> n{j.job_id};')
    lines.append('}');dot=figures/'selected-job-dag.dot';dot.write_text('\n'.join(lines)+'\n')
    import subprocess
    for ext in ('png','svg'):subprocess.run(['dot','-T'+ext,str(dot),'-o',str(dot.with_suffix('.'+ext))],check=True)
    print(json.dumps({k:v for k,v in summary.items() if k not in ('result','events','cooling_events','standalone_vertices')},indent=2))

if __name__=='__main__':main()
