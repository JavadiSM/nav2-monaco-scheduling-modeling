#!/usr/bin/env python3
"""Publish audited full-course results with run-level uncertainty and job-family shares."""
import argparse,collections,csv,hashlib,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.analyze_placement_trial import write_csv
from scripts.run_full_course_comparison import has_valid_completion
from tools.live_bridge.viewer import LABELS,COLORS
from tools.live_bridge.engine import TASK_NAMES
POLICIES=('local','random','offload','greedy');PALETTE={'local':'#c62828','random':'#1565c0','offload':'#238b45','greedy':'#e6a000'}
METRICS=[('completion_sim_s','Full-course completion time (s)',1),('overrun_count','Overruns per full course',1),('deadline_meet_rate','Deadline meet rate (%)',100),('average_response_s','Mean completed-job response (s)',1),('thermal_violations_per_device','Cooling entries per device',1),('overruns_per_1000_jobs','Overruns per 1000 entered jobs',1)]

def aggregate_policy(rows,cooling,p):
    group=[r for r in rows if r['placement']==p];row={'policy':p,'n_runs':len(group)}
    for key,_,scale in METRICS:
        values=[r[key] for r in group]
        row[key+'_mean']=float(np.mean(values)) if values else None;row[key+'_sample_sd']=float(np.std(values,ddof=1)) if len(values)>1 else None
    row['entered_jobs']=sum(r['jobs'] for r in group);row['completed_jobs']=sum(r['completed'] for r in group);row['unfinished_jobs']=sum(r['unfinished'] for r in group)
    row['pooled_response_s']=sum(r['average_response_s']*r['completed'] for r in group)/row['completed_jobs'] if row['completed_jobs'] else None
    denominator=sum(r['deadline_met']+r['deadline_missed'] for r in group);row['pooled_DMR']=sum(r['deadline_met'] for r in group)/denominator if denominator else None
    for kind in ('Cortex-A7','Cortex-A15'):
        c=[v for v in cooling if v['policy']==p and kind in v['core_types']]
        row[kind+'_cooling_entries_mean_per_run']=sum(int(v['cooling_entries']) for v in c)/len(group) if group else None
        row[kind+'_cooling_seconds_mean_per_run']=sum(float(v['cooling_duration_s']) for v in c)/len(group) if group else None
    return row


def deadline_floor_diagnostics(jobs,step_s):
    """Count evaluated jobs that fail optimistic bounds before contention/heat."""
    due=[j for j in jobs if j['deadline_status']!='pending_not_due']
    result=dict(evaluated_jobs=len(due),sub_step_deadline_jobs=0,upload_only_impossible=0,
                optimistic_service_impossible=0,upload_plus_service_impossible=0,
                reaction_service_impossible=0,upload_plus_reaction_service_impossible=0,
                sealed_budget_jobs=0)
    for j in due:
        deadline=float(j['D_i_s'])
        if deadline<step_s-1e-12:result['sub_step_deadline_jobs']+=1
        upload=float(j['upload_arrival_s'])-float(j['release_s'])
        result['upload_only_impossible']+=upload>deadline+1e-12
        if j['selected_budget_s'] in ('',None):continue
        result['sealed_budget_jobs']+=1
        work=float(j['selected_budget_s'])*1000.
        rate=1600. if int(j['execution_device'])==0 else 3600.
        service=max(0,math.ceil(work/(rate*step_s)-1e-9))*step_s
        result['optimistic_service_impossible']+=service>deadline+1e-12
        result['upload_plus_service_impossible']+=upload+service>deadline+1e-12
        # Single-core devices cannot inherit a boost from another running job.
        middle=1200. if int(j['execution_device'])==0 else 2700.
        lo_work=float(j.get('C_LO_s',j['selected_budget_s']))*1000.
        initial_work=min(work,lo_work)
        middle_ticks=max(0,math.ceil(initial_work/(middle*step_s)-1e-9))
        remaining=max(0.,work-middle_ticks*middle*step_s)
        maximum_ticks=max(0,math.ceil(remaining/(rate*step_s)-1e-9))
        reaction=(middle_ticks+maximum_ticks)*step_s
        result['reaction_service_impossible']+=reaction>deadline+1e-12
        result['upload_plus_reaction_service_impossible']+=upload+reaction>deadline+1e-12
    return result


def results_interpretation(summary,rows,diagnostics):
    available=[r for r in summary if r['n_runs']]
    if not available:return ['No balanced block has completed; a policy ranking is not available.']
    fastest=min(available,key=lambda r:r['completion_sim_s_mean'])
    best_dmr=max(available,key=lambda r:r['deadline_meet_rate_mean'])
    shortest=min(available,key=lambda r:r['average_response_s_mean'])
    local=next(r for r in available if r['policy']=='local')
    gain=100*(local['completion_sim_s_mean']-fastest['completion_sim_s_mean'])/local['completion_sim_s_mean']
    notes=[f"{fastest['policy']} has the shortest mean full-course time ({fastest['completion_sim_s_mean']:.3f} s; {gain:.2f}% below local). {best_dmr['policy']} has the highest run-mean DMR ({100*best_dmr['deadline_meet_rate_mean']:.3f}%), and {shortest['policy']} has the lowest mean completed-job response ({shortest['average_response_s_mean']:.6f} s). These are descriptive rankings."]
    if all(r['n_runs']==1 for r in available):notes.append('Each policy has one audited full course. The mean equals that single run; run-to-run sample standard deviation is undefined and no SD whiskers are drawn. Three repetitions per policy were planned, so these results remain descriptive.')
    for r in available:
        group=[v for v in rows if v['placement']==r['policy']]
        n=sum(v['jobs'] for v in group);off=sum(v['offloaded_jobs'] for v in group)
        diag=[v for v in diagnostics if v['policy']==r['policy']];due=sum(v['evaluated_jobs'] for v in diag)
        upload_only=sum(v['upload_only_impossible'] for v in diag)
        impossible=sum(v['upload_plus_service_impossible'] for v in diag)
        required=sum(v['upload_plus_reaction_service_impossible'] for v in diag)
        notes.append(f"{r['policy']}: {n:,} entered jobs across {len(group)} full courses; {100*off/n:.2f}% placed on RSUs. A7 cooling entries average {r['Cortex-A7_cooling_entries_mean_per_run']:.2f} per course; all A15 entries together average {r['Cortex-A15_cooling_entries_mean_per_run']:.2f}. Observed upload delay alone exceeds D_i for {upload_only:,}/{due:,} evaluated jobs ({100*upload_only/due:.2f}%). For {impossible:,}/{due:,} evaluated jobs ({100*impossible/due:.2f}%), even maximum-frequency service on the selected device plus its observed upload delay exceeds D_i, before parent waiting, queueing or cooling. Including the shared middle-then-maximum reaction makes this lower-bound count {required:,}/{due:,} ({100*required/due:.2f}%).")
    notes.append('The first comparison replicate of every policy records real camera views and the live Gantt. To limit software-rendering overhead, only the two presentation GUI processes are duty-cycled after 60 active SIM seconds; physics, navigation and the scheduling bridge continue independently. Later replicates run without camera GUIs. Presentation metadata and monotonic clock pairs retain these conditions and support the 8x SIM playback. Host CPU variability can affect the later LO/HI budget choice, while the reference budgets and deadlines remain fixed.')
    notes.append('A 1 ms lattice rounds positive modeled service upward to whole steps. Very short deadlines and upload delays can therefore dominate DMR even when the vehicle finishes successfully. Completion time, task mix, response and DMR answer different questions; similar DMR values alone do not establish equivalent navigation performance. The deadline lower-bound counts overlap actual misses and each other; they are diagnostics, not additive causal attributions.')
    return notes


def export_calibration_statistics(campaign,out):
    """Describe all retained samples without changing frozen scheduling inputs."""
    source=campaign/'calibration-samples.csv';parameters=campaign/'frozen-task-parameters.json'
    if not source.exists() or not parameters.exists():return []
    frozen=json.loads(parameters.read_text())['tasks'];groups=collections.defaultdict(list)
    with source.open() as stream:
        for row in csv.DictReader(stream):groups[row['run'],row['task']].append(row)
    exported=[]
    for task,reference in frozen.items():
        values=[];intervals=[];run_means=[]
        for (run,name),jobs in groups.items():
            if name!=task:continue
            cpu=[float(j['actual_CPU_s']) for j in jobs if j['actual_CPU_s']]
            if cpu:values.extend(cpu);run_means.append(float(np.mean(cpu)))
            intervals.extend(np.diff(sorted(float(j['release_sim_s']) for j in jobs)))
        if not values:raise ValueError('Missing calibration samples: '+task)
        mean=float(np.mean(values));maximum=max(values)
        if not math.isclose(mean,reference['C_LO_s'],rel_tol=1e-12) or maximum!=reference['C_HI_s']:raise ValueError('Frozen budget differs from calibration samples: '+task)
        exported.append(dict(task=task,activation=reference['class'],CPU_sample_count=len(values),runs=len(run_means),
            nominal_period_s=reference['T_nominal_s'],CPU_mean_pooled_s=mean,CPU_mean_of_run_means_s=float(np.mean(run_means)),
            CPU_min_s=min(values),CPU_max_s=maximum,CPU_median_s=float(np.median(values)),
            CPU_p95_linear_s=float(np.quantile(values,.95)),CPU_p99_linear_s=float(np.quantile(values,.99)),
            CPU_sample_sd_s=float(np.std(values,ddof=1)) if len(values)>1 else None,
            observed_SIM_inter_arrival_mean_s=float(np.mean(intervals)) if intervals else None,
            observed_SIM_inter_arrival_min_s=min(intervals) if intervals else None,
            observed_SIM_inter_arrival_max_s=max(intervals) if intervals else None,
            observed_SIM_inter_arrival_sample_sd_s=float(np.std(intervals,ddof=1)) if len(intervals)>1 else None,
            observed_SIM_inverse_mean_interval_hz=1/float(np.mean(intervals)) if intervals and np.mean(intervals)>0 else None))
    write_csv(out/'calibration-statistics.csv',exported)
    return exported


def report(campaign,out,partial=False):
    campaign=Path(campaign);out=Path(out);out.mkdir(parents=True,exist_ok=True);manifest=json.loads((campaign/'manifest.json').read_text())
    calibration_statistics=export_calibration_statistics(campaign,out)
    all_selected=[r for r in manifest['trials'] if r['phase']=='comparison' and r['status']=='complete']
    if any(not has_valid_completion(campaign,r) for r in all_selected):raise ValueError('Completed trial evidence failed its independent completion gate')
    balanced_blocks={r['replicate'] for r in all_selected if {v['policy'] for v in all_selected if v['replicate']==r['replicate']}==set(POLICIES)}
    selected=[r for r in all_selected if r['replicate'] in balanced_blocks]
    counts=collections.Counter(r['policy'] for r in selected)
    if not partial and (manifest['status']!='complete' or any(counts[p]!=3 for p in POLICIES)):raise ValueError('Three audited full courses per policy are required for a final report')
    rows=[];family=collections.Counter();jobs_by_policy=collections.defaultdict(list);cooling=[];evidence=[];diagnostics=[];task_metrics=[]
    for r in selected:
        trial=campaign/r['trial'];m=json.loads((trial/'metrics.json').read_text());v=json.loads((trial/'validation.json').read_text())
        if not v['passed'] or not m['target_reached'] or not m['full_course']:raise ValueError('Invalid full-course result')
        rows.append(dict(replicate=r['replicate'],**m))
        with (trial/'jobs.csv').open() as f:
            current_jobs=list(csv.DictReader(f))
        for j in current_jobs:family[r['policy'],j['task']]+=1;jobs_by_policy[r['policy']].append(j)
        diagnostics.append(dict(policy=r['policy'],replicate=r['replicate'],trial=r['trial'],**deadline_floor_diagnostics(current_jobs,.001)))
        with (trial/'task-metrics.csv').open() as f:task_metrics.extend(dict(policy=r['policy'],replicate=r['replicate'],trial=r['trial'],**j) for j in csv.DictReader(f))
        with (trial/'cooling-by-device.csv').open() as f:cooling.extend(dict(policy=r['policy'],replicate=r['replicate'],trial=r['trial'],**j) for j in csv.DictReader(f))
        names=['trace.json','jobs.csv','transfers.csv','clock-pairs.csv','native-timing.csv','run.json','red-mission.json','validation.json','task-parameters.json','hardware-config.json','source-hashes.json']
        names.extend(name for name in ('presentation.json','presentation-recovered.json','recording-recovery.json','runner-recovery.json','render-throttle.json') if (trial/name).exists())
        evidence.append(dict(trial=r['trial'],files_sha256={name:hashlib.sha256((trial/name).read_bytes()).hexdigest() for name in names}))
    write_csv(out/'all-valid-full-runs.csv',[dict(replicate=r['replicate'],**json.loads((campaign/r['trial']/'metrics.json').read_text())) for r in all_selected])
    write_csv(out/'per-run.csv',rows);write_csv(out/'cooling-by-device.csv',cooling)
    summary=[aggregate_policy(rows,cooling,p) for p in POLICIES]
    write_csv(out/'policy-means.csv',summary)
    write_csv(out/'deadline-floor-diagnostics.csv',diagnostics);write_csv(out/'task-metrics-per-run.csv',task_metrics)
    interpretation=results_interpretation(summary,rows,diagnostics)
    if partial:interpretation.insert(0,f"The campaign closed with {len(balanced_blocks)} complete balanced block(s), against the planned three, at the fixed cutoff {manifest['cutoff_iso']}. Failed and cutoff attempts are preserved and excluded from the primary comparison.")
    shares=[dict(policy=p,task=name,jobs=family[p,name],percentage=100*family[p,name]/sum(family[p,t] for t in TASK_NAMES) if counts[p] else 0) for p in POLICIES for name in TASK_NAMES];write_csv(out/'task-family-shares.csv',shares)
    plt.rcParams.update({'font.family':'serif','font.size':9,'mathtext.fontset':'stix','pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','axes.linewidth':.65,'savefig.bbox':'tight'})
    figures=[]
    def save(fig,name):
        for ext in ('png','svg','pdf'):
            target=out/(name+'.'+ext)
            fig.savefig(target,dpi=300)
            if ext=='svg':target.write_text('\n'.join(line.rstrip() for line in target.read_text().splitlines())+'\n')
        figures.append(fig)
    fig,axes=plt.subplots(3,2,figsize=(7.16,7.0))
    for ax,(key,label,scale) in zip(axes.flat,METRICS):
        means=[r[key+'_mean']*scale if r[key+'_mean'] is not None else 0 for r in summary]
        bars=ax.bar(POLICIES,means,color=[PALETTE[p] for p in POLICIES],edgecolor='black',linewidth=.45,width=.6)
        for n,p in enumerate(POLICIES):
            values=[r[key]*scale for r in rows if r['placement']==p]
            ax.scatter([n]*len(values),values,s=12,marker='o',facecolors='white',edgecolors='black',linewidths=.5,zorder=4)
            sd=summary[n][key+'_sample_sd']
            if sd is not None:ax.errorbar(n,means[n],yerr=sd*scale,fmt='none',ecolor='black',capsize=3,lw=.7)
            ax.annotate('N/A' if summary[n]['n_runs']==0 else f'{means[n]:.4f}' if key=='average_response_s' else f'{means[n]:.2f}',(n,means[n]),xytext=(0,5),textcoords='offset points',ha='center',fontsize=8)
        ax.set_ylabel(label);ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',color='.88',lw=.5);ax.set_axisbelow(True)
        top=max([r[key]*scale for r in rows] or [1]);ax.set_ylim(0,100 if key=='deadline_meet_rate' else max(1e-6,top)*1.3)
    fig.tight_layout(pad=1.2);save(fig,'comparison')
    pie,axes=plt.subplots(1,4,figsize=(7.16,2.8))
    for ax,p in zip(axes,POLICIES):
        values=[family[p,t] for t in TASK_NAMES];total=sum(values)
        if total:ax.pie(values,colors=COLORS,autopct=lambda x:f'{x:.1f}%' if x>=3 else '',pctdistance=.86,textprops={'fontsize':6.5},startangle=90,wedgeprops={'linewidth':.35,'edgecolor':'white'})
        else:ax.text(.5,.5,'No completed full-course runs',ha='center',transform=ax.transAxes,fontsize=8);ax.axis('off')
        ax.set_title(f'{p.upper()}\nN = {total:,}\nComplete laps: {counts[p]}',color=PALETTE[p],fontsize=8.5)
    pie.legend([plt.Line2D([0],[0],marker='s',color=c,lw=0,markersize=7) for c in COLORS],LABELS,loc='lower center',ncol=4,frameon=False,fontsize=7);pie.subplots_adjust(bottom=.3,top=.78,wspace=.18);save(pie,'task-family-pies')
    distribution,axes=plt.subplots(1,2,figsize=(7.16,3.0))
    for p in POLICIES:
        j=jobs_by_policy[p];response=np.sort([float(v['response_s']) for v in j if v['response_s']])
        if len(response):axes[0].plot(response,np.arange(1,len(response)+1)/len(response),label=p,color=PALETTE[p],lw=1.2)
        d=[float(v['response_s'])/float(v['D_i_s']) for v in j if v['response_s'] and float(v['D_i_s'])>0]
        if d:d=np.sort(d);axes[1].plot(d,np.arange(1,len(d)+1)/len(d),label=p,color=PALETTE[p],lw=1.2)
    axes[0].set_xlabel('Response time (s)');axes[1].set_xlabel('Response / relative deadline');axes[1].axvline(1,color='.4',lw=.7,ls='--')
    for ax in axes:ax.set_xscale('log');ax.set_ylabel('Empirical CDF');ax.set_ylim(0,1);ax.grid(color='.88',lw=.4);ax.legend(frameon=False,fontsize=8)
    distribution.tight_layout();save(distribution,'response-and-deadlines')
    digest=json.loads((campaign/'frozen-input-hashes.json').read_text()) if (campaign/'frozen-input-hashes.json').exists() else {}
    attempts=collections.Counter(r['status'] for r in manifest['trials']);calibration_count=sum(r['phase']=='calibration' and r['status']=='complete' for r in manifest['trials'])
    result=dict(calibration_statistics=calibration_statistics,calibration_laps_complete=calibration_count,attempt_outcomes=dict(attempts),interpretation=interpretation,deadline_floor_diagnostics=diagnostics,status='FINAL' if not partial else 'PARTIAL',balanced_blocks=sorted(balanced_blocks),all_success_counts=dict(collections.Counter(r['policy'] for r in all_selected)),counts=dict(counts),summary=summary,manifest=manifest,evidence=evidence,frozen_inputs_sha256=digest,uncertainty='Bars are run means; whiskers are sample SD; dots are individual full courses. No significance claim from this small sample.',response_denominator='Completed jobs; unfinished jobs retained separately.',deadline_denominator='All completed jobs and due unfinished jobs at mission completion.',thermal_denominator='All 26 devices; A7 and A15 breakdowns retained.',calibration=f'{calibration_count} completed local calibration laps using sealed measured CPU work at middle DVFS; three are required to freeze pooled mean LO and observed maximum HI before comparisons.')
    recovered_count=sum(bool(r.get('postprocessing_recovery')) for r in manifest['trials'])
    if recovered_count:interpretation.append(f'{recovered_count} successful full-course recording(s) required recovery of presentation postprocessing. Original run errors, presentation metadata and raw recordings remain preserved; native mission completion and scheduling audits passed. Recovered results use the original successful physical run and unchanged raw traces.')
    manager_recovered=sum(bool(r.get('manager_recovery')) for r in manifest['trials'])
    if manager_recovered:interpretation.append(f'A campaign-manager interruption affected {manager_recovered} retained trial(s). Their physical execution continued independently; process exit codes remain unavailable, while original evidence and subsequent audit results are retained. Only successful full native missions with passing scheduling audits enter the comparison.')
    diagnosis=campaign/'diagnostics/output-commit-diagnosis.json'
    if diagnosis.exists():
        issue=json.loads(diagnosis.read_text());late=sum(v['count'] for v in issue['publications'])
        interpretation.append(f"An initial random traversal reached the finish but failed exact output-gate auditing: {late} publications from {len(issue['publications'])} jobs were one 1 ms step late. A postcompletion mutex wait could temporarily clear the bridge busy flag. The bridge now keeps SIM frozen during output commit; precompletion waits still allow modeled work to progress. That traversal is preserved and excluded. Calibration and the earlier accepted local traversal passed their original exact-gate audits; frozen CPU budgets, deadlines and hardware were not changed. Per-trial source hashes retain this correction boundary.")
    (out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    lines=['# Full-course Nav2 placement comparison','',f"Status: {result['status']}. Successful full courses: "+', '.join(f'{p}={counts[p]}' for p in POLICIES)+'.',f"Complete calibration laps: {calibration_count}. Attempt outcomes across calibration and comparison: {dict(attempts)}. All valid comparison traversals, including supplementary incomplete blocks: {result['all_success_counts']}.",'', 'One A7 in the vehicle; one A15 in each of 25 RSUs. The circuit, navigation algorithms and 1 ms physics/hardware lattice are shared. Normal execution uses the middle paired operating point. An overrun boosts its entire device until the last active overrun completes.','',r'$W_i^{q}=1000 C_{i,ref}^{q},\ q\in\{LO,HI\};\quad D_i=U_i W_i^{HI}/3600,\ U_i\sim U(1.1,1.3);\quad d_j=r_j+D_i.$','', 'Reference CPU demand is measured on the host and converted through an explicit 1000 MHz, eta=1 normalization. Periods and deadlines remain seconds; hardware speed changes service time. Deadlines exclude thermal, queueing, dependency and communication costs. The calibration table remains fixed throughout comparisons; a later job may exceed the empirical HI maximum and is recorded as such without changing it.','', 'Placement is local; uniformly random among vehicle and covered RSUs; nearest covered RSU with local fallback; or greedy minimum current device temperature among vehicle and covered RSUs. Greedy ties use device ID. No placement rule optimizes queue lengths or parent locations. All devices share FIFO / shortest predicted finish-time queue mapping.','', '| Policy | Full courses | Completion (s), mean ± SD | Overruns, mean | DMR (%) | Mean response (s) | Cooling entries/device |','|---|---:|---:|---:|---:|---:|---:|']
    def number(v,d=3):return '—' if v is None else f'{v:.{d}f}'
    for r in summary:lines.append('| '+r['policy']+' | '+str(r['n_runs'])+' | '+number(r['completion_sim_s_mean'])+' ± '+number(r['completion_sim_s_sample_sd'])+' | '+number(r['overrun_count_mean'],1)+' | '+number(None if r['deadline_meet_rate_mean'] is None else 100*r['deadline_meet_rate_mean'])+' | '+number(r['average_response_s_mean'],6)+' | '+number(r['thermal_violations_per_device_mean'])+' |')
    parameters_path=campaign/'frozen-task-parameters.json'
    if parameters_path.exists():
        parameters=json.loads(parameters_path.read_text());deadline_rows=json.loads((campaign/'deadlines.json').read_text())['tasks']
        lines+=['','Frozen parameters (seconds):','', '| Task | Activation | Nominal T | Observed mean inter-arrival | C_LO (host CPU) | C_HI (host CPU) | D |','|---|---|---:|---:|---:|---:|---:|']
        for name,v in parameters['tasks'].items():
            lines.append(f"| {name} | {v['class']} | {number(v['T_nominal_s'],6)} | {number(v['observed_mean_inter_entry_s'],6)} | {number(v['C_LO_s'],6)} | {number(v['C_HI_s'],6)} | {number(deadline_rows[name]['D_i_s'],6)} |")
        lines+=['','P denotes a configured periodic family; A denotes an event/message-driven aperiodic family. Observed inter-arrival is descriptive, including for aperiodic families; it is not a guaranteed sporadic minimum. Nominal periodic T is a native configuration value, not a synthetic release generator. All comparisons create jobs from actual callback arrivals. C_HI is an empirical observed maximum, with later exceedances retained in per-run metrics.']
    lines+=['','\n\n'.join(interpretation),'','![Full-course metrics](figures/full-course-comparison/comparison.png)','','![Entered task-family shares](figures/full-course-comparison/task-family-pies.png)','','![Response distributions](figures/full-course-comparison/response-and-deadlines.png)','','The pie charts pool entered jobs across validated full courses and show their total count above each policy. Counts can change in a closed-loop system: scheduling changes output availability, which changes motion, duration, native action requests and message arrivals. Counts alone do not establish a cause; per-job, mission, clock and transfer evidence are retained for inspection.','', 'Primary comparisons use complete balanced blocks only. Additional valid full traversals from an incomplete block remain in all-valid-full-runs.csv. All failed or partial attempts remain in the campaign manifest and are excluded from full-course means. Completed-job response is censored by the final mission cutoff; due unfinished jobs count as deadline misses. Cooling entries count transitions into whole-device cooling, not hot samples. A7/A15 counts and cooling durations appear separately in policy-means.csv and cooling-by-device.csv.','', 'Raw per-job data, activation times, measured CPU demand, selected budgets, work/frequency intervals, parent transfers, native TF/sensor timestamps, physical/hardware clocks and mission trajectories remain in the local campaign archive. Published tables retain per-run results, shares and evidence hashes.']
    (ROOT/'docs/full-course-comparison.en.md').write_text('\n'.join(lines)+'\n')
    with PdfPages(out/'full-course-comparison.pdf') as pdf:
        for fig in figures:pdf.savefig(fig);plt.close(fig)
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);p.add_argument('--output',type=Path,default=ROOT/'docs/figures/full-course-comparison');p.add_argument('--partial',action='store_true');a=p.parse_args();r=report(a.campaign,a.output,a.partial);print(json.dumps({'status':r['status'],'counts':r['counts']}))
