#!/usr/bin/env python3
"""Export compact paper figures and transparent run-level scheduling statistics."""
import argparse,csv,hashlib,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.analyze_placement_trial import write_csv
POLICIES=('local','offload','random');COLOURS={'local':'#c62828','offload':'#238b45','random':'#1565c0'}
METRICS=[('overrun_count','Mean overrun count per run',1.),('deadline_meet_rate','Deadline meet rate (%)',100.),('average_response_s','Average response time (s)',1.),('thermal_violations_per_device','Mean cooling entries per device',1.)]


def export(campaign,out):
    campaign=Path(campaign).resolve();out=Path(out).resolve();out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((campaign/'manifest.json').read_text())
    if manifest['status']!='complete':raise ValueError('Finish and validate all 50 m trials before publishing the comparison')
    rows=[json.loads((campaign/r['trial']/'metrics.json').read_text()) for r in manifest['trials']]
    if any(r['validation_errors'] or not r['target_reached'] or r['completed_outputs_committed']!=r['completed'] for r in rows):raise ValueError('An invalid/incomplete trial cannot enter the comparison')
    write_csv(out/'per-run.csv',rows);summary=[]
    for policy in POLICIES:
        selected=[r for r in rows if r['placement']==policy];row=dict(policy=policy,n_runs=len(selected))
        for key,_,scale in METRICS:
            values=np.array([r[key] for r in selected],dtype=float)
            row[key+'_mean']=float(np.mean(values));row[key+'_sample_sd']=float(np.std(values,ddof=1)) if len(values)>1 else None
        row['completed_jobs']=sum(r['completed'] for r in selected);row['unfinished_jobs']=sum(r['unfinished'] for r in selected)
        row['pooled_response_s']=sum(r['average_response_s']*r['completed'] for r in selected)/row['completed_jobs']
        row['pooled_deadline_meet_rate']=sum(r['deadline_met'] for r in selected)/sum(r['deadline_met']+r['deadline_missed'] for r in selected)
        summary.append(row)
    write_csv(out/'policy-means.csv',summary)
    write_csv(out/'budget-mode-metrics.csv',[dict(policy=r['placement'],budget_mode=mode,**v) for r in rows for mode,v in r['selected_budget_modes'].items()])
    plt.rcParams.update({'font.family':'serif','font.serif':['DejaVu Serif'],'mathtext.fontset':'stix','font.size':9,'axes.labelsize':9,'xtick.labelsize':9,'ytick.labelsize':8,'axes.linewidth':.65,'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','savefig.bbox':'tight'})
    def chart(ax,key,label,scale,panel=None):
        means=[r[key+'_mean']*scale for r in summary];bars=ax.bar(POLICIES,means,color=[COLOURS[k] for k in POLICIES],width=.58,edgecolor='black',linewidth=.45)
        ax.set_ylabel(label);ax.set_axisbelow(True);ax.yaxis.grid(True,color='.87',lw=.45);ax.spines[['top','right']].set_visible(False)
        ax.set_ylim(0,100 if key=='deadline_meet_rate' else max(means)*1.28 or 1)
        for bar,value in zip(bars,means):
            text=f'{value:.4f}' if key=='average_response_s' else f'{value:.2f}' if key in ('deadline_meet_rate','thermal_violations_per_device') else f'{value:.0f}'
            ax.annotate(text,(bar.get_x()+bar.get_width()/2,value),xytext=(0,4),textcoords='offset points',ha='center',va='bottom',fontsize=8)
        deviations=[None if r[key+'_sample_sd'] is None else r[key+'_sample_sd']*scale for r in summary]
        if all(x is not None for x in deviations):ax.errorbar(range(3),means,yerr=deviations,fmt='none',ecolor='black',capsize=3,lw=.7)
        if panel:ax.text(.02,.97,panel,transform=ax.transAxes,va='top')
    figures=[]
    for key,label,scale in METRICS:
        fig,ax=plt.subplots(figsize=(3.5,2.55));chart(ax,key,label,scale);fig.tight_layout(pad=.6)
        for ext in ('png','svg','pdf'):fig.savefig(out/(key+'.'+ext),dpi=300)
        figures.append(fig)
    combined,axes=plt.subplots(2,2,figsize=(7.16,5.1))
    for ax,(key,label,scale),panel in zip(axes.flat,METRICS,['(a)','(b)','(c)','(d)']):chart(ax,key,label,scale,panel)
    combined.tight_layout(pad=.9,w_pad=2,h_pad=1.8)
    for ext in ('png','svg','pdf'):combined.savefig(out/('comparison.'+ext),dpi=300)
    inputs=dict(deadlines=json.loads((campaign/'deadlines.json').read_text()),hardware=json.loads((campaign/'hardware.json').read_text()))
    configs=[json.loads((campaign/r['trial']/'trial-config.json').read_text()) for r in manifest['trials']]
    if any(c['dvfs']!=configs[0]['dvfs'] for c in configs):raise ValueError('Inconsistent DVFS across policies')
    family=[]
    for r in manifest['trials']:
        with (campaign/r['trial']/'task-metrics.csv').open() as stream:
            family.extend(dict(policy=r['policy'],**v) for v in csv.DictReader(stream))
    write_csv(out/'task-family-metrics.csv',family)
    evidence=[]
    for r in manifest['trials']:
        trial=campaign/r['trial'];files={name:hashlib.sha256((trial/name).read_bytes()).hexdigest() for name in ['trace.json','jobs.csv','transfers.csv','clock-pairs.csv','protocol.jsonl','run.json','validation.json']}
        evidence.append(dict(trial=r['trial'],files_sha256=files))
    report=dict(manifest=manifest,summary=summary,inputs=inputs,experiment_settings=dict(intra_device_policy='ready_FIFO_shortest_finish_queue_nonpreemptive',dvfs=configs[0]['dvfs'],device_classes=inputs['hardware']['device_classes'],step_s=.001,upload_radius_m=5.,random_candidates='vehicle and covered RSUs',transfer_types=['task_upload','edge_data'],implicit_result_return_delay=False),evidence=evidence,uncertainty='Sample SD is undefined for one run; no error bars or confidence claim.',response_censoring='Completed jobs only; unfinished jobs retained separately.',deadline_denominator='All completed jobs plus unfinished jobs whose absolute deadlines have passed by the observation cutoff.',thermal_denominator='All 26 modeled devices, including idle devices, in every run.')
    (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    write_csv(out/'deadlines.csv',[dict(task=name,**row) for name,row in inputs['deadlines']['tasks'].items()])
    parameters=json.loads((campaign/manifest['trials'][0]['trial']/'task-parameters.json').read_text())['tasks']
    units=[]
    for name,row in inputs['deadlines']['tasks'].items():
        lo=parameters[name]['C_LO_s'];hi=row['C_HI_reference_s'];rate=row['reference_frequency_mhz']*row['reference_eta'];period=row['T_i_s']
        units.append(dict(task=name,C_LO_reference_s=lo,C_HI_reference_s=hi,W_LO_mcycles=lo*rate,W_HI_mcycles=hi*rate,
                          A7_middle_LO_s=lo*rate/1200,A7_middle_HI_s=hi*rate/1200,A15_middle_LO_s=lo*rate/2700,A15_middle_HI_s=hi*rate/2700,isolated_A7_boost_HI_s=lo*rate/1200+(hi-lo)*rate/1600,isolated_A15_boost_HI_s=lo*rate/2700+(hi-lo)*rate/3600,
                          ideal_A15_max_HI_s=hi*rate/3600,T_i_s=period,release_rate_hz=row['release_rate_hz'],
                          T_A7_middle_cycles=None if period is None else period*1200e6,T_A15_middle_cycles=None if period is None else period*1500e6,D_i_s=row['D_i_s']))
    write_csv(out/'task-timing-units.csv',units)
    # Compare deadlines against ideal service without queueing or thermal pauses.
    names=list(inputs['deadlines']['tasks']);drows=list(inputs['deadlines']['tasks'].values());x=np.arange(len(names))
    timing,ax=plt.subplots(figsize=(7.16,4.5))
    for offset,(label,values,colour) in zip([-.24,0,.24],[('Relative deadline',[r['D_i_s'] for r in drows],'#333333'),('Isolated HI: A15 boost',[r['isolated_A15_boost_HI_s'] for r in units],'#238b45'),('Isolated HI: A7 boost',[r['isolated_A7_boost_HI_s'] for r in units],'#c62828')]):
        ax.bar(x+offset,values,width=.24,label=label,color=colour)
    ax.set_yscale('log');ax.set_ylabel('Ideal duration (s)');ax.set_xticks(x,names,rotation=55,ha='right',fontsize=7);ax.legend(frameon=False,fontsize=8);ax.yaxis.grid(True,which='major',color='.88');ax.set_axisbelow(True);timing.tight_layout()
    for ext in ('png','svg','pdf'):timing.savefig(out/('deadline-service.'+ext),dpi=300)
    parameter_page=plt.figure(figsize=(11.69,8.27));parameter_page.text(.06,.94,'HI deadlines and processor-independent task work',fontsize=15)
    parameter_page.text(.06,.86,r'$W_i^{HI}=C_{i,ref}^{HI} f_{ref}\eta_{ref},\quad D_i=U_i W_i^{HI}/(2000\cdot1.8),\quad d_j=r_j+D_i$',fontsize=15)
    parameter_page.text(.06,.80,'Reference: 1000 MHz, eta = 1.   U in [1.1, 1.3].   Periods remain seconds.   No thermal costs in D.',fontsize=10)
    ax=parameter_page.add_axes([.06,.24,.88,.50]);ax.axis('off')
    cells=[[v['task'].replace('_',' '),f"{v['C_LO_reference_s']:.7f}",f"{v['C_HI_reference_s']:.7f}",f"{v['ideal_A15_max_HI_s']:.7f}",'—' if v['T_i_s'] is None else f"{v['T_i_s']:.3f}",f"{v['D_i_s']:.7f}"] for v in units]
    tab=ax.table(cellText=cells,colLabels=['Task','C ref LO (s)','C ref HI (s)','A15 max HI (s)','T (s)','D (s)'],loc='center',cellLoc='right',colWidths=[.29,.15,.15,.17,.09,.15]);tab.auto_set_font_size(False);tab.set_fontsize(9);tab.scale(1,1.75)
    parameter_page.text(.06,.12,'Execution: A7 1200 MHz / 1.0 V; A15 1500 MHz / 1.2 V. Normal execution uses the middle point; overrun boosts the whole device.\nThe response calculation includes actual transfers, queueing and cooling; D excludes them.\nCompletion uses 0.001 s ticks; D is unrounded. Full conversions are retained in task-timing-units.csv.',fontsize=10,linespacing=1.6)
    parameter_page.savefig(out/'deadline-parameters.png',dpi=180)
    # The PDF packet keeps figure text as vector glyphs and records sample sizes.
    with PdfPages(out/'placement-comparison.pdf') as pdf:
        cover=plt.figure(figsize=(8.27,11.69));cover.text(.08,.94,'Heterogeneous Nav2 placement: 50 m trials',fontsize=16)
        cover.text(.08,.90,'Single A7 vehicle / single A15 per RSU; middle DVFS; device-wide overrun boost',fontsize=11)
        y=.84
        for row in rows:
            lines=[f"{row['placement'].upper()} ({row['colour']}): {row['distance_m']:.3f} m; {row['race_seconds']:.3f} simulation s",f"{row['jobs']} jobs; {row['completed']} completed; {row['unfinished']} unfinished",f"Overruns: {row['overrun_count']}; deadline meet rate: {100*row['deadline_meet_rate']:.3f}%",f"Mean completed-job response: {row['average_response_s']:.6f} s",f"Cooling entries: {row['thermal_cooling_entries_total']} total / 26 = {row['thermal_violations_per_device']:.6f} per device",f"Validation errors: {row['validation_errors']}"]
            cover.text(.08,y,'\n'.join(lines),fontsize=10,va='top',linespacing=1.6);y-=.205
        cover.text(.08,.22,'Pilot runs: internal HOST-based replanning remains; see placement-diagnosis.en.md.',fontsize=9,color='#a32626')
        cover.text(.08,.17,'One run per policy: bars are descriptive; run-to-run variance is unavailable.\nAll deadlines and realized hardware values are shared across policies.\nReal CPU measurements and actual callback arrivals can differ between runs.\nUnfinished jobs are censored in mean response; due unfinished jobs count as misses.\nThe thermal metric counts whole-device cooling entries, not sampled hot ticks.',fontsize=9,va='top',linespacing=1.6)
        cover.savefig(out/'report-cover.png',dpi=160);pdf.savefig(cover);plt.close(cover);pdf.savefig(combined)
        for fig in figures:pdf.savefig(fig);plt.close(fig)
        pdf.savefig(timing);pdf.savefig(parameter_page)
    plt.close(combined);plt.close(timing);plt.close(parameter_page)
    # Ground-truth path samples captured at real callback entries.
    scene=json.loads((ROOT/'scenarios/monaco/scenario.json').read_text());route=np.array(scene['centreline']);fig,ax=plt.subplots(figsize=(7.16,3.8));ax.plot(route[:,0],route[:,1],color='.7',lw=2,label='Circuit centreline')
    for row in rows:
        t=json.loads((campaign/row['trial']/'trace.json').read_text());xy=np.array([j['placement_position_xy'] for j in sorted(t['jobs'],key=lambda j:(j['release_tick'],j['job_id']))]);ax.plot(xy[:,0],xy[:,1],color=COLOURS[row['placement']],lw=1.25,label=row['placement']);ax.plot(*xy[-1],marker='o',color=COLOURS[row['placement']],ms=4)
    ax.scatter([e['x'] for e in scene['servers']],[e['y'] for e in scene['servers']],s=10,marker='s',c='#417da3',label='RSU');ax.set_aspect('equal');ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.legend(frameon=False,ncol=3,fontsize=8);fig.tight_layout()
    for ext in ('png','svg','pdf'):fig.savefig(out/('trajectories.'+ext),dpi=300)
    plt.close(fig)
    table='\n'.join(f"| {r['placement']} | {r['distance_m']:.3f} | {r['race_seconds']:.3f} | {r['jobs']} | {r['overrun_count']} | {100*r['deadline_meet_rate']:.3f} | {r['average_response_s']:.6f} | {r['thermal_violations_per_device']:.4f} |" for r in rows)
    appendix='\n'.join(f"| {name} | {r['C_HI_reference_s']:.7f} | {r['W_HI_mcycles']:.5f} | {r['ideal_A15_max_HI_s']:.7f} | {'—' if r['T_i_s'] is None else format(r['T_i_s'],'.7f')} | {r['uniform_factor']:.7f} | {r['D_i_s']:.7f} |" for name,r in inputs['deadlines']['tasks'].items())
    text=f'''# Placement comparison on the fixed Monaco circuit

The three actual Nav2 runs each stop after at least 50 m of active-interval odometry travel. All runs use one vehicle and the same 25 RSUs, realized hardware parameters, relative deadline table, native navigation configuration and 1 ms physics/hardware lattice. The vehicle colour is red for `local`, green for `offload`, and blue for `random`.

## Observed results

| Policy | Distance (m) | Active time (s) | Jobs | Overruns | Deadline meet (%) | Mean response (s) | Cooling entries/device |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
{table}

![Four scheduling metrics](figures/placement-comparison/comparison.png)

The stopping distance is cumulative odometry travel, not a promise of identical trajectories; completed checkpoint counts are retained in the per-run CSV. These are **pilot runs**: a confirmed internal HOST-clock path in the replanning decorator affects workload timing. See the [diagnostic analysis](placement-diagnosis.en.md) before interpreting an algorithm ranking. There is **one run per policy**. All three traces pass the independent work, clock, placement, dependency-transfer, core-queue and exact output-gate checks. The current unit suite passes 89 tests. Mean overrun count per run is therefore the observed run count; run-to-run variance and confidence intervals cannot be estimated. The average response is the arithmetic mean over completed jobs. [Per-task metrics](figures/placement-comparison/task-family-metrics.csv), [per-run CSV](figures/placement-comparison/per-run.csv), [policy means](figures/placement-comparison/policy-means.csv), [vector PDF packet](figures/placement-comparison/placement-comparison.pdf) and [inputs/hashes](figures/placement-comparison/results.json) retain the denominators and evidence. Unfinished jobs remain in the raw CSV and are excluded from mean response, not replaced by zero.

Deadline meet rate includes completed jobs and unfinished jobs already past their absolute deadlines. Unfinished jobs not yet due are reported separately. The thermal metric is the number of **whole-device cooling entries**, divided by the fixed population of 26 devices. A sustained cooling interval counts once. This is the requested event-count metric, not a count of over-threshold temperature samples.

![Recorded ground-truth trajectories](figures/placement-comparison/trajectories.png)

Recorded scheduling windows: [local](figures/placement-comparison/local-first-window.png), [offload](figures/placement-comparison/offload-first-window.png), [random](figures/placement-comparison/random-first-window.png). Each window shows the vehicle and the latest executing RSU, labelled with its endpoint ID. The four nearest-RSU GIF slots in the README remain separate reserved presentation space.

## Placement and service

`local` always selects the vehicle. `offload` selects the minimum-distance RSU among those within 5 m when the job enters; it falls back to the vehicle if that set is empty. `random` chooses uniformly from the vehicle and RSUs within that same send-time range. Placement uses Gazebo ground-truth XY at the acknowledged physics tick, and remains fixed for the lifetime of the job. Random placement has no affinity to parent devices: a child may independently select a different covered RSU. Coverage is not checked again on reception.

Every device maps newly eligible jobs in FIFO order to the core queue with the smallest predicted finish tick, including the new job's service on that core. The prediction uses remaining service at the configured operating point plus FIFO reservations; actual execution still obeys device-wide cooling. The prediction holds the currently selected rate and does not anticipate future thermal suspensions or boost transitions. Each core serves its reserved jobs in FIFO order without preemption or migration. Running work and every frequency/voltage pair are recorded.

For job $j$ on selected device $m_j$, actual entry $r_j$, staged computation $b_j$, task-upload arrival $u_j$, and parent-edge arrivals $a_{{pj}}$:

$$A_j=\\max\\left(r_j,b_j,u_j,\\max_{{p\\in pred(j)}}a_{{pj}}\\right),\\qquad S_j\\ge A_j.$$

The data transfer $p\\to j$ can be sent only after parent completion and after the child's destination is known. If the child becomes known after the parent finishes, the transmission is sent then; it is never backdated. Each selected parent must have a recorded transfer, including zero-duration same-device transfers. The existing output gate releases the buffered host result at the selected device's modeled finish. This experiment charges the **two requested transfer types** (task upload and parent-edge data); it does not add an implicit third result-return delay.

## Communication delays

Both `task_upload_cost_s(*, distance_m, task_size_bytes=None, **kwargs)` and `edge_data_cost_s(*, distance_m, edge_size_bytes=None, **kwargs)` return finite nonnegative durations in seconds and can be replaced independently through `LiveEngine(upload_cost=..., data_cost=...)`. Size values are reserved extension inputs; the present model uses distance only. There is no modeled link contention, serialization or packet loss.

| Distance $d$ (m) | Delay (s) |
| --- | ---: |
| $0\\le d<1$ | 0 |
| $1\\le d<2$ | 0.003 |
| $2\\le d<3$ | 0.004 |
| $3\\le d\\le5$ | 0.005 |
| $d>5$ | 0.010 |

Same-device communication is zero. RSU-to-RSU parent data may travel farther than 5 m; the 5 m admission condition applies to the vehicle's initial task upload. Every transmission records its send-time endpoints, distance, delay and arrival tick.

## Deadlines and measurements

The vehicle has **one A7** and every RSU has **one A15**. Normal execution uses the middle paired point: A7 **1200 MHz / 1.0 V**, A15 **1500 MHz / 1.2 V**. At an HI-selected job's first LO-work crossing, a **system reaction** boosts every core of its device to its own maximum paired point. The device returns to normal selection after its last active overrun job finishes. This active set survives cooling pauses, and other devices are unaffected. This reaction is identical for all three algorithms; it does not alter placement or FIFO order. With one core per device, the shared FIFO/shortest-finish policy reduces to that device's FIFO queue.

The extracted $C_{{i,ref}}^{{LO}}$ and $C_{{i,ref}}^{{HI}}$ are reference CPU-time samples in seconds, not A7 execution times. The declared normalization uses $f_{{ref}}=1000$ MHz and $\\eta_{{ref}}=1$. Thus normalized work and ideal execution time are

$$W_i^{{HI}}=C_{{i,ref}}^{{HI}}f_{{ref}}\\eta_{{ref}}\\quad[\\text{{Mcycles}}],\\qquad E_i(m,q)=\\frac{{W_i}}{{f_{{m,q}}\\eta_m}}\\quad[\\text{{s}}].$$

The fastest A15 reference is $f_{{A15,max}}=2000$ MHz and $\\eta_{{A15}}=1.8$. Each task-family vertex receives one independently drawn $U_i\\sim\\mathcal{{U}}(1.1,1.3)$; the actual draws are saved and shared across policies. Random device choices use fresh system randomness. Relative and absolute deadlines are

$$D_i=U_i\\frac{{W_i^{{HI}}}}{{2000\\cdot1.8}},\\qquad d_j=r_j+D_i.$$

For an isolated HI-selected job starting at the normal middle point and triggering its own device boost, ideal continuous service (without waiting, communication, cooling or tick rounding) is

$$E_i^{{HI,boost}}=\\frac{{W_i^{{LO}}}}{{f_{{mid}}\\eta}}+\\frac{{W_i^{{HI}}-W_i^{{LO}}}}{{f_{{max}}\\eta}}.$$

Another active overrun on the same multicore device may boost a job sooner. The current devices each have one core. For A15 HI jobs, meeting the assigned deadline depends on the initial LO phase, transfer and waiting. A7 is slower even at its maximum point than the maximum-A15 deadline reference. The deadline remains independent of the overrun reaction.

This calculation excludes **queueing, dependencies, transfers, temperature and cooling**. It always uses HI work, even when a job selects LO work. It is not rounded to a physics tick and is not capped by $T_i$. Completion is observed on the 0.001 s lattice and compared against the unrounded absolute deadline. In particular, path-install deadlines are below one tick: any positive-work job of that family needs at least one execution tick and therefore cannot meet D, even before transfer or waiting. The experiment keeps these requested values unchanged.

$T_i$ is the fixed activation period in seconds. A configured release rate $h_i$ in Hz gives $T_i=1/h_i$; it is unrelated to the processor's clock frequency. For a target core, $T_i f_{{m,q}}10^6$ expresses the same interval in cycles; dividing by that same clock recovers $T_i$. Changing CPU frequency changes service time, not the ROS release period. Event-driven tasks retain no assumed period. The declared normalization is an abstract work model, not measured physical ARM instruction cycles.

![Deadline and ideal HI service](figures/placement-comparison/deadline-service.png)\n\n[Full LO/HI work, per-processor timing and period-cycle conversions](figures/placement-comparison/task-timing-units.csv).

| Task | $C_{{i,ref}}^{{HI}}$ (s) | $W_i^{{HI}}$ (Mcycles) | Ideal max A15 HI (s) | $T_i$ (s) | $U_i$ | $D_i$ (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
{appendix}

A job selects the measured-mean LO work if its sealed actual CPU time is at or below that mean, otherwise the observed-maximum HI work. All task families retain HI criticality. Task arrivals remain actual callback entries. Measured host CPU samples and navigation-triggered arrivals can differ between the three runs, so this experiment describes the closed-loop policies rather than a replay of an identical job stream. Counts, completed-job response and deadline denominators are all retained per task family. [LO/HI deadline metrics](figures/placement-comparison/budget-mode-metrics.csv) retain the selected-budget split.

### Why the earlier aggregate rates were similar

The [earlier experiment](placement-comparison-max-reference.en.md) used different hardware, maximum DVFS and reference-time deadlines. Its aggregate rates (82.117%, 82.053%, 82.485%) hid large per-task differences. Path-install deadline meet rates were 89.964% / 0% / 5.944% for local / offload / random, while control rates were 87.147% / 95.093% / 98.273%. BT job counts were 3,834 / 8,635 / 9,182. Therefore similar weighted aggregate rates did not imply similar per-family timing. The new experiment supersedes those settings, and the earlier evidence remains separately retained.

## Reproduction and retained evidence

```bash
source scripts/environment.sh
python3 scripts/build_live_bridge.py
python3 scripts/run_live_bridge.py --placement offload --distance 50 --seconds 600 --deadline-file config/task_deadlines.json --output artifacts/placement-comparison/<new-trial>
python3 scripts/analyze_placement_trial.py artifacts/placement-comparison/<new-trial>
# Run all three policies with shared saved hardware and deadlines:
python3 scripts/run_placement_comparison.py artifacts/placement-comparison/<new-campaign>
python3 scripts/report_placement_comparison.py artifacts/placement-comparison/<new-campaign>
```

The campaign uses a single explicit hardware realization in its saved `hardware.json`; supply `--hardware-config` with that file to reuse exactly those physical values. Each trial retains immutable inputs, original protocol/clock logs, all jobs, all transfers, per-device cooling counts and validation JSON under ignored `{campaign.relative_to(ROOT)}`. Original characterization CSVs remain intact. The route and navigation map are unchanged. Mission progress files retain historical hard-coded scheduler/communication labels; the immutable `trial-config.json`, executed jobs and transfer records establish the actual settings. Future mission summaries read those labels from the trial configuration. The 50 m observation bound intentionally interrupts the full-course action; its `Interrupted` marker is not a navigation failure. The earlier constant-middle cohort is superseded and excluded from these statistics; its original raw traces remain in a separate campaign directory. The cutoff freezes physics while already released outputs finish committing; incomplete modeled jobs are never forced to finish.
'''
    (ROOT/'docs/placement-comparison.en.md').write_text(text)
    for svg in out.glob('*.svg'):svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
    readme=ROOT/'README.md';content=readme.read_text()
    block='## Placement baselines: 50 m trials\n\nAll three runs reached 50 m of recorded odometry travel, with zero validation errors. The current unit suite passes 89 tests. The vehicle has one A7; each RSU has one A15. Normal execution uses the middle paired frequency/voltage point. Overrun causes a shared system reaction that boosts the entire affected device to maximum until its last active overrun finishes. Deadlines use HI work on an ideal maximum-frequency A15, multiplied by U(1.1,1.3), with no thermal or communication costs. Colours are red for **local**, green for **offload**, and blue for **random**. Each device uses the same FIFO / shortest predicted finish-time queues.\n\n'
    block+='**Pilot results:** internal replanning still uses a HOST clock. Read the [diagnostic analysis](docs/placement-diagnosis.en.md) before interpreting the ranking.\n\n'
    block+='![Placement comparison: overruns, deadlines, response and cooling](docs/figures/placement-comparison/comparison.png)\n\n'
    block+='One run per policy; the bars describe these runs. Deadlines and realized hardware values are shared. Response time averages completed jobs; due unfinished jobs count as deadline misses. Cooling entries are averaged over the same 26 devices.\n\n'
    block+='[English mathematical report](docs/placement-comparison.en.md) · [Vector PDF](docs/figures/placement-comparison/placement-comparison.pdf) · [Per-run statistics](docs/figures/placement-comparison/per-run.csv) · [Recorded trajectories](docs/figures/placement-comparison/trajectories.png)\n\n'
    if '## Placement baselines: 50 m trials' in content:
        start=content.index('## Placement baselines: 50 m trials');stop=content.index('## Main tools',start);content=content[:start]+block+content[stop:]
    else:content=content.replace('## Main tools',block+'## Main tools')
    readme.write_text(content)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);p.add_argument('--output',type=Path,default=ROOT/'docs/figures/placement-comparison');a=p.parse_args();export(a.campaign,a.output)
