#!/usr/bin/env python3
"""Finalize validated full-course tables, current parameters, recordings and README."""
import argparse,hashlib,json,shutil,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.report_full_course_comparison import report,POLICIES

def run(command,log):
    with log.open('w') as f:subprocess.run(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
def brief_readme_analysis(result):
    available=[row for row in result['summary'] if row['n_runs']]
    if not available:
        return 'No complete balanced comparison block was available at the cutoff.'
    fastest=min(available,key=lambda row:row['completion_sim_s_mean'])
    best_dmr=max(available,key=lambda row:row['deadline_meet_rate_mean'])
    shortest=min(available,key=lambda row:row['average_response_s_mean'])
    notes=[f"{fastest['policy']} completed the course fastest ({fastest['completion_sim_s_mean']:.3f} s), while {best_dmr['policy']} had the highest deadline meet rate ({100*best_dmr['deadline_meet_rate_mean']:.2f}%). {shortest['policy']} had the lowest mean completed-job response ({1000*shortest['average_response_s_mean']:.2f} ms). These are descriptive results from the completed runs."]
    offload=[row for row in result['deadline_floor_diagnostics'] if row['policy']=='offload']
    evaluated=sum(row['evaluated_jobs'] for row in offload)
    impossible=sum(row['upload_only_impossible'] for row in offload)
    if evaluated:
        notes.append(f"For offload, the observed task-upload delay alone exceeded the relative deadline for {impossible:,}/{evaluated:,} evaluated jobs ({100*impossible/evaluated:.2f}%). Deadlines are derived from HI work on a maximum-frequency A15 and exclude communication and cooling. The 1 ms execution lattice adds another timing floor, so low DMR can coexist with successful navigation; overlapping deadline diagnostics are not additive causal counts.")
    notes.append('Task totals and family shares differ across policies. Output availability can change native callback arrivals and closed-loop motion; host CPU variability also affects the later LO/HI budget choice. The reference budgets, deadlines, map and hardware stayed fixed. Per-job traces, per-device A7/A15 cooling and all failed attempts remain available locally.')
    if all(row['n_runs']==1 for row in available):
        notes.append('Only one complete balanced block finished before the 08:00 Tehran cutoff, against three planned blocks. Each policy therefore has N = 1: sample standard deviations are undefined and no SD whiskers are drawn. These data do not establish statistical significance.')
    if any('postcompletion mutex' in note for note in result['interpretation']):
        notes.append('An initial random traversal failed exact output-gate auditing and was excluded. A postcompletion mutex-wait defect was corrected before its accepted rerun. The earlier accepted local and calibration runs passed their exact-gate audits; per-trial source hashes preserve the correction boundary. One successful local recording needed presentation-only recovery, with its original physical and audit evidence unchanged. The report retains the recovery and interruption details.')
    return '\n\n'.join(notes)

def main():
    p=argparse.ArgumentParser();p.add_argument('campaign',type=Path);a=p.parse_args();campaign=a.campaign.resolve();manifest=json.loads((campaign/'manifest.json').read_text())
    if manifest['status']=='running':raise ValueError('Wait until simulation stops before finalizing')
    figures=ROOT/'docs/figures/full-course-comparison';result=report(campaign,figures,manifest['status']!='complete')
    publication=ROOT/'docs/evidence/full-course-parameters';publication.mkdir(parents=True,exist_ok=True)
    for name in ('frozen-task-parameters.json','frozen-task-parameters.csv','calibration-per-run.csv','frozen-input-hashes.json','hardware.json','deadlines.json','shutdown-verification.json','debug-resolution.json','native-clock-validation.json'):
        if (campaign/name).exists():shutil.copy2(campaign/name,publication/name)
    if (campaign/'frozen-task-parameters.json').exists():
        shutil.copy2(campaign/'frozen-task-parameters.json',ROOT/'docs/evidence/dual-budget-parameters.json');shutil.copy2(campaign/'frozen-task-parameters.csv',ROOT/'docs/evidence/dual-budget-parameters.csv');shutil.copy2(campaign/'deadlines.json',ROOT/'config/task_deadlines.json')
    media=campaign/'media';media.mkdir(exist_ok=True);movies={}
    for policy in POLICIES:
        candidates=[r for r in manifest['trials'] if r['phase']=='comparison' and r['policy']==policy and r['status']=='complete' and (campaign/r['trial']/'presentation.json').exists()]
        if not candidates:continue
        row=candidates[0];trial=campaign/row['trial'];metadata=trial/'presentation-recovered.json' if (trial/'presentation-recovered.json').exists() else trial/'presentation.json';presentation=json.loads(metadata.read_text())
        if not presentation.get('completed'):continue
        movies[policy]={}
        for kind,script in [('vehicle','resample_bridge_recording.py'),('rsu','render_nearest_rsu_gif.py')]:
            raw=media/f'{policy}-{kind}-sim-8x.gif';target=ROOT/'docs/media'/f'{policy}-{"full-course" if kind=="vehicle" else "nearest-rsus"}-8x.gif'
            if not raw.exists():run([sys.executable,str(ROOT/'scripts'/script),str(trial),'--output',str(raw)],media/f'{policy}-{kind}-render.log')
            raw_sha=hashlib.sha256(raw.read_bytes()).hexdigest();cache=media/f'{policy}-{kind}-publication.json'
            cached=json.loads(cache.read_text()) if cache.exists() else {}
            current=target.exists() and cached.get('source_sha256')==raw_sha and cached.get('publication_sha256')==hashlib.sha256(target.read_bytes()).hexdigest()
            if not current:
                try:run([sys.executable,str(ROOT/'scripts/compress_publication_gif.py'),str(raw),'--output',str(target),'--speed','1','--width','1440' if kind=='vehicle' else '1280','--fps','20','--colors','96','--dither','none'],media/f'{policy}-{kind}-compression.log')
                except subprocess.CalledProcessError:
                    if raw.stat().st_size>=95_000_000:raise
                    shutil.copy2(raw,target)
                    target.with_suffix('.json').write_text(json.dumps(dict(source_sha256=raw_sha,publication_sha256=raw_sha,publication_bytes=target.stat().st_size,compression='retained already compact rendered GIF',playback_clock='SIM',speed=8),indent=2)+'\n')
                cache.write_text(json.dumps(dict(source_sha256=raw_sha,publication_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),trial=row['trial']),indent=2)+'\n')
            if target.stat().st_size>=95_000_000:raise ValueError('Publication GIF is too large')
            for path in (target.with_suffix('.json'),raw.with_suffix('.json')):
                if path.exists():shutil.copy2(path,publication/f'{policy}-{kind}-{path.stem}.json')
            movies[policy][kind]=target.relative_to(ROOT).as_posix()
    original=(ROOT/'README.md').read_text();prefix=original.split('## Recorded local bridge preview')[0] if '## Recorded local bridge preview' in original else original.split('## Full-course scheduling comparison')[0]
    prefix=prefix.replace('three device-placement baselines','four device-placement baselines').replace('and `random` (blue).','`random` (blue), and minimum-temperature `greedy` (amber-yellow).')
    tail='## Main tools'+original.split('## Main tools',1)[1]
    if '## Run the coupled scheduler' in tail:tail=tail.split('## Run the coupled scheduler',1)[0]+'## Run locally'+tail.split('## Run locally',1)[1]
    lines=[prefix.rstrip(),'','## Full-course scheduling comparison','',f"**{result['status']}** — audited complete traversals in balanced blocks: "+', '.join(f"{p}: {result['counts'].get(p,0)}" for p in POLICIES)+'.','The fixed cutoff limited the comparison to one complete balanced block; the planned three comparison repetitions were not reached.' if result['status']=='PARTIAL' and len(result['balanced_blocks'])==1 else f"All valid full traversals, including supplementary incomplete blocks: {result['all_success_counts']}.",'', f"Completed local calibration laps: {result['calibration_laps_complete']}. The protocol requires three complete laps to freeze pooled mean CPU demand for LO and the observed maximum for HI. Calibration uses sealed measured CPU work at the middle operating point; subsequent comparisons use the fixed dual budgets. The four policies run in balanced local → random → offload → greedy blocks.",'', 'The vehicle has **one A7**, each of **25 RSUs has one A15**, and physics and hardware share **1 ms** steps. Normal execution uses the middle frequency/voltage point. Overrun boosts the entire affected device to maximum until its last active overrun finishes. Children wait for selected parent results and their transfers. A 5 m task-upload radius applies at send time.','', '![Full-course scheduling metrics](docs/figures/full-course-comparison/comparison.png)','','![Task-family percentages, with total job counts](docs/figures/full-course-comparison/task-family-pies.png)','','![Response and deadline distributions](docs/figures/full-course-comparison/response-and-deadlines.png)','','Bars show full-course run means and dots individual runs. Sample-SD whiskers are drawn only when a policy has at least two full runs. Response averages completed jobs; due unfinished jobs count as deadline misses. Cooling entries count whole-device transitions, with separate A7/A15 evidence retained. Different job counts can arise from scheduling changing closed-loop motion, mission duration and native event arrivals.','', '| Policy | Runs | Mean completion (s) | DMR (%) | Mean response (s) |','|---|---:|---:|---:|---:|']
    for row in result['summary']:
        value=lambda key,scale=1:'—' if row[key] is None else f'{row[key]*scale:.4f}'
        lines.append(f"| {row['policy']} | {row['n_runs']} | {value('completion_sim_s_mean')} | {value('deadline_meet_rate_mean',100)} | {value('average_response_s_mean')} |")
    lines+=['',brief_readme_analysis(result),'','[English mathematical report](docs/full-course-comparison.en.md) · [Vector figures](docs/figures/full-course-comparison/full-course-comparison.pdf) · [Per-run results](docs/figures/full-course-comparison/per-run.csv) · [Task shares](docs/figures/full-course-comparison/task-family-shares.csv) · [A7/A15 cooling evidence](docs/figures/full-course-comparison/cooling-by-device.csv) · [Deadline lower bounds](docs/figures/full-course-comparison/deadline-floor-diagnostics.csv) · [Per-family results](docs/figures/full-course-comparison/task-metrics-per-run.csv) · [Calibration statistics](docs/figures/full-course-comparison/calibration-statistics.csv)','','## Full-course vehicle recordings','', '**8× simulation-time playback — the recordings are displayed eight times faster than simulation time.** Real chase/overview camera images are sampled against retained clock pairs. The chase camera remains approximately 0.75 m behind the vehicle. The live Gantt uses 2 s windows.','']
    for policy in POLICIES:
        if policy in movies:lines += [f'### {policy.title()}','',f'![{policy} — full course, 8× simulation-time playback]({movies[policy]["vehicle"]})','']
    lines+=['## Four nearest RSUs for each policy','', 'Each recording shows the four currently nearest RSUs, their identifiers, distances, actual modeled A15 schedules, temperatures and cooling state. Panels follow geometric proximity, including endpoints outside the 5 m task-upload range. **8× simulation-time playback.**','']
    for policy in POLICIES:
        if policy in movies:lines += [f'### {policy.title()} RSUs','',f'![{policy} — four nearest RSUs, 8× simulation-time playback]({movies[policy]["rsu"]})','']
    lines+=['[Bridge semantics](docs/live-bridge.en.md) · [Scheduling and communication interfaces](docs/scheduling-interfaces.en.md) · [Task model](docs/task-execution.en.md)','','The next step is to compare additional scheduling policies and a proposed algorithm on the same circuit, hardware and frozen task parameters.','',tail]
    example='''## Run the coupled scheduler

```bash
source scripts/environment.sh
python3 scripts/build_live_bridge.py
python3 scripts/run_live_bridge.py --placement local --full-course --seconds 1200 --views --output artifacts/live-bridge/local-full-course
```

Choose `local`, `random`, `offload` or `greedy` for placement and a new output directory for each run. The 1200 s limit is a safety cap; a successful full-course mission stops at the finish. Run one simulation at a time. The default parameter and deadline files are the frozen calibration inputs used in this comparison.

'''
    lines[-1]=lines[-1].replace('## Run locally',example+'## Run locally',1)
    (ROOT/'README.md').write_text('\n'.join(lines).rstrip()+'\n')
    (campaign/'publication.json').write_text(json.dumps(dict(status=result['status'],counts=result['counts'],movies=movies),indent=2)+'\n')
    print(json.dumps(dict(status=result['status'],counts=result['counts'],movies=movies),indent=2))
if __name__=='__main__':main()
