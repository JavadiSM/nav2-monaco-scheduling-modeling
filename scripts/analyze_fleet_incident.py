#!/usr/bin/env python3
"""Analyze a recorded fleet attempt without launching or changing the simulation."""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
import re
import zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
COLORS={'red':'#de4058','blue':'#2779cf','white':'#626f80','green':'#1a9956'}


def evidence_hash(path):
    if path.exists():
        with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
    name=str(path.relative_to(ROOT))
    for archive in (ROOT/'.private/media-backups').glob('*.zip'):
        with zipfile.ZipFile(archive) as bundle:
            if name in bundle.namelist():
                with bundle.open(name) as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
    raise FileNotFoundError(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'artifacts/fleet')
    parser.add_argument('--max-bracket-s',type=float,default=1.)
    args=parser.parse_args()
    root=args.input.resolve();out=root/'analysis';out.mkdir(parents=True,exist_ok=True)
    report_dir=ROOT/'docs/evidence';figures=ROOT/'docs/figures/fleet'
    report_dir.mkdir(exist_ok=True);figures.mkdir(parents=True,exist_ok=True)
    files=[root/'mission'/f'{c}.json' for c in COLORS]+[root/'capture/launch.log',root/'capture/capture-evidence.json',root/'capture/dual-view.gif']
    original_hashes={str(p.relative_to(ROOT)):evidence_hash(p) for p in files}
    reports={c:json.loads((root/'mission'/f'{c}.json').read_text()) for c in COLORS}
    epoch=min(r['start_wall_time'] for r in reports.values())
    accepted=min(r['goal_accepted_mono_ns'] for r in reports.values())
    samples={c:np.array([[p['wall_time'],p['x'],p['y']] for p in r['pose_samples']]) for c,r in reports.items()}
    distances={};rows=[];pair_stats={}
    for a,b in itertools.combinations(COLORS,2):
        aa,bb=samples[a],samples[b]
        time=np.arange(max(aa[0,0],bb[0,0]),min(aa[-1,0],bb[-1,0]),.1)
        valid=np.ones(len(time),dtype=bool);positions=[]
        for source in (aa,bb):
            indices=np.clip(np.searchsorted(source[:,0],time,side='right'),1,len(source)-1)
            valid&=(source[indices,0]-source[indices-1,0])<=args.max_bracket_s
            positions.append(np.column_stack([np.interp(time,source[:,0],source[:,k]) for k in (1,2)]))
        distance=np.linalg.norm(positions[0]-positions[1],axis=1)
        selected=np.flatnonzero(valid);minimum=selected[np.argmin(distance[valid])]
        key=a+'-'+b
        stat=dict(minimum_amcl_distance_m=float(distance[minimum]),minimum_elapsed_host_s=float(time[minimum]-epoch),valid_samples=int(valid.sum()),total_samples=len(time),first_below_m={})
        for threshold in (.44,.30,.265,.20):
            close=np.flatnonzero(valid&(distance<threshold))
            stat['first_below_m'][str(threshold)]=float(time[close[0]]-epoch) if len(close) else None
        pair_stats[key]=stat;distances[key]=(time-epoch,np.where(valid,distance,np.nan))
        rows.extend(dict(pair=key,elapsed_host_s=float(t-epoch),amcl_distance_m=float(d),valid_short_brackets=bool(v)) for t,d,v in zip(time,distance,valid))
    with (out/'pairwise-amcl-distances.csv').open('w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    log=(root/'capture/launch.log').read_text();events=[]
    patterns={'control_warning':'Control loop missed','planning_failure':'failed to plan','spin':'Running spin','backup':'Running backup','halt_result_error':'Failed to get result for follow_path'}
    for line in log.splitlines():
        match=re.search(r'\[(179\d+\.\d+)\]',line)
        if not match:continue
        for kind,pattern in patterns.items():
            if pattern in line:
                component=re.search(r'\[(?:INFO|WARN|ERROR)\] \[[^]]+\] \[([^]]+)\]',line)
                events.append(dict(elapsed_host_s=float(match[1])-epoch,kind=kind,component=component[1] if component else '',text=line))
    first_failure=min(e['elapsed_host_s'] for e in events if e['kind']=='planning_failure')
    vehicle_results={c:{**{k:r.get(k) for k in ('passed','ordered_targets_passed','requested_targets','odometry_distance_m','navigation_recoveries','elapsed_wall_seconds','error')},'accept_offset_host_s':(r['goal_accepted_mono_ns']-accepted)/1e9,'accept_sim_s':r['goal_accepted_sim_ns']/1e9,'amcl_samples':len(r['pose_samples'])} for c,r in reports.items()}
    analysis=dict(analysis_only=True,new_simulations_launched=False,behavior_changes_applied=False,
        epoch_host_s=epoch,clock='HOST wall-clock at AMCL callback receipt and ROS log emission',
        interpolation_step_s=.1,max_interpolation_bracket_s=args.max_bracket_s,
        localization_is_ground_truth=False,physical_contact_recorded=False,
        pairwise_statistics=pair_stats,vehicle_results=vehicle_results,
        first_planning_failure_elapsed_host_s=first_failure,
        event_counts={kind:sum(e['kind']==kind for e in events) for kind in patterns},
        accepted_goal_spread_host_s=max(v['accept_offset_host_s'] for v in vehicle_results.values()),
        custom_compute_scheduler_coupled=False,
        missing=['Gazebo contact events','Gazebo ground-truth poses and orientations','raw LaserScan and costmap history','collision monitor state','command publish/take timestamps','per-thread CPU/runtime and OS run-queue trace','synchronized capture-start timestamp'],
        source_sha256=original_hashes)
    (out/'incident-summary.json').write_text(json.dumps(analysis,indent=2)+'\n')
    (report_dir/'fleet-incident.json').write_text(json.dumps(analysis,indent=2)+'\n')
    (out/'events.json').write_text(json.dumps(events,indent=2)+'\n')
    fig,(axis,timeline)=plt.subplots(2,1,figsize=(10,6.7),sharex=True,gridspec_kw={'height_ratios':[2,1]},layout='constrained')
    t,d=distances['white-green'];axis.plot(t,d,color='#268d68',lw=2,label='White–green: interpolated AMCL distance')
    axis.axhline(.44,color='#b46b24',ls='--',label='2 × configured navigation radius (not a contact test)')
    axis.axvline(first_failure,color='#cb4355',ls=':',label='First planner failure')
    stat=pair_stats['white-green'];axis.scatter([stat['minimum_elapsed_host_s']],[stat['minimum_amcl_distance_m']],color='#132b43',zorder=4)
    axis.set(ylabel='Estimated centre separation (m)',ylim=(0,1.5),title='Recorded fleet incident — observations, not causal proof')
    axis.legend(loc='upper right',fontsize=8);axis.grid(alpha=.18)
    labels={'control_warning':0,'planning_failure':1,'spin':2,'backup':3,'halt_result_error':4}
    for kind,y in labels.items():
        selected=[e for e in events if e['kind']==kind]
        timeline.scatter([e['elapsed_host_s'] for e in selected],[y]*len(selected),marker='|',s=75,label=kind)
    timeline.set_yticks(list(labels.values()),['Control-rate warning','Planner failure','Recovery spin','Recovery backup','Halt result error'])
    timeline.set(xlabel='Elapsed HOST time from earliest mission client start (s)',xlim=(0,140));timeline.grid(axis='x',alpha=.18)
    for ax in (axis,timeline):ax.spines[['top','right']].set_visible(False)
    fig.savefig(figures/'incident-timeline.png',dpi=160);plt.close(fig)
    fig,axis=plt.subplots(figsize=(9,5),layout='constrained')
    scene=json.loads((ROOT/'scenarios/monaco/scenario.json').read_text());route=np.array(scene['centreline']);axis.plot(route[:,0],route[:,1],color='#ced5df',lw=10,zorder=0,label='Circuit centreline')
    for c,values in samples.items():
        axis.plot(values[:,1],values[:,2],color=COLORS[c],lw=1.8,label=c.title());axis.scatter(values[0,1],values[0,2],color=COLORS[c],s=25)
    checkpoint=scene['checkpoints'][0];axis.scatter(checkpoint['x'],checkpoint['y'],marker='x',color='black',s=65,label='Checkpoint 1')
    axis.set(xlim=(-17.5,-6.5),ylim=(2,9),xlabel='Map x (m)',ylabel='Map y (m)',title='AMCL callback positions near the starting merge and first checkpoint');axis.set_aspect('equal');axis.legend(fontsize=8);axis.grid(alpha=.18)
    fig.savefig(figures/'incident-trajectories.png',dpi=160);plt.close(fig)
    assert original_hashes=={str(p.relative_to(ROOT)):evidence_hash(p) for p in files},'Analysis changed evidence'
    print(json.dumps({k:analysis[k] for k in ('vehicle_results','first_planning_failure_elapsed_host_s','event_counts','accepted_goal_spread_host_s')},indent=2))
    print('All seven input evidence files preserved byte-for-byte.')


if __name__=='__main__':main()
