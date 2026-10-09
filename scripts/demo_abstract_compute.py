#!/usr/bin/env python3
"""Generate a synthetic CPU/power/cooling demonstration, independent of ROS."""
from pathlib import Path
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.abstract_compute import Job, load_platform
from tools.abstract_compute.thermal_scheduler import ThermalScheduler


def main():
    devices = load_platform()
    runtime = ThermalScheduler(devices)
    jobs = [Job(1, 3600, 0, 1, activity_factor=.5),
            Job(2, 3600, 0, 0, activity_factor=.5),
            Job(3, 80, 1, 0, dvfs_level_id=0, activity_factor=.5)]
    results = runtime.run(jobs, sample_period_s=.01)
    output = ROOT / 'artifacts/abstract-compute'
    output.mkdir(parents=True, exist_ok=True)
    (output/'demo-trace.json').write_text(json.dumps({'jobs':results,'samples':runtime.samples,
        'events':runtime.events,'work_segments':runtime.work_segments},indent=2)+'\n')
    selected = [x for x in runtime.samples if x['device_id']==0]
    time = [x['time_s'] for x in selected]
    cooling = [x['cooling'] for x in selected]
    fig, axes = plt.subplots(4,1,figsize=(12,12),gridspec_kw={'height_ratios':[.9,1.1,1.2,1]})
    for cid, color in [(0,'#1787a1'),(1,'#d45829')]:
        core = devices[0].cores[cid]
        levels=core.core_type.dvfs_levels
        axes[0].plot([x.frequency_mhz/1000 for x in levels],[x.voltage_v for x in levels],'o-',color=color,label=core.core_type.name)
        axes[1].plot(time,[x['temperature_c'][cid] for x in selected],color=color,label=core.core_type.name)
        axes[2].plot(time,[x['power_w'][cid] for x in selected],color=color,lw=1,label=core.core_type.name)
    axes[0].set_xlabel('Frequency (GHz)');axes[0].set_ylabel('Voltage (V)')
    axes[0].set_title('Independent virtual hardware — 5 operating points per core type')
    axes[1].axhline(55.6,ls='--',color='#b82d37',label='Tmax = 55.6 C')
    axes[1].axhline(55.45,ls=':',color='#207e39',label='Tbalance = 55.45 C')
    axes[1].fill_between(time,55.0,55.7,where=cooling,color='#bdbdbd',alpha=.24,label='Entire vehicle cooling')
    axes[1].set_ylabel('Temperature (C)');axes[1].set_xlabel('Virtual time (s)')
    axes[2].set_ylabel('Power (W)');axes[2].set_xlabel('Virtual time (s)')
    rows={(0,0):0,(0,1):1,(1,0):2};colors={1:'#d45829',2:'#1787a1',3:'#36935a'}
    for seg in runtime.work_segments:
        axes[3].broken_barh([(seg['start_s'],seg['end_s']-seg['start_s'])],(rows[seg['device_id'],seg['core_id']]-.25,.5),facecolors=colors[seg['job_id']],linewidth=0)
    axes[3].set_yticks([0,1,2],['Vehicle A7','Vehicle A15','Server A7'])
    axes[3].set_xlabel('Virtual time (s)');axes[3].set_title('Only coloured segments execute work; cooling gaps preserve remaining work',fontsize=10)
    for ax in axes:
        ax.grid(alpha=.2)
    for ax in axes[:3]:ax.legend(fontsize=8,loc='upper right')
    fig.tight_layout();target=ROOT/'docs/figures/abstract-compute';target.mkdir(parents=True,exist_ok=True)
    fig.savefig(target/'hardware-power-cooling.png',dpi=170);fig.savefig(target/'hardware-power-cooling.svg');plt.close(fig)
    summary={'synthetic_demo':True,'ros_connected':False,'task_graph_connected':False,
        'jobs':[{'job_id':x['job_id'],'device_id':x['device_id'],'core_id':x['core_id'],
                 'demand_mcycles':x['demand_mcycles'],'start_s':x['start_s'],'finish_s':x['finish_s'],
                 'active_time_s':x['active_time_s'],'executed_mcycles':x['executed_mcycles']} for x in results],
        'vehicle_cooling_entries':sum(e['device_id']==0 and e['kind']=='cooling_start' for e in runtime.cooling_events),
        'vehicle_cooling_recoveries':sum(e['device_id']==0 and e['kind']=='cooling_end' for e in runtime.cooling_events),
        'cooling_equilibrium_c':{d.device_class:runtime.models[did].validate_idle_recovery().tolist() for did,d in devices.items()},
        'actual_device_core_counts':{d.device_class:len(d.cores) for d in devices.values()}}
    (ROOT/'docs/evidence/abstract-compute-demo.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
