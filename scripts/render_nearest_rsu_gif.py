#!/usr/bin/env python3
"""Replay actual four-nearest-RSU schedules at 8x common simulation time."""
import argparse,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.live_bridge.rsu_viewer import NearestRSUGantt

def render(trial,target):
    trial=Path(trial);target=Path(target);target.parent.mkdir(parents=True,exist_ok=True)
    cfg=json.loads((trial/'trial-config.json').read_text());hardware=json.loads((trial/'hardware-config.json').read_text())
    cfg['thermal']={**hardware['thermal'],**hardware['device_classes']['vehicle'].get('thermal',{})};cfg['server_thermal']={**hardware['thermal'],**hardware['device_classes']['server'].get('thermal',{})}
    cfg['core_layouts']={k:['A7']*v['lp_cores']+['A15']*v['hp_cores'] for k,v in hardware['device_classes'].items()};model=NearestRSUGantt(cfg)
    t=json.loads((trial/'trace.json').read_text());origin=t['origin_tick'];end=round(t['sim_seconds']/t['step_s']);next_frame=origin;frame_step=round(.4/t['step_s']);frames=0
    cmd=['ffmpeg','-y','-hide_banner','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','1280x720','-framerate','20','-i','-','-filter_complex_threads','1','-filter_complex','split[a][b];[a]palettegen=max_colors=96:stats_mode=single[p];[b][p]paletteuse=new=1:dither=none','-loop','0',str(target)]
    with target.with_suffix('.encoding.log').open('w') as log:
        process=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=log)
        def frame(tick):
            nonlocal frames
            previous=model.tick;model.tick=tick;process.stdin.write(model.image().tobytes());model.tick=previous;frames+=1
            low=tick-round(model.window_s/model.dt)
            model.segments=[s for s in model.segments if s[2]>=low]
            model.cooling_spans=[s for s in model.cooling_spans if s[2]>=low]
            keep={s[0] for s in model.segments}|set(model.active)
            model.jobs={j:r for j,r in model.jobs.items() if not r.get('finished') or j in keep}
        with (trial/'live-events.jsonl').open() as stream:
            for line in stream:
                event=json.loads(line)
                while event['tick']>next_frame and next_frame<=end:frame(next_frame);next_frame+=frame_step
                model.consume(event)
        while next_frame<=end:frame(next_frame);next_frame+=frame_step
        frame(end);process.stdin.close()
        if process.wait():raise RuntimeError('GIF encoding failed; inspect encoding log')
    target.with_suffix('.json').write_text(json.dumps(dict(source_trial=str(trial),source='actual broker events',clock='SIM',speed=8,frame_sim_interval_s=.4,frame_playback_interval_s=.05,frames=frames),indent=2)+'\n')
    return target
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();print(render(a.trial,a.output))
