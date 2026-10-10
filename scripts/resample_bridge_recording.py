#!/usr/bin/env python3
"""Resample real camera recordings onto the retained common simulation clock."""
import argparse,bisect,csv,io,json,subprocess,sys
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.make_gif_speed_variants import delays
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

def resample(trial,target):
    trial=Path(trial);target=Path(target);target.parent.mkdir(parents=True,exist_ok=True)
    metadata=trial/'presentation-recovered.json' if (trial/'presentation-recovered.json').exists() else trial/'presentation.json'
    presentation=json.loads(metadata.read_text());root=Path(__file__).resolve().parents[1]
    source=root/presentation['raw_gif_archive'];capture_start=presentation['capture_host_monotonic_ns']
    t=json.loads((trial/'trace.json').read_text());origin=t['origin_tick']*round(t['step_s']*1e9);finish=round(t['sim_seconds']*1e9)
    host=[];simulation=[]
    with (trial/'clock-pairs.csv').open() as stream:
        for row in csv.DictReader(stream):host.append(int(row['host_monotonic_ns']));simulation.append(int(row['physics_ns']))
    data=source.read_bytes();images=Image.open(io.BytesIO(data));index=[];elapsed=0.;last_sim=-1
    for n,offset in enumerate(delays(data)):
        mono=capture_start+round(elapsed*8e9);i=min(len(host)-1,max(0,bisect.bisect_right(host,mono)-1));sim=simulation[i]
        if sim>=last_sim:index.append((sim,n));last_sim=sim
        elapsed+=int.from_bytes(data[offset:offset+2],'little')/100
    ticks=[r[0] for r in index];cmd=['ffmpeg','-y','-hide_banner','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','1440x540','-framerate','20','-i','-','-filter_complex_threads','1','-filter_complex','split[a][b];[a]palettegen=max_colors=96:stats_mode=single[p];[b][p]paletteuse=new=1:dither=none','-loop','0',str(target)]
    font=ImageFont.truetype(FONT,19);frames=0;previous=-1;picture=None
    with target.with_suffix('.encoding.log').open('w') as log:
        process=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=log)
        for sim in range(origin,finish+1,400_000_000):
            pos=max(0,bisect.bisect_right(ticks,sim)-1);frame=index[pos][1]
            if frame!=previous:images.seek(frame);picture=images.convert('RGB').resize((1440,540));previous=frame
            im=picture.copy();g=ImageDraw.Draw(im);g.rectangle((0,0,1440,35),fill='#101820');g.text((10,7),f"{t['placement_policy'].upper()} | 8x SIMULATION-TIME PLAYBACK | t = {(sim-origin)/1e9:.3f} s",font=font,fill='white');process.stdin.write(im.tobytes());frames+=1
        images.seek(index[-1][1]);im=images.convert('RGB').resize((1440,540));g=ImageDraw.Draw(im);g.rectangle((0,0,1440,35),fill='#101820');g.text((10,7),f"{t['placement_policy'].upper()} | FINISHED | 8x SIMULATION-TIME PLAYBACK | t = {(finish-origin)/1e9:.3f} s",font=font,fill='white');process.stdin.write(im.tobytes());frames+=1
        process.stdin.close()
        if process.wait():raise RuntimeError('Camera GIF encoding failed')
    target.with_suffix('.json').write_text(json.dumps(dict(trial=str(trial),source=str(source),source_frames=len(index),frames=frames,playback_clock='SIM',speed=8,frame_sim_interval_s=.4,frame_playback_interval_s=.05,alignment='Recorded monotonic frame times mapped to acknowledged physics clock pairs; native image sampling is approximate.'),indent=2)+'\n')
    return target
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args();print(resample(a.trial,a.output))
