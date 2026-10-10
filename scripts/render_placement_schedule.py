#!/usr/bin/env python3
"""Draw a recorded scheduling window, preserving actual endpoint identifiers."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.live_bridge.viewer import LiveGantt

def main():
    p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--start',type=float,default=0.);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    cfg=json.loads((a.trial/'trial-config.json').read_text());hardware=json.loads((a.trial/'hardware-config.json').read_text())
    cfg['thermal']={**hardware['thermal'],**hardware['device_classes']['vehicle']['thermal']};cfg['server_thermal']={**hardware['thermal'],**hardware['device_classes']['server']['thermal']}
    cfg['core_layouts']={k:['A7']*v['lp_cores']+['A15']*v['hp_cores'] for k,v in hardware['device_classes'].items()}
    model=LiveGantt(cfg);t=json.loads((a.trial/'trace.json').read_text());end=t['origin_tick']+round((a.start+cfg['gantt_window_s'])/model.dt)
    with (a.trial/'live-events.jsonl').open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['tick']>end:break
            model.consume(event)
    model.tick=end;a.output.parent.mkdir(parents=True,exist_ok=True);model.image(a.start).save(a.output);print(a.output)
if __name__=='__main__':main()
