#!/usr/bin/env python3
"""Limit only viewer CPU activity while physics and computation remain independent."""
import argparse,json,os,signal,struct,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.live_bridge.broker import runtime_paths
STOP=False

def stop(*_):
    global STOP;STOP=True

def identity(pid):
    try:
        folder=Path('/proc')/str(pid)
        command=folder.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode()
        if 'gz sim -g ' not in command:return None
        return int(folder.joinpath('stat').read_text().split()[21])
    except FileNotFoundError:return None

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--trial',required=True,type=Path);parser.add_argument('--pid',action='append',required=True,type=int);parser.add_argument('--after-sim',type=float,default=60.)
    args=parser.parse_args();trial=args.trial.resolve();targets={p:identity(p) for p in args.pid}
    if any(t is None for t in targets.values()):raise ValueError('Only live Gazebo GUI processes may be throttled')
    for pid in targets:
        command=(Path('/proc')/str(pid)/'cmdline').read_bytes().replace(b'\0',b' ').decode()
        if str(trial/'views') not in command:raise ValueError('Viewer belongs to a different trial')
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    _,clock=runtime_paths(trial);result=dict(viewer_pids=targets,after_active_SIM_s=args.after_sim,active_host_s=.05,paused_host_s=.15,started=False)
    output=trial/'render-throttle.json'
    def save():
        tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2)+'\n');tmp.replace(output)
    def send(sig):
        for pid,start in targets.items():
            if identity(pid)==start:
                try:os.kill(pid,sig)
                except ProcessLookupError:pass
    save()
    try:
        while not STOP and any(identity(p)==s for p,s in targets.items()):
            if not result['started']:
                try:
                    origin=int((trial/'armed').read_text());now=struct.unpack('<Q',clock.read_bytes()[:8])[0]
                except (FileNotFoundError,ValueError,struct.error):time.sleep(.05);continue
                if (now-origin)/1e9<args.after_sim:time.sleep(.02);continue
                result.update(started=True,started_sim_ns=now,started_host_monotonic_ns=time.monotonic_ns());save()
            send(signal.SIGCONT);time.sleep(.05)
            if STOP:break
            send(signal.SIGSTOP);time.sleep(.15)
    finally:
        send(signal.SIGCONT);result['ended_host_monotonic_ns']=time.monotonic_ns();save()
if __name__=='__main__':main()
