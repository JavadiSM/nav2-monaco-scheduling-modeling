#!/usr/bin/env python3
"""Show two real Gazebo views and the live scheduler; optionally capture all three."""
import argparse,json,os,re,signal,subprocess,sys,time,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.record_dual_view import windows
from tools.live_bridge.viewer import TITLE

def main():
    p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True);p.add_argument('--record',action='store_true');a=p.parse_args();trial=a.trial.resolve();out=trial/'views';out.mkdir(parents=True,exist_ok=True)
    owned=[];handles=[];recorder=None;result={'scheduler_coupled_to_robot':True,'playback':'6x host recording; simulation clock remains visible','completed':False}
    def start(cmd,label):
        f=(out/(label+'.log')).open('w');handles.append(f);proc=subprocess.Popen(cmd,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);owned.append(proc);return proc
    try:
        original=(ROOT/'config/monaco-gui.config').read_text();overview=re.sub(r'  <plugin filename="CameraTracking".*?</plugin>','',original,flags=re.S)
        (out/'overview.config').write_text(overview);(out/'chase.config').write_text(original)
        viewer=start([sys.executable,'-m','tools.live_bridge.viewer','--trial',str(trial)],'gantt')
        ids={}
        for name in ('overview','chase'):
            before=windows();proc=start(['gz','sim','-g','-v','2','--gui-config',str(out/(name+'.config'))],name);deadline=time.monotonic()+45
            while time.monotonic()<deadline:
                created=windows()-before
                if created:ids[name]=sorted(created)[0];break
                if proc.poll() is not None:raise RuntimeError(name+' view exited')
                time.sleep(.25)
            if name not in ids:raise RuntimeError(name+' view did not open')
        deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            q=subprocess.run(['xdotool','search','--name','^'+TITLE+'$'],capture_output=True,text=True)
            if q.stdout.strip() and (trial/'viewer-ready').exists():ids['gantt']=q.stdout.split()[0];break
            if viewer.poll() is not None:raise RuntimeError('Live Gantt exited')
            time.sleep(.25)
        if 'gantt' not in ids:raise RuntimeError('Live Gantt did not open')
        # Camera services belong only to the chase view; the overview has no tracking plugin.
        time.sleep(3)
        for service,request_type,request in [('/gui/follow','gz.msgs.StringMsg','data: "racecar"'),('/gui/follow/offset','gz.msgs.Vector3d','x: -0.75 y: 0 z: 0.25')]:
            r=subprocess.run(['gz','service','-s',service,'--reqtype',request_type,'--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],capture_output=True,text=True,check=True)
            if 'true' not in r.stdout:raise RuntimeError('Camera did not acknowledge '+service)
        track='track_mode: FOLLOW_LOOK_AT follow_target: {name: "racecar"} track_target: {name: "racecar"} follow_offset: {x: -0.75 y: 0 z: 0.25} track_offset: {x: 0 y: 0 z: 0.10} follow_pgain: 1 track_pgain: 1'
        for _ in range(3):subprocess.run(['gz','topic','-t','/gui/track','-m','gz.msgs.CameraTrack','-p',track],check=True);time.sleep(.25)
        result['window_ids']=ids
        if a.record:
            cfg=json.loads((trial/'trial-config.json').read_text())
            while True:
                ready=set()
                for line in (trial/'protocol.jsonl').read_text().splitlines():
                    try:row=json.loads(line)
                    except json.JSONDecodeError:continue
                    if row['op']==11:ready.add(row['args'][1])
                try:progress=json.loads((trial/'progress.json').read_text())
                except (FileNotFoundError,json.JSONDecodeError):progress={'sim_s':0}
                if len(ready)==cfg['vehicle_count'] and (trial/'nav2-ready').exists() and progress['sim_s']>=cfg['bootstrap_sim_s']:break
                if any(p.poll() is not None for p in owned):raise RuntimeError('A view exited before mission initialization')
                time.sleep(.25)
            cmd=['ffmpeg','-y','-hide_banner','-loglevel','warning']
            for label,size in (('chase','1600x900'),('overview','1600x900'),('gantt','1280x720')):cmd+=['-thread_queue_size','512','-f','x11grab','-framerate','3','-window_id',ids[label],'-video_size',size,'-i',os.environ['DISPLAY']]
            graph="[0:v]setpts=PTS-STARTPTS,scale=640:360[c];[1:v]setpts=PTS-STARTPTS,scale=640:360[o];[2:v]setpts=PTS-STARTPTS[g];[c][o]vstack[left];[left][g]hstack,scale=1440:540,drawtext=text='LIVE bridge | 6x host playback':x=8:y=8:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.8,setpts=PTS/6,split[a][b];[a]palettegen=max_colors=128:stats_mode=single[p];[b][p]paletteuse=new=1:dither=bayer:bayer_scale=4"
            cmd+=['-filter_complex_threads','1','-filter_complex',graph,'-r','12','-fps_mode','cfr','-loop','0',str(out/'live-three-view.gif')]
            recorder=start(cmd,'capture');time.sleep(.5)
            if recorder.poll() is not None:raise RuntimeError('Three-view capture did not start')
        (trial/'views-ready').touch()
        while not (trial/'trace.json').exists():
            if any(p.poll() is not None for p in owned):raise RuntimeError('A presentation process exited early')
            time.sleep(.25)
        time.sleep(.5)
        if recorder:
            recorder.send_signal(signal.SIGINT);recorder.wait(timeout=60)
            if recorder.returncode not in (0,255):raise RuntimeError('Capture failed')
            # Capture has ended; release rendering resources during GIF processing.
            for proc in owned:
                if proc is not recorder and proc.poll() is None:
                    try:os.killpg(proc.pid,signal.SIGINT)
                    except ProcessLookupError:pass
            from PIL import Image
            with Image.open(out/'live-three-view.gif') as im:
                result['size']=im.size;frames=0
                while True:
                    im.load();frames+=1
                    if frames%24==1:im.convert('RGB').save(trial/'three-view-preview.png')
                    try:im.seek(im.tell()+1)
                    except EOFError:break
                result['frames']=frames
                if frames<2:raise RuntimeError('Capture has fewer than two frames')
            target=ROOT/'artifacts/videos/live-fifo-single-6x.gif';target.parent.mkdir(parents=True,exist_ok=True)
            # The capture already has 6x host timing; compression preserves that speed.
            with (out/'compression.log').open('w') as log:
                subprocess.run([sys.executable,str(ROOT/'scripts/compress_publication_gif.py'),str(out/'live-three-view.gif'),'--output',str(target),'--speed','1','--width','1440','--fps','12','--colors','96','--dither','none'],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=150)
            metadata=target.with_suffix('.json');result['compression']=json.loads(metadata.read_text())
            shutil.move(metadata,out/'compression.json');result['gif']=str(target.relative_to(ROOT))
            archive=ROOT/'.private/media-backups/live-bridge'/(trial.name+'.gif')
            if archive.exists():raise RuntimeError('Raw recording archive already exists')
            archive.parent.mkdir(parents=True,exist_ok=True);shutil.move(out/'live-three-view.gif',archive)
            result['raw_gif_archive']=str(archive.relative_to(ROOT))
        result['completed']=True
    finally:
        for proc in reversed(owned):
            if proc.poll() is None:
                try:os.killpg(proc.pid,signal.SIGINT)
                except ProcessLookupError:pass
        time.sleep(1)
        for sig in (signal.SIGTERM,signal.SIGKILL):
            for proc in reversed(owned):
                try:os.killpg(proc.pid,sig)
                except ProcessLookupError:pass
        for proc in owned:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:pass
        for f in handles:f.close()
        (trial/'presentation.json').write_text(json.dumps(result,indent=2)+'\n')
if __name__=='__main__':main()
