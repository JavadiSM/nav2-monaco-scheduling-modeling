#!/usr/bin/env python3
"""Record an unchanged Gazebo scene with a rigid chase and overview camera."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]

def windows():
    p=subprocess.run(['xdotool','search','--name','^Gazebo Sim$'],text=True,capture_output=True)
    return set(p.stdout.split())

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--fleet',action='store_true');parser.add_argument('--seconds',type=float,default=20);parser.add_argument('--full-course',action='store_true');parser.add_argument('--timeout',type=float,default=1200)
    args=parser.parse_args(); owned=[]; handles=[]
    out=ROOT/('artifacts/fleet/capture' if args.fleet else 'artifacts/dual-view');out.mkdir(parents=True,exist_ok=True)
    media=ROOT/'docs/media';media.mkdir(parents=True,exist_ok=True)
    if not os.environ.get('DISPLAY'):raise SystemExit('Source scripts/environment.sh first')
    if windows():raise SystemExit('Close existing Gazebo windows before this isolated capture')
    existing=subprocess.check_output(['ps','-eo','args'],text=True)
    if any(('gz sim -s' in line or 'gz sim -r -s' in line or 'ros2 launch' in line) and 'ps -eo' not in line for line in existing.splitlines()):
        raise SystemExit('Another simulation appears active; refusing an overlapping launch')
    original=(ROOT/'config/monaco-gui.config').read_text()
    # Remove only the tracking plugin from the overview, avoiding shared topic effects.
    import re
    overview=re.sub(r'  <plugin filename="CameraTracking".*?</plugin>','',original,flags=re.S)
    (out/'overview.config').write_text(overview)
    (out/'chase.config').write_text(original)
    def launch(cmd,name):
        h=(out/(name+'.log')).open('w');handles.append(h)
        p=subprocess.Popen(cmd,cwd=ROOT,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
        owned.append(p);return p
    evidence={'world_changed':args.fleet, 'world_change_scope':'local starting apron only' if args.fleet else 'none','capture_type':'actual_Gazebo_pixels','source_gif':str((out/'dual-view.gif').relative_to(ROOT)),
              'left':'rigid third-person, 0.75 m behind in vehicle local x; height offset 0.25 m',
              'right':'fixed existing overview', 'scheduler_coupled_to_robot':False,
              'note':'Camera/motion demonstration; local FIFO is separately replayed from measured tasks.'}
    mission_command = ['python3', str(ROOT/'scripts/run_monaco_fleet.py')] if args.fleet else ['bash', str(ROOT/'scripts/run_monaco.sh')]
    evidence['moving_vehicles']=4 if args.fleet else 1
    try:
        launch(['ros2','launch',str(ROOT/'launch/monaco.launch.py'),'gui:=false','fleet:='+str(args.fleet).lower()],'launch')
        time.sleep(12)
        ids={}
        for label in ('overview','chase'):
            before=windows();p=launch(['gz','sim','-g','-v','2','--gui-config',str(out/(label+'.config'))],label)
            deadline=time.monotonic()+35
            while time.monotonic()<deadline:
                after=windows()-before
                if after:
                    ids[label]=sorted(after)[0];break
                if p.poll() is not None:raise RuntimeError(label+' GUI exited')
                time.sleep(.5)
            if label not in ids:raise RuntimeError('No window for '+label)
        time.sleep(12)
        track='track_mode: FOLLOW_LOOK_AT follow_target: {name: "racecar"} track_target: {name: "racecar"} follow_offset: {x: -0.75 y: 0 z: 0.25} track_offset: {x: 0 y: 0 z: 0.10} follow_pgain: 1 track_pgain: 1'
        for service, request_type, request in [
            ('/gui/follow', 'gz.msgs.StringMsg', 'data: "racecar"'),
            ('/gui/follow/offset', 'gz.msgs.Vector3d', 'x: -0.75 y: 0 z: 0.25')]:
            response=subprocess.run(['gz','service','-s',service,'--reqtype',request_type,
                '--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],capture_output=True,text=True,check=True)
            if 'true' not in response.stdout:
                raise RuntimeError('Camera service did not confirm: '+service)
        for _ in range(3):
            subprocess.run(['gz','topic','-t','/gui/track','-m','gz.msgs.CameraTrack','-p',track],check=True)
            time.sleep(.5)
        capture_seconds=args.timeout if args.full_course else args.seconds
        command=['ffmpeg','-y','-hide_banner','-loglevel','warning']
        for label in ('chase','overview'):
            command+=['-thread_queue_size','512','-f','x11grab','-framerate','6','-window_id',ids[label],'-video_size','1600x900','-t',str(capture_seconds),'-i',os.environ['DISPLAY']]
        views=("[0:v]setpts=PTS-STARTPTS,scale=640:360,drawtext=text='Third person - 0.75 m behind':x=12:y=12:fontcolor=white:box=1:boxcolor=black@0.7[a];"
               "[1:v]setpts=PTS-STARTPTS,scale=640:360,drawtext=text='Fixed overview':x=12:y=12:fontcolor=white:box=1:boxcolor=black@0.7[b];[a][b]hstack")
        command+=['-t',str(capture_seconds),'-filter_complex_threads','1']
        if args.full_course:
            temporary=out/'full-course-capture.mkv'
            command+=['-filter_complex',views,'-an','-c:v','libx264','-preset','ultrafast','-crf','20',str(temporary)]
            with (out/'capture.log').open('w') as log:
                recorder=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
                try:
                    time.sleep(1)
                    if recorder.poll() is not None:raise RuntimeError('Capture did not start')
                    mission=launch(mission_command+['--timeout',str(args.timeout)],'mission')
                    mission.wait(timeout=args.timeout+30)
                    time.sleep(2)
                finally:
                    if recorder.poll() is None:recorder.send_signal(signal.SIGINT)
                    recorder.wait(timeout=40)
            report_path = ROOT/('artifacts/fleet/mission/summary.json' if args.fleet else 'artifacts/monaco-run.json')
            report=json.loads(report_path.read_text())
            if mission.returncode or not report.get('passed') or not report.get('full_course'):
                raise RuntimeError('Mission did not finish successfully; local recording retained')
            (out/'full-course-mission.json').write_text(json.dumps(report,indent=2)+'\n')
            # Two passes avoid retaining an entire lap of decoded frames in memory.
            palette=out/'palette.png'
            subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(temporary),
                '-vf','fps=6,palettegen=stats_mode=diff','-frames:v','1',str(palette)],check=True)
            subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(temporary),'-i',str(palette),
                '-filter_complex_threads','1','-filter_complex','[0:v]fps=6[v];[v][1:v]paletteuse=dither=bayer:bayer_scale=3',
                '-loop','0',str(out/'dual-view.gif')],check=True)
            evidence.update(full_course=True,mission_passed=True,ordered_targets_passed=report['ordered_targets_passed'],
                elapsed_mission_wall_s=report['elapsed_wall_seconds'],navigation_recoveries=report['navigation_recoveries'])
            from PIL import Image
            with Image.open(out/'dual-view.gif') as image:
                for frame in range(image.n_frames):image.seek(frame);image.load()
            temporary.unlink()
        else:
            command+=['-filter_complex',views+',split[c][d];[c]palettegen=stats_mode=diff[p];[d][p]paletteuse=dither=bayer:bayer_scale=3',
                '-loop','0',str(out/'dual-view.gif')]
            with (out/'capture.log').open('w') as log:
                recorder=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
                try:
                    time.sleep(1)
                    if recorder.poll() is not None:raise RuntimeError('Capture did not start')
                    mission=launch(mission_command+['--timeout',str(max(args.seconds+90,120))],'mission')
                    recorder.wait(timeout=args.seconds+60)
                    if recorder.returncode:raise RuntimeError('Capture failed')
                finally:
                    if recorder.poll() is None:
                        recorder.send_signal(signal.SIGINT)
                        recorder.wait(timeout=40)
        from PIL import Image
        with Image.open(out/'dual-view.gif') as im:
            evidence['frames']=im.n_frames;evidence['dimensions']=list(im.size)
            im.seek(min(120 if args.full_course else 18,im.n_frames-1));im.convert('RGB').save(out/'preview.png' if args.fleet else media/'dual-view-frame.png')
        evidence['window_ids']=ids;evidence['requested_duration_s']=None if args.full_course else args.seconds
        evidence['mission_completed_during_clip']=mission.poll()==0
    finally:
        for p in reversed(owned):
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGINT)
                except ProcessLookupError:pass
        time.sleep(2)
        # A launch leader may exit while its children still occupy its group.
        for p in reversed(owned):
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        time.sleep(2)
        for p in reversed(owned):
            try:os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            p.wait(timeout=5)
        for h in handles:h.close()
        (out/'capture-evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence,indent=2))

if __name__=='__main__':main()
