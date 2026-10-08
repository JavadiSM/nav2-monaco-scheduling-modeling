#!/usr/bin/env python3
"""Record the real Gazebo window throughout an upstream Nav2 mission."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--window-id', default=None)
    p.add_argument('--timeout', type=float, default=1500)
    args = p.parse_args()
    project = Path(__file__).resolve().parents[1]
    artifacts = project / 'artifacts'
    artifacts.mkdir(exist_ok=True)
    if not os.environ.get('DISPLAY'):
        raise SystemExit('Source scripts/environment.sh first.')
    if args.window_id is None:
        windows = subprocess.check_output(['xdotool', 'search', '--name', '^Gazebo Sim$'], text=True).splitlines()
        if len(windows) != 1:
            raise SystemExit('Select one Gazebo window explicitly with --window-id.')
        args.window_id = windows[0]
    metadata = {'start_wall_time': time.time(), 'fps': 12, 'window_id': args.window_id, 'raw_file': 'monaco-race-raw.mp4'}
    (artifacts / 'monaco-recording.json').write_text(json.dumps(metadata, indent=2) + '\n')
    with (artifacts / 'monaco-recording.log').open('w') as capture_log:
        recorder = subprocess.Popen(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'warning', '-f', 'x11grab', '-framerate', '12', '-window_id', args.window_id, '-video_size', '1600x900', '-i', os.environ['DISPLAY'], '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '22', '-pix_fmt', 'yuv420p', str(artifacts / metadata['raw_file'])], stdout=capture_log, stderr=subprocess.STDOUT)
        try:
            time.sleep(1)
            if recorder.poll() is not None:
                raise RuntimeError('Capture could not start; check monaco-recording.log.')
            with (artifacts / 'monaco-mission.log').open('w') as mission_log:
                mission = subprocess.run(['bash', str(project / 'scripts/run_monaco.sh'), '--timeout', str(args.timeout)], stdout=mission_log, stderr=subprocess.STDOUT)
            time.sleep(2)
        finally:
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
                try:
                    recorder.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    recorder.terminate()
                    recorder.wait(timeout=5)
    metadata['end_wall_time'] = time.time()
    metadata['mission_exit_code'] = mission.returncode
    (artifacts / 'monaco-recording.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2), flush=True)
    return mission.returncode


if __name__ == '__main__':
    raise SystemExit(main())
