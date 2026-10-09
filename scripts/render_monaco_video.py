#!/usr/bin/env python3
"""Overlay recorded Gazebo video with a labeled AMCL trajectory map."""
import argparse
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    artifacts = project / 'artifacts'
    report = json.loads((artifacts / 'monaco-run.json').read_text())
    metadata = json.loads((artifacts / 'monaco-recording.json').read_text())
    if not report.get('passed') or not report.get('full_course'):
        raise SystemExit('A successful full-course report is required for this completed-course video.')
    source_file = artifacts / metadata['raw_file']
    if not source_file.exists():
        source_file = source_file.with_suffix('.gif')
    cap = cv2.VideoCapture(str(source_file))
    if not cap.isOpened():
        raise SystemExit('Recorded Gazebo video could not be opened.')
    width, height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (width, height) not in ((1600, 900), (960, 540)):
        raise SystemExit('Zoom calibration requires the configured 1600x900 top-down view.')
    fps = cap.get(cv2.CAP_PROP_FPS)
    scene_path = artifacts / metadata['scene_file'] if 'scene_file' in metadata else project / 'scenarios/monaco/scenario.json'
    scene = json.loads(scene_path.read_text())
    route = np.array(scene['centreline'])
    lower, upper = route.min(axis=0)-1.2, route.max(axis=0)+1.2
    scale = min(380/(upper[0]-lower[0]), 196/(upper[1]-lower[1]))
    def pixel(x, y):
        return int(10+(x-lower[0])*scale), int(32+(upper[1]-y)*scale)
    track_pixels = np.array([pixel(*p) for p in route], dtype=np.int32)
    map_base = np.full((240, 400, 3), (32, 29, 24), np.uint8)
    cv2.polylines(map_base, [track_pixels], False, (100, 95, 78), 8, cv2.LINE_AA)
    cv2.putText(map_base, 'AMCL estimated position / trace', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .48, (245,245,245), 1, cv2.LINE_AA)
    times = np.array([p['wall_time'] for p in report['pose_samples']])
    xs, ys = np.array([p['x'] for p in report['pose_samples']]), np.array([p['y'] for p in report['pose_samples']])
    events = report['target_events']
    event_times = np.array([e['wall_time'] for e in events])
    success_time = report.get('success_wall_time', metadata['end_wall_time'] - 2.0)
    output = (artifacts / metadata.get('output_file', 'monaco-start-to-finish.gif')).with_suffix('.gif')
    width, height = 1600, 900
    encoder = subprocess.Popen(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'warning', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{width}x{height}', '-r', str(fps), '-i', '-', '-an', '-filter_complex_threads', '1', '-filter_complex', 'fps=5,scale=960:540,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3', '-loop', '0', str(output)], stdin=subprocess.PIPE)
    frame_number = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if frame.shape[:2] != (900,1600):
                frame = cv2.resize(frame, (1600,900))
            wall_time = metadata['start_wall_time'] + frame_number / fps
            x, y = np.interp(wall_time, times, xs), np.interp(wall_time, times, ys)
            index = np.searchsorted(event_times, wall_time, side='right') - 1
            completed = events[index]['index'] if index >= 0 else 0
            if wall_time >= success_time:
                status = 'FINISHED | 19/19 checkpoints + finish confirmed'
            elif index < 0:
                status = 'START | localization and upstream Nav2 activation'
            else:
                status = f'Checkpoint targets completed: {completed}/19 | ' + (f'Heading to CP{completed+1:02d}' if completed < 19 else 'Heading to FINISH')
            mini = map_base.copy()
            sample_index = np.searchsorted(times, wall_time, side='right')
            trace = np.array([pixel(xx, yy) for xx, yy in zip(xs[:sample_index], ys[:sample_index])], dtype=np.int32)
            if len(trace)>1:
                cv2.polylines(mini, [trace], False, (220,205,70), 2, cv2.LINE_AA)
            for cp in scene['checkpoints']:
                point = pixel(cp['x'], cp['y'])
                colour = (80,225,130) if cp['id'] <= completed or wall_time >= success_time else (40,195,240)
                cv2.circle(mini, point, 3, colour, -1, cv2.LINE_AA)
            for label, point, colour in [('S', scene['start'], (80,225,130)), ('F', scene['finish'], (70,80,245))]:
                pos = pixel(point['x'],point['y'])
                cv2.putText(mini, label, (pos[0]-10,pos[1]-4), cv2.FONT_HERSHEY_SIMPLEX,.35,colour,1,cv2.LINE_AA)
            cv2.circle(mini, pixel(x,y), 5, (40,220,255), -1, cv2.LINE_AA)
            frame[65:305,1180:1580] = mini
            cv2.rectangle(frame,(1179,64),(1581,306),(220,210,75),1)
            cv2.rectangle(frame, (0, 0), (1600, 47), (26, 22, 18), -1)
            cv2.putText(frame, 'MONACO-INSPIRED CIRCUIT  |  Upstream Nav2  |  1 moving car  |  19 parked edge cars', (22, 31), cv2.FONT_HERSHEY_SIMPLEX, .75, (244, 245, 247), 2, cv2.LINE_AA)
            cv2.rectangle(frame, (0, 835), (1600, 899), (26, 22, 18), -1)
            cv2.putText(frame, status, (22, 863), cv2.FONT_HERSHEY_SIMPLEX, .70, (80, 235, 175), 2, cv2.LINE_AA)
            elapsed = frame_number / fps
            cv2.putText(frame, f'REAL RECORDING  |  {int(elapsed)//60:02d}:{int(elapsed)%60:02d}  |  Communication and custom scheduling inactive', (22, 887), cv2.FONT_HERSHEY_SIMPLEX, .52, (205, 205, 205), 1, cv2.LINE_AA)
            encoder.stdin.write(frame.tobytes())
            frame_number += 1
    finally:
        cap.release()
        encoder.stdin.close()
    if encoder.wait() != 0:
        raise SystemExit('Video encoding failed.')
    print(json.dumps({'file': str(output), 'frames': frame_number, 'duration_seconds': frame_number/fps, 'source': metadata['raw_file'], 'note': 'Main image is captured Gazebo pixels; the inset explicitly shows AMCL estimates and target progress uses Nav2 feedback.'}, indent=2))


if __name__ == '__main__':
    main()
