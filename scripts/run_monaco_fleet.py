#!/usr/bin/env python3
"""Send independent unchanged Nav2 checkpoint missions to four real simulated cars."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.monaco_fleet import VEHICLES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=1800)
    args = parser.parse_args()
    out = ROOT/'artifacts/fleet/mission'
    out.mkdir(parents=True, exist_ok=True)
    processes = []
    started = time.monotonic()
    try:
        for color in VEHICLES:
            report = out/(color+'.json')
            if report.exists():
                report.rename(out/(color+'-previous.json'))
            log = (out/(color+'.log')).open('w')
            command = [sys.executable, str(ROOT/'scripts/drive_monaco.py'), '--timeout', str(args.timeout),
                '--fleet-color', color, '--robot-name', 'racecar' if color == 'red' else 'racecar_'+color,
                '--output', str(report)]
            if color != 'red':command += ['--namespace', color]
            processes.append((color, subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT), log))
        while any(p.poll() is None for _, p, _ in processes):
            time.sleep(.5)
    except KeyboardInterrupt:
        pass
    finally:
        for _, p, _ in processes:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for _, p, log in processes:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
            log.close()
        results = {}
        for color, p, _ in processes:
            path = out/(color+'.json')
            report = json.loads(path.read_text()) if path.exists() else {}
            results[color] = {k:report.get(k) for k in ('passed','full_course','ordered_targets_passed',
                'requested_targets','odometry_distance_m','navigation_recoveries','elapsed_wall_seconds','error')}
        summary = dict(moving_vehicles=4, camera_target='racecar', scheduler_coupled_to_robot=False,
                       passed=all(v.get('passed') for v in results.values()), results=results)
        summary.update(full_course=all(v.get('full_course') for v in results.values()),
            ordered_targets_passed=sum(v.get('ordered_targets_passed') or 0 for v in results.values()),
            navigation_recoveries=sum(v.get('navigation_recoveries') or 0 for v in results.values()),
            elapsed_wall_seconds=round(time.monotonic()-started,3))
        (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary,indent=2),flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
