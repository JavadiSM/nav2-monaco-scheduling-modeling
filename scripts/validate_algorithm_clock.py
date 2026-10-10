#!/usr/bin/env python3
"""Exercise the installed Nav2 replanning decorator against a controlled clock."""
import json,os,subprocess,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    include=Path('/opt/ros/jazzy/include')
    with tempfile.TemporaryDirectory(prefix='nav2-clock-test-') as folder:
        path=Path(folder);binary=path/'rate-clock';clock=path/'clock';clock.write_bytes(bytes(32))
        flags=['-I'+str(p) for p in [include,*[p for p in include.iterdir() if p.is_dir()]]]
        subprocess.run(['g++','-O2','-std=c++17',str(ROOT/'tests/native/rate_clock_fixture.cpp'),*flags,'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib','-lbehaviortree_cpp','-o',str(binary)],check=True)
        env={**os.environ,'LD_PRELOAD':str(ROOT/'build/live-bridge/libnav2_live_bridge.so'),'NAV2_LIVE_CLOCK':str(clock)}
        result=subprocess.run([str(binary)],env=env,text=True,capture_output=True,check=True)
        print(json.dumps(json.loads(result.stdout),indent=2))
if __name__=='__main__':main()
