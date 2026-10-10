#!/usr/bin/env python3
"""Build the optional live adapter without replacing installed ROS binaries."""
import json,os,subprocess,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    out=ROOT/'build/live-bridge';out.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env['PKG_CONFIG_PATH']=':'.join(str(p) for p in Path('/opt/ros/jazzy/opt').glob('*/lib/pkgconfig'))+':'+env.get('PKG_CONFIG_PATH','')
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','gz-transport13','gz-msgs10'],env=env,text=True).split()
    inc=Path('/opt/ros/jazzy/include');includes=['-I'+str(p) for p in [inc,*sorted(p for p in inc.iterdir() if p.is_dir()),Path('/usr/include/eigen3')]]
    commands=[['g++','-O2','-std=c++17','-pthread',str(ROOT/'tools/live_bridge/stepper.cpp'),'-o',str(out/'stepper'),*flags],
      ['g++','-O2','-g','-std=c++17','-pthread','-shared','-fPIC',str(ROOT/'tools/live_bridge/nav2_adapter.cpp'),str(ROOT/'tools/live_bridge/algorithm_clock.cpp'),str(ROOT/'tools/live_bridge/tf_audit.cpp'),'-o',str(out/'libnav2_live_bridge.so'),*includes,'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib','-lrclcpp','-lrcl','-lrcutils','-lrmw','-lrosidl_runtime_c','-ldl']]
    metadata=[]
    for cmd in commands:
        p=subprocess.run(cmd,env=env,text=True,capture_output=True)
        (out/(Path(cmd[cmd.index('-o')+1]).name+'.build.log')).write_text(p.stdout+p.stderr)
        if p.returncode:print(p.stderr);return p.returncode
        metadata.append({'command':cmd,'sources_sha256':{str(Path(x).relative_to(ROOT)):hashlib.sha256(Path(x).read_bytes()).hexdigest() for x in cmd if x.endswith('.cpp')},'sha256':hashlib.sha256(Path(cmd[cmd.index('-o')+1]).read_bytes()).hexdigest()})
    (out/'build.json').write_text(json.dumps(metadata,indent=2)+'\n');print(out);return 0
if __name__=='__main__':raise SystemExit(main())
