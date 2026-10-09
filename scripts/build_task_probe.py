#!/usr/bin/env python3
"""Build the optional measurement shim; never install over ROS packages."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path)
    parser.add_argument('--source',type=Path)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    output=args.output or root/'build/task-probe'
    output.mkdir(parents=True,exist_ok=True)
    source=args.source or root/'tools/profiling/nav2_task_probe.cpp'
    include_root=Path('/opt/ros/jazzy/include')
    includes=[include_root]+sorted(p for p in include_root.iterdir() if p.is_dir())+[Path('/usr/include/eigen3')]
    command=['g++','-O2','-std=c++17','-shared','-fPIC','-pthread','-o',str(output/'libnav2_task_probe.tmp.so'),str(source),'-ldl']+['-I'+str(p) for p in includes]
    proc=subprocess.run(command,capture_output=True,text=True)
    (output/'build.log').write_text(proc.stdout+proc.stderr)
    if proc.returncode:
        print(proc.stderr)
        return proc.returncode
    binary=output/'libnav2_task_probe.so'
    (output/'libnav2_task_probe.tmp.so').replace(binary)
    meta={'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'command':command,'built_unix_s':time.time(),'nav2_expected_version':'1.3.13','note':'LD_PRELOAD observational interposition; no scheduler replacement; CPU costs include residual measurement overhead.'}
    (output/'build-metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    harness=output/'overhead.cpp'
    harness.write_text('''#include <dlfcn.h>
#include <cstdio>
#include <time.h>
long long n(clockid_t c){timespec t{};clock_gettime(c,&t);return t.tv_sec*1000000000LL+t.tv_nsec;}
int main(int argc,char**argv){auto h=dlopen(argv[1],RTLD_NOW);if(!h){puts(dlerror());return 1;}auto f=(void(*)())dlsym(h,"nav2_probe_noop");long long w=n(CLOCK_MONOTONIC),c=n(CLOCK_THREAD_CPUTIME_ID);for(int i=0;i<100000;i++)f();printf("{\\\"calls\\\":100000,\\\"wall_ns\\\":%lld,\\\"cpu_ns\\\":%lld}\\n",n(CLOCK_MONOTONIC)-w,n(CLOCK_THREAD_CPUTIME_ID)-c);return 0;}
''')
    subprocess.run(['g++','-O2',str(harness),'-o',str(output/'overhead'),'-ldl'],check=True)
    print(binary)
    return 0

if __name__=='__main__':
    raise SystemExit(main())
