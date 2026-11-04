#!/usr/bin/env python3
"""Offline stage-0 build, golden tests, and ASan/UBSan; no ROS or network."""
from pathlib import Path
import hashlib
import json
import os
import platform
import subprocess
import time

root=Path(__file__).resolve().parent
pkg=root/'package'
report={'stage':2,'host':platform.machine(),'board_colcon':'NOT_RUN','runs':[]}
for mode in ('normal','asan-ubsan'):
    build=root/'build'/mode
    results=root/'reports'/mode
    build.mkdir(parents=True,exist_ok=True)
    results.mkdir(parents=True,exist_ok=True)
    flags=['-O2','-g','-Wall','-Wextra','-Wpedantic','-Werror']
    if mode=='asan-ubsan':
        flags+=['-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie']
    commands=[
      ['gcc','-std=c99',*flags,'-c',str(pkg/'tests/reference/nav3/candidate26/softcore/firmware/nav_exec_protocol.c'),'-o',str(build/'guard.o')],
      ['g++','-std=c++17',*flags,'-I'+str(pkg/'include'),'-c',str(pkg/'src/nav3_ethernet.cpp'),'-o',str(build/'ethernet.o')],
      ['gcc','-std=c99',*flags,'-c',str(pkg/'tests/reference/nav3/nav_exec_wire.c'),'-o',str(build/'golden.o')],
      ['g++','-std=c++17',*flags,'-I'+str(pkg/'include'),'-I'+str(pkg/'tests/reference/nav3'),
       str(pkg/'src/nav3_wire.cpp'),str(pkg/'tests/test_nav3_wire.cpp'),str(build/'golden.o'),
       *(['-no-pie'] if mode=='asan-ubsan' else []),'-o',str(build/'test_nav3_wire')],
      [str(build/'test_nav3_wire'),str(results)],
      ['g++','-std=c++17',*flags,'-I'+str(pkg/'include'),str(pkg/'src/nav3_wire.cpp'),str(pkg/'tests/test_nav3_client.cpp'),str(build/'golden.o'),str(build/'guard.o'),*(['-no-pie'] if mode=='asan-ubsan' else []),'-pthread','-o',str(build/'test_nav3_client')],
      [str(build/'test_nav3_client'),str(results/'client')],
      *[['gcc','-std=c99',*flags,'-c',str(pkg/'tests/reference/nav3/candidate26/softcore/firmware'/ (name+'.c')),'-o',str(build/(name+'.o'))] for name in ('astar','nav_protocol','nav_exec')],
      ['g++','-std=c++17',*flags,'-I'+str(pkg/'include'),str(pkg/'src/nav3_wire.cpp'),str(pkg/'tests/test_nav3_map.cpp'),str(build/'golden.o'),str(build/'guard.o'),*[str(build/(name+'.o')) for name in ('astar','nav_protocol','nav_exec')],*(['-no-pie'] if mode=='asan-ubsan' else []),'-pthread','-o',str(build/'test_nav3_map')],
      [str(build/'test_nav3_map'),str(results/'map')]]
    run={'mode':mode,'steps':[],'lsan':'DISABLED (sandbox ptrace); ASan/UBSan enabled' if mode=='asan-ubsan' else 'not applicable'}
    for i,cmd in enumerate(commands):
        env=os.environ.copy()
        if mode=='asan-ubsan':
            env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
            env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
        started=time.monotonic()
        r=subprocess.run(cmd,capture_output=True,text=True,env=env,timeout=120)
        log=results/f'step-{i}.log';log.write_text(r.stdout+r.stderr)
        run['steps'].append({'argv':cmd,'exit_code':r.returncode,'elapsed_s':time.monotonic()-started,'log':str(log.relative_to(root))})
        print(r.stdout+r.stderr,end='')
        if r.returncode:
            run['pass']=False;report['runs'].append(run)
            (root/'reports/host-verification.json').write_text(json.dumps(report,indent=2)+'\n')
            raise SystemExit(r.returncode)
    run['summary']=json.loads((results/'summary.json').read_text())
    run['client']=json.loads((results/'client/summary.json').read_text());run['pass']=run['summary']['pass'] and run['client']['pass'];report['runs'].append(run)
for run in report['runs']:
    run['map']=json.loads((root/'reports'/run['mode']/'map/summary.json').read_text());run['pass']=run['pass'] and run['map']['pass']
report['pass']=all(r['pass'] for r in report['runs'])
report['source_hashes']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in pkg.rglob('*') if p.is_file()}
(root/'reports/host-verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'pass':report['pass'],'runs':[{k:v for k,v in r.items() if k in ('mode','summary','pass')} for r in report['runs']]}))
