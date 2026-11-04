#!/usr/bin/env python3
"""Test the next stopped-task pose seed against the actual v51 firmware ledger."""
from pathlib import Path
import hashlib
import json
import os
import subprocess

r = Path(__file__).resolve().parents[1]
v = r.parents[1] / 'fpga/robot_navigation_v51_local_pose_sonar50/verification'
p = r / 'acg138_fpga_map_bridge'
out = r / 'reports/offline'
out.mkdir(parents=True, exist_ok=True)
records = []
for mode in ('normal', 'asan-ubsan'):
    flags = ['-O1', '-g', '-Wall', '-Wextra']
    if mode != 'normal':
        flags += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie']
    objects = [v / mode / (n + '.o') for n in (
        'nav_exec', 'nav_exec_protocol', 'nav_protocol', 'astar', 'nav_avoid',
        'nav_obstacle', 'nav_track', 'nav_stop', 'nav_math', 'layered_control', 'nav_exec_wire')]
    assert all(x.is_file() for x in objects), 'Run the v51 firmware layer first'
    exe = out / ('local-reseed-' + mode)
    argv = ['g++', '-std=c++17', *flags, '-I' + str(p / 'include'),
            str(r / 'tests/test_local_reseed.cpp'), str(p / 'src/nav3_wire.cpp'),
            *map(str, objects), '-pthread', '-o', str(exe)]
    env = {**os.environ, 'ASAN_OPTIONS': 'detect_leaks=0:halt_on_error=1',
           'UBSAN_OPTIONS': 'halt_on_error=1:print_stacktrace=1'}
    for step, command in enumerate((argv, [str(exe)])):
        x = subprocess.run(command, capture_output=True, text=True, env=env, timeout=120)
        (out / f'local-reseed-{mode}-{step}.log').write_text(x.stdout + x.stderr)
        records.append({'mode': mode, 'step': step, 'argv': command, 'returncode': x.returncode})
        if x.returncode:
            (out / 'local-reseed-summary.json').write_text(json.dumps({'status': 'FAIL', 'steps': records}, indent=2) + '\n')
            raise SystemExit(x.returncode)
    (out / f'local-reseed-{mode}.jsonl').write_text(x.stdout)
report = {'status': 'PASS', 'steps': records, 'hardware_commands': False,
          'object_sha256': {str(x): hashlib.sha256(x.read_bytes()).hexdigest() for x in objects},
          'scope': 'Actual v51 C state admission and current RK NAV3 client; virtual wire transport only'}
(out / 'local-reseed-summary.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps({'status': 'PASS', 'runs': 2}))
