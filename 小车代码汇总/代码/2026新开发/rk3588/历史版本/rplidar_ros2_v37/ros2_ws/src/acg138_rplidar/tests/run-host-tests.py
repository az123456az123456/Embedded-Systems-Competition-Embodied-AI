#!/usr/bin/env python3
"""Real host compile/tests; never opens a serial device or reports hardware success."""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path)
args = parser.parse_args()
package = Path(__file__).resolve().parents[1]
output = (args.output or package.parents[2] / 'build' / 'driver-host').resolve()
output.mkdir(parents=True, exist_ok=True)
cxx = shutil.which(os.environ.get('HOST_CXX', 'g++'))
ar = shutil.which(os.environ.get('HOST_AR', 'ar'))
if not cxx or not ar:
    raise SystemExit('Host C++ compiler and ar required')
commands = []

def run(command, name, extra_env=None):
    commands.append(command)
    env = dict(os.environ)
    if extra_env:
        env.update(extra_env)
    completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, env=env, timeout=240)
    (output / (name + '.log')).write_text(completed.stdout)
    if completed.returncode:
        print(completed.stdout[-6000:])
        raise RuntimeError(f'{name} failed ({completed.returncode})')
    return completed.stdout.strip()

started = time.monotonic()
include = ['-I' + str(package / 'include')]
common = [cxx, '-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror']
run(common + ['-O2'] + include + [str(package / 'tests/scan_core_test.cpp'),
    '-o', str(output / 'scan_core_test')], 'compile-core')
core_stdout = run([str(output / 'scan_core_test')], 'test-core')
run(common + ['-O1', '-g', '-fno-omit-frame-pointer', '-fsanitize=address,undefined']
    + include + [str(package / 'tests/scan_core_test.cpp'), '-o', str(output / 'scan_core_sanitized')],
    'compile-sanitized')
sanitizer_stdout = run([str(output / 'scan_core_sanitized')], 'test-sanitized',
    {'ASAN_OPTIONS': 'detect_leaks=0:halt_on_error=1', 'UBSAN_OPTIONS': 'halt_on_error=1'})
sdk = package / 'vendor/sl_lidar_sdk'
sdk_include = ['-isystem', str(sdk / 'include'), '-I' + str(sdk / 'src')]
run(common + sdk_include + ['-c', str(package / 'tests/sdk_api_contract.cpp'),
    '-o', str(output / 'sdk-api-contract.o')], 'compile-api-contract')
sources = sorted(set(p for pattern in ('src/*.cpp', 'src/arch/linux/*.cpp', 'src/hal/*.cpp',
                'src/dataunpacker/*.cpp', 'src/dataunpacker/unpacker/*.cpp') for p in sdk.glob(pattern)))
sdk_objects = output / 'sdk-objects'
sdk_objects.mkdir(exist_ok=True)

def compile_sdk(source):
    name = str(source.relative_to(sdk)).replace('/', '_')
    target = sdk_objects / (name + '.o')
    run([cxx, '-std=c++17', '-O2', '-pthread'] + sdk_include + ['-c', str(source), '-o', str(target)],
        'compile-' + name)
    return target

with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
    objects = list(executor.map(compile_sdk, sources))
run([ar, 'rcs', str(output / 'libsl_lidar_sdk.a')] + list(map(str, objects)), 'archive-real-sdk')
hashes = {str(p.relative_to(package)): hashlib.sha256(p.read_bytes()).hexdigest()
          for p in sorted(package.rglob('*')) if p.is_file() and 'build' not in p.parts}
report = {'status': 'PASS', 'hardware_tested': False, 'ros2_node_compiled': False,
          'reason_ros2_node_not_compiled': 'ROS 2 headers/runtime are not installed on this host',
          'host_compiler': cxx, 'sdk_source_files_compiled': len(sources),
          'core': core_stdout, 'asan_ubsan': sanitizer_stdout,
          'api_contract': 'PASS compiled against real vendored SDK headers',
          'elapsed_s': round(time.monotonic() - started, 3), 'source_sha256': hashes,
          'commands': commands}
(output / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')
print(core_stdout)
print(sanitizer_stdout)
print('PASS real SDK source compile:', len(sources), 'files; no ROS runtime or physical serial test claimed')
print(output / 'verification.json')
