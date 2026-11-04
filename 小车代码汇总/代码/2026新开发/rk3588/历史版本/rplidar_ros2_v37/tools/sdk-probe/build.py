#!/usr/bin/env python3
"""Build native Buildroot ARM64 diagnostic with the user's BSP GCC 10.3 toolchain."""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

root = Path(__file__).resolve().parents[2]
workspace = root.parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--cross-prefix', default=os.environ.get('ACG138_CROSS_COMPILE'))
args = parser.parse_args()
if args.cross_prefix:
    prefix = Path(args.cross_prefix)
else:
    candidates = sorted(workspace.glob('bsp_r4/*/sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/'
        'gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++'))
    if len(candidates) != 1:
        raise SystemExit('Set --cross-prefix /absolute/path/aarch64-none-linux-gnu-')
    prefix = Path(str(candidates[0])[:-3])
cxx = str(prefix) + 'g++'
ar = str(prefix) + 'ar'
readelf = str(prefix) + 'readelf'
strip = str(prefix) + 'strip'
for program in (cxx, ar, readelf, strip):
    if not Path(program).is_file():
        raise SystemExit('Missing toolchain program: ' + program)
package = root / 'ros2_ws/src/acg138_rplidar'
sdk = package / 'vendor/sl_lidar_sdk'
source = root / 'tools/sdk-probe/lidar_scan_probe.cpp'
build = root / 'build/sdk-probe-arm64'
build.mkdir(parents=True, exist_ok=True)
binary = root / 'deploy/bin/lidar-scan-probe.aarch64'
binary.parent.mkdir(parents=True, exist_ok=True)
commands=[]

def run(command, name):
    commands.append(command)
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=240)
    (build / (name+'.log')).write_text(result.stdout)
    if result.returncode:
        print(result.stdout[-5000:])
        raise RuntimeError(name+' failed')
    return result.stdout

started=time.monotonic()
includes=['-isystem', str(sdk/'include'), '-I'+str(sdk/'src')]
sources=sorted(set(p for pattern in ('src/*.cpp','src/arch/linux/*.cpp','src/hal/*.cpp',
    'src/dataunpacker/*.cpp','src/dataunpacker/unpacker/*.cpp') for p in sdk.glob(pattern)))

def compile_one(path):
    name=str(path.relative_to(sdk)).replace('/','_')
    output=build/(name+'.o')
    run([cxx,'-std=c++17','-O2','-pthread','-ffunction-sections','-fdata-sections']+includes+
        ['-c',str(path),'-o',str(output)],'compile-'+name)
    return output

with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    objects=list(pool.map(compile_one,sources))
archive=build/'libsl_lidar_sdk.a'
run([ar,'rcs',str(archive)]+list(map(str,objects)),'archive')
run([cxx,'-std=c++17','-O2','-Wall','-Wextra','-Wpedantic','-Werror','-pthread',
     '-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-static-libstdc++','-static-libgcc']+
    includes+['-I'+str(package/'include'),str(source),str(archive),'-o',str(binary)],'link-probe')
unstripped=build/'lidar-scan-probe.unstripped.aarch64'
import shutil
shutil.copy2(binary,unstripped)
run([strip,'--strip-unneeded',str(binary)],'strip')
elf=run([readelf,'-h','-l','-d','--version-info',str(binary)],'readelf')
if 'AArch64' not in elf:
    raise RuntimeError('Output is not AArch64')
needed=[]
glibc_versions=set()
import re
for line in elf.splitlines():
    if '(NEEDED)' in line:
        needed.extend(re.findall(r'\[([^\]]+)\]',line))
    glibc_versions.update(re.findall(r'GLIBC_[0-9.]+',line))
if any('libstdc++' in name or 'libgcc_s' in name for name in needed):
    raise RuntimeError('C++ runtime unexpectedly dynamically linked')
interpreter=re.search(r'Requesting program interpreter: ([^\]]+)',elf)
if not interpreter or interpreter.group(1) != '/lib/ld-linux-aarch64.so.1':
    raise RuntimeError('Unexpected ARM64 dynamic loader')
paths=[source,Path(__file__).resolve(),package/'include/acg138_rplidar/scan_core.hpp',package/'vendor/provenance.json']+sources
paths+=sorted(sdk.rglob('*.h')) + sorted(sdk.rglob('*.hpp'))
paths+=sorted((root/'tools/sdk-probe').glob('*.txt'))
paths=sorted(set(paths))
report={'status':'PASS','hardware_tested':False,'executed_on_rk3588':False,
    'toolchain_version':run([cxx,'--version'],'toolchain-version').splitlines()[0],
    'output':str(binary.relative_to(root)),'output_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
    'fixed_baudrate':460800,'sdk_source_files':len(sources),'elf_machine':'AArch64',
    'binary_size_bytes':binary.stat().st_size,'interpreter':interpreter.group(1),
    'needed':needed,'glibc_symbol_versions':sorted(glibc_versions),
    'static_cpp_runtime':True,'elapsed_s':round(time.monotonic()-started,3),
    'source_sha256':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
    'commands':commands}
(build/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print('PASS native Buildroot probe cross-build:',binary)
print('DT_NEEDED:',', '.join(needed))
print('Report:',build/'verification.json')
