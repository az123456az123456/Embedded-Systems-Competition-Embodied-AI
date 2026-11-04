#!/usr/bin/env python3
"""PC-only BSP cross-build; the RK program itself requires no Python."""
from pathlib import Path
import hashlib,json,os,subprocess
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[1]
prefix=os.environ.get('CROSS_COMPILE')
cxx=Path(prefix+'g++') if prefix else next(ROOT.glob('bsp_r4/*/sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/*/bin/aarch64-none-linux-gnu-g++'))
prefix=str(cxx)[:-3];(BASE/'bin').mkdir(exist_ok=True);(BASE/'reports').mkdir(exist_ok=True)
binary=BASE/'bin/rawcap.aarch64'
args=[str(cxx),'-std=c++17','-O2','-static','-Wall','-Wextra','-Wpedantic','-Werror','-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-I',str(BASE/'vendor'),str(BASE/'src/rawcap.cpp'),'-o',str(binary)]
subprocess.run(args,check=True);subprocess.run([prefix+'strip','--strip-unneeded',str(binary)],check=True)
elf=subprocess.check_output([prefix+'readelf','-h','-l','-d',str(binary)],text=True)
assert 'AArch64' in elf and 'INTERP' not in elf and 'NEEDED' not in elf
(BASE/'reports/elf.txt').write_text(elf)
report={'result':'PASS','machine':'AArch64','static':True,'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'source_sha256':hashlib.sha256((BASE/'src/rawcap.cpp').read_bytes()).hexdigest(),'hardware_tested':False}
(BASE/'reports/build.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
