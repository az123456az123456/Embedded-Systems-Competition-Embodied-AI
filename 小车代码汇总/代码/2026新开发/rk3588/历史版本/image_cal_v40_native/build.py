#!/usr/bin/env python3
"""PC-only cross-build using the user's BSP; RK runtime needs no Python."""
from pathlib import Path
import hashlib,json,os,subprocess
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[1]
prefix=os.environ.get('CROSS_COMPILE')
if prefix:
    cxx=Path(prefix+'g++')
else:
    cxx=next(ROOT.glob('bsp_r4/*/sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/*/bin/aarch64-none-linux-gnu-g++'))
prefix=str(cxx)[:-3]
(BASE/'bin').mkdir(exist_ok=True);(BASE/'reports').mkdir(exist_ok=True)
binary=BASE/'bin/fpc1.aarch64'
args=[str(cxx),'-std=c++17','-O2','-Wall','-Wextra','-Wpedantic','-Werror','-static',
      '-ffunction-sections','-fdata-sections','-Wl,--gc-sections','-I',str(BASE/'vendor'),str(BASE/'src/fpc1.cpp'),'-o',str(binary)]
subprocess.run(args,check=True);subprocess.run([prefix+'strip','--strip-unneeded',str(binary)],check=True)
elf=subprocess.check_output([prefix+'readelf','-h','-l','-d',str(binary)],text=True)
assert 'AArch64' in elf and 'INTERP' not in elf and 'NEEDED' not in elf
(BASE/'reports/elf.txt').write_text(elf)
report={'result':'PASS','machine':'AArch64','static':True,'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
        'binary_size':binary.stat().st_size,'compiler':subprocess.check_output([str(cxx),'--version'],text=True).splitlines()[0],
        'source_sha256':{str(p.relative_to(BASE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [BASE/'src/fpc1.cpp',BASE/'vendor/picojson.h']},
        'hardware_tested':False,'commands':args}
(BASE/'reports/build.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
