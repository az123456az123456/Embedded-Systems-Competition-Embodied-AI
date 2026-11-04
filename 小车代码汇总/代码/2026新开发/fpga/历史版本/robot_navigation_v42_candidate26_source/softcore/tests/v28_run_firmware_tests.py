#!/usr/bin/env python3
"""Reproduce v28 host and real 50MHz RV32 firmware tests; no hardware access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'softcore/firmware'
REPORTS=ROOT/'reports'

def run(cmd,log,cwd=ROOT):
    started=time.monotonic()
    with (REPORTS/log).open('w') as out:
        subprocess.run(cmd,cwd=cwd,check=True,stdout=out,stderr=subprocess.STDOUT,
                       env={**os.environ,'ASAN_OPTIONS':'detect_leaks=0','CCACHE_DISABLE':'1','CCACHE_TEMPDIR':'/tmp'})
    return round(time.monotonic()-started,3)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host-only',action='store_true')
    args=ap.parse_args();REPORTS.mkdir(exist_ok=True)
    timings={}
    timings['build']=run(['make','all','size'],'v28_firmware_build.log',FW)
    cflags=['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-pedantic',
            '-fsanitize=address,undefined','-fno-omit-frame-pointer']
    timings['host_compile']=run(cflags+['softcore/tests/v28_layered_control_test.c',
        'softcore/firmware/layered_control.c','-o','build/v28-control-test'],'v28_firmware_host_compile.log')
    timings['host_control']=run(['build/v28-control-test'],'v28_firmware_host.log')
    timings['host_astar']=run(['make','-C','softcore/tests','-f','test_astar.mk','test'],'v28_firmware_astar_host.log')
    timings['host_nav_compile']=run(cflags+['softcore/tests/test_nav_protocol.c',
        'softcore/firmware/astar.c','softcore/firmware/nav_protocol.c','-o','build/v28-nav-protocol-test'],
        'v28_firmware_nav_compile.log')
    timings['host_nav_protocol']=run(['build/v28-nav-protocol-test'],'v28_firmware_nav_protocol_host.log')
    if not args.host_only:
        timings['verilate']=run(['verilator','--binary','--timing','-Wno-fatal',
          '--top-module','v28_tb_layered_firmware','--Mdir','build/v28_firmware_verilator',
          '-CFLAGS','-std=c++20','softcore/vendor/picorv32.v','softcore/robot_nav_soc.v',
          'softcore/tests/v28_tb_layered_firmware.sv'],'v28_firmware_compile.log')
        timings['real_rv32']=run(['build/v28_firmware_verilator/Vv28_tb_layered_firmware'],
                                 'v28_firmware_rv32_final.log')
    source={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(FW.iterdir()) if p.suffix in ('.c','.h','.S','.ld') or p.name=='Makefile'}
    record={'passed':True,'hardware_tested':False,'host_only':args.host_only,
            'wall_seconds':timings,'source_sha256':source,
            'firmware_hex_sha256':hashlib.sha256((FW/'firmware.hex').read_bytes()).hexdigest()}
    if not args.host_only:record['actual_rv32']=json.loads((REPORTS/'v28_firmware_rv32.json').read_text())
    (REPORTS/'v28_firmware_tests.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps({k:v for k,v in record.items() if k!='source_sha256'},indent=2))

if __name__=='__main__':main()
