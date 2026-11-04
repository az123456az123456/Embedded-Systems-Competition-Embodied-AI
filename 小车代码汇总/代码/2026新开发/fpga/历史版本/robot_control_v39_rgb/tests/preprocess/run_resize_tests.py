#!/usr/bin/env python3
"""Reproducible actual-RTL resize/padding regression; no board speed claim."""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess
import time

ROOT=Path(__file__).resolve().parents[2]
BUILD=ROOT/'build'
BUILD.mkdir(exist_ok=True)
FILES=['preprocess/rgb_bilinear_resize.v','preprocess/rgb_letterbox.v',
       'preprocess/generate_resize_coeffs.py','preprocess/resize_x_q11.hex',
       'preprocess/resize_coefficients.json','tests/preprocess/resize_top.sv',
       'tests/preprocess/test_resize_stream.cpp','tests/preprocess/compare_resize_reference.py',
       'tests/preprocess/run_resize_tests.py']
def hashes():return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in FILES}
before=hashes()
env=dict(os.environ,CCACHE_DISABLE='1',CCACHE_TEMPDIR='/tmp')
commands=[
    ('resize-coefficients.log',['python3','preprocess/generate_resize_coeffs.py']),
    ('preprocess-resize-compile.log',['verilator','--cc','--exe','--build','-j','2','-Wno-fatal',
     '--top-module','resize_top','--Mdir','build/preprocess-resize','-CFLAGS','-O2 -std=c++17',
     'preprocess/rgb_bilinear_resize.v','preprocess/rgb_letterbox.v','tests/preprocess/resize_top.sv',
     str(ROOT/'tests/preprocess/test_resize_stream.cpp')]),
    ('preprocess-resize-test.log',['./build/preprocess-resize/Vresize_top']),
    ('resize-reference.log',['python3','tests/preprocess/compare_resize_reference.py']),
    ('resize-yosys-gw2a.log',['yosys','-Q','-T','-p',
     'read_verilog preprocess/rgb_bilinear_resize.v preprocess/rgb_letterbox.v; '
     'hierarchy -top rgb_bilinear_resize; proc; opt; memory_dff; memory_collect; '
     'setattr -unset syn_romstyle -unset syn_ramstyle; '
     'synth_gowin -family gw2a -top rgb_bilinear_resize -noiopads; stat']),
]
checks=[]
for log,command in commands:
    start=time.monotonic()
    with (BUILD/log).open('w') as output:
        result=subprocess.run(command,cwd=ROOT,env=env,stdout=output,stderr=subprocess.STDOUT,timeout=180)
    checks.append({'command':command,'log':log,'exit_code':result.returncode,'elapsed_s':round(time.monotonic()-start,3)})
    if result.returncode:break
after=hashes()
text=(BUILD/'preprocess-resize-test.log').read_text() if (BUILD/'preprocess-resize-test.log').exists() else ''
mapping=(BUILD/'resize-yosys-gw2a.log').read_text() if (BUILD/'resize-yosys-gw2a.log').exists() else ''
match=re.search(r'checks=(\d+)',text)
peak=re.search(r'30 full .*fifo_peak=(\d+)',text)
report={'status':'PASS' if len(checks)==len(commands) and all(c['exit_code']==0 for c in checks) and before==after else 'FAIL',
        'source_unchanged_during_tests':before==after,'source_sha256':after,'checks':checks,
        'rtl_assertions':int(match[1]) if match else None,'fifo_peak_continuous':int(peak[1]) if peak else None,
        'coverage':['30 full frames continuous valid without Hblank or inter-frame gaps','3 complete frames with valid bubbles',
                    'RGB flat/ramp/checkerboard/random/edge impulse/step inputs','Every tensor output byte and coordinate checked against independent integer reference',
                    'Explicit abort, missing pixel, unexpected SOF interruption and recovery','Deliberate padding-FIFO overflow abort and subsequent recovery',
                    'Fixed Q11 versus OpenCV full-size reference PSNR'],
        'mapping_note':'Independent experimental Yosys GW2A mapping, vendor style attributes removed for Yosys only. Official Gowin integrated P&R remains authoritative.',
        'mapping_cells_last_stat':dict((name,int(count)) for count,name in re.findall(r'^\s+(\d+)\s+(SPX9|MULT18X18|MULT9X9|LUT4|DFFE|DFFR|DFFRE)\s*$',mapping,re.M)),
        'hardware_speed_measured':False,'minimum_input_clock_mhz_for_30fps_active_RGB':960*1080*30/1e6,
        'pixel_stream_backpressure':False,'tensor_payload_bps_two_eyes_30fps':640*640*3*8*30*2}
(BUILD/'resize-verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('checks','source_sha256')},indent=2),flush=True)
raise SystemExit(report['status']!='PASS')
