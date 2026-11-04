#!/usr/bin/env python3
"""Actual DVP -> preprocessing -> typed RGB FIFO RTL, independent C++ golden."""
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
RTL=['preprocess/bayer_demosaic.v','preprocess/rgb_bilinear_resize.v',
     'preprocess/rgb_letterbox.v','preprocess/rgb_capture_bridge.v']
FILES=RTL+['preprocess/resize_x_q11.hex','tests/preprocess/test_capture_bridge.cpp',
           'tests/preprocess/run_capture_bridge_tests.py']
def hashes():return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in FILES}
before=hashes()
env=dict(os.environ,CCACHE_DISABLE='1',CCACHE_TEMPDIR='/tmp')
commands=[
 ('preprocess-capture-lint.log',['verilator','--lint-only','-Wno-fatal','--top-module','rgb_capture_bridge',*RTL]),
 ('preprocess-capture-compile.log',['verilator','--cc','--exe','--build','-j','2','-Wno-fatal','--top-module',
  'rgb_capture_bridge','--Mdir','build/preprocess-capture','-CFLAGS','-O2 -std=c++17',*RTL,
  str(ROOT/'tests/preprocess/test_capture_bridge.cpp')]),
 ('preprocess-capture-test.log',['./build/preprocess-capture/Vrgb_capture_bridge']),
 ('capture-yosys-gw2a.log',['yosys','-Q','-T','-p','read_verilog '+' '.join(RTL)+'; '
  'hierarchy -top rgb_capture_bridge; proc; opt; memory_dff; memory_collect; '
  'setattr -unset syn_romstyle -unset syn_ramstyle; synth_gowin -family gw2a -top rgb_capture_bridge -noiopads; stat'])]
results=[]
for log,command in commands:
    start=time.monotonic()
    with (BUILD/log).open('w') as output:
        result=subprocess.run(command,cwd=ROOT,env=env,stdout=output,stderr=subprocess.STDOUT,timeout=180)
    results.append({'command':command,'log':log,'exit_code':result.returncode,'elapsed_s':round(time.monotonic()-start,3)})
    if result.returncode:break
after=hashes()
text=(BUILD/'preprocess-capture-test.log').read_text() if (BUILD/'preprocess-capture-test.log').exists() else ''
mapping=(BUILD/'capture-yosys-gw2a.log').read_text() if (BUILD/'capture-yosys-gw2a.log').exists() else ''
lint=(BUILD/'preprocess-capture-lint.log').read_text()
warning_types=sorted(set(re.findall(r'%Warning-([^:]+):',lint)))
match=re.search(r'checks=(\d+)',text)
report={'status':'PASS' if len(results)==len(commands) and all(c['exit_code']==0 for c in results) and before==after else 'FAIL',
 'source_unchanged_during_tests':before==after,'source_sha256':after,'checks':results,
 'rtl_assertions':int(match[1]) if match else None,
 'coverage':['Two full-size DVP Bayer gradient/checker frames, all 640x640 RGB tensor bytes compared with independent reflect101 demosaic and Q11 golden',
  'Source sequence/stamp precede RGB, correct EOF sequence and exact pixel count required',
  'VSYNC truncation aborts old frame and immediately next healthy frame succeeds',
  'VSYNC coincident with a complete final HREF falling edge is legal',
  'Short and long HREF abort without fabricating a complete tensor',
  'Almost-full FIFO during continuing DVP input aborts frame and later recovers',
  'One bad source frame increments dropped_frames only once despite pipeline abort propagation',
  'New epoch update strobe discards partial old frame, resets source sequence, waits for next physical VSYNC',
  'Same-epoch update strobe during a complete frame does not interrupt it',
  'Registered FIFO write observed on the next clock edge, matching actual receiving FIFO'],
 'gray_timestamp_review':{'source_clock_mhz':125,'source_update_period_ms':1,
  'source_gray_registered':True,'destination':'camera pclk, two synchronizer flops then Gray decode at raw SOF',
  'scope':'Functional simulation checks stamp/frame association; not analog metastability proof. Source Gray registration independently inspected in camera/ov5640x2_udp_rgmii.v. Update can quantize source capture stamp by one millisecond.'},
 'lint_warning_types':warning_types,
 'lint_note':'TIMESCALEMOD is a simulation time-unit declaration mismatch; all tested logic is edge-triggered and has no delay operators.',
 'mapping_cells_last_stat':dict((name,int(count)) for count,name in re.findall(r'^\s+(\d+)\s+(SPX9|SDPX9B|MULT18X18|MULT9X9|LUT4|DFFE|DFFR|DFFRE)\s*$',mapping,re.M)),
 'mapping_note':'Independent experimental Yosys GW2A synthesis after removing vendor memory style attributes for Yosys. Official integrated Gowin P&R is authoritative.',
 'hardware_speed_measured':False}
(BUILD/'capture-bridge-verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('checks','source_sha256')},indent=2),flush=True)
raise SystemExit(report['status']!='PASS')
