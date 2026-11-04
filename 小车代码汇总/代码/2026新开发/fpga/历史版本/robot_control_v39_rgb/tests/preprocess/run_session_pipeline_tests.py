#!/usr/bin/env python3
"""Registered coherent-session pipeline; functional control test, not STA."""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
RTL = ['preprocess/bayer_demosaic.v', 'preprocess/rgb_bilinear_resize.v',
       'preprocess/rgb_letterbox.v', 'preprocess/rgb_capture_bridge.v']
FILES = RTL + ['tests/preprocess/tb_session_pipeline.sv',
               'tests/preprocess/run_session_pipeline_tests.py',
               'preprocess/resize_x_q11.hex']
BUILD = ROOT / 'build'
BUILD.mkdir(exist_ok=True)
def hashes():
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in FILES}
before = hashes()
commands = [
    ['iverilog', '-g2012', '-s', 'tb_session_pipeline', '-o',
     'build/session-pipeline-directed.vvp', *RTL, 'tests/preprocess/tb_session_pipeline.sv'],
    ['vvp', 'build/session-pipeline-directed.vvp'],
]
results = []
for label, command in zip(['compile', 'test'], commands):
    log = BUILD / f'session-pipeline-{label}.log'
    with log.open('w') as output:
        result = subprocess.run(command, cwd=ROOT, stdout=output,
                                stderr=subprocess.STDOUT, timeout=30)
    results.append({'command': command, 'exit_code': result.returncode,
                    'log': str(log.relative_to(ROOT)),
                    'log_sha256': hashlib.sha256(log.read_bytes()).hexdigest()})
    if result.returncode:
        break
text = (BUILD/'session-pipeline-test.log').read_text()
found = re.search(r'PASS registered session pipeline checks=(\d+) changes=(\d+)', text)
after = hashes()
okay = len(results)==2 and all(item['exit_code']==0 for item in results) and before==after and bool(found)
report = {'status':'PASS' if okay else 'FAIL', 'result':'PASS' if okay else 'FAIL',
          'source_sha256':after, 'source_unchanged_during_test':before==after,
          'checks':int(found[1]) if found else 0, 'accepted_unique_changes':int(found[2]) if found else 0,
          'commands':results,
          'coverage':['mailbox held changes without valid cannot affect active epoch',
                      'sample, local comparison pulse and ISP apply use separate destination-clock edges',
                      'continuous same-epoch updates never repeat abort or reset source sequence',
                      'back-to-back different epochs apply in order',
                      'A/A/B/A preserves the true return to A and suppresses only the duplicate',
                      'reset after sampling clears the pending request',
                      'session change coincident with source SOF aborts it and waits for next physical VSYNC',
                      'next healthy physical SOF recovers without relabeling old pixels'],
          'scope':'Real bridge and ISP RTL, functional directed session control test. Not an analog CDC proof, hardware test or STA signoff; full tensors/RAW40 are independently checked by other runners.',
          'hardware_tested':False}
p = BUILD/'session-pipeline-verification.json'
p.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({'status':report['status'],'checks':report['checks'],'report':str(p)}))
raise SystemExit(not okay)
