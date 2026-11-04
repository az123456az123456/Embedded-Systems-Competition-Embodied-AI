#!/usr/bin/env python3
"""Actual production CLI, in-memory100-frame endpoint, no physical/network IO."""
from pathlib import Path
import subprocess

exe=Path(__file__).resolve().parent/'test-monitor-runtime'
for mode,expected in [('trap',3),('stalled',3),('healthy',0),('none',2)]:
    r=subprocess.run([str(exe),mode],capture_output=True,text=True,timeout=3)
    assert r.returncode==expected,(mode,r.returncode,r.stdout,r.stderr)
    if mode!='none':
        assert 'valid=100' in r.stdout,(mode,r.stdout)
        assert 'no data' not in r.stderr.lower() and 'no valid' not in r.stderr.lower(),r.stderr
    if mode=='trap':
        assert 'hardware_trap=yes' in r.stderr and 'heartbeat=0' in r.stderr and 'hardware CPU trap detected' in r.stderr,r.stderr
    if mode=='stalled':assert 'heartbeat is not advancing' in r.stderr,r.stderr
    if mode=='none':assert 'No valid NAV1 data received' in r.stderr,r.stderr
print('PASS actual monitor main:100 valid trap frames,100 nonadvancing frames,100 healthy frames, true no-data; no hardware')
