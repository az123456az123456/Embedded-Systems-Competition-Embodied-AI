#!/usr/bin/env python3
"""Repeat the mapped setup-controller proof against the delivered production netlist."""
from pathlib import Path
import gzip, hashlib, json, os, subprocess, tempfile
BASE=Path(__file__).resolve().parents[1];ROOT=BASE.parents[1]
proof=json.loads((BASE/'reports/gate-verification.json').read_text())
netlist=gzip.decompress((BASE/'reports/production-netlist.vg.gz').read_bytes())
assert hashlib.sha256(netlist).hexdigest()==proof['netlist_sha256']
assert hashlib.sha256((BASE/'bitstream/raw_calibration.fs').read_bytes()).hexdigest()==proof['bitstream_sha256']
prim=Path(os.environ.get('GOWIN_IDE',str(ROOT/'Gowin_V1.9.12.03/IDE')))/'simlib/gw5a/prim_sim.v'
with tempfile.TemporaryDirectory(prefix='rawcal-gate-') as d:
    d=Path(d);(d/'production.vg').write_bytes(netlist)
    with (BASE/'reports/setup-gate-compile.log').open('w') as log:
        subprocess.run(['iverilog','-g2012','-s','tb_setup','-s','GSR','-o',str(d/'test'),str(BASE/'tests/tb_setup_gate.sv'),str(d/'production.vg'),str(prim)],stdout=log,stderr=subprocess.STDOUT,check=True)
    p=subprocess.run(['vvp',str(d/'test')],text=True,capture_output=True,check=True)
    assert 'PASS all startup/lock/readback/fault cases' in p.stdout
    (BASE/'reports/setup-gate-simulation.log').write_text(p.stdout)
    print(p.stdout)
