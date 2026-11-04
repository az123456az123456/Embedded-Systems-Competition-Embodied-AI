#!/usr/bin/env python3
"""Exercise real async RGB FIFO FWFT, wrap, full/empty, clock ratios and reset."""
from pathlib import Path
import hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build/rgb-fifo-tests';OUT.mkdir(parents=True,exist_ok=True)
FILES=['preprocess/rgb_stream_fifo.v','tests/preprocess/tb_rgb_stream_fifo.sv','tests/preprocess/run_rgb_fifo_tests.py']
def hashes():return {n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}
before=hashes();results=[]
for aw in (6,11):
 target=OUT/('fifo-'+str(aw))
 subprocess.run(['iverilog','-g2012','-s','tb_rgb_stream_fifo','-Ptb_rgb_stream_fifo.AW='+str(aw),'-o',str(target),str(ROOT/FILES[0]),str(ROOT/FILES[1])],check=True)
 run=subprocess.run(['vvp',str(target)],capture_output=True,text=True,check=True)
 assert 'PASS' in run.stdout
 (OUT/('fifo-'+str(aw)+'.log')).write_text(run.stdout+run.stderr)
 results.append({'address_width':aw,'capacity_words':1<<aw,'result':'PASS','log':run.stdout.strip()})
assert before==hashes(),'FIFO sources changed while testing'
report={'status':'PASS','result':'PASS','hardware_tested':False,'source_sha256':before,'cases':results,
 'checked':['FWFT against independent randomized write scoreboard','full and empty read/write rejection','pointer wrap at two capacities','write-fast and read-fast unrelated clock ratios','common asynchronous reset with unread data','registered next-address and conservative occupancy invariants'],
 'scope':'RTL Icarus four-state simulation. Does not replace real CDC STA or metastability analysis.'}
(OUT/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
