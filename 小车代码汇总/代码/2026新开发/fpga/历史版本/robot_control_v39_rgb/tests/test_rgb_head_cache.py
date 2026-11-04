#!/usr/bin/env python3
"""Check four-state input caching/tag flags independently of packet golden."""
from pathlib import Path
import hashlib,json,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
RTL=ROOT/'preprocess/rgb_packet_tx.v'
TB=ROOT/'tests/tb_rgb_head_cache.sv'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    sources={str(p.relative_to(ROOT)):sha(p) for p in (RTL,TB,Path(__file__))}
    with tempfile.TemporaryDirectory(prefix='v35-head-cache-') as tmp:
        sim=Path(tmp)/'sim'
        subprocess.run(['iverilog','-g2012','-s','tb_rgb_head_cache','-o',str(sim),str(RTL),str(TB)],check=True)
        result=subprocess.run(['vvp',str(sim)],check=True,stdout=subprocess.PIPE,text=True)
    assert result.stdout.startswith('PASS '),result.stdout
    assert sources=={str(p.relative_to(ROOT)):sha(p) for p in (RTL,TB,Path(__file__))},'source changed during test'
    out=ROOT/'reports/rgb-head-cache';out.mkdir(parents=True,exist_ok=True)
    report={'result':'PASS','hardware_tested':False,'simulation':'Icarus four-state','coverage':'all 16 tags, empty source, valid hold, consume followed by refill, no same-cycle late refill, repeated reset, epoch invalidation; 1-word finite cache per eye','output':result.stdout.strip(),'source_sha256':sources}
    (out/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
