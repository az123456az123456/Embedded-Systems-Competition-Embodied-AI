#!/usr/bin/env python3
"""Check total64 capacity, collision-free prefetch and 4-state reset/SOF data."""
from pathlib import Path
import hashlib,json,subprocess

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'preprocess/rgb_letterbox.v'
TB=ROOT/'tests/preprocess/tb_letterbox_bram_safety.sv'
OUT=ROOT/'build/letterbox-bram-safety'
OUT.mkdir(parents=True,exist_ok=True)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
inputs={str(p.relative_to(ROOT)):sha(p) for p in (SOURCE,TB,Path(__file__))}
results=[]
for pad in (0,35):
    exe=OUT/('four-state-'+str(pad))
    subprocess.run(['iverilog','-g2012','-s','tb','-Ptb.PAD='+str(pad),'-o',str(exe),str(SOURCE),str(TB)],check=True)
    result=subprocess.run(['vvp',str(exe)],check=True,text=True,stdout=subprocess.PIPE)
    (OUT/('four-state-'+str(pad)+'.log')).write_text(result.stdout)
    results.append({'pad_left':pad,'result':result.stdout.strip()})

# Inductive transition proof over every count/ring position and legal push/pop.
# The forwarded copy does not increase count or create extra queue capacity.
trials=0
for count in range(65):
    for rd in range(64):
        wr=(rd+count)%64
        for pop in (0,1):
            if pop and count==0:continue
            for push in (0,1):
                abort=bool(count==64 and push and not pop)
                read=not abort and count>pop
                store=not abort and push and (count<64 or pop)
                forward=store and count==pop
                if abort:newcount=0;newrd=0;newwr=0;valid=False
                else:
                    newcount=count+push-pop;newrd=(rd+pop)%64;newwr=(wr+store)%64
                    valid=read or forward
                assert 0<=newcount<=64 and bool(valid)==(newcount!=0)
                assert newwr==(newrd+newcount)%64
                assert not(read and store and ((rd+pop)%64)==wr)
                trials+=1
assert inputs=={str(p.relative_to(ROOT)):sha(p) for p in (SOURCE,TB,Path(__file__))}
report={'result':'PASS','scope':'Inductive queue-state enumeration and actual RTL four-state simulation; no hardware measurement',
        'source_sha256':inputs,'state_transition_cases':trials,'four_state_simulations':results,
        'properties':['Total capacity64 includes every prefetched/forwarded pixel',
                      'head_valid iff total count is nonzero',
                      'No simultaneous RAM same-address read/write for all legal push/pop and ring positions',
                      'No uninitialized RAM leak after reset/first SOF, including PAD_LEFT0',
                      'Abort/reset/new SOF invalidate old queue and recover exact colored pixels']}
(OUT/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
