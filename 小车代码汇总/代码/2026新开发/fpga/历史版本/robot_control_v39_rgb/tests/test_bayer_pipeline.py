#!/usr/bin/env python3
"""Real pipelined demosaic: exact EOF/drain preemption and abort boundaries."""
from pathlib import Path
import hashlib,json,tempfile
import numpy as np
from test_bayer_demosaic import PROJECT,RTL,TB,stream,golden,run_case

def hashes():
    return {str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [RTL,TB,PROJECT/'tests/test_bayer_demosaic.py',Path(__file__).resolve()]}
before=hashes();results=[];rng=np.random.default_rng(3502)
with tempfile.TemporaryDirectory(prefix='bayer-pipeline-') as temporary:
    folder=Path(temporary);w,h=17,12
    for phase in range(4):
        a=rng.integers(0,256,(h,w),dtype=np.uint8)
        b=rng.integers(0,256,(h,w),dtype=np.uint8)
        ga,gb=golden(a,phase),golden(b,phase)
        # Final RAW at t0; W flush reads, strip/window alignment and three
        # interpolation registers produce real EOF at t0+W+5. A coincident
        # SOF must cancel that old EOF; one clock later must not abort it.
        for gap in range(w,w+7):
            old_complete=gap>=w+5
            results.append(run_case(folder,f'sof-at-final-pipeline-{phase}-{gap}',w,h,phase,
                stream(a)+[(0,0,0,0,0)]*gap+stream(b),
                [ga,gb] if old_complete else [gb],0 if old_complete else 1))
        # Explicit abort independently clears each late interpolation stage,
        # including an old would-be EOF; the following real SOF survives.
        for delay in range(w+1,w+5):
            results.append(run_case(folder,f'abort-at-final-pipeline-{phase}-{delay}',w,h,phase,
                stream(a)+[(0,0,0,0,0)]*delay+[(0,0,0,1,0)]+stream(b),[gb],1))
after=hashes();assert before==after,'source changed during pipeline regression'
report={'status':'PASS','result':'PASS','hardware_tested':False,'source_sha256':after,
        'source_unchanged_during_tests':True,'cases':results,
        'checked_pixels':sum(r['checked_pixels']for r in results),
        'coverage':['All four Bayer phases and two independent random images',
                    'SOF preemption at each final window/sum/average/output pipeline boundary',
                    'SOF coincident with would-be old EOF suppresses that old frame',
                    'SOF one clock after real EOF preserves both full frames',
                    'Explicit abort clears every late valid; following SOF succeeds without stale pixels'],
        'scope':'Real RTL and independent NumPy reflect101 reference; functional pixel/valid test, not STA.'}
out=PROJECT/'reports/bayer-pipeline-v35/drain-boundaries.json';out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'status':'PASS','cases':len(results),'checked_pixels':report['checked_pixels'],'report':str(out)}))
