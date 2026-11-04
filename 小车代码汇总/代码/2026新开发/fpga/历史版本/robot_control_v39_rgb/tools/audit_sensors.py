#!/usr/bin/env python3
"""v15 source/bitstream regression summary; run after tools/build_softcore.sh."""
import json,hashlib
from pathlib import Path
root=Path(__file__).resolve().parents[1]
r=json.loads((root/'reports/release_verification.json').read_text())
for f,h in {**r['synthesis_inputs'],**r['auxiliary_inputs'],r['bitstream']:r['bitstream_sha256']}.items():
 assert hashlib.sha256((root/f).read_bytes()).hexdigest()==h,f
logs=['encoder-sync-unit.log','encoder-motor-integration.log','motor-regressions.log','preserved-regressions.log']
result={}
for n in logs:
 t=(root/'reports'/n).read_text()
 assert 'PASS' in t and not any(s in t for s in ['FATAL:','AssertionError','make: ***']),n
 result[n]=[l for l in t.splitlines()if l.startswith('PASS')]
(root/'reports/regression_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print('PASS v15 source/bitstream hashes and encoder + compatibility regressions')
