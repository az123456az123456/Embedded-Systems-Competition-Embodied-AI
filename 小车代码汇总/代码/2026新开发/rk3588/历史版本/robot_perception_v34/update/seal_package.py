#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,sys
PAYLOAD_FILES=tuple('payload/'+n for n in ('dashboard','navigation','model','labels','manifest'))
RUNTIME_FILES=('install.sh','rollback.sh','v34-common.sh','transform-launcher.awk','baseline-pairs.sha256','使用说明.txt','benchmark-perception-v34.sh','compare-performance.py','summarize-perception-v34.py','diagnose-performance.sh',*PAYLOAD_FILES)
def seal(root):
 root=Path(root);sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
 m=json.loads((root/'payload/manifest').read_text());assert sha(root/'payload/model')==m['model_sha256'] and sha(root/'payload/labels')==m['labels_sha256']
 for r in ('dashboard','navigation'):
  raw=(root/'payload'/r).read_bytes();assert raw[:6]==b'\x7fELF\x02\x01' and raw[18:20]==b'\xb7\0'
 (root/'SHA256SUMS').write_text(''.join(f'{sha(root/n)}  {n}\n' for n in RUNTIME_FILES))
if __name__=='__main__':seal(sys.argv[1])
