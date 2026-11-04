#!/usr/bin/env python3
"""Apply an evidence-hashed, stage-specific overlay to canonical board sources."""
from pathlib import Path
import hashlib,json,shutil,tarfile,time
r=Path(__file__).resolve().parent
target=Path('/opt/acg138_ws/src/acg138_fpga_map_bridge')
m=json.loads((r/'overlay-manifest.json').read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
for name,entry in m['files'].items():
    if Path(name).is_absolute() or '..' in Path(name).parts:raise SystemExit('Bad manifest path')
    if sha(target/name)!=entry['before']:raise SystemExit('Board baseline differs: '+name)
    if sha(r/'overlay'/name)!=entry['after']:raise SystemExit('Delivery corrupted: '+name)
backup=target.parent.parent/('nav3-before-stage'+str(m['stage'])+'-'+str(time.time_ns())+'.tar.gz')
with tarfile.open(backup,'x:gz') as t:t.add(target,arcname=target.name)
for name,entry in m['files'].items():
    dst=target/name;dst.parent.mkdir(parents=True,exist_ok=True)
    temp=dst.with_name(dst.name+'.nav3-new')
    if temp.exists():raise SystemExit('Unfinished previous write: '+str(temp))
    shutil.copyfile(r/'overlay'/name,temp);temp.replace(dst)
record={'stage':m['stage'],'backup':str(backup),'files':m['files'],
        'target':str(target),'verified':all(sha(target/n)==e['after'] for n,e in m['files'].items())}
(r/'apply-record.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'stage':m['stage'],'backup':str(backup),'verified':record['verified'],'changed_files':list(m['files'])}))
