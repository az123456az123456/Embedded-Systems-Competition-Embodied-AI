#!/usr/bin/env python3
"""Add stage0 to canonical chroot source, with a complete pre-change backup.
Run inside the RK chroot. No runtime deploy, network, ROS launch or process kill.
"""
from pathlib import Path
import datetime
import hashlib
import json
import shutil
import tarfile

root=Path(__file__).resolve().parent
target=Path('/opt/acg138_ws/src/acg138_fpga_map_bridge')
if not target.is_dir():
    raise SystemExit('Canonical board package missing; no changes made')
cmake=target/'CMakeLists.txt'
before=cmake.read_text()
marker='include(cmake/Nav3Stage0.cmake)'
if marker in before:
    raise SystemExit('Stage0 already applied; inspect before reapplying')
if 'add_library(acg138_map_core ' not in before or before.count('ament_package()')!=1:
    raise SystemExit('Unexpected board CMake structure; inspect before patching')
files=[p for p in (root/'package').rglob('*') if p.is_file() and p.name!='CMakeLists.txt']
for p in files:
    dest=target/p.relative_to(root/'package')
    if dest.exists():
        raise SystemExit('Refusing to overwrite existing file: '+str(dest))
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
backup=target.parent.parent/('nav3-stage0-backup-'+stamp+'.tar.gz')
with tarfile.open(backup,'x:gz') as archive:
    archive.add(target,arcname=target.name)
pre_hashes={str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in target.rglob('*') if p.is_file()}
for p in files:
    dest=target/p.relative_to(root/'package')
    dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(p,dest)
cmake.write_text(before.replace('ament_package()',marker+'\nament_package()'))
after_hashes={str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in target.rglob('*') if p.is_file()}
changed=[p for p,h in pre_hashes.items() if after_hashes.get(p)!=h]
assert changed==['CMakeLists.txt'],changed
record={'canonical_source':str(target),'backup':str(backup),'pre_hashes':pre_hashes,
        'post_hashes':after_hashes,'changed_existing':changed,
        'added':sorted(set(after_hashes)-set(pre_hashes)),
        'nav2_unchanged':all(after_hashes[p]==h for p,h in pre_hashes.items() if 'nav2_wire.' in p),
        'bridge_node_unchanged':pre_hashes['src/bridge_node.cpp']==after_hashes['src/bridge_node.cpp']}
(root/'install-record.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'backup':str(backup),'changed_existing':changed,
                  'added':record['added'],'nav2_unchanged':record['nav2_unchanged']}))
