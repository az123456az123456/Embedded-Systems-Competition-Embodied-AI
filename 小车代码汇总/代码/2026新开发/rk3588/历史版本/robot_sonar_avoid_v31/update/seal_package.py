#!/usr/bin/env python3
"""Seal the fixed three-file v31-three-sonar-20261028 dashboard/navigation pairing update."""
from pathlib import Path
import argparse, hashlib, json

PAYLOAD_FILES=('payload/perception-dashboard.aarch64','payload/robot-navigation.aarch64')
RUNTIME_FILES=('common.sh','install.sh','rollback.sh','baselines.sha256','baseline-pairs.sha256','package-mode.txt','profile.json',
               '使用说明.txt','故障分类与恢复边界.txt','diagnose-phone.sh','export-wrong.sh','LICENSE-picojson.txt','payload.sha256','tools/softcore-monitor.aarch64','tools/softcore-monitor.c','tools/README.txt','tools/verification.json',*PAYLOAD_FILES)

def seal(root):
    root=Path(root);sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
    assert (root/'package-mode.txt').read_text()=='dashboard-navigation\n'
    old={line.split()[0] for line in (root/'baselines.sha256').read_text().splitlines() if line.split()[1] in ('dashboard','navigation')}
    for name in PAYLOAD_FILES:
        p=root/name;data=p.read_bytes()
        assert not p.is_symlink() and data[:6]==b'\x7fELF\x02\x01' and data[18:20]==b'\xb7\x00'
        assert sha(p) not in old
    tool=root/'tools/softcore-monitor.aarch64';raw=tool.read_bytes()
    assert not tool.is_symlink() and raw[:6]==b'\x7fELF\x02\x01' and raw[18:20]==b'\xb7\x00'
    evidence=json.loads((root/'tools/verification.json').read_text())
    assert evidence['status']=='PASS' and evidence['passive_only']
    assert evidence['arm64_sha256']==sha(tool) and evidence['source_sha256']==sha(root/'tools/softcore-monitor.c')
    (root/'payload.sha256').write_text(''.join(f'{sha(root/f)}  {f}\n' for f in PAYLOAD_FILES))
    (root/'SHA256SUMS').write_text(''.join(f'{sha(root/f)}  {f}\n' for f in RUNTIME_FILES))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);seal(p.parse_args().directory)
