#!/usr/bin/env python3
"""Package only the audited v20 build, RK receiver update and v18 rollback."""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[3]
PROJECT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/01a13005-8bda-7393-95ca-5157476f342f'
NAME = 'acg138-camera40-timing-v20-20261025'


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    report = json.loads((PROJECT / 'reports/release_verification.json').read_text())
    fs = PROJECT / report['bitstream']
    assert sha(fs) == report['bitstream_sha256']
    assert report['timing']['setup_violated_endpoints'] == 0
    assert report['timing']['hold_violated_endpoints'] == 0
    assert report['timing']['setup_minimum_slack_ns'] > 0.040
    gray = json.loads((PROJECT / 'reports/gray-cdc-verification.json').read_text())
    assert gray['status'] == 'PASS' and gray['paths_verified'] == 88
    assert gray['bitstream_sha256'] == sha(fs)
    ram = json.loads((PROJECT / 'reports/ram_init_verification.json').read_text())
    assert gray['netlist_sha256'] == ram['netlist_sha256']
    rk = ROOT / 'rk3588/camera-fastpath-highfps/dist/camera-fastpath-40fps-rk3588-v20-20261025.tar.gz'
    rollback = ROOT / 'fpga/robot_navigation_v18/impl/pnr/robot_navigation_v18.fs'
    assert sha(rollback) == '8d9ddd806258445b09c5c2d30581cc9348e0d6b1f526675fa4bb43bb6aee15e5'
    dest = OUT / NAME
    assert not dest.exists(), f'Package already exists: {dest}'
    dest.mkdir(parents=True)
    shutil.copytree(PROJECT, dest / 'fpga/robot_navigation_v20',
                    ignore=shutil.ignore_patterns('build', '__pycache__'))
    shutil.copy2(fs, dest / 'robot_navigation_v20.fs')
    shutil.copy2(rollback, dest / 'robot_navigation_v18_30fps_rollback.fs')
    shutil.copy2(rk, dest / rk.name)
    shutil.copy2(PROJECT / 'docs/安装与验收.txt', dest / '安装与验收.txt')
    shutil.copy2(ROOT / 'rk3588/camera-fastpath-highfps/dist/camera-fastpath-40fps-rk3588-v20-20261025/使用说明.txt',
                 dest / 'RK详细说明.txt')
    files = sorted(p for p in dest.rglob('*') if p.is_file())
    manifest = {str(p.relative_to(dest)): sha(p) for p in files}
    (dest / 'SHA256SUMS').write_text(''.join(f'{h}  {p}\n' for p, h in manifest.items()))
    for p, h in manifest.items():
        assert sha(dest / p) == h
    archive = OUT / (NAME + '.zip')
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in sorted(dest.rglob('*')):
            if p.is_file():
                z.write(p, str(p.relative_to(OUT)))
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for p, h in manifest.items():
            assert hashlib.sha256(z.read(NAME + '/' + p)).hexdigest() == h
    result = {'status': 'PASS', 'archive': str(archive), 'sha256': sha(archive),
              'files_verified': len(manifest), 'fpga_sha256': sha(fs),
              'rk_archive_sha256': sha(rk), 'rollback_sha256': sha(rollback),
              'hardware_tested': False}
    (OUT / (NAME + '-verification.json')).write_text(json.dumps(result, indent=2) + '\n')
    (OUT / (NAME + '.SHA256SUMS')).write_text(f'{sha(archive)}  {archive.name}\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
