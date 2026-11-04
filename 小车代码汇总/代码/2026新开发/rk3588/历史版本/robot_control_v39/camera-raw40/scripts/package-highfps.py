#!/usr/bin/env python3
"""Build a small reversible receiver update; no model or system library bundled."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[1]
NAME = 'camera-fastpath-40fps-rk3588-v20-20261025'
DIST = ROOT / 'dist'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def manifest(path):
    (path / 'SHA256SUMS').write_text(''.join(f'{sha(p)}  {p.relative_to(path)}\n' for p in sorted(path.rglob('*'))
                                          if p.is_file() and p.name != 'SHA256SUMS'))


def main():
    binary = ROOT / 'build/isp-probe'
    elf = subprocess.check_output(['readelf', '-h', str(binary)], text=True)
    assert 'AArch64' in elf
    tests = json.loads((ROOT / 'reports/highfps-installer-verification.json').read_text())
    assert tests['status'] == 'PASS' and tests['receiver_sha256'] == sha(binary)
    package = DIST / NAME
    if package.exists():
        shutil.rmtree(package)
    package.mkdir(parents=True)
    for name in ['install-highfps.sh', 'rollback-highfps.sh', 'highfps-common.sh', 'patch-fps-launcher.awk', 'diagnose-fps.sh']:
        copy(ROOT / 'scripts' / name, package / name)
    copy(binary, package / 'isp-probe')
    copy(ROOT / 'README.md', package / '使用说明.txt')
    report = {
        'date': '2026-10-25', 'status': 'PASS_OFFLINE', 'hardware_tested': False,
        'target': '40FPS engineering trial, matching robot_navigation_v20 FPGA required',
        'receiver_default_fps': 40, 'accepted_nominal_fps': [15, 20, 30, 35, 40],
        'frame_contract': '1920x1080 RAW8 input, 1920x1080 NV12 output; one output per completed ISP frame',
        'receiver_sha256': sha(binary), 'retained_fakecam_sha256': tests['retained_fakecam_sha256'],
        'compiler': 'BSP aarch64-none-linux-gnu GCC10.3-2021.07; -O2 -std=gnu11 -Wall -Wextra -Werror',
        'needed': ['librkaiq.so', 'libm.so.6', 'libpthread.so.0', 'libc.so.6'],
        'checks': {'installer': tests, 'receiver': 'PASS full 1920x1080 local datagrams, bounded latest-complete buffering, slow consumer, missing/reordered/truncated rows, stop/join',
                   'fps': 'PASS source/build defaults, runtime15/20/30/35/40, real S_PARM negotiation, no synthetic repeats, invalid-rate pre-mutation guard',
                   'shell': 'PASS sh -n'},
        'preserved': ['custom 13-class YOLOv8s INT8 model, labels and application', 'phone pairing and motor preferences',
                      'navigation parameters and encoder mapping', 'isolated FakeCam library and IQ', 'OS, kernel, Wi-Fi driver and system RKAIQ'],
        'limitations': ['Actual RAW/ISP/capture/inference FPS and image integrity require board validation',
                        '35 is only a supported nominal interval; no35FPS sensor image is supplied',
                        'Nominal30 metadata alone does not restore a40FPS sensor: restore original FPGA image for genuine30FPS rollback',
                        'Legacy RAW datagrams have row numbers but no frame identifiers; not all cross-frame mismatches are detectable',
                        'The retained FakeCam offline exposure descriptor is25FPS; this is not a RAW input frame-rate limiter']}
    (package / 'verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    for directory in ['src', 'tests', 'scripts', 'patches', 'reports']:
        for p in sorted((ROOT / directory).rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts:
                copy(p, package / 'source' / p.relative_to(ROOT))
    for name in ['Makefile', 'run_isp_camera.sh', 'README.md']:
        copy(ROOT / name, package / 'source' / name)
    for p in package.iterdir():
        if p.suffix == '.sh' or p.name == 'isp-probe':
            p.chmod(0o755)
    manifest(package)
    archive = DIST / (NAME + '.tar.gz')
    with tarfile.open(archive, 'w:gz') as output:
        output.add(package, arcname=NAME)
    (DIST / (NAME + '.SHA256SUMS')).write_text(f'{sha(archive)}  {archive.name}\n')
    print(json.dumps({'archive': str(archive), 'bytes': archive.stat().st_size,
                      'sha256': sha(archive), 'receiver_sha256': sha(binary)}, indent=2))


if __name__ == '__main__':
    main()
