#!/usr/bin/env python3
"""Verify the competition source snapshot without requiring board/SDK access."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / '源码清单.json'
BAD_EXT = {'.fs', '.apk', '.aab', '.o', '.a', '.so', '.elf', '.bin', '.deb',
           '.class', '.dex', '.jar', '.zip', '.rar', '.gz', '.7z', '.pyc', '.pyo',
           '.vcd', '.fst', '.log', '.console', '.rknn', '.rkllm', '.onnx', '.pt',
           '.pth', '.keystore', '.jks', '.pr', '.map', '.d', '.su', '.rpt', '.htmlz'}
BAD_DIR = {'.git', '.cache', '.local', '.gradle', '.pytest_cache', '__pycache__',
           'build', 'dist', 'install', 'impl', 'reports', 'outputs', 'logs', 'bin'}
SECRET = [re.compile(rb'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----'),
          re.compile(rb'(?:github_pat_|gh[pousr]_)[A-Za-z0-9_]{25,}'),
          re.compile(rb'Bearer [a-f0-9]{32,}')]


def inventory():
    errors, files = [], {}
    for p in sorted(ROOT.rglob('*')):
        rel = p.relative_to(ROOT)
        if p == MANIFEST:
            continue
        if p.is_symlink():
            errors.append(f'symlink: {rel}')
            continue
        if p.is_dir():
            if p.name in BAD_DIR:
                errors.append(f'output/cache directory: {rel}')
            continue
        if not p.is_file():
            continue
        if p.suffix.lower() in BAD_EXT or (p.suffix.lower() == '.hex' and p.name != 'ov5640_af.hex'):
            errors.append(f'output/binary extension: {rel}')
        if re.search(r'private|pair-token|credentials', p.name, re.I):
            errors.append(f'private runtime filename: {rel}')
        data = p.read_bytes()
        if b'\0' in data or data.startswith((b'\x7fELF', b'!<arch>\n', b'PK\x03\x04', b'Rar!', b'\x1f\x8b')):
            errors.append(f'binary content: {rel}')
        if any(pattern.search(data) for pattern in SECRET):
            errors.append(f'credential content (value hidden): {rel}')
        if len(data) > 10 * 1024 * 1024:
            errors.append(f'unexpected large source file: {rel}')
        files[str(rel)] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    return files, errors


def structural_checks():
    errors = []
    index = json.loads((ROOT / '版本索引.json').read_text())
    for e in index['versions']:
        d = ROOT / e['path']
        if not (d / 'README.md').is_file():
            errors.append(f'missing version README: {e["path"]}')
    for platform in ['fpga', 'rk3588', '上位机']:
        if not (ROOT / platform / 'README.md').is_file():
            errors.append(f'missing platform README: {platform}')
    current = ROOT / index['current']['fpga']
    for project in current.glob('*.gprj'):
        for item in ET.parse(project).getroot().iter('File'):
            path = item.get('path')
            if path and not (project.parent / path).is_file():
                errors.append(f'missing FPGA project input: {path}')
    for line in (current / 'build.tcl').read_text().splitlines():
        if line.startswith('add_file '):
            path = line[len('add_file '):].strip().strip('"{}')
            if not (current / path).is_file():
                errors.append(f'missing FPGA build input: {path}')
    required_entries = [
        'fpga/v54_150_70_20261104/softcore/firmware/Makefile',
        'rk3588/v143_20261104/robot-fpga-integration-20261102/perception/scripts/build-arm64.sh',
        'rk3588/v143_20261104/robot-fpga-integration-20261102/tools/build_gateway.py',
        'rk3588/v143_20261104/ros2/acg138_semantic_20261103/build-ros.sh',
        '上位机/v143_1.2.43_20261104/build-apk.sh',
    ]
    for entry in required_entries:
        if not (ROOT / entry).is_file():
            errors.append(f'missing documented build entry: {entry}')
    ns = '{http://schemas.android.com/apk/res/android}'
    app = ET.parse(ROOT / index['current']['上位机'] / 'AndroidManifest.xml').getroot()
    if (app.get(ns + 'versionName'), app.get(ns + 'versionCode')) != ('1.2.43', '56'):
        errors.append('unexpected current Android version')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--update', action='store_true', help='explicitly rewrite reviewed file inventory')
    args = parser.parse_args()
    files, errors = inventory()
    errors += structural_checks()
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    if args.update:
        MANIFEST.write_text(json.dumps({'schema': 1, 'algorithm': 'sha256',
                                       'note': 'Self-excluding source identity inventory; not a compiler output.',
                                       'files': files}, ensure_ascii=False, indent=2) + '\n')
        print(f'Updated source inventory: {len(files)} files')
        return 0
    expected = json.loads(MANIFEST.read_text())['files']
    changed = sorted(set(expected) ^ set(files))
    changed += [p for p in files.keys() & expected.keys() if files[p] != expected[p]]
    if changed:
        print('Source inventory mismatch:\n' + '\n'.join(changed[:80]), file=sys.stderr)
        return 1
    print(f'PASS source-only rules, version entries, current FPGA inputs, Android version, {len(files)} hashes')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
