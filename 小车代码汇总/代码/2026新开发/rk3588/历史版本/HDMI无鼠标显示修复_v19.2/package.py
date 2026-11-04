#!/usr/bin/env python3
"""Build the small, single-binary runtime update and test the extracted archive."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parent
NAME = 'robot-hdmi-display-fix-v19.2-20261025'
EXPECTED_PAYLOAD_SHA = 'f382e95b982ce4b9501e4e08d2b077a8b7f689e14438877c7eea204de06924fb'
payload = ROOT / 'dashboard/build/perception-dashboard.aarch64'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(payload) == EXPECTED_PAYLOAD_SHA, 'The audited payload has changed.'
assert (ROOT / '使用说明.txt').is_file(), 'Write the installation guide first.'
report = json.loads((ROOT / 'reports/installer-tests.json').read_text())
assert report['result'] == 'PASS' and report['payload_sha256'] == EXPECTED_PAYLOAD_SHA
dest = ROOT / 'dist'
dest.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix='package-hdmi-display-') as temporary:
    stage = Path(temporary) / NAME
    stage.mkdir()
    for name in ('common.sh', 'install.sh', 'rollback.sh', 'diagnose-display.sh', '使用说明.txt'):
        shutil.copy2(ROOT / name, stage / name)
    shutil.copy2(payload, stage / payload.name)
    for name in ('common.sh', 'install.sh', 'rollback.sh', 'diagnose-display.sh', payload.name):
        (stage / name).chmod(0o755)
    (stage / 'dashboard.sha256').write_text(EXPECTED_PAYLOAD_SHA + '\n')
    (stage / 'reports').mkdir()
    shutil.copy2(ROOT / 'reports/installer-tests.json', stage / 'reports/installer-tests.json')
    (stage / 'licenses').mkdir()
    for source, name in (
        ('dashboard/assets/FONT-LICENSE', 'FONT-LICENSE'),
        ('dashboard/vendor/jpeg_turbo/LICENSE.md', 'libjpeg-turbo-LICENSE.md'),
    ):
        shutil.copy2(ROOT / source, stage / 'licenses' / name)
    workspace = ROOT.parents[1]
    shutil.copy2(workspace / 'outputs/01a13005-8bda-7393-95ca-5157476f342f/acg138-rk-full-v19.1-20261025/update/THIRD_PARTY_NOTICES.txt',
                 stage / 'THIRD_PARTY_NOTICES.txt')
    shutil.copy2(ROOT.parent / 'robot_industrial_v19/calibration/LICENSE-picojson.txt', stage / 'licenses/LICENSE-picojson.txt')
    for stem in ('xdg-output', 'xdg-shell'):
        source = (ROOT / f'dashboard/vendor/{stem}-protocol.c').read_text()
        notice = re.search(r'/\*\n \* Copyright.*?\*/', source, re.S)
        assert notice, f'Missing license in {stem}'
        (stage / 'licenses' / (stem + '-LICENSE.txt')).write_text(notice.group(0) + '\n')
    entries = sorted(path for path in stage.rglob('*') if path.is_file())
    (stage / 'SHA256SUMS').write_text(''.join(f'{sha(path)}  {path.relative_to(stage)}\n' for path in entries))
    archive = dest / (NAME + '.tar.gz')
    with tarfile.open(archive, 'w:gz') as tar:
        tar.add(stage, arcname=NAME)
    unpack = Path(temporary) / 'unpacked'
    unpack.mkdir()
    with tarfile.open(archive) as tar:
        assert all(not Path(member.name).is_absolute() and '..' not in Path(member.name).parts
                   and not member.issym() and not member.islnk() for member in tar.getmembers())
        tar.extractall(unpack, filter='data')
    subprocess.run(['python3', str(ROOT / 'tests/test_installer.py'), '--package', str(unpack / NAME),
                    '--report', str(ROOT / 'reports/packaged-installer-tests.json')], check=True)
    result = {'result': 'PASS', 'archive': archive.name, 'sha256': sha(archive),
              'bytes': archive.stat().st_size, 'payload_sha256': EXPECTED_PAYLOAD_SHA,
              'actual_archive_checks': json.loads((ROOT / 'reports/packaged-installer-tests.json').read_text())['checks'],
              'hardware_tested': False, 'arm_code_executed_on_host': False}
    (dest / (NAME + '.sha256')).write_text(f'{sha(archive)}  {archive.name}\n')
    (ROOT / 'reports/package-verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
