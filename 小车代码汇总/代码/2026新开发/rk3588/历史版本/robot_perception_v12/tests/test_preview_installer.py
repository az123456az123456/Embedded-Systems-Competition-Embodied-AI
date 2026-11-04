"""Update and rollback in isolated temporary directories; no board mutation."""
from pathlib import Path
import hashlib
import os
import shutil
import subprocess
import tempfile
app = Path(__file__).resolve().parents[1]
def sums(package):
    (package/'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in sorted(package.iterdir()) if p.name!='SHA256SUMS'))
with tempfile.TemporaryDirectory(prefix='preview-v8-install-') as directory:
    root=Path(directory);package=root/'release';dash=root/'board app';fake=root/'tools'
    for path in (package,dash,fake):path.mkdir()
    (fake/'uname').write_text('#!/bin/sh\necho aarch64\n');(fake/'uname').chmod(0o755)
    for source,name in ((app/'scripts/install-preview-v8.sh','install.sh'),(app/'scripts/rollback-preview-v8.sh','rollback.sh'),(app/'build/perception-dashboard.aarch64','perception-dashboard.aarch64')):
        shutil.copy2(source,package/name)
    sums(package)
    (dash/'perception-dashboard.aarch64').write_bytes(b'previous working executable');(dash/'perception-dashboard.aarch64').chmod(0o755)
    preserved={name:('preserve '+name).encode() for name in ('run-perception.sh','run-normal.sh','run-rotate90.sh','model.rknn','librknnrt.so','weston.ini')}
    for name,data in preserved.items():(dash/name).write_bytes(data)
    env=dict(os.environ,DASHBOARD_DIR=str(dash),PATH=str(fake)+':'+os.environ['PATH'])
    def run(name):return subprocess.run(['sh',str(package/name)],env=env,capture_output=True,text=True)
    result=run('install.sh');assert result.returncode==0,result.stdout+result.stderr
    backups=list(dash.glob('preview-v8-backup.*'));assert len(backups)==1
    assert (backups[0]/'perception-dashboard.aarch64').read_bytes()==b'previous working executable'
    assert (dash/'perception-dashboard.aarch64').read_bytes()==(package/'perception-dashboard.aarch64').read_bytes()
    for name,data in preserved.items():assert (dash/name).read_bytes()==data
    result=run('rollback.sh');assert result.returncode==0,result.stdout+result.stderr
    assert (dash/'perception-dashboard.aarch64').read_bytes()==b'previous working executable'
    print('PASS binary-only atomic install, backup, permissions, launcher/model/rotation/library preserved, rollback')
    (package/'perception-dashboard.aarch64').write_bytes(b'bad checksum')
    result=run('install.sh');assert result.returncode!=0
    assert list(dash.glob('preview-v8-backup.*'))==backups
    assert (dash/'perception-dashboard.aarch64').read_bytes()==b'previous working executable'
    print('PASS checksum error rejected before backup or replacement')
    sums(package);shutil.copy2(shutil.which('sleep'),dash/'perception-dashboard.aarch64')
    before=(dash/'perception-dashboard.aarch64').read_bytes()
    process=subprocess.Popen([str(dash/'perception-dashboard.aarch64'),'30'])
    try:
        for script in ('install.sh','rollback.sh'):
            result=run(script);assert result.returncode!=0 and 'Close the YOLO/IMU window' in result.stderr
            assert (dash/'perception-dashboard.aarch64').read_bytes()==before
        assert list(dash.glob('preview-v8-backup.*'))==backups
    finally:process.terminate();process.wait(timeout=5)
    print('PASS install and rollback refuse a running application')
    (backups[0]/'perception-dashboard.aarch64').write_bytes(b'corrupt backup')
    result=run('rollback.sh');assert result.returncode!=0
    assert (dash/'perception-dashboard.aarch64').read_bytes()==before
    print('PASS corrupt backup rejected without replacing current executable')
