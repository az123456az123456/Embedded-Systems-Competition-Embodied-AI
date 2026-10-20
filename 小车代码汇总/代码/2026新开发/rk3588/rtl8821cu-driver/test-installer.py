#!/usr/bin/env python3
"""Exercise installation/rollback in a temporary fake board, never host /etc."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

here = Path(__file__).resolve().parent
config = Path(sys.argv[1]).resolve()
results = []
srcversion = json.loads((here/'reports/verification.json').read_text())['module']['srcversion']
with tempfile.TemporaryDirectory(prefix='rtl8821cu-install-test-') as td:
    work = Path(td)
    fake = work/'fake-board'
    pkg = work/'package'
    mock = work/'mock-bin'
    mock.mkdir()
    shutil.copytree(here/'board', pkg)
    shutil.copy2(here/'build/source/8821cu.ko', pkg/'payload/8821cu.ko')
    # Redirect all absolute system accesses in this test copy, including its init script.
    for p in pkg.rglob('*'):
        if not p.is_file() or p.suffix == '.ko':
            continue
        data = p.read_text()
        for prefix in ['/lib/modules/', '/etc/', '/proc/', '/sys/']:
            data = data.replace(prefix, str(fake)+prefix)
        p.write_text(data)
    def checksums():
        lines = [hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(pkg))
                 for p in sorted(pkg.rglob('*')) if p.is_file() and p.name != 'SHA256SUMS']
        (pkg/'SHA256SUMS').write_text('\n'.join(lines)+'\n')
    def executable(name, body):
        p = mock/name
        p.write_text('#!/bin/sh\nset -eu\n'+body+'\n')
        p.chmod(0o755)
    executable('uname', 'case "$1" in -m) echo "${MOCK_ARCH:-aarch64}";; -r) echo "${MOCK_RELEASE:-6.1.75}";; esac')
    executable('id', 'echo 0')
    executable('insmod', '''echo insmod >> "$MOCKROOT/events"
[ "${MOCK_FAIL:-0}" = 0 ] || exit 1
mkdir -p "$MOCKROOT/sys/module/8821cu/parameters"
echo "$MOCK_SRCVERSION" > "$MOCKROOT/sys/module/8821cu/srcversion"
for p in rtw_power_mgnt rtw_ips_mode rtw_enusbss; do echo 0 > "$MOCKROOT/sys/module/8821cu/parameters/$p"; done
''')
    executable('rmmod', '''echo rmmod >> "$MOCKROOT/events"
[ "${MOCK_BUSY:-0}" = 0 ] || exit 1
rm -r "$MOCKROOT/sys/module/8821cu"
''')
    executable('depmod', 'echo depmod >> "$MOCKROOT/events"')
    executable('dmesg', 'echo "mock kernel log"')
    executable('iw', 'echo "mock iw"')
    executable('ip', 'echo "mock ip"')
    env = dict(os.environ, PATH=str(mock)+':'+os.environ['PATH'], MOCKROOT=str(fake), MOCK_SRCVERSION=srcversion)
    def reset():
        if fake.exists(): shutil.rmtree(fake)
        (fake/'proc').mkdir(parents=True)
        (fake/'etc/init.d').mkdir(parents=True)
        (fake/'sys/bus/usb/devices').mkdir(parents=True)
        (fake/'proc/config.gz').write_bytes(gzip.compress(config.read_bytes()))
    def run(script, *args, ok=True, **settings):
        p = subprocess.run(['sh', str(pkg/script), *args], env=dict(env, **settings), text=True, capture_output=True)
        assert (p.returncode == 0) == ok, (script, args, settings, p.returncode, p.stdout, p.stderr)
        return p.stdout+p.stderr
    targets = ['lib/modules/6.1.75/extra/acg138-rtl8821cu/8821cu.ko', 'etc/init.d/S35rtl8821cu', 'etc/modprobe.d/acg138-rtl8821cu.conf']
    def absent():
        assert all(not (fake/p).exists() for p in targets)
    checksums()
    reset()
    run('install.sh', '--check'); absent()
    assert not (fake/'events').exists()
    results.append('check-only makes no system changes')
    for settings in [{'MOCK_ARCH':'x86_64'}, {'MOCK_RELEASE':'6.1.76'}]:
        run('install.sh', ok=False, **settings); absent()
    results.append('wrong architecture and kernel rejected before mutation')
    (fake/'proc/config.gz').write_bytes(gzip.compress(b'wrong config\n'))
    run('install.sh', ok=False); absent()
    reset(); results.append('mismatched runtime config rejected')
    checksum = (pkg/'SHA256SUMS').read_text()
    (pkg/'payload/acg138-rtl8821cu.conf').write_text('tampered\n')
    run('install.sh', ok=False); absent()
    shutil.copy2(here/'board/payload/acg138-rtl8821cu.conf', pkg/'payload/acg138-rtl8821cu.conf')
    (pkg/'SHA256SUMS').write_text(checksum)
    results.append('corrupt package rejected before mutation')
    foreign = fake/'etc/init.d/S35rtl8821cu'
    foreign.write_text('user file\n')
    run('install.sh', ok=False)
    assert foreign.read_text() == 'user file\n'
    reset(); results.append('foreign destination preserved')
    conflict = fake/'lib/modules/6.1.75/extra/8821cu.ko'
    conflict.parent.mkdir(parents=True); conflict.write_bytes(b'old driver')
    run('install.sh', ok=False); assert conflict.read_bytes() == b'old driver'
    reset(); results.append('duplicate module blocked')
    usb = fake/'sys/bus/usb/devices/5-1'
    usb.mkdir()
    (usb/'idVendor').write_text('0bda\n'); (usb/'idProduct').write_text('c820\n')
    intf = fake/'sys/bus/usb/devices/5-1:1.0'
    intf.mkdir(); (intf/'bInterfaceClass').write_text('ff\n')
    other_driver = fake/'sys/bus/usb/drivers/other'
    other_driver.mkdir(parents=True)
    (intf/'driver').symlink_to(other_driver)
    (other_driver/'module').symlink_to(fake/'sys/module/other')
    run('install.sh', ok=False); absent()
    reset(); results.append('USB interface bound to another driver is not stolen')
    run('install.sh', ok=False, MOCK_FAIL='1'); absent()
    reset(); results.append('insmod failure rolls back new files')
    run('install.sh')
    assert all((fake/p).is_file() for p in targets)
    assert (fake/'etc/init.d/S35rtl8821cu').stat().st_mode & 0o777 == 0o755
    run('install.sh')
    assert (fake/'events').read_text().count('insmod') == 1
    results.append('install succeeds and repeat install does not reload')
    usb = fake/'sys/bus/usb/devices/5-1'; usb.mkdir()
    (usb/'idVendor').write_text('0bda\n'); (usb/'idProduct').write_text('c820\n')
    intf = fake/'sys/bus/usb/devices/5-1:1.0'; (intf/'net/wlan0').mkdir(parents=True)
    (intf/'bInterfaceClass').write_text('ff\n')
    own_driver = fake/'sys/bus/usb/drivers/rtl8821cu'; own_driver.mkdir(parents=True)
    (intf/'driver').symlink_to(own_driver)
    (own_driver/'module').symlink_to(fake/'sys/module/8821cu')
    assert 'READY: USB driver bound' in run('status.sh')
    run('install.sh', '--check')
    results.append('bound target USB interface is reported ready and permits recheck')
    loaded = fake/'sys/module/8821cu'
    shutil.rmtree(loaded)
    run('payload/S35rtl8821cu', 'start', ok=False, MOCK_RELEASE='6.1.76')
    assert not loaded.exists()
    run('payload/S35rtl8821cu', 'start')
    assert loaded.exists()
    results.append('boot loader refuses changed kernel and loads matching kernel')
    run('uninstall.sh', ok=False, MOCK_BUSY='1')
    assert all((fake/p).is_file() for p in targets)
    results.append('busy module preserves installed files')
    run('uninstall.sh'); absent()
    run('uninstall.sh'); absent()
    results.append('uninstall removes only package files and is repeatable')
    for p in pkg.rglob('*'):
        if p.is_file() and (p.suffix == '.sh' or p.name == 'S35rtl8821cu'):
            subprocess.run(['/usr/bin/busybox','sh','-n',str(p)], check=True)
    results.append('BusyBox sh syntax checks passed')
report = {'test_type': 'isolated fake-board filesystem and mocked module operations', 'hardware_test': False, 'passed': len(results), 'checks': results}
(here/'reports/installer-tests.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
