#!/usr/bin/env python3
"""Exercise the real shell update using temporary roots; never execute ARM code."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import tempfile

SRC = Path(__file__).resolve().parents[1]
WS = SRC.parents[1]
BASELINE = WS / 'rk3588/robot_industrial_v19/dashboard/build/perception-dashboard.aarch64'
BASELINE_SHA = 'c36444374e98274d9ef6d048d1671a33c9a3ab0f8602b578c828c307d05fd959'
SCRIPTS = ('common.sh', 'install.sh', 'rollback.sh', 'diagnose-display.sh')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--payload', type=Path, help='Actual new ARM binary; if omitted, an inert baseline-plus-marker fixture is used.')
    parser.add_argument('--package', type=Path, help='Test scripts and payload from an extracted, checksummed runtime archive.')
    parser.add_argument('--report', type=Path, default=SRC / 'reports/installer-tests.json')
    args = parser.parse_args()
    checks = 0
    cases = []

    def ck(value, message=''):
        nonlocal checks
        assert value, message
        checks += 1

    ck(sha(BASELINE) == BASELINE_SHA)
    with tempfile.TemporaryDirectory(prefix='hdmi-display-installer-') as temporary:
        tmp = Path(temporary)
        package = tmp / 'package'
        package.mkdir()
        runtime_source = args.package or SRC
        if args.package:
            subprocess.run(['sha256sum', '-c', 'SHA256SUMS'], cwd=args.package, stdout=subprocess.DEVNULL, check=True)
            ck(True)
        for name in SCRIPTS:
            shutil.copy2(runtime_source / name, package / name)
            subprocess.run(['sh', '-n', str(package / name)], check=True)
        payload = package / 'perception-dashboard.aarch64'
        actual_payload = args.payload or (args.package / payload.name if args.package else None)
        if actual_payload:
            shutil.copy2(actual_payload, payload)
        else:
            payload.write_bytes(BASELINE.read_bytes() + b'\nINERT-HOST-TEST-DISPLAY-FIX\n')
        payload.chmod(0o755)
        new_sha = sha(payload)
        ck(new_sha != BASELINE_SHA)
        (package / 'dashboard.sha256').write_text(new_sha + '\n')

        def manifest():
            files = (*SCRIPTS, 'dashboard.sha256', 'perception-dashboard.aarch64')
            (package / 'SHA256SUMS').write_text(''.join(f'{sha(package / name)}  {name}\n' for name in files))

        manifest()
        tools = tmp / 'tools'
        tools.mkdir()
        for name, body in (
            ('uname', 'if [ "$1" = -m ]; then echo "${TEST_ARCH:-aarch64}"; else exec /usr/bin/uname "$@"; fi'),
            ('id', 'if [ "$1" = -u ]; then echo "${TEST_UID:-0}"; else exec /usr/bin/id "$@"; fi'),
            ('mv', '''last=
for last; do :; done
case "$last" in */perception-dashboard.aarch64)
    if [ -n "${TEST_MV_MODE:-}" ] && [ ! -f "$TEST_MV_MARKER" ]; then
        : > "$TEST_MV_MARKER"
        case "$TEST_MV_MODE" in
            fail-before) exit 73 ;;
            fail-after) /usr/bin/mv "$@"; exit 73 ;;
            corrupt-after) /usr/bin/mv "$@"; printf corrupt > "$last"; exit 0 ;;
        esac
    fi ;;
esac
exec /usr/bin/mv "$@"'''),
        ):
            path = tools / name
            path.write_text('#!/bin/sh\nset -eu\n' + body + '\n')
            path.chmod(0o755)

        def fixture(name, alias=False):
            root = tmp / name
            app = root / 'root' / ('real-app' if alias else 'robot-perception')
            app.mkdir(parents=True)
            if alias:
                (root / 'root/robot-perception').symlink_to(app, target_is_directory=True)
            (root / 'proc').mkdir()
            shutil.copy2(BASELINE, app / 'perception-dashboard.aarch64')
            (app / 'perception-dashboard.aarch64').chmod(0o751)
            (app / 'lib').mkdir()
            (app / 'model').mkdir()
            for relative, content in {
                'run-dashboard.sh': b'unchanged launch -90 --left-mask 12 --invert-mask 15\n',
                'run-perception.sh': b'unchanged launch CAMERA_TARGET_FPS=40\n',
                'lib/librknnrt.so': b'existing-rknn-runtime\n',
                'lib/libwayland-client.so.0': b'existing-wayland-runtime\n',
                'model/industrial-yolov8s-int8-rk3588.rknn': b'existing-custom-model\n',
                'pair-token.txt': b'048731\n',
                'phone-preferences.json': b'{"left_mask":12,"invert_mask":15}\n',
                'camera-fps.conf': b'CAMERA_TARGET_FPS=40\n',
            }.items():
                (app / relative).write_bytes(content)
            for relative, content in {
                'root/robot-navigation/config.json': b'{"encoder_ticks_per_turn":60000,"calibration_confirmed":false}\n',
                'root/robot-navigation/follow_geometry.json': b'{"measured":true}\n',
                'opt/fpga-camera-isp-1080/run_isp_camera.sh': b'CAMERA_TARGET_FPS=40\n',
                'etc/xdg/weston/weston.ini': b'[core]\nidle-time=300\n',
                'root/.config/weston.ini': b'[output]\nname=HDMI-A-1\n',
            }.items():
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(content)
            env = dict(os.environ, PATH=str(tools) + ':' + os.environ['PATH'], HDMI_FIX_DESTDIR=str(root))
            return root, app, env

        def state(root):
            return {str(path.relative_to(root)): (sha(path), path.stat().st_mode & 0o777)
                    for path in root.rglob('*') if path.is_file() and not path.is_symlink()
                    and 'robot-display-v19.2-backup.' not in str(path.relative_to(root))
                    and '/proc/' not in str(path)}

        def run(script, env, *arguments, ok=True):
            result = subprocess.run(['sh', str(package / script), *map(str, arguments)], env=env,
                                    text=True, capture_output=True)
            ck(result.returncode == 0 if ok else result.returncode != 0, result.stdout + result.stderr)
            return result

        def backups(root):
            return list((root / 'root').glob('robot-display-v19.2-backup.*'))

        for alias in (False, True):
            root, app, env = fixture('normal-alias-' + str(alias), alias)
            before = state(root)
            original_stat = (app / payload.name).stat()
            run('install.sh', env, '--check-only')
            ck(state(root) == before and not backups(root), 'check-only mutated files')
            run('install.sh', env)
            ck(sha(app / payload.name) == new_sha)
            after = state(root)
            binary_key = str((app / payload.name).relative_to(root))
            ck({k: v for k, v in after.items() if k != binary_key} == {k: v for k, v in before.items() if k != binary_key}, 'unrelated state changed')
            history = backups(root)
            ck(len(history) == 1)
            backup = history[0]
            ck(sha(backup / payload.name) == BASELINE_SHA)
            ck((backup / payload.name).stat().st_mode & 0o777 == 0o751)
            backup_stat = (backup / payload.name).stat()
            ck((backup_stat.st_uid, backup_stat.st_gid, backup_stat.st_mtime_ns) ==
               (original_stat.st_uid, original_stat.st_gid, original_stat.st_mtime_ns), 'backup lost ownership or timestamp')
            receipt = (backup / 'receipt.txt').read_bytes()
            run('install.sh', env)
            ck(backups(root) == history and (backup / 'receipt.txt').read_bytes() == receipt, 'idempotence overwrote receipt')
            run('rollback.sh', env, backup)
            ck(state(root) == before, 'rollback did not restore exact baseline state')
            restored_stat = (app / payload.name).stat()
            ck((restored_stat.st_uid, restored_stat.st_gid, restored_stat.st_mtime_ns) ==
               (original_stat.st_uid, original_stat.st_gid, original_stat.st_mtime_ns), 'rollback lost original ownership or timestamp')
            run('rollback.sh', env, backup)
            ck(state(root) == before)
            cases.append('install/idempotence/exact rollback with ' + ('resolved directory alias' if alias else 'regular directory'))

        for reason in ('arch', 'uid', 'missing-runtime', 'unknown-baseline', 'binary-symlink', 'active', 'active-deleted', 'hardlink-active', 'lock'):
            root, app, env = fixture('reject-' + reason)
            if reason == 'arch': env['TEST_ARCH'] = 'x86_64'
            elif reason == 'uid': env['TEST_UID'] = '1000'
            elif reason == 'missing-runtime': (app / 'lib/librknnrt.so').unlink()
            elif reason == 'unknown-baseline': (app / payload.name).write_bytes(b'newer unrelated dashboard')
            elif reason == 'binary-symlink':
                (app / payload.name).rename(app / 'real-binary')
                (app / payload.name).symlink_to(app / 'real-binary')
            elif reason in ('active', 'active-deleted', 'hardlink-active'):
                pid = root / 'proc/1234'; pid.mkdir()
                if reason == 'hardlink-active': os.link(app / payload.name, pid / 'exe')
                else: (pid / 'exe').symlink_to(str(app / payload.name) + (' (deleted)' if reason.endswith('deleted') else ''))
            elif reason == 'lock': (root / 'root/.robot-display-hdmi-v19.2.lock').mkdir()
            before = state(root)
            run('install.sh', env, ok=False)
            ck(state(root) == before and not backups(root), reason + ' changed files')
            cases.append('reject ' + reason)

        root, app, env = fixture('bad-package')
        before = state(root)
        payload.write_bytes(payload.read_bytes() + b'corrupt')
        run('install.sh', env, ok=False)
        ck(state(root) == before and not backups(root))
        payload.write_bytes(payload.read_bytes()[:-7])
        manifest()
        cases.append('reject corrupt package before writes')

        root, app, env = fixture('wrong-elf')
        before = state(root)
        original_payload = payload.read_bytes()
        payload.write_bytes(b'not an AArch64 ELF file')
        (package / 'dashboard.sha256').write_text(sha(payload) + '\n')
        manifest()
        run('install.sh', env, ok=False)
        ck(state(root) == before and not backups(root))
        payload.write_bytes(original_payload)
        (package / 'dashboard.sha256').write_text(new_sha + '\n')
        manifest()
        cases.append('reject wrong ELF even with correct checksums')

        for mode in ('fail-before', 'fail-after', 'corrupt-after'):
            root, app, env = fixture('install-failure-' + mode)
            before = state(root)
            env.update(TEST_MV_MODE=mode, TEST_MV_MARKER=str(tmp / ('install-' + mode + '.marker')))
            run('install.sh', env, ok=False)
            ck(state(root) == before, 'failed replacement not recovered: ' + mode)
            ck(len(backups(root)) == 1 and sha(backups(root)[0] / payload.name) == BASELINE_SHA)
            ck(not (root / 'root/.robot-display-hdmi-v19.2.lock').exists())
            ck(not list(app.glob('.display-hdmi-v19.2.*')))
            cases.append('installation recovers ' + mode)

        for mode in ('fail-before', 'fail-after', 'corrupt-after'):
            root, app, env = fixture('rollback-failure-' + mode)
            run('install.sh', env)
            before = state(root)
            env.update(TEST_MV_MODE=mode, TEST_MV_MARKER=str(tmp / ('rollback-' + mode + '.marker')))
            run('rollback.sh', env, backups(root)[0], ok=False)
            ck(state(root) == before, 'failed rollback not recovered: ' + mode)
            ck(not (root / 'root/.robot-display-hdmi-v19.2.lock').exists())
            ck(not list(app.glob('.display-hdmi-v19.2.*')))
            cases.append('rollback recovers ' + mode)

        for reason in ('later-update', 'corrupt-backup', 'wrong-receipt', 'active-deleted'):
            root, app, env = fixture('rollback-reject-' + reason)
            run('install.sh', env)
            backup = backups(root)[0]
            if reason == 'later-update': (app / payload.name).write_bytes(b'later-app-update')
            elif reason == 'corrupt-backup': (backup / payload.name).write_bytes(b'bad-backup')
            elif reason == 'wrong-receipt':
                receipt = backup / 'receipt.txt'
                receipt.write_text(receipt.read_text().replace(new_sha, '0' * 64))
            elif reason == 'active-deleted':
                pid = root / 'proc/1234'; pid.mkdir()
                (pid / 'exe').symlink_to(str(app / payload.name) + ' (deleted)')
            before = state(root)
            run('rollback.sh', env, backup, ok=False)
            ck(state(root) == before)
            cases.append('rollback rejects ' + reason)

        report = {
            'result': 'PASS', 'checks': checks, 'cases': cases,
            'baseline_sha256': BASELINE_SHA, 'payload_sha256': new_sha,
            'payload': str(actual_payload) if actual_payload else 'inert AArch64 ELF fixture; rerun with actual binary before packaging',
            'tested_package': str(args.package) if args.package else None,
            'executed_arm_code': False, 'hardware_tested': False,
            'scope': 'actual shell installers; safe temporary destination roots and fake uname/id/mv only',
            'preserved': ['launchers', '40 FPS configuration', 'models', 'IMU and navigation configuration', 'phone pairing and wheel masks', 'Weston files'],
        }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
