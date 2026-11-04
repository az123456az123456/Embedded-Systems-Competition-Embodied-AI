#!/usr/bin/env python3
"""Exercise the real portable installer in isolated simulated board directories."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parents[1]
LIB = Path(os.environ.get('FAKECAM_TEST_LIBRARY', str(WS / 'outputs/01a13005-8bda-7393-95ca-5157476f342f/acg138-rk-full-v19.1-20261025/base/payload/isp/lib/librkaiq.so')))
FILES = ('install-highfps.sh', 'rollback-highfps.sh', 'highfps-common.sh', 'patch-fps-launcher.awk')
checks = 0


def check(value, message):
    global checks
    checks += 1
    assert value, message


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def manifest(area):
    (area / 'SHA256SUMS').write_text(''.join(f'{sha(area / name)}  {name}\n' for name in (*FILES, 'isp-probe')))


def snapshot(isp, app):
    return {str(p.relative_to(isp if p.is_relative_to(isp) else app)): (p.read_bytes(), p.stat().st_mode & 0o777)
            for p in (isp / 'isp-probe', isp / 'run_isp_camera.sh', app / 'run-perception.sh')}


with tempfile.TemporaryDirectory(prefix='camera-highfps-test-') as temp:
    area = Path(temp)
    toolbin = area / 'bin'
    toolbin.mkdir()
    for name, result in [('uname', 'aarch64'), ('id', '0')]:
        p = toolbin / name
        p.write_text(f'#!/bin/sh\nprintf \'%s\\n\' \'{result}\'\n')
        p.chmod(0o755)
    source = area / 'sleeper.c'
    source.write_text('#include <unistd.h>\nint main(void){for(;;)pause();}\n')
    subprocess.run(['cc', '-O2', str(source), '-o', str(area / 'sleeper')], check=True)
    native = (area / 'sleeper').read_bytes()
    package = area / 'package'
    package.mkdir()
    for name in FILES:
        shutil.copy2(ROOT / 'scripts' / name, package / name)
    shutil.copy2(ROOT / 'build/isp-probe', package / 'isp-probe')
    manifest(package)
    isp = area / 'isp'
    app = area / 'app'
    isp.mkdir()
    app.mkdir()
    (isp / 'lib').mkdir()
    shutil.copy2(LIB, isp / 'lib/librkaiq.so')
    (isp / 'isp-probe').write_bytes(native + b'ORIGINAL-30FPS')
    (isp / 'isp-probe').chmod(0o751)
    shutil.copy2(ROOT / 'tests/fixtures/run_isp_camera.sh', isp / 'run_isp_camera.sh')
    shutil.copy2(ROOT / 'tests/fixtures/run-perception.sh', app / 'run-perception.sh')
    immutable = {app / 'perception-dashboard.aarch64': b'CUSTOM-13-CLASS-APP', app / 'model.rknn': b'UNCHANGED-MODEL',
                 app / 'pair-token.txt': b'123456', app / 'motor-settings.txt': b'left_mask=12 invert_mask=15',
                 isp / 'lib/librkaiq.so': LIB.read_bytes(), isp / 'IQ.json': b'ORIGINAL-IQ'}
    for path, content in immutable.items():
        path.write_bytes(content)
    env = dict(os.environ, PATH=f'{toolbin}:/usr/bin:/bin', ISP_DIR=str(isp), DASHBOARD_DIR=str(app))

    def run(script='install-highfps.sh', *args, success=True):
        result = subprocess.run(['/bin/sh', str(package / script), *map(str, args)], env=env,
                                text=True, capture_output=True, timeout=30)
        check((result.returncode == 0) == success, (script, result.returncode, result.stdout, result.stderr))
        check(all(path.read_bytes() == content for path, content in immutable.items()), 'App/model/IQ/library/preferences changed')
        return result

    def backups():
        return set(isp.glob('camera-highfps-backup.*'))

    original = snapshot(isp, app)
    old = backups()
    result = run()
    backup, = backups() - old
    check((isp / 'camera-target-fps').read_text() == '40\n', '40FPS default not installed')
    check((isp / 'isp-probe').read_bytes() == (package / 'isp-probe').read_bytes(), 'Wrong receiver bytes')
    installed = snapshot(isp, app)
    check(installed != original and 'nominal target=40' in result.stdout, 'No actual install')
    # Invert only the documented FPS edits; all other bytes must match fixtures.
    for kind, path, fixture in [('isp', isp / 'run_isp_camera.sh', 'run_isp_camera.sh'),
                                ('app', app / 'run-perception.sh', 'run-perception.sh')]:
        transformed = path.read_text()
        start = transformed.index('# CAMERA_HIGHFPS_CONFIG_V20\n')
        end = transformed.index('\nfi\n', start) + len('\nfi\n')
        old_default = 'CAMERA_TARGET_FPS=${CAMERA_TARGET_FPS:-30}\n' if kind == 'app' else ''
        transformed = transformed[:start] + old_default + transformed[end:]
        transformed = transformed.replace('15|20|30|35|40)', '15|20|30)').replace('15, 20, 30, 35 or 40', '15, 20 or 30')
        check(transformed == (ROOT / 'tests/fixtures' / fixture).read_text(), f'{kind} non-FPS bytes changed')

    def fps_prefix(path, kind):
        text = path.read_text()
        start = text.index('# CAMERA_HIGHFPS_CONFIG_V20')
        if kind == 'isp':
            end = text.index('case "$MODE" in', start)
        else:
            end = text.index('export CAMERA_TARGET_FPS', start) + len('export CAMERA_TARGET_FPS')
        return text[start:end] + '\nprintf \'%s\\n\' "$CAMERA_TARGET_FPS"\n'

    for kind, path in [('isp', isp / 'run_isp_camera.sh'), ('app', app / 'run-perception.sh')]:
        script = fps_prefix(path, kind)
        for config, override, expected in [('40', None, '40'), ('30', None, '30'), ('35', None, '35'),
                                           ('40', '30', '30'), ('30', '40', '40')]:
            (isp / 'camera-target-fps').write_text(config + '\n')
            variables = dict(env, BASE=str(isp), ISP_CAMERA_SCRIPT=str(isp / 'run_isp_camera.sh'))
            variables.pop('CAMERA_TARGET_FPS', None)
            if override is not None:
                variables['CAMERA_TARGET_FPS'] = override
            p = subprocess.run(['/bin/sh', '-c', script], env=variables, text=True, capture_output=True)
            check(p.returncode == 0 and p.stdout.strip() == expected, (kind, config, override, p.stderr))
        for invalid in ['0', '31', '60', '040', '', '40\n40']:
            (isp / 'camera-target-fps').write_text(invalid + '\n')
            variables = dict(env, BASE=str(isp), ISP_CAMERA_SCRIPT=str(isp / 'run_isp_camera.sh'))
            variables.pop('CAMERA_TARGET_FPS', None)
            p = subprocess.run(['/bin/sh', '-c', script], env=variables, text=True, capture_output=True)
            check(p.returncode == 2 and not p.stdout, (kind, invalid, p.stderr))

    # Reinstall is idempotent in launchers and saves/restores an existing config.
    (isp / 'camera-target-fps').write_text('40\n')
    old = backups()
    run('install-highfps.sh', '30')
    second, = backups() - old
    check((isp / 'camera-target-fps').read_text() == '30\n', '30FPS metadata fallback missing')
    check(snapshot(isp, app) == installed, 'Reinstall duplicated or altered patch')
    run('rollback-highfps.sh', second)
    check((isp / 'camera-target-fps').read_text() == '40\n', 'Existing config not restored')
    run('rollback-highfps.sh', backup)
    check(snapshot(isp, app) == original and not (isp / 'camera-target-fps').exists(), 'Original exact bytes/modes/absence not restored')

    # Invalid input/package/runtime/launcher all fail before target mutation.
    for target in ['0', '60', '040', '35.0']:
        before = backups()
        run('install-highfps.sh', target, success=False)
        check(snapshot(isp, app) == original and backups() == before, 'Invalid target changed files')
    data = (package / 'isp-probe').read_bytes()
    (package / 'isp-probe').write_bytes(data + b'CORRUPTION')
    before = backups()
    run(success=False)
    check(snapshot(isp, app) == original and backups() == before, 'Checksum failure changed files')
    (package / 'isp-probe').write_bytes(data)
    old_app = (app / 'run-perception.sh').read_bytes()
    (app / 'run-perception.sh').write_bytes(old_app.replace(b'15|20|30)', b'15|20|30|60)'))
    before = backups()
    run(success=False)
    check(backups() == before and not (isp / 'camera-target-fps').exists(), 'Unknown launcher partially installed')
    (app / 'run-perception.sh').write_bytes(old_app)
    (isp / 'camera-target-fps').symlink_to(app / 'model.rknn')
    run(success=False)
    check((app / 'model.rknn').read_bytes() == immutable[app / 'model.rknn'], 'Followed config symlink')
    (isp / 'camera-target-fps').unlink()

    # Dashboard and foreign ISP are never killed by an update.
    for folder, name in [('live', 'perception-dashboard.aarch64'), ('foreign', 'isp-probe')]:
        directory = area / folder
        directory.mkdir()
        program = directory / name
        program.write_bytes(native)
        program.chmod(0o755)
        proc = subprocess.Popen([str(program)])
        try:
            time.sleep(.05)
            before = backups()
            run(success=False)
            check(proc.poll() is None and backups() == before and snapshot(isp, app) == original, 'Unexpected process stop or file replacement')
        finally:
            proc.terminate()
            proc.wait(timeout=2)

    # Stop only this exact ISP process before replacing its executable.
    proc = subprocess.Popen([str(isp / 'isp-probe')])
    try:
        time.sleep(.05)
        before = backups()
        run()
        check(proc.wait(timeout=2) == -15, 'Target ISP did not receive TERM')
        active_backup, = backups() - before
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=2)
    run('rollback-highfps.sh', active_backup)
    check(snapshot(isp, app) == original, 'Exact ISP stop/install/rollback changed baseline')

    # Inject one failed rename after binary commit; transaction restores originals.
    mv = toolbin / 'mv'
    mv.write_text('#!/bin/sh\nfor last do :; done\nif [ "$last" = "$DASHBOARD_DIR/run-perception.sh" ] && [ ! -f "$MV_FAILURE_MARK" ]; then touch "$MV_FAILURE_MARK"; exit 71; fi\nexec /usr/bin/mv "$@"\n')
    mv.chmod(0o755)
    env['MV_FAILURE_MARK'] = str(area / 'rename-failed-once')
    result = run(success=False)
    check('original camera files restored' in result.stderr, result.stderr)
    check(snapshot(isp, app) == original and not (isp / 'camera-target-fps').exists(), 'Partial commit not restored')
    mv.unlink()

    # Corrupt backup is rejected before rollback touches current targets.
    (backup / 'isp-probe').write_bytes(b'BROKEN')
    result = run('rollback-highfps.sh', backup, success=False)
    check('Backup checksum verification failed' in result.stderr and snapshot(isp, app) == original, 'Corrupt backup changed targets')

    report = {'status': 'PASS', 'checks': checks, 'hardware_tested': False,
              'scope': 'Real POSIX installer and launcher FPS logic in isolated host fixture directories; no board ARM execution',
              'cases': ['default40/config30/config35/env override/invalid config', 'one real output per ISP frame',
                        'exact original bytes/modes/config absence restored', 'idempotent reinstall and existing config rollback',
                        'checksum/unknown launcher/symlink/live dashboard/foreign ISP guards', 'exact target ISP TERM',
                        'partial commit failure auto-restore', 'corrupt backup refusal'],
              'receiver_sha256': sha(ROOT / 'build/isp-probe'), 'retained_fakecam_sha256': sha(LIB)}
    (ROOT / 'reports').mkdir(exist_ok=True)
    (ROOT / 'reports/highfps-installer-verification.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
