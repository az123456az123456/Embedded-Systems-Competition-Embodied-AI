#!/usr/bin/env python3
"""Export only diagnostic files; no board programs or control requests."""
from pathlib import Path
import json
import os
import subprocess
import tarfile
import tempfile

SCRIPT = Path(__file__).resolve().parents[1] / 'export-wrong.sh'
checks = 0


def check(value):
    global checks
    assert value
    checks += 1


def main():
    with tempfile.TemporaryDirectory(prefix='wrong-export-') as temporary:
        root = Path(temporary)
        nav = root / 'navigation files'
        out = root / 'exports'
        tools = root / 'tools'
        for p in (nav, out, tools):
            p.mkdir()
        # Test does not read the execution host's driver log.
        dmesg = tools / 'dmesg'
        dmesg.write_text("#!/bin/sh\nprintf '%s\\n' 'unrelated driver event' 'rtw_usb: simulated disconnect'\n")
        dmesg.chmod(0o755)
        env = dict(os.environ, WRONG_EXPORT_NAVDIR=str(nav), WRONG_EXPORT_OUTDIR=str(out),
                   PATH=str(tools) + ':' + os.environ['PATH'])
        for suffix in ('', '.1'):
            (nav / ('wrong.log' + suffix)).write_text(json.dumps({'reason': 'test_' + suffix}) + '\n')
        secret = root / 'pair-token.txt'
        secret.write_text('must-not-collect-012345\n')
        (nav / 'wrong.log.2').symlink_to(secret)
        (nav / 'config.json').write_text('{"secret":"must-not-collect"}')
        result = subprocess.run(['sh', str(SCRIPT)], env=env, capture_output=True, text=True)
        check(result.returncode == 0)
        check('Saved:' in result.stdout)
        archives = list(out.glob('wrong-diagnostics-*.tar.gz'))
        check(len(archives) == 1)
        with tarfile.open(archives[0]) as tar:
            files = {m.name: tar.extractfile(m).read() for m in tar.getmembers() if m.isfile()}
        check('wrong-diagnostics/wrong.log' in files)
        check('wrong-diagnostics/wrong.log.1' in files)
        check('wrong-diagnostics/wrong.log.2' not in files)
        check(all('config.json' not in name and 'token' not in name for name in files))
        check(all(b'must-not-collect' not in content for content in files.values()))
        check(b'rtw_usb: simulated disconnect' in files['wrong-diagnostics/driver-events.txt'])
        check(b'unrelated driver' not in files['wrong-diagnostics/driver-events.txt'])
        check((archives[0].stat().st_mode & 0o077) == 0)
        (nav / 'wrong.log').unlink()
        (nav / 'wrong.log.1').unlink()
        result = subprocess.run(['sh', str(SCRIPT)], env=env, capture_output=True, text=True)
        check(result.returncode != 0)
        check(len(list(out.glob('*.tar.gz'))) == 1)
        env['ROBOT_WRONG_LOG_PATH'] = 'relative/wrong.log'
        result = subprocess.run(['sh', str(SCRIPT)], env=env, capture_output=True, text=True)
        check(result.returncode == 2)
        check(len(list(out.glob('*.tar.gz'))) == 1)
    print(f'PASS {checks} export checks: bounded selection, symlink skip, no token/config, private archive and missing-path failure')


if __name__ == '__main__':
    main()
