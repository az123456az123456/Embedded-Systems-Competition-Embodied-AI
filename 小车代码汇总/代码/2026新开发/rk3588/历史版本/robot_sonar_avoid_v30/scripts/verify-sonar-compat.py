#!/usr/bin/env python3
"""Offline compatibility tests, not a new RK release or an installer.

All production files and ARM64 payloads must match the published v29 baseline.
The only new executable built here is a host-side test, never run on the car.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--v29-runtime', type=Path,
                        help='optional original v29 archive to verify byte-for-byte')
    args = parser.parse_args()
    reports = ROOT / 'reports'
    build = ROOT / 'build/sonar-compat'
    reports.mkdir(exist_ok=True)
    build.mkdir(parents=True, exist_ok=True)
    # A failed rerun must not leave an earlier PASS report looking current.
    (reports / 'rk-compatibility.json').unlink(missing_ok=True)
    baseline_file = ROOT / 'scripts/v29-production-baseline.json'
    baseline = json.loads(baseline_file.read_text())
    production = baseline['source_sha256']
    for name, value in production.items():
        assert sha(ROOT / name) == value, f'production v29 byte mismatch: {name}'
    binaries = {}
    for role, value in baseline['binaries'].items():
        path = ROOT / 'update/payload' / Path(value['path']).name
        assert sha(path) == value['sha256'], path
        raw = path.read_bytes()
        assert raw[:6] == b'\x7fELF\x02\x01' and raw[18:20] == b'\xb7\x00'
        binaries[role] = dict(path=str(path.relative_to(ROOT)), sha256=sha(path),
                              bytes=path.stat().st_size, unchanged_from_v29=True)
    if args.v29_runtime:
        assert sha(args.v29_runtime) == baseline['runtime_sha256']
    origin_file = ROOT / 'navigation/tests/fixtures/v29_1-sns1-origin.json'
    origin = json.loads(origin_file.read_text())
    fixture = ROOT / origin['fixture']
    assert sha(fixture) == origin['fixture_sha256']
    rows = fixture.read_text().splitlines()
    assert len(rows) == 39 and all(re.fullmatch(r'SNS1 t_ms=\d+ packet=[0-9a-f]{128}', x) for x in rows)
    producer_root = ROOT.parents[1]
    producer_available = all((producer_root / p).exists() for p in origin['producer_source_sha256'])
    if producer_available:
        for name, value in origin['producer_source_sha256'].items():
            assert sha(producer_root / name) == value, f'fixture producer source changed: {name}'
        for name, value in origin['producer_evidence_sha256'].items():
            assert sha(producer_root / name) == value, f'fixture producer evidence changed: {name}'
    commands = []
    def run(name, command, marker=None):
        log = reports / (name + '.log')
        with log.open('w') as out:
            out.write('COMMAND ' + shlex.join(map(str, command)) + '\n')
            out.flush()
            result = subprocess.run(list(map(str, command)), cwd=ROOT, stdout=out,
                                    stderr=subprocess.STDOUT)
        body = log.read_text()
        commands.append(dict(name=name, command=list(map(str, command)),
                             exit_code=result.returncode, log=str(log.relative_to(ROOT)),
                             log_sha256=sha(log)))
        if result.returncode or (marker and marker not in body):
            raise RuntimeError(f'{name} failed: see {log}')
        if marker:
            print(next(line for line in reversed(body.splitlines()) if marker in line))
    cc = shlex.split(os.environ.get('CC', 'cc'))
    cxx = shlex.split(os.environ.get('CXX', 'g++'))
    cflags = ['-O2', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Idashboard/src']
    cppflags = ['-O2', '-std=c++17', '-pthread', '-Wall', '-Wextra', '-Werror', '-Idashboard/src']
    obj = build / 'sensor-data.o'
    run('sonar-c-compile', cc + cflags + ['-c', 'dashboard/src/sensor_data.c', '-o', obj])
    host = build / 'test-sonar-zero-compat'
    run('sonar-pipeline-compile', cxx + cppflags + [
        'navigation/navcore.cc', 'navigation/imu_alignment.cc', 'navigation/sensor_bridge.cc',
        'navigation/protocol.cc', 'common/wrong_log.cc',
        'navigation/tests/test_sonar_zero_compat.cc', obj, '-o', host])
    run('sonar-zero-compat', [host, fixture], 'PASS unchanged-v29 sonar-zero compatibility')
    for name in ['display', 'observation_age']:
        target = build / ('test-sonar-' + name)
        run('sonar-' + name + '-compile', cc + cflags + ['dashboard/src/sensor_data.c',
            f'dashboard/tests/test_sonar_{name}.c', '-o', target])
        run('sonar-' + name, [target], 'PASS')
    for name, sources in [
        ('censored', ['navigation/navcore.cc', 'navigation/tests/test_sonar_censored.cc', 'common/wrong_log.cc']),
        ('diagnostics', ['navigation/tests/test_sonar_diagnostics.cc', 'common/wrong_log.cc']),
    ]:
        target = build / ('test-sonar-' + name)
        run('sonar-' + name + '-compile', cxx + cppflags + sources + ['-o', target])
        run('sonar-' + name, [target], 'PASS')
    run('sonar-android', ['sh', 'navigation/android/test-sonar-host.sh'], 'PASS')
    for name, value in production.items():
        assert sha(ROOT / name) == value, f'test changed production file: {name}'
    if producer_available:
        for name, value in origin['producer_source_sha256'].items():
            assert sha(producer_root / name) == value, f'producer changed during compatibility run: {name}'
    additions = ['scripts/verify-sonar-compat.py', 'scripts/v29-production-baseline.json',
                 'navigation/tests/test_sonar_zero_compat.cc',
                 'v29.1超声波兼容验证说明.txt']
    sources = dict(production)
    sources.update({p: sha(ROOT / p) for p in additions})
    evidence = {str(fixture.relative_to(ROOT)): sha(fixture),
                str(origin_file.relative_to(ROOT)): sha(origin_file)}
    evidence.update({row['log']: row['log_sha256'] for row in commands})
    report = dict(result='PASS', test_version='v29.1-sonar-compatibility',
        created_utc=datetime.now(timezone.utc).isoformat(),
        rk_update_required=False, apk_update_required=False, new_rk_release=False,
        production_rk_changes=False, production_baseline_files_verified=len(production),
        hardware_tested=False, arm_programs_executed=False, network_accessed=False,
        baseline_runtime=dict(filename=baseline['runtime_filename'], sha256=baseline['runtime_sha256'],
                              supplied_archive_verified=bool(args.v29_runtime)),
        baseline_verification_report_sha256=baseline['verification_report_sha256'],
        binaries=binaries, source_sha256=sources, evidence_sha256=evidence,
        integration_evidence_sha256=origin['producer_evidence_sha256'],
        actual_rtl_fixture=dict(rows=len(rows), sha256=sha(fixture),
            producer_sources_available_and_verified=producer_available,
            producer_source_sha256=origin['producer_source_sha256'],
            provenance='actual three UARTs -> production receiver/capture -> unchanged C Kalman DPI feedback'),
        tests=commands,
        semantics=dict(zero='checksum-valid FF0000FF maps to valid 4500mm by user-selected project convention',
            not_official_proof_of_clear_4_5m=True,
            map='valid >4m is censored: observed near cone only; no occupied endpoint, no clearing beyond4m',
            invalid='bad checksum, missing UART frame, 1..29mm and FFFF are never synthesized into fresh far',
            age='source age and observation ID retained independently of new transport snapshots',
            control='genuine new near zeroes immediately; transient source expiry zero WAIT then continuous5s fault; no automatic restart'),
        limits=['offline replay and host tests, not physical obstacle safety certification',
                'the source fixture uses real RTL UARTs; synthetic extended replay tests existing source-age timeout boundaries',
                'existing v29 installation/rollback and APK remain unchanged; install no new RK runtime for this FPGA-only fix'])
    dest = reports / 'rk-compatibility.json'
    dest.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(f'PASS unchanged v29 RK / APK; {len(production)} original files, two ARM64 payloads and39 actual SNS1 fixtures verified')
    print(dest)


if __name__ == '__main__':
    main()
