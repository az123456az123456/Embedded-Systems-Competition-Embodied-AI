#!/usr/bin/env python3
"""Compiled production CLI migration tests; no sockets, motors or board required."""
import copy
import hashlib
import json
import math
from pathlib import Path
import stat
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / 'build/robot-navigation'


def run(*args):
    return subprocess.run([str(BINARY), *map(str, args)], text=True, capture_output=True, timeout=5)


def main():
    checks = 0

    def check(condition, label):
        nonlocal checks
        assert condition, label
        checks += 1

    raw = run('--dump-profile')
    check(raw.returncode == 0, 'profile readable without config/network/hardware')
    profile = json.loads(raw.stdout)
    check(profile['encoder_ticks_per_turn'] == 60527.7, 'fixed CPR decimal retained')
    check(profile['encoder_signs'] == [-1, 1, -1, 1], 'fixed signal-order signs')
    check(profile['robot_radius_m'] == math.hypot(.16725, .137), 'full extended-body rectangle sweep radius')
    check(abs(profile['body_width_m'] - .252) < 1e-15, 'left board extends total width to252mm')
    check(abs(profile['body_length_m'] - .312667) < 1e-15, 'asymmetric body length')
    check(all(math.isclose(actual, expected, rel_tol=0, abs_tol=1e-12) for row, expected_row in zip(profile['sonar_mounts'], [[.145417, 0, 0], [.097417, .057, 30], [.097417, -.057, -30]]) for actual, expected in zip(row, expected_row)), 'diagonal side sonar mounts')
    check(profile['sonar_origins_measured'] is False and profile['effective_turn_track_measured'] is False, 'CAD provenance not promoted to measured physical calibration')
    check(profile['wheel_mapping'] == 'phone_verified_live_FPGA_ack', 'signal signs independent of live wheel-side grouping')
    check(profile['dynamic_control']['calibration_confirmed'] is False and profile['dynamic_control']['drive_mps_per_full_pwm'] == 0, 'fixed geometry never auto-authorizes driving')
    fixed_keys = set(profile) - {'dynamic_control', 'wheel_mapping'}
    with tempfile.TemporaryDirectory(prefix='nav-v22-fixed-profile-') as td:
        folder = Path(td)
        source = folder / 'old config.json'
        staged = folder / 'staged.json'
        original = dict(calibration_confirmed=False, encoder_ticks_per_turn=60000,
                        encoder_signs=[1, 1, 1, 1], wheel_circumference_m=.20106193,
                        track_m=.177, robot_radius_m=.18,
                        sonar_mounts=[[0, 0, 0], [0, 0, 90], [0, 0, -90]],
                        drive_mps_per_full_pwm=.623, brake_mps2=.57,
                        reaction_s=.63, sonar_latency_s=.21, imu_yaw_sign=-1,
                        max_pwm=.38, calibration_notes={'owner': 'test', 'values': [None, True, 5]},
                        left_mask=12, invert_mask=15,
                        stop_clearance_m=.2, side_clearance_m=.45, avoidance_step_deg=29)
        source.write_text(json.dumps(original))
        input_bytes = source.read_bytes()
        migrated = run('--migrate-config', source, '--output', staged)
        check(migrated.returncode == 0, 'migration runs without network/motor access')
        check(source.read_bytes() == input_bytes, 'migration never modifies input')
        check(stat.S_IMODE(staged.stat().st_mode) == 0o600, 'staged config owner-only')
        result = json.loads(staged.read_text())
        for key in fixed_keys:
            check(result[key] == profile[key], 'migration literal matches compiled profile: ' + key)
        for key, value in original.items():
            if key not in fixed_keys | {'stop_clearance_m', 'side_clearance_m', 'avoidance_step_deg'}:
                check(result[key] == value, 'dynamic/unknown setting preserved: ' + key)
        check(result['calibration_confirmed'] is False, 'no calibration auto-enable')
        check(result['stop_clearance_m'] == .30 and result['side_clearance_m'] == .30 and result['avoidance_step_deg'] == 25, 'requested three-beam avoidance defaults')
        check(run('--check-profile', staged).returncode == 0, 'actual migrated config validates')
        check(run('--check-profile', source).returncode == 2, 'legacy config profile mismatch detected')
        effective = run('--config', source, '--dump-profile')
        check(effective.returncode == 0, 'legacy config inspectable')
        effective = json.loads(effective.stdout)
        for key in fixed_keys:
            check(effective[key] == profile[key], 'legacy geometry cannot override compiled profile: ' + key)
        check(effective['dynamic_control']['drive_mps_per_full_pwm'] == .623 and effective['dynamic_control']['brake_mps2'] == .57 and effective['dynamic_control']['imu_yaw_sign'] == -1, 'effective runtime measured control preserved')
        second = folder / 'second.json'
        check(run('--migrate-config', staged, '--output', second).returncode == 0, 'migration can rerun to fresh staging path')
        check(json.loads(second.read_text()) == result, 'migration idempotent over canonical profile')
        check(run('--migrate-config', source, '--output', source).returncode != 0 and source.read_bytes() == input_bytes, 'in-place source output rejected')
        check(run('--migrate-config', source, '--output', folder / '.' / source.name).returncode != 0, 'alias source rejected')
        symlink = folder / 'symlink.json'; symlink.symlink_to(source)
        check(run('--migrate-config', source, '--output', symlink).returncode != 0 and source.read_bytes() == input_bytes, 'symlink output rejected')
        check(run('--migrate-config', source, '--output', staged).returncode != 0 and json.loads(staged.read_text()) == result, 'existing staging file not clobbered')
        check(run('--config', source, '--check-config').returncode == 0, 'v28 physical velocity needs no legacy PWM calibration; phone/ACK4 still required')
        original['calibration_confirmed'] = True
        source.write_text(json.dumps(original))
        check(run('--config', source, '--check-config').returncode == 0, 'measured configured controls remain structurally valid')
        for value in ['fast', None, True, -1]:
            bad = dict(original, drive_mps_per_full_pwm=value)
            source.write_text(json.dumps(bad))
            attempt = folder / ('bad-' + str(checks) + '.json')
            check(run('--migrate-config', source, '--output', attempt).returncode != 0 and not attempt.exists(), 'invalid configured speed does not install: ' + str(value))
        for text in ['{}', '[]', '{"calibration_confirmed":true,"brake_mps2":1e9999}', '{"calibration_confirmed":false} trailing']:
            source.write_text(text)
            check(run('--config', source, '--dump-profile').returncode != 0, 'invalid structural JSON/nonfinite values rejected')
        bad = dict(result, encoder_ticks_per_turn=60000)
        source.write_text(json.dumps(bad))
        check(run('--check-profile', source).returncode == 2, 'changed fixed CPR detected by installer checker')
        bad = dict(result, calibration_confirmed=False, drive_mps_per_full_pwm=0,
                   encoder_ticks_per_turn='old invalid value', encoder_signs=[0], sonar_mounts=None)
        source.write_text(json.dumps(bad))
        check(json.loads(run('--config', source, '--dump-profile').stdout)['encoder_ticks_per_turn'] == 60527.7, 'legacy fixed-field types cannot alter production constants')
    print(f'PASS {checks} compiled profile/migration/CLI checks; no hardware/network')
    report = {'checks': checks, 'binary_sha256': hashlib.sha256(BINARY.read_bytes()).hexdigest(), 'board_tested': False, 'profile': profile}
    (ROOT / 'reports' / 'vehicle-profile-tests.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
