#!/usr/bin/env python3
"""Reproduce v28.1 firmware and actual RTL simulations; never access hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'softcore/firmware'
REPORTS = ROOT / 'reports'
SOURCE_RTL = [
    'softcore/vendor/picorv32.v', 'softcore/robot_nav_soc.v',
    'imu/i2c_reg_master.v', 'imu/mpu9250_reader.v', 'imu/imu_sample_filter.v',
    'imu/imu_gmii_source.v', 'imu/imu_ethernet_tx.v',
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command, name):
    started = time.monotonic()
    with (REPORTS / name).open('w') as out:
        subprocess.run(command, cwd=ROOT, check=True, stdout=out,
                       stderr=subprocess.STDOUT, env={**os.environ,
                       'ASAN_OPTIONS': 'detect_leaks=0', 'CCACHE_DISABLE': '1',
                       'CCACHE_TEMPDIR': '/tmp'})
    return round(time.monotonic() - started, 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reuse-completed-simulations', action='store_true',
                        help='Only bind already completed same-source core and producer simulations')
    args = parser.parse_args()
    REPORTS.mkdir(exist_ok=True)
    timings = {}
    simulation_sources = SOURCE_RTL + [
        'softcore/tests/v28_tb_layered_firmware.sv',
        'softcore/tests/v28_1_tb_source_mount.sv',
        'softcore/firmware/firmware.hex', 'softcore/firmware/firmware_symbols.vh',
    ]
    starting_source = {name: digest(ROOT / name) for name in simulation_sources}
    if args.reuse_completed_simulations:
        # This mode is for completing a record after independent jobs were
        # launched with these commands. Refuse an older executable if any
        # source or firmware image has subsequently changed.
        core_binary = ROOT / 'build/v28_firmware_verilator/Vv28_tb_layered_firmware'
        producer_binary = ROOT / 'build/v28_1_source_mount/Vv28_1_tb_source_mount'
        for name in simulation_sources:
            # make may re-emit an identical HEX after the HDL executable was
            # compiled. HEX is read at simulator start, not baked into C++;
            # its content is separately bound to the actual core report.
            if name.startswith('softcore/firmware/'):
                continue
            binary = producer_binary if name.startswith('imu/') or name.endswith('v28_1_tb_source_mount.sv') else core_binary
            assert (ROOT / name).stat().st_mtime_ns <= binary.stat().st_mtime_ns, name
    if not args.reuse_completed_simulations:
        timings['base_control_astar_protocol_rv32'] = run(
            ['python3', 'softcore/tests/v28_run_firmware_tests.py'],
            'v28_1-final-firmware-runner.log')
    previous = json.loads((REPORTS / 'v28_firmware_tests.json').read_text())
    assert previous['passed'] and not previous['host_only']
    assert previous['firmware_hex_sha256'] == digest(FW / 'firmware.hex')
    for name, value in previous['source_sha256'].items():
        assert digest(ROOT / name) == value, name
    core = previous['actual_rv32']
    assert core['passed'] and core['astar_expanded'] == 2048
    assert core['max_commit_gap_ms'] < 24 and core['max_service_gap_cycles'] < 500000
    cflags = ['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic',
              '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
    timings['mount_compile'] = run(cflags + [
        'softcore/tests/v28_1_mount_readiness_test.c',
        'softcore/firmware/layered_control.c', '-lm', '-o', 'build/v28_1-mount-test'],
        'v28_1-mount-readiness-compile.log')
    timings['mount_sanitizers'] = run(['build/v28_1-mount-test'], 'v28_1-mount-readiness.log')
    if not args.reuse_completed_simulations:
        timings['producer_compile'] = run([
            'verilator', '--binary', '--timing', '-Wno-fatal', '--top-module',
            'v28_1_tb_source_mount', '--Mdir', 'build/v28_1_source_mount',
            '-CFLAGS', '-std=c++20', *SOURCE_RTL,
            'softcore/tests/v28_1_tb_source_mount.sv'],
            'v28_1-source-mount-final-compile.log')
        timings['producer_actual_rv32'] = run([
            'build/v28_1_source_mount/Vv28_1_tb_source_mount'],
            'v28_1-source-mount-final.log')
    log = (REPORTS / 'v28_1-source-mount-final.log').read_text()
    assert 'PASS actual source/filter/IMU9 monitor->MMIO->publishedRV32' in log
    match = re.search(r'positive physical90deg turn with negativeZ mount: '
                      r'yaw_mdeg=(\d+) flags=([0-9a-f]+) cycles=(\d+)', log)
    assert match, 'Actual producer/turn test did not finish'
    yaw, flags, cycles = int(match[1]), int(match[2], 16), int(match[3])
    assert 89500 <= yaw <= 90500 and flags & 0x03000003 == 0x03000003
    source_paths = [p for p in FW.iterdir() if p.suffix in ('.c', '.h', '.S', '.ld')
                    or p.name in ('Makefile', 'verify_v28_firmware.py')]
    source_paths += [ROOT / p for p in SOURCE_RTL]
    source_paths += [ROOT / p for p in (
        'softcore/tests/v28_run_firmware_tests.py',
        'softcore/tests/v28_1_run_firmware_tests.py',
        'softcore/tests/v28_tb_layered_firmware.sv',
        'softcore/tests/v28_layered_control_test.c',
        'softcore/tests/v28_1_mount_readiness_test.c',
        'softcore/tests/v28_1_tb_source_mount.sv',
    )]
    logs = ['v28_firmware_host.log', 'v28_firmware_astar_host.log',
            'v28_firmware_nav_protocol_host.log', 'v28_firmware_rv32_final.log',
            'v28_1-mount-readiness.log', 'v28_1-source-mount-final.log']
    for name in logs:
        assert 'PASS' in (REPORTS / name).read_text(), name
    assert starting_source == {name: digest(ROOT / name) for name in simulation_sources}, 'Simulation inputs changed while running'
    record = {
        'version': 'v28.1-mount-readiness-dff-registerfile',
        'passed': True, 'hardware_tested': False,
        'wall_seconds': timings, 'reused_completed_simulations': args.reuse_completed_simulations,
        'firmware_hex_sha256': digest(FW / 'firmware.hex'),
        'source_sha256': {str(p.relative_to(ROOT)): digest(p) for p in sorted(set(source_paths))},
        'evidence_sha256': {name: digest(REPORTS / name) for name in logs},
        'actual_rv32': core,
        'production_imu_producer_rv32': {
            'passed': True, 'calibration_ready_ms': 2024,
            'actual_rv32_cycles': cycles, 'physical_ccw_turn_deg': 90,
            'final_yaw_mdeg': yaw, 'committed_flags': flags,
            'mount_raw_acceleration': [1561, 497, -16384],
            'stationary_raw_gyro': [-300, -304, 30],
            'boundary': 'force raw reader outputs; actual production filter/IMU9 monitor/MMIO/RV32',
            'not_tested': ['physical I2C bus', 'physical board', 'complete synthesized graph'],
        },
    }
    (REPORTS / 'v28_1_firmware_tests.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps({k: v for k, v in record.items()
                      if k not in ('source_sha256', 'evidence_sha256')}, indent=2))


if __name__ == '__main__':
    main()
