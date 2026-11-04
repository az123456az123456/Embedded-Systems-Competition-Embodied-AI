#!/usr/bin/env python3
"""Use actual sibling RK encoder/decoder and actual FPGA RX/control/ACK RTL."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess

PROJECT = Path(__file__).resolve().parents[1]
RK = PROJECT.parent.parent / 'rk3588/robot_perception_v12'
IDE = PROJECT.parents[4] / 'Gowin_V1.9.12.03/IDE'
BUILD = PROJECT / 'build/motor_crosswire'
REPORT = PROJECT / 'reports/motor_crosswire'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    REPORT.mkdir(parents=True, exist_ok=True)
    pins = dict(re.findall(r'IO_LOC "([^"]+)" (\w+);', (PROJECT / 'robot_fmc_remote.cst').read_text()))
    expected = {'motor_pwm[0]': 'J15', 'motor_in1[0]': 'H15', 'motor_in2[0]': 'Y21',
                'motor_pwm[1]': 'Y22', 'motor_in1[1]': 'AB21', 'motor_in2[1]': 'AA19',
                'motor_pwm[2]': 'AB20', 'motor_in1[2]': 'M16', 'motor_in2[2]': 'K21',
                'motor_pwm[3]': 'G20', 'motor_in1[3]': 'G17', 'motor_in2[3]': 'N18'}
    assert {key: pins[key] for key in expected} == expected
    commands = [
        ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', '-I', str(RK / 'phone'),
         'tests/cross_motor_vectors.cc', str(RK / 'phone/motor_link.cc'), '-o', str(BUILD / 'vectors')],
        [str(BUILD / 'vectors'), 'generate', str(BUILD / 'mot1.hex')],
        ['iverilog', '-g2012', '-s', 'tb_motor_crosswire', '-o', str(BUILD / 'test'),
         'rtl/motor_packet_rx.v', 'rtl/motor_remote_control.v', 'rtl/motor_status_tx.v',
         'imu/imu_ethernet_tx.v', 'tests/tb_motor_crosswire.v', str(IDE / 'simlib/gw5a/prim_sim.v')],
        ['vvp', str(BUILD / 'test')],
        [str(BUILD / 'vectors'), 'verify', str(BUILD / 'ack1.hex')],
    ]
    results = []
    for command in commands:
        result = subprocess.run(command, cwd=PROJECT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        results.append(result.stdout)
        print(result.stdout, end='')
        if result.returncode:
            raise RuntimeError('Crosswire command failed: ' + repr(command))
    (REPORT / 'regression.log').write_text(''.join(results))
    for name in ('mot1.hex', 'ack1.hex'):
        shutil.copy2(BUILD / name, REPORT / name)
    report = {'date': '2026-10-20', 'physical_hardware_tested': False, 'passed': True,
              'scope': 'actual RK wire encoder -> vendor IDDR -> parser/FCS -> command CDC -> motor controller -> status CDC/ACK -> actual RK decoder',
              'clock_model': '100kHz motor and250kHz RGMII; production50:125 ratio retained; actual50MHz20kHz covered separately',
              'left_mask': 10, 'invert_mask': 0, 'motor_pins': expected,
              'states': ['STOP/config', 'arm-zero', 'forward', 'reverse', 'left-differential',
                         'right-differential', 'CW', 'CCW', 'armed-zero', 'STOP', 'forward-then-500ms-timeout'],
              'ack_frames_verified': sum(line == 'END' for line in (BUILD / 'ack1.hex').read_text().splitlines()),
              'source_sha256': {str(path.relative_to(PROJECT)): sha(path) for path in (
                  PROJECT / 'rtl/motor_packet_rx.v', PROJECT / 'rtl/motor_remote_control.v',
                  PROJECT / 'rtl/motor_status_tx.v', PROJECT / 'imu/imu_ethernet_tx.v',
                  PROJECT / 'tests/cross_motor_vectors.cc', PROJECT / 'tests/tb_motor_crosswire.v')},
              'rk_encoder_decoder_source': str(RK / 'phone/motor_link.cc'),
              'rk_encoder_decoder_sha256': sha(RK / 'phone/motor_link.cc'),
              'bitstream_sha256': sha(PROJECT / 'impl/pnr/robot_fmc_remote_softcore_v13.fs')}
    (REPORT / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
