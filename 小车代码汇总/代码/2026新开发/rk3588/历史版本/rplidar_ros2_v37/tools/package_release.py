#!/usr/bin/env python3
"""Verify frozen evidence and create a source release; never builds or claims ROS runtime."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads((ROOT / path).read_text())


def check_hashes(mapping, base):
    for name, expected in mapping.items():
        actual = base / name
        if not actual.is_file() or digest(actual) != expected:
            raise RuntimeError('Stale verification evidence: ' + str(actual))
    return len(mapping)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # Keep the original compiler/test records, but not transient object files.
    for folder, target in [('driver-host', 'driver-host.json'),
                           ('sdk-probe-arm64', 'sdk-probe-arm64.json'),
                           ('sdk-probe-host-cli', 'sdk-probe-host-cli.json')]:
        source = ROOT / 'build' / folder / 'verification.json'
        if source.is_file():
            shutil.copyfile(source, ROOT / 'reports' / target)
    driver = read('reports/driver-host.json')
    arm = read('reports/sdk-probe-arm64.json')
    cli = read('reports/sdk-probe-host-cli.json')
    bridge = read('ros2_ws/src/acg138_fpga_map_bridge/tests/reports/host-verification.json')
    base = read('reports/base-test.json')
    bringup = read('reports/bringup-test.json')
    deploy = read('reports/deploy-verification.json')
    assert driver['status'] == arm['status'] == cli['status'] == 'PASS'
    assert base['result'] == bringup['result'] == 'PASS'
    assert all(case['passed'] for case in bridge['results'])
    assert deploy['passed'] == len(deploy['checks']) == 48
    count = check_hashes(driver['source_sha256'], ROOT / 'ros2_ws/src/acg138_rplidar')
    count += check_hashes(arm['source_sha256'], ROOT)
    count += check_hashes(bridge['sha256'], ROOT / 'ros2_ws/src/acg138_fpga_map_bridge')
    count += check_hashes(base['source_sha256'], ROOT)
    count += check_hashes(bringup['source_sha256'], ROOT)
    assert digest(ROOT / 'tools/sdk-probe/lidar_scan_probe.cpp') == cli['source_sha256']
    assert digest(ROOT / arm['output']) == arm['output_sha256']
    assert digest(ROOT / 'deploy/bin/mountns-exec.aarch64') == deploy['helper_sha256']
    assert all(case['actual_exit'] == case['expected_exit'] for case in cli['cases'])
    for name in ['deploy/bin/lidar-scan-probe.aarch64', 'deploy/bin/mountns-exec.aarch64']:
        elf = (ROOT / name).read_bytes()
        assert elf[:5] == b'\x7fELF\x02' and int.from_bytes(elf[18:20], 'little') == 183
    evidence = {
        'release': 'acg138-rplidar-ros2-v37-20261029',
        'generated_for': 'Existing RK3588 Buildroot Linux 6.1.75 aarch64',
        'laser_mount_m': [0.0, 0.0, 0.175], 'laser_yaw_rad': 0.0,
        'baudrate': 460800,
        'evidence_source_hash_entries_verified': count,
        'host_validation': {
            'scan_core_regular_and_sanitizers': driver['core'],
            'real_sdk_linux_sources_compiled': driver['sdk_source_files_compiled'],
            'sdk_api_contract': driver['api_contract'],
            'probe_host_cli_cases': len(cli['cases']),
            'map_bridge_regular_and_sanitizers': [x['output'] for x in bridge['results']],
            'base_odometry_and_real_kernel_timestamp': base['output'],
            'bringup_static_composition_checks': bringup['checks'],
            'deployment_fault_injection_checks': deploy['passed']
        },
        'arm64_binaries_cross_compiled': {
            arm['output']: arm['output_sha256'],
            'deploy/bin/mountns-exec.aarch64': deploy['helper_sha256']
        },
        'not_completed_or_not_verified': [
            'Full ROS 2 image build and real ROS node compilation',
            'ROS synthetic LaserScan -> slam_toolbox -> saved map runtime smoke test',
            'Real C1 ROS scan, TF, wheel odometry, mapping and AMCL board acceptance',
            'Live RGMII map upload ACK with the user flashed bitstream',
            'YOLO coexistence latency and memory measurement on the board',
            'Buildroot-native complete ROS core/DDS/Python/Ceres port'
        ],
        'default_behavior': 'Read-only scan/odometry and dry-run map bridge; no vehicle motor messages',
        'fpga_scope': 'Reuse existing NAV2 64x64 A* protocol. No new bitstream, no path follower, no automatic arm.',
        'full_ros_image_included': False,
        'host_system_replacement_included': False,
        'hardware_tested': False
    }
    (ROOT / 'reports/交付验证.json').write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + '\n')
    excluded = {'build', '__pycache__', '.git', 'CHECKSUMS.sha256'}
    files = [p for p in sorted(ROOT.rglob('*')) if p.is_file() and not any(part in excluded for part in p.relative_to(ROOT).parts)]
    if any(p.is_symlink() for p in files):
        raise RuntimeError('Unexpected symlink in source release')
    manifest = ''.join(digest(p) + '  ' + p.relative_to(ROOT).as_posix() + '\n' for p in files)
    (ROOT / 'CHECKSUMS.sha256').write_text(manifest)
    files.append(ROOT / 'CHECKSUMS.sha256')
    archive = args.output / 'acg138-rplidar-ros2-v37-20261029.tar.gz'
    if archive.exists():
        raise RuntimeError('Refusing to overwrite existing release ' + str(archive))
    with tarfile.open(archive, 'w:gz') as tf:
        for p in files:
            tf.add(p, arcname='rplidar-ros2-v37/' + p.relative_to(ROOT).as_posix(), recursive=False)
    # Validate the archive bytes independently, not just the source tree.
    with tarfile.open(archive, 'r:gz') as tf:
        members = tf.getmembers()
        assert len(members) == len(files) and all(m.isfile() for m in members)
        names = {m.name: m for m in members}
        for row in manifest.splitlines():
            expected, name = row.split('  ', 1)
            content = tf.extractfile(names['rplidar-ros2-v37/' + name]).read()
            assert hashlib.sha256(content).hexdigest() == expected
    (Path(str(archive) + '.sha256')).write_text(digest(archive) + '  ' + archive.name + '\n')
    shutil.copyfile(ROOT / 'README_部署与联调.txt', args.output / 'rplidar-v37_部署与联调.txt')
    print(json.dumps({'archive': str(archive), 'bytes': archive.stat().st_size, 'files': len(files),
                      'sha256': digest(archive), 'report': str(ROOT / 'reports/交付验证.json')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
