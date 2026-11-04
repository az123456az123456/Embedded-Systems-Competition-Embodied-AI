#!/usr/bin/env python3
"""Package the audited v26 FPGA image, source and frozen verification baseline."""
import hashlib
import json
import re
from pathlib import Path
import shutil
import subprocess
import tempfile
import zipfile

PROJECT = Path(__file__).resolve().parents[1]
WORKSPACE = PROJECT.parents[1]
OUT = WORKSPACE / 'outputs/01a13005-8bda-7393-95ca-5157476f342f'
NAME = 'acg138-sonar-moving-average-v26-20261027'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    audit = json.loads((PROJECT/'reports/release_verification.json').read_text())
    assert audit['result'] == 'PASS' and audit['hardware_tested'] is False
    assert audit['timing']['setup_violated_endpoints'] == audit['timing']['hold_violated_endpoints'] == 0
    for name, expected in {**audit['synthesis_inputs'], **audit['auxiliary_inputs'],
                           audit['bitstream']: audit['bitstream_sha256']}.items():
        assert sha(PROJECT/name) == expected, name
    tests = ['sonar-ma-v26.log', 'sonar-capture-v26.log', 'sonar-uart.log',
             'selected-regressions.log', 'rk-sns-replay.log']
    for name in tests:
        text = (PROJECT/'reports'/name).read_text()
        assert 'PASS' in text and not any(bad in text for bad in ('FATAL:', 'make: ***', 'AssertionError')), name
    model = re.search(r'PASS sonar MA v26: (\d+) checks, (\d+) frames \((\d+) seeded random\)',
                      (PROJECT/'reports/sonar-ma-v26.log').read_text())
    capture = re.search(r'real UART -> SNS1: (\d+) checks, (\d+) packets, (\d+)',
                        (PROJECT/'reports/sonar-capture-v26.log').read_text())
    replay = re.search(r'trace: (\d+) checks, (\d+) snapshots',
                       (PROJECT/'reports/rk-sns-replay.log').read_text())
    assert model and int(model[1]) >= 16000 and int(model[3]) >= 1200
    assert capture and int(capture[1]) >= 40 and int(capture[2]) >= 100
    assert replay and int(replay[1]) >= 8000 and int(replay[2]) >= 100
    for name in ('encoder-sync-unit.log', 'encoder-motor-integration.log'):
        assert 'PASS' in (PROJECT/'reports'/name).read_text(), name
    image = OUT/'robot_control_v26_sonar_ma.fs'
    archive = OUT/(NAME+'.zip')
    notes = OUT/'超声波滑动平均_v26_烧录说明.txt'
    proof_path = OUT/(NAME+'-verification.json')
    checksums = OUT/(NAME+'.SHA256SUMS')
    for path in (image, archive, notes, proof_path, checksums):
        assert not path.exists(), path
    OUT.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='acg138-v26-delivery-') as tmp:
        stage = Path(tmp)/NAME
        target = stage/'project'/PROJECT.name
        excluded = {'build', 'impl', 'reports', '__pycache__', '.git', '.local'}
        shutil.copytree(PROJECT, target, ignore=lambda folder, names: [n for n in names if n in excluded])
        shutil.copytree(PROJECT/'reports', target/'reports')
        (target/'impl/pnr').mkdir(parents=True)
        shutil.copy2(PROJECT/audit['bitstream'], target/audit['bitstream'])
        baseline = json.loads((PROJECT/'tools/baseline_v25_manifest.json').read_text())
        original = (PROJECT/baseline['baseline_directory']).resolve()
        baseline_target = target.parent/original.name
        required = {**baseline['synthesis_inputs'], **baseline['auxiliary_inputs'],
                    **baseline['firmware_sources'], **baseline['firmware_artifacts'],
                    **baseline['rtl_source_tree'],
                    baseline['release_verification']: baseline['release_verification_sha256'],
                    baseline['bitstream']: baseline['bitstream_sha256']}
        for name, expected in required.items():
            assert sha(original/name) == expected, name
            path = baseline_target/name
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(original/name, path)
        # Confirm the delivered relative directory layout can run the source gate.
        subprocess.run(['python3', 'tools/audit_release.py', '--prepare-only'], cwd=target,
                       check=True, stdout=subprocess.DEVNULL)
        shutil.copy2(PROJECT/audit['bitstream'], stage/image.name)
        rk_base = WORKSPACE/'rk3588/robot_straight_v27_2/dashboard'
        rk_sources = ('src/sensor_data.c', 'src/sensor_data.h', 'tests/test_fpga_sonar_trace.c')
        rk_hashes = {}
        for name in rk_sources:
            target_source = stage/'rk-replay'/Path(name).name
            target_source.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(rk_base/name, target_source)
            rk_hashes[name] = sha(rk_base/name)
        (stage/'rk-replay/README.txt').write_text(
            '原样保留的现有RK解析/回放源码，仅用于开发电脑复现协议兼容检查，不是RK安装更新。\n'
            'cc -O2 -I . sensor_data.c test_fpga_sonar_trace.c -o replay\n'
            './replay ../project/robot_control_v26_sonar_ma/reports/sonar-v26-sns.hex\n')
        text = (PROJECT/'README.md').read_text()
        text += '\n\n交付目录说明\n只烧录根目录的 robot_control_v26_sonar_ma.fs 即可启用本版。'
        text += '\n完整源码位于 project/robot_control_v26_sonar_ma。相邻的robot_control_v25仅是冻结基线校验所需文件，请保留二者相对位置；不要将旧基线位流当成本次更新。'
        text += f'\n验证通过：滑动平均{model[1]}项检查；实际UART到SNS1的{capture[1]}项检查、{capture[2]}条快照；现有RK解码回放{replay[1]}项检查。'
        text += f"\n实际内部STA：setup违例0，最小余量{audit['timing']['setup_minimum_slack_ns']:.3f}ns；hold违例0，最小余量{audit['timing']['hold_minimum_slack_ns']:.3f}ns。"
        text += '\n资源：' + '，'.join(f"{key} {v['used']}/{v['total']}" for key, v in audit['resources'].items()) + '。'
        text += '\n此包未进行实体车测试。\n'
        (stage/notes.name).write_text(text)
        report = {'result': 'PASS', 'hardware_tested': False,
                  'bitstream_sha256': audit['bitstream_sha256'], 'project': PROJECT.name,
                  'sources_sha256': {**audit['synthesis_inputs'], **audit['auxiliary_inputs']},
                  'test_logs_sha256': {name: sha(PROJECT/'reports'/name) for name in tests},
                  'timing': audit['timing'], 'resources': audit['resources'],
                  'pins_verified': audit['pins_verified'], 'sonar_contract': audit['sonar_v26'],
                  'unchanged_rk_replay_sources_sha256': rk_hashes,
                  'tests': {'mean_checks': int(model[1]), 'mean_frames': int(model[2]),
                            'random_frames': int(model[3]), 'uart_scenarios': int(capture[1]),
                            'uart_packets': int(capture[2]), 'uart_age_checks': int(capture[3]),
                            'rk_replay_checks': int(replay[1]), 'rk_replay_snapshots': int(replay[2])},
                  'rk_update_required': False, 'apk_update_required': False,
                  'portable_baseline_source_gate': 'PASS', 'production_source_changes': audit['changed_sources']}
        (stage/'verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        hashes = {str(p.relative_to(stage)): sha(p) for p in sorted(stage.rglob('*')) if p.is_file()}
        assert not any(p.is_symlink() for p in stage.rglob('*'))
        assert not any(name.endswith(('.key', '.pem', '.jks', '.keystore', '.lic')) for name in hashes)
        (stage/'SHA256SUMS').write_text(''.join(f'{digest}  {name}\n' for name, digest in hashes.items()))
        staged_archive = Path(tmp)/(NAME+'.zip')
        with zipfile.ZipFile(staged_archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
            for path in sorted(stage.rglob('*')):
                if path.is_file():
                    z.write(path, str(path.relative_to(Path(tmp))))
        with zipfile.ZipFile(staged_archive) as z:
            assert z.testzip() is None
            for name, expected in hashes.items():
                assert hashlib.sha256(z.read(NAME+'/'+name)).hexdigest() == expected, name
        shutil.copy2(stage/image.name, image)
        shutil.copy2(stage/notes.name, notes)
        shutil.copy2(staged_archive, archive)
        report.update(archive_sha256=sha(archive), files_verified=len(hashes),
                      archive=str(archive), bitstream=str(image), notes=str(notes))
        proof_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        checksums.write_text(''.join(f'{sha(p)}  {p.name}\n' for p in (image, archive, notes)))
        print(json.dumps({k: report[k] for k in ('result', 'hardware_tested', 'bitstream',
              'bitstream_sha256', 'archive', 'archive_sha256', 'files_verified')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
