#!/usr/bin/env python3
"""Package verified fast-v10.1 artifacts. Models, system libraries and secrets are excluded."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile
import zipfile

ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    parser=argparse.ArgumentParser();parser.add_argument('destination',type=Path)
    out=parser.parse_args().destination.resolve();out.mkdir(parents=True,exist_ok=True)
    binary=ROOT/'build/perception-dashboard.aarch64'
    elf=subprocess.check_output(['readelf','-h',str(binary)],text=True)
    assert 'AArch64' in elf
    deps=subprocess.check_output(['readelf','-d',str(binary)],text=True)
    libraries=re.findall(r'Shared library: \[([^\]]+)\]',deps)
    assert 'librknnrt.so' in libraries and 'libwayland-client.so.0' in libraries
    tests=['host-tests.log','ui-render-tests.log','npu-performance-tests.log','cpu-performance-tests.log','window-tests.log','diagnostics-tests.log',
           'compare-tests.log','launcher-tests.log','install-tests.log']
    for name in tests:
        value=(ROOT/'reports'/name).read_text()
        assert ('PASS' in value or '\nOK\n' in value) and 'Traceback' not in value and 'FAIL' not in value,name
    code={str(p.relative_to(ROOT)):sha(p) for d in ['src','scripts','tests'] for p in sorted((ROOT/d).rglob('*'))
          if p.is_file() and '__pycache__' not in p.parts and p.suffix not in {'.pyc'}}
    verification=dict(version='perception-dashboard fast-v10.1 20261019',base='fast-v10',
        architecture='AArch64',binary_sha256=sha(binary),runtime_dependencies=libraries,
        hardware_tested=False,model_changed=False,resolution_changed=False,
        capture_resolution=[1920,1080],preview_resolution=[864,486],default_npu_workers=3,
        optimized=['paired/NEON YUV conversion','cached pixel-wise bilinear plan','byte-exact opaque UI fills','persistent normalized UINT8 RKNN input'],
        fallback='legacy input for missing APIs or query/alloc/bind/cache/run failure; restore original core',
        capture_timestamps='trusted driver monotonic timestamp; else pre-conversion dequeue time',
        frequency_changes_on_install=False,optional_performance_governors=True,local_test_logs=tests,source_sha256=code)
    (ROOT/'reports/verification.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2)+'\n')
    artifacts=[]
    with tempfile.TemporaryDirectory(prefix='perception-fast-v10.1-package-') as temp:
        update=Path(temp)/'perception-dashboard-v10.1-update';update.mkdir()
        mapping={'build/perception-dashboard.aarch64':'perception-dashboard.aarch64',
                 'run-dashboard.sh':'run-dashboard.sh','scripts/install-v10.1.sh':'install.sh',
                 'scripts/rollback-v10.1.sh':'rollback.sh','scripts/diagnose-performance.sh':'diagnose-performance.sh',
                 'scripts/compare-performance.py':'compare-performance.py','README.md':'安装与对比说明.md',
                 'build/bench-camera.aarch64':'bench-camera.aarch64',
                 'build/bench-preprocess.aarch64':'bench-preprocess.aarch64',
                 'build/bench-ui-render.aarch64':'bench-ui-render.aarch64',
                 'scripts/npu-performance.sh':'npu-performance.sh',
                 'scripts/cpu-big-performance.sh':'cpu-big-performance.sh'}
        for source,name in mapping.items():
            shutil.copyfile(ROOT/source,update/name)
            (update/name).chmod(0o755 if name.endswith(('.sh','.aarch64','.py')) else 0o644)
        (update/'reports').mkdir()
        for name in tests+['verification.json','benchmark-host.log']:
            shutil.copyfile(ROOT/'reports'/name,update/'reports'/name)
        (update/'SHA256SUMS').write_text(''.join(sha(p)+'  '+str(p.relative_to(update))+'\n'
            for p in sorted(update.rglob('*')) if p.is_file()))
        archive=out/'perception-dashboard-fast-v10.1-arm64-20261019.tar.gz'
        with tarfile.open(archive,'w:gz') as tar:tar.add(update,arcname=update.name)
        with tarfile.open(archive) as tar:
            for line in tar.extractfile(update.name+'/SHA256SUMS').read().decode().splitlines():
                expected,name=line.split('  ',1)
                assert hashlib.sha256(tar.extractfile(update.name+'/'+name).read()).hexdigest()==expected
            assert not any(m.name.endswith(('.rknn','.onnx','.so','.fs','.keystore')) for m in tar.getmembers())
        artifacts.append(archive)
    source=out/'ACG138_YOLO集中优化_源码_v10.1_20261019.zip'
    with zipfile.ZipFile(source,'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(ROOT.rglob('*')):
            relative=p.relative_to(ROOT)
            if not p.is_file() or {'build','dist','__pycache__','.git','.local'}.intersection(relative.parts):continue
            if p.suffix in {'.rknn','.onnx','.pyc','.so','.keystore','.o'}:continue
            z.write(p,str(Path(ROOT.name)/relative))
        assert z.testzip() is None
    artifacts.append(source)
    readme=out/'YOLO集中优化_v10.1_安装与对比说明_20261019.md'
    shutil.copyfile(ROOT/'README.md',readme);artifacts.append(readme)
    for name in ['npu-performance.sh','cpu-big-performance.sh']:
        artifact=out/name;shutil.copyfile(ROOT/'scripts'/name,artifact);artifacts.append(artifact)
    manifest={'date':'2026-10-19','hardware_tested':False,'files':{p.name:{'bytes':p.stat().st_size,'sha256':sha(p)} for p in artifacts}}
    (out/'YOLO集中优化_v10.1_交付清单_20261019.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (out/'YOLO集中优化_v10.1_SHA256SUMS_20261019').write_text(''.join(sha(p)+'  '+p.name+'\n' for p in artifacts))
    print(json.dumps(manifest,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
