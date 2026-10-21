#!/usr/bin/env python3
"""Package the compiled module separately from rebuild sources; run all checks."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile

root = Path(__file__).resolve().parent
bsp, symvers, output = (Path(p).resolve() for p in sys.argv[1:])
subprocess.run([sys.executable, str(root/'verify-build.py'), str(bsp), str(symvers)], check=True, stdout=subprocess.DEVNULL)
report = json.loads((root/'reports/verification.json').read_text())
module_hash = report['module']['sha256']
srcversion = report['module']['srcversion']
board = root/'board'
init = board/'payload/S35rtl8821cu'
data = init.read_text()
old_module_hash = re.search(r'^module_hash=([0-9a-f]+)$', data, re.M).group(1)
old_srcversion = re.search(r'= ([0-9A-F]{16,32}) \]', data).group(1)
init.write_text(data.replace(old_module_hash, module_hash).replace(old_srcversion, srcversion))
common = board/'common.sh'
common.write_text(re.sub(r'^SRCVERSION=[0-9A-F]+$', 'SRCVERSION='+srcversion, common.read_text(), flags=re.M))
readme = board/'安装说明.txt'
readme.write_text(readme.read_text().replace(old_module_hash, module_hash))
subprocess.run([sys.executable, str(root/'test-installer.py'), str(bsp/'configs/kernel.config')], check=True)

dist = root/'dist'
dist.mkdir(exist_ok=True)
output.mkdir(parents=True, exist_ok=True)
runtime = dist/'rtl8821cu-rk3588-6.1.75'
source = dist/'rtl8821cu-rk3588-6.1.75-source'
for folder in (runtime, source):
    if folder.exists():
        shutil.rmtree(folder)  # Only these generated staging folders.
shutil.copytree(board, runtime)
shutil.copy2(root/'build/source/8821cu.ko', runtime/'payload/8821cu.ko')
shutil.copy2(root/'build/source/LICENSE', runtime/'LICENSE')
(runtime/'reports').mkdir()
for name in ['verification.json', 'installer-tests.json', 'modinfo.txt', 'elf-header.txt']:
    shutil.copy2(root/'reports'/name, runtime/'reports'/name)
for p in runtime.rglob('*'):
    if p.is_file(): p.chmod(0o755 if p.suffix == '.sh' or p.name == 'S35rtl8821cu' else 0o644)

source.mkdir()
for name in ['build-driver.sh', 'verify-build.py', 'test-installer.py', 'package.py', 'README.md']:
    shutil.copy2(root/name, source/name)
shutil.copy2(root/'build/source/LICENSE', source/'LICENSE')
shutil.copytree(board, source/'board')
shutil.copytree(root/'reports', source/'reports')
(source/'downloads').mkdir()
archive_name = 'rtl8821cu-5b39398e2de146edeb76716420f3288f508bea61.tar.gz'
shutil.copy2(root/'downloads'/archive_name, source/'downloads'/archive_name)
(source/'inputs').mkdir()
shutil.copy2(symvers, source/'inputs/Module.symvers')
shutil.copy2(bsp/'configs/kernel.config', source/'inputs/runtime.config')
shutil.copy2(root/'build/kernel/.config', source/'inputs/prepared.config')
for name in ['rtl8821cu.mk','rtl8821cu.hash']:
    shutil.copy2(bsp/'sdk/yy3588_buildroot/package/rtl8821cu'/name, source/'inputs'/name)

def checksums(folder):
    (folder/'SHA256SUMS').write_text(''.join(
        hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(folder))+'\n'
        for p in sorted(folder.rglob('*')) if p.is_file() and p.name != 'SHA256SUMS'))
def tarball(folder, dest):
    def normalize(info):
        info.uid = info.gid = 0
        info.uname = info.gname = 'root'
        info.mtime = 0
        return info
    checksums(folder)
    with dest.open('wb') as out:
        with gzip.GzipFile(fileobj=out, filename='', mode='wb', mtime=0) as gz:
            with tarfile.open(fileobj=gz, mode='w') as tf:
                tf.add(folder, arcname=folder.name, filter=normalize)

runtime_archive = output/'rtl8821cu-rk3588-6.1.75-driver-20261017.tar.gz'
source_archive = output/'rtl8821cu-rk3588-6.1.75-source-20261017.tar.gz'
tarball(runtime, runtime_archive)
tarball(source, source_archive)
instructions = output/'RTL8821CU驱动安装说明-20261017.txt'
shutil.copy2(readme, instructions)
(output/'RTL8821CU-SHA256SUMS-20261017.txt').write_text(''.join(
    hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n'
    for p in (runtime_archive, source_archive, instructions)))
for p in (runtime_archive, source_archive, instructions):
    print(p, p.stat().st_size)
