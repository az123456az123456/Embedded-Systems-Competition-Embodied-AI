#!/usr/bin/env python3
"""Validate the actual module against the BSP export table and frozen inputs."""
import hashlib
import json
import pathlib
import subprocess
import sys
import tarfile

root = pathlib.Path(__file__).resolve().parent
bsp = pathlib.Path(sys.argv[1]).resolve()
symvers = pathlib.Path(sys.argv[2]).resolve()
cross = bsp / 'sdk/yy3588_extras/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-'
module = root / 'build/source/8821cu.ko'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
def run(*args):
    return subprocess.check_output(list(map(str, args)), text=True).strip()
def info(field):
    return run('modinfo', '-F', field, module)

assert info('vermagic') == '6.1.75 SMP mod_unload aarch64'
assert info('name') == '8821cu'
assert info('depends') == '', 'Unexpected separate module dependency'
assert 'usb:v0BDApC820d*dc*dsc*dp*icFFiscFFipFFin*' in info('alias').splitlines()
assert 'Machine:                           AArch64' in run(str(cross)+'readelf', '-h', module)
for param in ['rtw_ips_mode', 'rtw_enusbss', 'rtw_power_mgnt']:
    assert any(s.startswith(param+':') for s in info('parm').splitlines())
exports = {line.split()[1]: line.split()[2] for line in symvers.read_text().splitlines()}
undefined = sorted(set(line.split()[-1] for line in run(str(cross)+'nm', '-u', module).splitlines()))
missing = sorted(set(undefined) - exports.keys())
assert not missing, missing
nonbuiltin = {name: exports[name] for name in undefined if exports[name] != 'vmlinux'}
assert not nonbuiltin, nonbuiltin
runtime = (bsp/'configs/kernel.config').read_text()
prepared = (root/'build/kernel/.config').read_text()
assert prepared == runtime.replace('CONFIG_LOCALVERSION_AUTO=y', '# CONFIG_LOCALVERSION_AUTO is not set')
assert '# CONFIG_MODVERSIONS is not set' in runtime
assert not (bsp/'sdk/yy3588_linux/.config').exists(), 'BSP source tree should stay unconfigured'
archive = root/'downloads/rtl8821cu-5b39398e2de146edeb76716420f3288f508bea61.tar.gz'
assert sha(archive) == '3ccec54462a8668142a9c2b2ddfffe4e0434feebe6caf6a7de6812dc8713c37f'
with tarfile.open(archive) as tf:
    checked = 0
    for member in tf.getmembers():
        if member.isfile():
            relative = pathlib.PurePosixPath(*pathlib.PurePosixPath(member.name).parts[1:])
            assert (root/'build/source'/relative).read_bytes() == tf.extractfile(member).read(), str(relative)
            checked += 1
assert sha(root/'build/source/LICENSE') == '22cdb0a9cf492dbf05c5b27381a5c0e73ae0d4622427b522b0ba92b7495e5fb0'
log = (root/'reports/build.log').read_text()
assert not any(token in log for token in ['error:', 'ERROR:', 'warning:', 'WARNING:'])
report = {
    'date': '2026-10-17',
    'target': {'usb_id': '0bda:c820', 'arch': 'aarch64', 'kernel': '6.1.75'},
    'source': {'repository': 'https://github.com/morrownr/8821cu-20210916', 'commit': '5b39398e2de146edeb76716420f3288f508bea61', 'archive_sha256': sha(archive), 'license': 'GPL-2.0', 'unmodified_upstream_files_verified': checked},
    'bsp': {'name': 'ACG138-Offline-BSP-20260923-r4', 'runtime_config_sha256': hashlib.sha256(runtime.encode()).hexdigest(), 'prepared_config_sha256': hashlib.sha256(prepared.encode()).hexdigest(), 'symvers_sha256': sha(symvers), 'symvers_source': 'rk3588_acg138/13_FPGA_RGMII/03_测试模块/Module.symvers', 'symvers_source_commit': '1ccf6a8559a963e97328228704fd9e08ece3f830', 'compiler': run(str(cross)+'gcc', '--version').splitlines()[0]},
    'board_evidence': {'source': 'User supplied terminal output, 2026-10-17', 'config_matches': True, 'kernel_build': '#1 SMP Wed Sep 23 06:54:46 UTC 2026', 'driver_load_test': 'NOT PERFORMED: board not accessible from this workspace'},
    'module': {'sha256': sha(module), 'size': module.stat().st_size, 'vermagic': info('vermagic'), 'srcversion': info('srcversion'), 'dependencies': [], 'firmware': 'embedded in module', 'usb_alias_c820': True, 'undefined_symbols': len(undefined), 'missing_exports': missing, 'nonbuiltin_dependencies': nonbuiltin},
    'build': {'external_module_only': True, 'bsp_source_modified': False, 'warnings': 0, 'errors': 0},
}
(root/'reports/verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
(root/'reports/required-symbols.txt').write_text('\n'.join(undefined)+'\n')
print(json.dumps(report, ensure_ascii=False, indent=2))
