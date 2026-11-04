#!/usr/bin/env python3
"""Bind completed shell fixture tests to final payloads and sealed update bytes."""
from pathlib import Path
import hashlib,json
P=Path(__file__).resolve().parents[1];R=P/'reports';h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
pairs=[line.split() for line in (P/'baseline-pairs.sha256').read_text().splitlines() if line.strip()]
combinations=len(pairs);mixed=len({v[0] for v in pairs})*len({v[1] for v in pairs})-combinations
install_cases=combinations*4
records={n:json.loads((R/n).read_text()) for n in ('installer-posix.json','installer-busybox.json')}
for name,r in records.items():
 assert r['status']=='passed' and not r['arm_programs_executed'] and not r['hardware_accessed']
 assert r['config_tokens_launchers_byte_exact'] and r['later_config_edits_retained_on_rollback']
 assert r['tested_runtime_manifest_sha256']==h(P/'SHA256SUMS')
 for path,digest in r['tested_runtime_files_sha256'].items():assert h(P/path)==digest,path
 assert sum(c.startswith('reject mixed known baseline') for c in r['cases'])==mixed
 assert sum(c.startswith('complete three-file exact install/rollback') for c in r['cases'])==install_cases
assert records['installer-posix.json']['shell']=='host sh' and records['installer-busybox.json']['shell']=='busybox sh'
for n,build in [('perception-dashboard.aarch64','dashboard'),('robot-navigation.aarch64','navigation')]:
 p=P/'payload'/n;assert h(p)==h(P.parent/build/'build'/n);data=p.read_bytes();assert data[:6]==b'\x7fELF\x02\x01' and data[18:20]==b'\xb7\x00'
assert 'PASS 15 export checks' in (R/'export-tests.log').read_text()
sources=['common.sh','install.sh','rollback.sh','seal_package.py','tests/test_installer.py','tests/test_export.py','tests/collect_installer_verification.py','diagnose-phone.sh','export-wrong.sh','baselines.sha256','baseline-pairs.sha256','package-mode.txt','profile.json','使用说明.txt','故障分类与恢复边界.txt']
r={'result':'PASS','version':'v33-scan-lease-20261028','hardware_tested':False,'arm_payload_executed':False,'runtime_manifest_sha256':h(P/'SHA256SUMS'),'dashboard_sha256':h(P/'payload/perception-dashboard.aarch64'),'navigation_sha256':h(P/'payload/robot-navigation.aarch64'),'source_sha256':{n:h(P/n) for n in sources},'evidence_sha256':{str(p.relative_to(P)):h(p) for p in (R/'installer-posix.json',R/'installer-busybox.json',R/'export-tests.log')},'per_shell_checks':{n:r['checks'] for n,r in records.items()},'per_shell_cases':{n:len(r['cases']) for n,r in records.items()},'approved_pair_fixture_combinations':combinations,'install_alias_calibration_cases_per_shell':install_cases,'rejected_mixed_known_pairs_per_shell':mixed,'behavior':'Only dashboard/navigation/dashboard.sha256 change as an atomic group with backup/verified rollback; config/tokens/models/data/launchers unchanged; unknown/mixed/active/symlink/corrupt/newer modifications rejected; failure injection before/after/corruption/signal on each file recovers whole group','limitations':['Shell tests execute host POSIX/BusyBox against temporary fixture roots with mocked uname/id','Actual AArch64 ELF payload bytes are read/copied/hashed only, never executed','No physical RK3588, motor command, network connection, FPGA flash or APK install performed']}
(R/'installer-verification.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n');print(h(R/'installer-verification.json'))
