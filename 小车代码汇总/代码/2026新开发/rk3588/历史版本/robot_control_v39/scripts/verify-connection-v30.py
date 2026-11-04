#!/usr/bin/env python3
"""Verify frozen connection/lifecycle proof; --run repeats host tests.
The native HTTP tests bind only loopback; constrained sandboxes may need local
socket permission. No radio, Android device, motor or FPGA is accessed.
"""
from pathlib import Path
import argparse,hashlib,json,subprocess
R=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');args=p.parse_args()
if args.run:
 subprocess.run(['sh',str(R/'navigation/android/test-connection-v30.sh')],cwd=R,check=True)
 for name in ['test_gateway_concurrency','test_gateway_wait_liveness']:
  dest=R/'navigation/build'/name.replace('_','-');dest.parent.mkdir(exist_ok=True,parents=True)
  subprocess.run(['c++','-O2','-std=c++17','-Wall','-Wextra','-pthread','navigation/gateway.cc','navigation/vendor/http_server.cc','navigation/vendor/json_flat.cc','common/wrong_log.cc','navigation/tests/'+name+'.cc','-o',str(dest)],cwd=R,check=True)
  subprocess.run([str(dest)],cwd=R,check=True)
report=json.loads((R/'reports/connection-v30-verification.json').read_text())
for kind in ['source_sha256','evidence_sha256']:
 for name,sha in report[kind].items():
  assert hashlib.sha256((R/name).read_bytes()).hexdigest()==sha,(kind,name)
assert report['result']=='PASS' and not report['hardware_tested']
assert report['same_signer'] and report['certificate_sha256']==report['previous_certificate_sha256']
assert hashlib.sha256((R/'release/ACG138-Robot-Control-v30.apk').read_bytes()).hexdigest()==report['android_apk_sha256']
print('PASS frozen source, evidence, APK and same-signer connection/lifecycle verification')
