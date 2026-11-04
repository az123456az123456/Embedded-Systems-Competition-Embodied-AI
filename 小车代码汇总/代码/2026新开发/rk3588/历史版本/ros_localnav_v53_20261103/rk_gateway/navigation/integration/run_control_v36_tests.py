#!/usr/bin/env python3
"""Build/run production control regressions; binds loopback only, never physical IO."""
import hashlib,json,pathlib,subprocess,time
root=pathlib.Path(__file__).resolve().parents[2]
d=root/'dashboard';n=root/'navigation';reports=n/'reports';reports.mkdir(exist_ok=True)
input_files=[*sorted((d/'phone').glob('*.[ch]*')), *sorted((n/'vendor').glob('*.[ch]*')), n/'gateway.cc',n/'gateway.h',n/'integration/test_control_session_v36.cc',d/'tests/test_control_authority_v36.cc',pathlib.Path(__file__).resolve()]
input_hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in input_files if p.is_file()}
subprocess.run(['make','-C',str(d),'build/phone-publish-probe','-j2'],check=True)
common=['-std=c++17','-O2','-Wall','-Wextra','-Werror','-pthread']
authority=d/'build/test-control-authority-v36'
subprocess.run(['g++',*common,str(d/'tests/test_control_authority_v36.cc'),str(d/'phone/controller.cc'),str(d/'phone/follow_geometry.cc'),'-o',str(authority)],check=True)
production=n/'build/test-control-session-v36'
sources=[n/'integration/test_control_session_v36.cc',n/'gateway.cc',d/'phone/controller.cc',d/'phone/follow_geometry.cc',d/'phone/motor_link.cc',d/'phone/http_server.cc',d/'phone/json_flat.cc']
objects=[d/'build/phone-publish-imu.o',d/'build/phone-publish-sensors.o',d/'build/phone-publish-jpeg-turbo-x64.o',d/'vendor/jpeg_turbo/Linux/x64/libturbojpeg.a',d/'build/obj-host/common/wrong_log.o']
subprocess.run(['g++',*common,*['-I'+str(d/p) for p in ('phone','src','vendor','assets')],*map(str,sources+objects),'-lm','-o',str(production)],check=True)
results=[]
for name,binary in [('authority',authority),('production-http',production)]:
 t=time.monotonic();log=reports/('control-v36-'+name+'.log')
 with log.open('w') as out:r=subprocess.run([str(binary)],stdout=out,stderr=subprocess.STDOUT,timeout=65)
 results.append({'name':name,'returncode':r.returncode,'elapsed_s':time.monotonic()-t,'log':str(log.relative_to(root))})
 if r.returncode:print(log.read_text());break
for relative,expected in input_hashes.items():
 assert hashlib.sha256((root/relative).read_bytes()).hexdigest()==expected,'Source changed during build/test: '+relative
files=[*sources,d/'phone/controller.h',d/'phone/motor_link.h',d/'phone/phone_service.cc',d/'phone/control_audit.h',n/'gateway.h',d/'tests/test_control_authority_v36.cc',pathlib.Path(__file__).resolve()]
report={'status':'PASS' if len(results)==2 and all(x['returncode']==0 for x in results) else 'FAIL','scope':'production Controller + production phone_service + production Gateway, loopback only, no camera/NPU/FPGA','cases':results,'source_sha256':input_hashes}
(reports/'control-v36-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2));raise SystemExit(0 if report['status']=='PASS' else 1)
