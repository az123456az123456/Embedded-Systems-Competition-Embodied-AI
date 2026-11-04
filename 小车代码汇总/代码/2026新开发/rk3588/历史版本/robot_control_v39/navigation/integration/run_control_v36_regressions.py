#!/usr/bin/env python3
import hashlib,json,pathlib,subprocess,time
root=pathlib.Path(__file__).resolve().parents[2];d=root/'dashboard';n=root/'navigation';reports=n/'reports';reports.mkdir(exist_ok=True)
targets=['test-phone-controller','test-velocity-control','test-manual-arc','test-angle-follow','test-fpga-follow-geometry','test-control-clock-race','test-phone-wire','test-softcore-nav1','test-control-audit']
files=[*sorted((d/'phone').glob('*.[ch]*')),d/'Makefile',n/'Makefile',root/'common/wrong_log.cc',root/'common/wrong_log.h',n/'gateway.cc',n/'gateway.h',n/'tests/test_gateway_concurrency.cc',pathlib.Path(__file__).resolve(),d/'tests/test_control_backpressure_v36.py',*[d/'tests'/x for x in ('test_phone_controller.cc','test_velocity_control.cc','test_manual_arc.cc','test_angle_follow.cc','test_fpga_follow_geometry.cc','test_control_clock_race.cc','test_phone_wire.cc','test_softcore_nav1.cc','test_control_audit.cc')]]
inputs={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}
subprocess.run(['make','-C',str(d),*[f'build/{t}' for t in targets],'-j2'],check=True)
subprocess.run(['make','-C',str(n),'build/wrong-log.o'],check=True)
concurrent=n/'build/test-gateway-concurrency-v36'
subprocess.run(['g++','-O2','-std=c++17','-Wall','-Wextra','-pthread',str(n/'gateway.cc'),str(n/'vendor/http_server.cc'),str(n/'vendor/json_flat.cc'),str(n/'tests/test_gateway_concurrency.cc'),str(n/'build/wrong-log.o'),'-o',str(concurrent)],check=True)
results=[]
for name,binary in [(t,d/'build'/t) for t in targets]+[('gateway-concurrency',concurrent),('stderr-backpressure',d/'tests/test_control_backpressure_v36.py')]:
 start=time.monotonic();log=reports/('control-v36-'+name+'.log')
 with log.open('w') as out:result=subprocess.run((['python3',str(binary)] if binary.suffix=='.py' else [str(binary)]),stdout=out,stderr=subprocess.STDOUT,timeout=20)
 results.append({'name':name,'returncode':result.returncode,'elapsed_s':time.monotonic()-start,'log':str(log.relative_to(root))})
 if result.returncode:print(log.read_text());break
for relative,expected in inputs.items():assert hashlib.sha256((root/relative).read_bytes()).hexdigest()==expected,'Sources changed during regressions: '+relative
report={'status':'PASS' if len(results)==len(targets)+2 and all(r['returncode']==0 for r in results) else 'FAIL','scope':'native control, existing tracking/bearing/geometry/manual slew, real ACK/NAV codecs, clock schedule and real localhost delayed ARM/DRIVE/GET/STOP races','cases':results,'source_sha256':inputs}
(reports/'control-v36-regressions.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2));raise SystemExit(0 if report['status']=='PASS' else 1)
