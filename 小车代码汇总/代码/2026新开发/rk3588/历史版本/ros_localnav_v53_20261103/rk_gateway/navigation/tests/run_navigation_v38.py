#!/usr/bin/env python3
"""Build actual v38 production code; test fixtures use only host/loopback IO.

Use --http-only in an environment allowing local TCP sockets. No physical
interface, robot, remote endpoint, Wi-Fi setting or motor is touched.
"""
import argparse, concurrent.futures, hashlib, json, os, subprocess, tempfile, time
from pathlib import Path
NAV=Path(__file__).resolve().parents[1]
RK=NAV.parent
ROOT=NAV.parents[2]
FW=ROOT/'fpga/robot_control_v38/softcore/firmware'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--http-only',action='store_true');p.add_argument('--skip-http',action='store_true');p.add_argument('--output',type=Path);p.add_argument('--reuse-compiled',type=Path,help='Reuse only SHA-verified binaries whose compiled inputs are unchanged; all tests execute again');args=p.parse_args()
 label='http' if args.http_only else 'core' if args.skip_http else 'all'
 output=(args.output or NAV/'tests/reports'/('v38-'+label)).resolve();output.mkdir(parents=True,exist_ok=True)
 build=Path(tempfile.mkdtemp(prefix='robot-nav-v38-'));wrong=RK/'common/wrong_log.cc'
 core=[NAV/'navcore.cc'];gateway=[NAV/'gateway.cc',NAV/'vendor/http_server.cc',NAV/'vendor/json_flat.cc']
 full=core+[NAV/'imu_alignment.cc',NAV/'sensor_bridge.cc',NAV/'protocol.cc',NAV/'map_snapshot.cc']+gateway
 cases=[]
 def add(name,file,sources,extra=(),sanitize=False,http=False):
  cases.append(dict(name=name,test=NAV/'tests'/file,sources=sources,extra=list(extra),sanitize=sanitize,http=http))
 add('source-grace-v38','test_source_grace_v38.cc',core)
 add('source-grace-v38-asan-ubsan','test_source_grace_v38.cc',core,sanitize=True)
 add('navcore','test_navcore.cc',core)
 add('straight-assist','test_straight_assist.cc',core,extra=('ASTAR',))
 add('reactive-avoidance','test_reactive_avoidance.cc',core,extra=('ASTAR',))
 add('reactive-velocity','test_reactive_avoidance.cc',core,extra=('ASTAR','-DV28_TEST_VELOCITY'))
 add('sonar-policy','test_sonar_avoidance_policy.cc',core)
 add('three-sonar-policy','test_three_sonar_policy.cc',core)
 add('direction-fallback','test_direction_fallback_v36.cc',core)
 add('direction-fallback-asan-ubsan','test_direction_fallback_v36.cc',core,sanitize=True)
 add('reactive-safety','test_reactive_safety_v33.cc',core)
 add('rotation-geometry','test_local_rotation_map.cc',core)
 add('encoder-window','test_encoder_window.cc',core)
 add('sonar-diagnostics','test_sonar_diagnostics.cc',[])
 add('sensor-bridge','test_sensor_bridge.cc',core+[NAV/'imu_alignment.cc',NAV/'sensor_bridge.cc'])
 add('imu-alignment','test_imu_alignment.cc',core+[NAV/'imu_alignment.cc'])
 add('source-clock','test_gateway_source_clock.cc',gateway,http=True)
 add('ack-recovery-v38','test_ack_recovery_v38.cc',gateway,http=True)
 add('video-proxy-v38','test_video_proxy_v38.cc',gateway,http=True)
 add('gateway-concurrency','test_gateway_concurrency.cc',gateway,http=True)
 add('gateway-straight','test_gateway_straight.cc',gateway,http=True)
 add('core-handoff','test_gateway_core_handoff.cc',full,http=True)
 add('preflight','test_gateway_preflight.cc',full,http=True)
 add('observation-barrier','test_gateway_observation_barrier.cc',gateway,http=True)
 add('wrong-log-causality','test_wronglog_navigation.cc',full,http=True)
 add('straight-runtime','test_straight_runtime.cc',full)
 add('navigation-timeline','test_navigation_timeline.cc',core,extra=('ASTAR',))
 cases=[c for c in cases if (c['http'] if args.http_only else not c['http'] if args.skip_http else True)]
 bound=set(NAV.glob('*.cc'))|set(NAV.glob('*.h'))|set(NAV.glob('*.inc'))|set((NAV/'vendor').glob('*.cc'))|set((NAV/'vendor').glob('*.h'))
 bound|={wrong,RK/'common/wrong_log.h',RK/'common/sonar_range.h',FW/'astar.c',FW/'astar.h',Path(__file__).resolve()}
 bound|={c['test'] for c in cases}
 if any(c['http'] for c in cases):bound|={NAV/'tests/test_gateway_straight.cc',NAV/'tests/test_gateway_core_handoff.cc'}
 bound|=set((NAV/'tests/fixtures').rglob('*.json'))
 source={str(x.relative_to(ROOT)):sha(x) for x in sorted(bound)}
 reused={};reuse_sha=None
 if args.reuse_compiled:
  original=args.reuse_compiled.resolve();reuse_sha=sha(original);old=json.loads(original.read_text())
  if old.get('result')!='PASS':raise RuntimeError('Only a prior fully passing build may be reused')
  for name,digest in source.items():
   if ROOT/name==Path(__file__).resolve():continue # orchestration is not a compiled input
   if old['source_sha256'].get(name)!=digest:raise RuntimeError('Compiled input changed: '+name)
  reused={c['name']:c for c in old['cases']}
  if set(reused)!={c['name'] for c in cases}:raise RuntimeError('Case selection changed')
 else:
  subprocess.run(['gcc','-O2','-c',str(FW/'astar.c'),'-o',str(build/'astar.o')],check=True)
 def execute(c):
  binary=build/c['name'];cmd=['g++','-std=c++17','-pthread','-Wall','-Wextra','-I'+str(FW)]
  cmd+=['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer'] if c['sanitize'] else ['-O2']
  cmd += [v for v in c['extra'] if v!='ASTAR']+[str(x) for x in c['sources']]+[str(c['test']),str(wrong)]
  if 'ASTAR' in c['extra']:cmd.append(str(build/'astar.o'))
  cmd+=['-o',str(binary)];began=time.monotonic();record=dict(name=c['name'],compile_command=cmd,http_loopback=c['http'])
  if reused:
   previous=reused[c['name']];previous_cmd=previous['compile_command'];binary=Path(previous_cmd[previous_cmd.index('-o')+1])
   if sha(binary)!=previous['executable_sha256']:raise RuntimeError('Cached executable changed: '+c['name'])
   # Every selected C/C++ source must be present in the recorded real compiler
   # command. Production/header/fixture hashes above bind transitive includes.
   for value in c['sources']+[c['test'],wrong]:
    if str(value) not in previous_cmd:raise RuntimeError('Cached compiler inputs do not match: '+str(value))
   record.update(compile_command=previous_cmd,compile_returncode=previous['compile_returncode'],compilation_reused_after_sha_verification=True,prior_report_sha256=reuse_sha)
  else:
   result=subprocess.run(cmd,capture_output=True,text=True,timeout=240);record['compile_returncode']=result.returncode
   (output/(c['name']+'.compile.log')).write_text(result.stdout+result.stderr)
  if not record['compile_returncode']:
   env=os.environ.copy()
   if c['sanitize']:env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
   result=subprocess.run([str(binary)],cwd=NAV,env=env,capture_output=True,text=True,timeout=90)
   log=output/(c['name']+'.log');log.write_text(result.stdout+result.stderr)
   record.update(returncode=result.returncode,output_tail=(result.stdout+result.stderr).splitlines()[-3:],executable_sha256=sha(binary),log=log.name,log_sha256=sha(log))
  record['duration_s']=round(time.monotonic()-began,3);print(c['name'],record.get('returncode',record['compile_returncode']),flush=True);return record
 normal=[c for c in cases if not c['sanitize']]
 with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(execute,normal))
 results.extend(execute(c) for c in cases if c['sanitize'])
 unchanged=all(sha(ROOT/f)==h for f,h in source.items())
 passed=unchanged and all(c.get('returncode')==0 and c['compile_returncode']==0 for c in results)
 report=dict(compilation_reuse_policy='If requested: SHA-verify original passing ELF and every compiled source/header, then execute every test again; no test result is reused',version='robot-navigation-v38-sonar-source-grace-ack-recovery',result='PASS' if passed else 'FAIL',source_sha256=source,source_unchanged=unchanged,hardware_tested=False,ros2_runtime_tested=False,network_scope='loopback only' if any(c['http'] for c in cases) else 'none',sanitizer_scope='ASAN/UBSAN; leak scan disabled because traced executor does not support LSan',cases=results)
 (output/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(report['result'],len(results),str(output/'verification.json'))
 return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
