#!/usr/bin/env python3
from pathlib import Path
import os,subprocess,json,hashlib,time
r=Path(__file__).resolve().parents[1];p=r/'acg138_fpga_map_bridge';g=r/'gateway'
attempt=r/'reports/delegation'/('attempt-'+str(time.time_ns()));attempt.mkdir(parents=True)
records=[]
for mode in ('normal','asan-ubsan'):
 out=attempt/mode;out.mkdir(parents=True,exist_ok=True)
 build=r/'build-offline'/mode;build.mkdir(parents=True,exist_ok=True)
 flags=['-O1','-g','-Wall','-Wextra','-Wpedantic']
 if mode!='normal':flags+=['-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie']
 env={**os.environ,'ASAN_OPTIONS':'detect_leaks=0:halt_on_error=1','UBSAN_OPTIONS':'halt_on_error=1:print_stacktrace=1'}
 ref=p/'tests/reference/nav3'
 commands=[]
 for target,source in [('golden',ref/'nav_exec_wire.c'),('guard',ref/'candidate26/softcore/firmware/nav_exec_protocol.c')]+[(n,ref/'candidate26/softcore/firmware'/(n+'.c')) for n in ('astar','nav_protocol','nav_exec')]:
  commands.append(['gcc','-std=c99',*flags,'-c',str(source),'-o',str(build/(target+'.o'))])
 objects=[str(build/(n+'.o')) for n in ('golden','guard','astar','nav_protocol','nav_exec')]
 commands.append(['g++','-std=c++17',*flags,'-I'+str(p/'include'),'-c',str(g/'nva_socket.cc'),'-o',str(build/'nva_socket.o')])
 for name in ('authority','delegation','adapter_http','state_delivery'):
  sources=[g/'nav_authority.cc',p/'src/nav3_wire.cpp',g/('test_'+name+'.cpp')]
  if name in ('delegation','adapter_http','state_delivery'):sources.append(g/'fpga_delegation.cc')
  if name=='adapter_http':sources.extend([g/'phone_nav_adapter.cc',r/'common/ros_bridge_codec.cc'])
  exe=build/('test_'+name)
  commands.extend([['g++','-std=c++17',*flags,'-I'+str(p/'include'),*[str(x) for x in sources],*objects,'-pthread','-o',str(exe)],[str(exe),str(out/name)]])
 for i,command in enumerate(commands):
  began=time.monotonic();c=subprocess.run(command,capture_output=True,text=True,env=env,timeout=120)
  (out/f'step-{i}.log').write_text(c.stdout+c.stderr)
  records.append({'mode':mode,'step':i,'argv':command,'returncode':c.returncode,'seconds':time.monotonic()-began})
  if c.returncode:
   (attempt/'summary.json').write_text(json.dumps({'status':'FAIL','steps':records},indent=2)+'\n')
   (r/'reports/delegation/summary.json').write_text((attempt/'summary.json').read_text())
   print(c.stdout+c.stderr);raise SystemExit(c.returncode)
report={'status':'PASS','attempt':str(attempt),'steps':records,'sources':{str(x.relative_to(r)):hashlib.sha256(x.read_bytes()).hexdigest() for x in g.glob('*') if x.is_file()},'actual_NVA_family_oracle':'unchanged candidate26 C firmware, host execution','FPGA_files_modified':False}
(attempt/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
(r/'reports/delegation/summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'status':'PASS','groups':[str(attempt/m/n/'summary.json') for m in ('normal','asan-ubsan') for n in ('authority','delegation','adapter_http','state_delivery')]}))
