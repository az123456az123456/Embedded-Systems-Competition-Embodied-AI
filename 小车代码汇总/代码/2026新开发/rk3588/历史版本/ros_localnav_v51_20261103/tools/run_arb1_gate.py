from pathlib import Path
import os,subprocess,json,hashlib,time
r=Path(__file__).resolve().parents[1];repo=r.parents[1];out=r/'reports/arb1-current/full';out.mkdir(exist_ok=True,parents=True)
files=list((r/'common').glob('ros_bridge_*'));before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files};records=[]
for variant,flags in [('normal',['-O2']),('asan-ubsan',['-O1','-g','-fno-pie','-no-pie','-fsanitize=address,undefined','-fno-omit-frame-pointer'])]:
 for name,src,wrap in [('T01',repo/'integration/ros_bridge_v41/tests/test_codec.cc',True),('T02',repo/'integration/ros_bridge_v41/tests/test_session.cc',True),('extension',r/'tests/test_arb1_nav3.cc',False)]:
  dest=out/variant/name;dest.mkdir(parents=True,exist_ok=True);binary=dest/'test'
  command=['c++','-std=c++17','-Wall','-Wextra','-Werror','-pthread',*flags,'-I'+str(r/'common'),str(src),str(r/'common/ros_bridge_codec.cc'),str(r/'common/ros_bridge_session.cc')]
  if wrap:command+=['-Wl,--wrap=send']
  command+=['-o',str(binary)];start=time.monotonic();c=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT);(dest/'build.log').write_bytes(c.stdout)
  if c.returncode:raise SystemExit(c.stdout.decode())
  args=[str(binary),str(dest)]
  if name=='T01':args=[str(binary),str(dest/'cases.json'),str(dest/'corpus')]
  env=os.environ.copy();env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
  p=subprocess.run(args,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=60);(dest/'run.log').write_bytes(p.stdout)
  record={'group':name,'variant':variant,'returncode':p.returncode,'seconds':time.monotonic()-start,'log':str(dest/'run.log')};records.append(record);print(json.dumps(record),flush=True)
  (out/'summary.json').write_text(json.dumps({'status':'RUNNING' if not p.returncode else 'FAIL','runs':records,'source_sha256':before},indent=2)+'\n')
  if p.returncode:raise SystemExit(1)
after={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files};assert before==after
(out/'summary.json').write_text(json.dumps({'status':'PASS','runs':records,'source_sha256':before,'source_stable':True,'root_positive_peercred':'requires board UID0 test'},indent=2)+'\n')
