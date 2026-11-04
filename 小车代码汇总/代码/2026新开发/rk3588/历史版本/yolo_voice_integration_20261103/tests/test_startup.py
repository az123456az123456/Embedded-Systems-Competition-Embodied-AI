from pathlib import Path
import json, os, signal, subprocess, tempfile, time
root=Path(__file__).resolve().parents[1]; retry=root/'candidate/deploy/retry-dh-until-ready.sh'; lib=root/'candidate/deploy/stack-readiness.sh'
records=[]; children=[]

def check(name, okay, **details):
 records.append({'case':name,'pass':bool(okay),**details})
 if not okay:raise AssertionError(name)

def wait_for(test, timeout=15):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if test():return
  time.sleep(.05)
 raise TimeoutError('expected state missing')

def state(d):
 p=d/'retry-status.txt'
 return p.read_text() if p.exists() else ''

def fixture(d, failures=0, quick=False):
 e=d/'entry.sh'; log=d/'ui.log'; log.write_text('PHONE unified service on 127.0.0.1:8082; test\nSTAGE ui_hz=30.0\n')
 e.write_text(f'''#!/bin/sh
set -eu
n=0;[ ! -f "{d}/count" ] || n=$(cat "{d}/count")
n=$((n+1));echo "$n" > "{d}/count"
[ "$n" -gt {failures} ] || exit 7
. "{lib}"
stack_readiness_init
trap 'stack_readiness_cleanup; echo stopped >> "{d}/child-stopped"; exit 0' TERM INT
stack_try_ready "{log}"
{ 'stack_readiness_cleanup; exit 0' if quick else 'while :;do sleep 1;done' }
''')
 return e

def launch(d,entry):
 env={**os.environ,'ROBOT_BOOT_STATE':str(d),'ROBOT_DH_ENTRY':str(entry)}
 p=subprocess.Popen(['sh',str(retry)],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True);children.append(p);return p

def stop(p):
 if p.poll() is None:p.terminate()
 try:return p.communicate(timeout=5)[0]
 except subprocess.TimeoutExpired:p.kill();return p.communicate()[0]

with tempfile.TemporaryDirectory(prefix='voice-boot-test-') as tmp:
 base=Path(tmp)
 try:
  for f in (root/'candidate/deploy').glob('*.sh'):
   x=subprocess.run(['sh','-n',str(f)],capture_output=True,text=True);check('syntax_'+f.name,x.returncode==0)
  d=base/'retry';d.mkdir();e=fixture(d,failures=2);p=launch(d,e)
  wait_for(lambda:'phase=ENTERED' in state(d),timeout=15)
  check('two_failures_then_entry_only_one_active_attempt',int((d/'count').read_text())==3,attempts=3)
  stop(p);check('SIGTERM_stops_owned_application_without_retry',(d/'child-stopped').exists() and 'phase=STOPPED' in state(d))
  d=base/'quick';d.mkdir();e=fixture(d,quick=True);p=launch(d,e);output=p.communicate(timeout=5)[0]
  check('entry_then_immediate_exit_does_not_restart',p.returncode==0 and (d/'count').read_text().strip()=='1' and 'phase=FINISHED' in state(d),output=output)
  d=base/'zero';d.mkdir();e=fixture(d)
  s=e.read_text().replace('[ "$n" -gt 0 ] || exit 7','[ "$n" -gt 1 ] || exit 0');e.write_text(s);p=launch(d,e)
  wait_for(lambda:'phase=ENTERED' in state(d));check('exit_zero_before_entry_still_retries',(d/'count').read_text().strip()=='2');stop(p)
  d=base/'stale';d.mkdir();e=fixture(d);(d/'app-ready').write_text(f'{os.getpid()} 1 invalid-boot\n');p=launch(d,e)
  wait_for(lambda:'phase=ENTERED' in state(d));check('stale_ready_boot_and_PID_ignored',(d/'count').read_text().strip()=='1');stop(p)
  d=base/'single';d.mkdir();e=fixture(d);p=launch(d,e);wait_for(lambda:'phase=ENTERED' in state(d));inode=(d/'retry.lock').stat().st_ino
  q=launch(d,e);output=q.communicate(timeout=3)[0]
  check('second_retry_instance_does_not_launch_or_replace_lock',q.returncode==0 and (d/'count').read_text().strip()=='1' and inode==(d/'retry.lock').stat().st_ino,lock_inode=inode,output=output)
  # Release only the retrier lock in a separate process, leaving a manually
  # entered application alive; an observer must not kill or relaunch it.
  stop(p)
  d=base/'adopt';d.mkdir();e=fixture(d)
  env={**os.environ,'ROBOT_STARTUP_READY_FILE':str(d/'app-ready')}
  manual=subprocess.Popen(['sh',str(e)],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);children.append(manual)
  wait_for(lambda:(d/'app-ready').exists());p=launch(d,e);p.communicate(timeout=3)
  check('existing_entered_manual_application_left_running',p.returncode==0 and manual.poll() is None and (d/'count').read_text().strip()=='1');stop(manual)
  d=base/'markers';d.mkdir();log=d/'ui.log';ready=d/'ready'
  def probe(text):
   log.write_text(text)
   return subprocess.run(['sh','-c',f'. "{lib}"; stack_readiness_init; stack_try_ready "{log}"'],env={**os.environ,'ROBOT_STARTUP_READY_FILE':str(ready)},capture_output=True,text=True)
  probe('STAGE ui_hz=30\n');check('UI_alone_is_not_application_entry',not ready.exists())
  probe('PHONE unified service on 127.0.0.1:8082\nSTAGE ui_hz=0.00\n');check('zero_render_rate_is_not_entry',not ready.exists())
  probe('PHONE unified service on 127.0.0.1:8082\nSTAGE ui_hz=29.84\n');check('phone_and_render_interval_publish_ready',ready.exists())
 finally:
  for p in reversed(children):stop(p)
  out=root/'reports/startup-tests.jsonl';out.write_text(''.join(json.dumps(x)+'\n' for x in records))
  (root/'reports/startup-summary.json').write_text(json.dumps({'status':'PASS' if records and all(x['pass'] for x in records) else 'FAIL','cases':len(records),'scope':'Actual new shell workers; fake application/temporary paths, no robot/ROS/audio started'},indent=2)+'\n')
print(json.dumps({'cases':len(records),'failed':sum(not x['pass'] for x in records)}))
