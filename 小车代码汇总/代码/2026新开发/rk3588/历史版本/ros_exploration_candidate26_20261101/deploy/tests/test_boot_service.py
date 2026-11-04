import os,pathlib,subprocess,tempfile,time,json,socket
r=pathlib.Path(__file__).resolve().parents[1]
records=[]
def check(name,ok):
 records.append(dict(case=name,pass_=bool(ok)))
 if not ok:raise AssertionError(name)
with tempfile.TemporaryDirectory(prefix='robot-boot-test-') as tmp:
 t=pathlib.Path(tmp);(t/'deploy').mkdir();(t/'deploy/boot-run-stack.sh').write_text('#!/bin/sh\necho "started:$#" >> "$ROBOT_BOOT_MARKER"\nexec sleep 120\n')
 env=dict(os.environ,ROBOT_STACK=str(t),ROBOT_BOOT_STATE=str(t/'run'),ROBOT_BOOT_LOG=str(t/'boot.log'),ROBOT_BOOT_MARKER=str(t/'launched'))
 def run(op):return subprocess.run(['sh',str(r/'S99robot-stack'),op],env=env,capture_output=True,text=True,timeout=25)
 try:
  check('POSIX syntax',subprocess.run(['sh','-n',str(r/'S99robot-stack')]).returncode==0)
  check('start background returns',run('start').returncode==0)
  time.sleep(.15);owner=(t/'run/owner').read_text()
  check('one no-argument launch', (t/'launched').read_text()=='started:0\n')
  inode=(t/'run/service.lock').stat().st_ino
  check('duplicate start idempotent',run('start').returncode==0 and (t/'run/owner').read_text()==owner)
  check('status running',run('status').returncode==0)
  check('stop owned process',run('stop').returncode==0)
  check('status stopped',run('status').returncode==3)
  check('stable lock inode',inode==(t/'run/service.lock').stat().st_ino)
  # A stale file must not signal an unrelated live PID.
  (t/'run/owner').write_text(f'{os.getpid()} 1 invalid-boot\n')
  check('stale PID identity not signalled',run('stop').returncode==0)
  check('not automatically restarted',(t/'launched').read_text()=='started:0\n')
 finally:run('stop')
check('worker POSIX syntax',subprocess.run(['sh','-n',str(r/'boot-run-stack.sh')]).returncode==0)
worker=(r/'boot-run-stack.sh').read_text();check('same entry without recording flag','exec sh "$BASE/deploy/run-yolo.sh"\n' in worker)
check('no shell-controlled launch arguments','"$@"' not in worker)
print(json.dumps({'status':'PASS','cases':records,'real_robot_started':False},indent=2))
