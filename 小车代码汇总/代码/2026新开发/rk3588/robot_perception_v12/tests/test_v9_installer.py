from pathlib import Path
import hashlib,os,shutil,subprocess,tempfile
app=Path(__file__).resolve().parents[1]
def sums(path):
 (path/'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in sorted(path.iterdir()) if p.is_file() and p.name!='SHA256SUMS'))
with tempfile.TemporaryDirectory(prefix='v9-install-') as temp:
 root=Path(temp);pkg=root/'update';dash=root/'app with spaces';fake=root/'bin'
 for p in (pkg,dash,fake):p.mkdir()
 (fake/'uname').write_text('#!/bin/sh\necho aarch64\n');(fake/'uname').chmod(0o755)
 for source,target in ((app/'scripts/install-v9.sh','install.sh'),(app/'scripts/rollback-v9.sh','rollback.sh'),(app/'build/perception-dashboard.aarch64','perception-dashboard.aarch64'),(app/'run-dashboard.sh','run-dashboard.sh')):shutil.copy2(source,pkg/target)
 sums(pkg)
 launcher=b'#!/bin/sh\nprintf "%s\\n" "$PERCEPTION_BINARY" "$@"\n'
 (dash/'run-perception.sh').write_bytes(launcher);(dash/'run-perception.sh').chmod(0o755)
 (dash/'model.rknn').write_bytes(b'unchanged model')
 for name in ('run-normal.sh','run-rotate90.sh'):(dash/name).write_bytes(b'preserved historical '+name.encode())
 env=dict(os.environ,DASHBOARD_DIR=str(dash),PATH=str(fake)+':'+os.environ['PATH'])
 def run(script):return subprocess.run(['sh',str(pkg/script)],env=env,capture_output=True,text=True)
 for existing_entry in (False,True):
  (dash/'perception-dashboard.aarch64').write_bytes(b'old executable');(dash/'perception-dashboard.aarch64').chmod(0o755)
  if existing_entry:(dash/'run-dashboard.sh').write_bytes(b'original entry')
  result=run('install.sh');assert result.returncode==0,result.stdout+result.stderr
  assert (dash/'perception-dashboard.aarch64').read_bytes()==(pkg/'perception-dashboard.aarch64').read_bytes()
  assert (dash/'run-perception.sh').read_bytes()==launcher
  assert (dash/'model.rknn').read_bytes()==b'unchanged model'
  for args,angle in ((['-0'],'0'),(['-90'],'90'),([],'0')):
   result=subprocess.run([str(dash/'run-dashboard.sh'),*args,'--npu-workers','3','--camera','node with spaces'],env=env,capture_output=True,text=True)
   assert result.returncode==0,result.stderr
   assert result.stdout.splitlines()==[str(dash/'perception-dashboard.aarch64'),'--fullscreen','--rotate',angle,'--npu-workers','3','--camera','node with spaces']
  result=subprocess.run([str(dash/'run-dashboard.sh'),'-180'],env=env,capture_output=True,text=True)
  assert result.returncode==2 and 'Select -0 or -90' in result.stderr
  result=subprocess.run([str(dash/'run-dashboard.sh'),'--help'],env=env,capture_output=True,text=True)
  assert result.returncode==0 and '[-0|-90]' in result.stdout
  for name in ('run-normal.sh','run-rotate90.sh'):assert (dash/name).read_bytes()==b'preserved historical '+name.encode()
  result=run('rollback.sh');assert result.returncode==0,result.stdout+result.stderr
  assert (dash/'perception-dashboard.aarch64').read_bytes()==b'old executable'
  if existing_entry:assert (dash/'run-dashboard.sh').read_bytes()==b'original entry'
  else:assert not (dash/'run-dashboard.sh').exists()
  for name in ('run-normal.sh','run-rotate90.sh'):assert (dash/name).read_bytes()==b'preserved historical '+name.encode()
 print('PASS single script -0/-90/default/help/invalid angle; original launcher/model preserved; quoted arguments; restore existing/missing entry')
 backups=list(dash.glob('v9-backup.*'))
 (pkg/'perception-dashboard.aarch64').write_bytes(b'damaged')
 result=run('install.sh');assert result.returncode!=0
 assert list(dash.glob('v9-backup.*'))==backups
 assert (dash/'perception-dashboard.aarch64').read_bytes()==b'old executable'
 print('PASS checksum rejection before mutation')
 sums(pkg);shutil.copy2(shutil.which('sleep'),dash/'perception-dashboard.aarch64')
 before=(dash/'perception-dashboard.aarch64').read_bytes();process=subprocess.Popen([str(dash/'perception-dashboard.aarch64'),'30'])
 try:
  for script in ('install.sh','rollback.sh'):
   result=run(script);assert result.returncode!=0 and 'Close the YOLO/IMU window' in result.stderr
   assert (dash/'perception-dashboard.aarch64').read_bytes()==before
 finally:process.terminate();process.wait(timeout=5)
 print('PASS active process guards for install and rollback')
 backup=Path((dash/'.v9-last-backup').read_text().strip());(backup/'perception-dashboard.aarch64').write_bytes(b'corrupt')
 result=run('rollback.sh');assert result.returncode!=0
 assert (dash/'perception-dashboard.aarch64').read_bytes()==before
 print('PASS corrupted backup cannot overwrite current binary')
