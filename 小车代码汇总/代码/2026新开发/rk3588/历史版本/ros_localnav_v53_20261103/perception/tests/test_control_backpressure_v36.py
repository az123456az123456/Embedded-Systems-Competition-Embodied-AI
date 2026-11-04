#!/usr/bin/env python3
"""Intentionally block the real service's stderr pipe; control must stay live."""
import fcntl,http.client,json,pathlib,socket,subprocess,tempfile,time
root=pathlib.Path(__file__).resolve().parents[2];binary=root/'dashboard/build/phone-publish-probe'
checks=0;worst_ms=0
with tempfile.TemporaryDirectory(prefix='v36-stderr-') as tmp:
 token=pathlib.Path(tmp)/'token';token.write_text('123456\n')
 sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close()
 process=subprocess.Popen([str(binary),str(port),str(token)],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
 fcntl.fcntl(process.stderr.fileno(),fcntl.F_SETPIPE_SZ,4096)
 def request(method,path,body=None):
  global checks,worst_ms
  connection=http.client.HTTPConnection('127.0.0.1',port,timeout=.8);start=time.monotonic()
  try:
   encoded=json.dumps(body) if body is not None else None
   connection.request(method,path,encoded,{'Authorization':'Bearer 123456','Content-Type':'application/json'})
   response=connection.getresponse();data=json.loads(response.read());assert response.status==200,(response.status,data)
   worst_ms=max(worst_ms,(time.monotonic()-start)*1000);checks+=1;return data
  finally:connection.close()
 try:
  deadline=time.monotonic()+2
  while True:
   try:state=request('POST','/api/connect',{});break
   except (ConnectionRefusedError,ConnectionResetError):
    if time.monotonic()>=deadline:raise
    time.sleep(.02)
  session=state['session'];seq=0
  def command(op,**fields):
   global seq
   seq+=1;return request('POST','/api/control',dict(session=session,seq=seq,op=op,**fields))
  time.sleep(.04);assert command('arm')['logical_armed']
  began=time.monotonic();late_requests=0
  while time.monotonic()-began<6.7:
   # Keep real owner commands arriving while several log reports fill the pipe.
   result=command('drive',throttle=.2,steer=0);assert result['logical_armed']
   state=request('GET','/api/state');assert state['logical_armed']
   if time.monotonic()-began>4.5:late_requests+=1
   time.sleep(.045)
  assert late_requests>=25,'insufficient control attempts while diagnostic publisher is blocked'
  assert command('stop')['logical_armed'] is False
  print(f'PASS blocked-stderr real-service {checks} HTTP checks; late_requests={late_requests} max_rtt_ms={worst_ms:.2f}; 4096-byte pipe intentionally never drained')
 finally:
  process.terminate()
  try:process.wait(timeout=1)
  except subprocess.TimeoutExpired:process.kill();process.wait(timeout=1)
  process.stderr.close()
