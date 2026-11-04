"""Environment tuning through the real headless phone service, without hardware."""
import http.client
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'build/perception-dashboard'
checks=0

def ck(value,label):
 global checks
 checks+=1
 assert value,label

def environment(extra):
 env=dict(os.environ)
 for name in ['PERCEPTION_TURN_INNER_RATIO','PERCEPTION_TURN_FULL_DEG']:env.pop(name,None)
 env.update(extra)
 return env

def args(token,port):
 return [str(APP),'--demo','--headless','--phone-bind','127.0.0.1','--phone-port',str(port),'--pair-token-file',str(token),'--seconds','8']

def main():
 invalid=[]
 with tempfile.TemporaryDirectory(prefix='strong-arc-tuning-') as temporary:
  token=Path(temporary)/'token';token.write_text('123456\n')
  for name,values in [('PERCEPTION_TURN_INNER_RATIO',['','.099','.801','-1','1','nan','inf','nonsense','0.3x']),('PERCEPTION_TURN_FULL_DEG',['','15','90.1','nan','inf','nonsense','45x'])]:
   for text in values:
    result=subprocess.run(args(token,18562),capture_output=True,text=True,env=environment({name:text}),timeout=4)
    ck(result.returncode==1,'invalid steering tuning refuses startup before motor controls')
    ck('Phone service: Invalid '+name in result.stderr,'invalid tuning names the setting in error')
    invalid.append({'setting':name,'value':text,'status':result.returncode})
  valid=[]
  for ratio,full in [(.10,30.),(.80,60.),(.25,15.001),(.25,90.)]:
   with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
   with (Path(temporary)/'app.log').open('w+') as log:
    process=subprocess.Popen(args(token,port),stdout=log,stderr=log,env=environment({'PERCEPTION_TURN_INNER_RATIO':str(ratio),'PERCEPTION_TURN_FULL_DEG':str(full)}))
    def req(method,path,body=None):
     c=http.client.HTTPConnection('127.0.0.1',port,timeout=2)
     c.request(method,path,None if body is None else json.dumps(body),{'Authorization':'Bearer 123456','Content-Type':'application/json'})
     r=c.getresponse();out=(r.status,json.loads(r.read()));c.close();return out
    try:
     for _ in range(100):
      try:
       if req('GET','/api/state')[1]['motor']['link_ok']:break
      except OSError:pass
      time.sleep(.02)
     else:raise AssertionError('tuned server did not start')
     _,s=req('POST','/api/connect',{});session=s['session'];seq=0
     time.sleep(.05)
     def control(op,**fields):
      nonlocal seq
      seq+=1;return req('POST','/api/control',dict(session=session,seq=seq,op=op,**fields))
     ck(control('arm')[1]['armed'],'valid tuning permits normal explicit arm')
     rad=math.radians(full)
     for _ in range(13):
      code,s=control('drive',throttle=math.cos(rad),steer=math.sin(rad));time.sleep(.05)
     m=req('GET','/api/state')[1]['motor']
     ck(round(m['left']*1000)==600 and abs(round(m['right']*1000)-round(600*ratio))<=1,'HTTP tuned sharp duties reach configured minimum')
     ck(m['turn_inner_ratio']==ratio and abs(m['turn_full_deg']-full)<.0001,'state reports effective tuned values')
     ck(m['straight_cone_deg']==15,'tuning preserves requested 15 degree cone')
     ck(control('drive',throttle=0,steer=0)[1]['motor']['left']==0,'tuned release immediate zero')
     log.flush();log.seek(0);text=log.read()
     ck('MANUAL_ARC straight_cone_deg=15.0' in text and 'sharp_wire=MOT1-exact-wheels' in text,'tuned configuration startup log')
     valid.append({'inner_ratio':ratio,'full_degrees':full,'left':m['left'],'right':m['right']})
    finally:
     process.terminate()
     try:process.wait(timeout=3)
     except subprocess.TimeoutExpired:process.kill();process.wait()
  (ROOT/'reports').mkdir(exist_ok=True)
  (ROOT/'reports/arc-tuning-verification.json').write_text(json.dumps({'passed':True,'checks':checks,'invalid_cases':invalid,'valid_cases':valid,'scope':'host DEMO real HTTP and phone service; no motor hardware'},indent=2)+'\n')
  print('PASS actual environment/HTTP steering tuning',checks,'checks')
if __name__=='__main__':main()
