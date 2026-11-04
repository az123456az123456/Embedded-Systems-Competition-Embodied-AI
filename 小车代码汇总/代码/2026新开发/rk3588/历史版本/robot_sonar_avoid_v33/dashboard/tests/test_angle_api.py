"""Real HTTP service, synthetic frames/sensors only; never opens motor sockets."""
import base64, http.client, json, os
from pathlib import Path
import socket, subprocess, tempfile, time
ROOT=Path(__file__).resolve().parents[1]
checks=0
def ck(value,message):
 global checks
 assert value,message
 checks+=1

def run(configured):
 with tempfile.TemporaryDirectory(prefix='follow-angle-api-') as td:
  tmp=Path(td);token='045678';(tmp/'token').write_text(token+'\n')
  with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
  geom=ROOT/'tests/synthetic-bearing-fixture.json' if configured else tmp/'missing.json'
  proc=subprocess.Popen([str(ROOT/'build/perception-dashboard'),'--demo','--headless','--phone-bind','127.0.0.1','--phone-port',str(port),'--pair-token-file',str(tmp/'token'),'--seconds','15'],env=dict(os.environ,FOLLOW_GEOMETRY_FILE=str(geom)),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
  def request(method,path,data=None):
   c=http.client.HTTPConnection('127.0.0.1',port,timeout=2);headers={'Authorization':'Bearer '+token}
   c.request(method,path,None if data is None else json.dumps(data),headers);r=c.getresponse();v=json.loads(r.read());status=r.status;c.close();return status,v
  sequence=0;session=''
  def cmd(op,**kw):
   nonlocal sequence
   sequence+=1;return request('POST','/api/control',dict(session=session,seq=sequence,op=op,**kw))
  video=None
  try:
   for _ in range(150):
    try:
     _,state=request('GET','/api/state')
     if state['imu']['fresh'] and state['motor']['link_ok']:break
    except (OSError,http.client.HTTPException):pass
    time.sleep(.02)
   else:raise AssertionError('server startup timeout')
   ck(state['follow_geometry']['configured']==configured,'actual configured status')
   ck(state['motor']['angle_protocol_supported'] and not state['motor']['enabled'],'demo capability never real motor socket')
   _,state=request('POST','/api/connect',{});session=state['session'];time.sleep(.1)
   video=http.client.HTTPConnection('127.0.0.1',port,timeout=2);video.request('GET','/api/video.mjpg',headers={'Authorization':'Bearer '+token});stream=video.getresponse()
   def frame():
    for _ in range(3):
     if stream.readline().strip()==b'--robotframe':break
    else:raise AssertionError('missing frame delimiter')
    headers={}
    while True:
     line=stream.readline()
     if line in (b'\r\n',b'\n'):break
     k,v=line.decode().split(':',1);headers[k.lower()]=v.strip()
    stream.read(int(headers['content-length']))
    return json.loads(base64.b64decode(headers['x-frame-meta']))
   meta=frame();box=meta['detections'][0]
   code,state=cmd('select',frame_id=meta['frame_id'],target_id=box['id'])
   ck(code==200 and not state['armed'] and state['selected_id']==box['id'],'disarmed select for measurement')
   selected=state['selected_target'];x1,y1,x2,y2=box['box'];u=((x1+x2)/2-(box['eye']*432))/.45
   ck(selected['eye']==box['eye'] and abs(selected['u_px']-u)<6,'native source coordinates exposed; fresh detections may advance')
   ck(not state['follow']['angle_mode'] and state['motor']['left']==state['motor']['right']==0,'measurement does not move')
   if not configured:ck(not selected['angle_valid'] and selected['geometry_reason']=='follow_geometry_disabled','uncalibrated selected box readable with honest reason')
   code,state=cmd('arm');ck(code==200 and state['selected_id']==0,'arm clears measurement selection')
   meta=frame();box=meta['detections'][0];code,state=cmd('select',frame_id=meta['frame_id'],target_id=box['id']);ck(code==200,'fresh armed selection')
   code,state=cmd('follow')
   if configured:
    ck(code==200 and state['mode']=='follow' and state['follow']['angle_mode'],'real service uses angle mode')
    ck(abs(state['follow']['turn_angle_deg']-state['selected_target']['turn_angle_deg'])<.011,'MOT2 requested angle is RK computed bearing')
    ck(0<=state['follow']['forward_duty']<=.22,'forward hard bound')
    (ROOT/'reports/follow-api-contract.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n')
    cmd('zero');time.sleep(.1);_,state=request('GET','/api/state')
    ck(not state['armed'] and not state['follow']['angle_mode'],'pose zero disarms and invalidates angle epoch')
   else:
    ck(code==409 and state['error']=='follow_geometry_disabled' and not state['follow']['angle_mode'],'missing lens calibration returns specific error and no motion')
    ck(cmd('drive',throttle=.2,steer=0,hold=False)[0]==200,'uncalibrated camera does not disable manual mode')
   cmd('stop')
  finally:
   if video:video.close()
   proc.terminate();err=proc.communicate(timeout=5)[1];ck(token not in err,'pair code not logged')
run(False);run(True)
print(f'PASS actual angle HTTP API {checks} checks')
