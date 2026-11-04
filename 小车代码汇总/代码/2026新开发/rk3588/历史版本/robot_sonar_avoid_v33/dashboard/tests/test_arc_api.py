"""Real HTTP -> continuous controller -> exact MOT1/MOT3 wheel requests; host DEMO."""
import http.client,json,math,os,socket,subprocess,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
checks=0

def ck(v,label):
 global checks
 checks+=1
 assert v,label

def main():
 with socket.socket() as p:
  p.bind(('127.0.0.1',0));port=p.getsockname()[1]
 with tempfile.TemporaryDirectory() as directory:
  token=Path(directory)/'token';token.write_text('123456\n')
  with open(Path(directory)/'app.log','w+') as log:
   process=subprocess.Popen([str(ROOT/'build/perception-dashboard'),'--demo','--headless','--phone-bind','127.0.0.1','--phone-port',str(port),'--pair-token-file',str(token),'--seconds','60'],stdout=log,stderr=log)
   def req(method,path,body=None):
    c=http.client.HTTPConnection('127.0.0.1',port,timeout=2)
    data=None if body is None else json.dumps(body)
    c.request(method,path,data,{'Authorization':'Bearer 123456','Content-Type':'application/json'})
    r=c.getresponse();out=(r.status,json.loads(r.read()));c.close();return out
   try:
    for _ in range(100):
     try:
      if req('GET','/api/state')[1]['motor']['link_ok']:break
     except OSError:pass
     time.sleep(.02)
    else:raise AssertionError('no API')
    status,s=req('POST','/api/connect',{});session=s['session'];seq=0
    def cmd(op,path='/api/control',**kwargs):
     nonlocal seq
     seq+=1;return req('POST',path,dict(op=op,session=session,seq=seq,**kwargs))
    time.sleep(.05);ck(cmd('arm')[0]==200,'arm')
    vectors=[(1,0,600,0,600,600),(-1,0,-600,0,-600,-600),(1,-1,600,-999,150,600),(1,1,600,999,600,150),(-1,-1,-600,-999,-150,-600),(-1,1,-600,999,-600,-150),(0,-1,600,-999,150,600),(0,1,600,999,600,150)]
    observed=[]
    for y,x,speed,curve,l,r in vectors:
     ck(cmd('drive',throttle=0,steer=0)[0]==200,'release before each independent gesture')
     for _ in range(15):
      code,state=cmd('drive',throttle=y,steer=x,hold=False);ck(code==200,'continuous drive accepted');time.sleep(.05)
     state=req('GET','/api/state')[1];m=state['motor']
     ck(m['arc_protocol_supported'],'HTTP retains v21 capability prerequisite')
     ck(m['arc_mode']==(curve==0),'HTTP selects representable MOT3 or exact sharp MOT1')
     if curve==0:ck(m['arc_outer_permille']==speed and m['arc_curve_q8']==curve,'straight API arc fields')
     ck(round(m['left']*1000)==l and round(m['right']*1000)==r,'8 directions physical request')
     observed.append({'throttle':y,'steer':x,'outer':speed,'curve':curve,'left':l,'right':r})
    cmd('drive',throttle=0,steer=0)
    for angle in [0,10,14,15,-10,-14,-15]:
     for _ in range(2):
      _,state=cmd('drive',throttle=math.cos(math.radians(angle)),steer=math.sin(math.radians(angle)));time.sleep(.05)
     ck(state['motor']['arc_curve_q8']==0 and state['motor']['left']==state['motor']['right'],'straight cone exact equal API')
    for sign in [-1,1]:
     for steer_sign in [-1,1]:
      for angle in [15,31,35,45]:
       cmd('drive',throttle=0,steer=0)
       radians=math.radians(angle)
       for _ in range(12):
        code,state=cmd('drive',throttle=sign*math.cos(radians),steer=steer_sign*math.sin(radians));time.sleep(.05)
       m=req('GET','/api/state')[1]['motor']
       inner=round(600*(1-.75*min(1,max(0,(angle-15)/30))))
       left=sign*(inner if steer_sign<0 else 600);right=sign*(600 if steer_sign<0 else inner)
       ck(round(m['left']*1000)==left and round(m['right']*1000)==right,'HTTP 15/31/35/45 forward and reverse physical goldens')
       ck(m['arc_mode']==(inner>=300),'HTTP real physical 50 percent wire boundary')
       ck(m['turn_inner_ratio']==.25 and m['turn_full_deg']==45 and m['straight_cone_deg']==15,'HTTP exposes effective steering defaults')
    for value in [-.061,-.08,-.119,.061,.08,.119]:
     cmd('drive',throttle=0,steer=0)
     cmd('drive',throttle=value,steer=0);time.sleep(.08)
     m=req('GET','/api/state')[1]['motor']
     ck(m['left']*value>0 and m['right']*value>0,'small first straight gesture has correct forward/reverse sign')
    ck(cmd('drive',throttle=0,steer=0)[0]==200,'stop for navigation claim')
    code,s=cmd('claim','/api/navigation/claim');ck(code==200 and len(s['navigation_ticket'])==48,'loopback private nav claim')
    ticket=s['navigation_ticket'];ck('navigation_ticket' not in req('GET','/api/state')[1],'telemetry never leaks navigation capability')
    for _ in range(8):
     code,s=cmd('wheels','/api/navigation/wheels',left=-.11,right=-.29,navigation_ticket=ticket);ck(code==200,'private nav wheels accepted');time.sleep(.05)
    ck(not s['motor']['arc_mode'] and round(s['motor']['left']*1000)==-110 and round(s['motor']['right']*1000)==-290,'navigation exact duties bypass cone and nonlinearity')
    code,s=cmd('drive',throttle=.4,steer=0);ck(code==200,'manual takeover')
    code,s=cmd('wheels','/api/navigation/wheels',left=.2,right=.2,navigation_ticket=ticket);ck(code==409 and not s['armed'],'manual takeover revokes private navigation ticket')
    cmd('arm');code,s=cmd('wheels',left=.2,right=.2,navigation_ticket=ticket);ck(code==409,'public phone endpoint cannot invoke raw-wheel operation')
    cmd('arm');cmd('drive',throttle=.5,steer=.5);time.sleep(.4)
    s=req('GET','/api/state')[1];ck(not s['armed'] and s['motor']['left']==s['motor']['right']==0,'API command lease stops movement without stale renewal')
    cmd('drive',throttle=0,steer=0)
    log.flush();log.seek(0);text=log.read()
    ck('MANUAL_ARC straight_cone_deg=15.0 inner_min_ratio=0.250 full_deflection_deg=45.0' in text,'startup reports actual steering configuration')
    result={'checks':checks,'directions':observed,'passed':True,'scope':'host demo HTTP, real controller + field codec path; no board motors'}
    (ROOT/'reports').mkdir(exist_ok=True);(ROOT/'reports/arc-api-verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print('PASS actual arc HTTP API',checks,'checks')
   finally:
    process.terminate()
    try:process.wait(timeout=3)
    except subprocess.TimeoutExpired:process.kill();process.wait()
    if process.returncode not in (0,-15):
     log.seek(0);print(log.read())
if __name__=='__main__':main()
