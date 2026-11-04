"""Real existing phone HTTP flow. ONLY run with wheels lifted and power stable.

Pairing secret arrives on stdin and is never logged. No direct motor, ROS speed,
raw Ethernet, or FPGA reconfiguration API is used by this client.
"""
import json,sys,time,threading,urllib.request,urllib.error,signal,math
token=sys.stdin.readline().strip()
assert token
records=[];session='';sequence=0;nav_sequence=0;hb=False;done=False
lock=threading.Lock()
def record(case,passed,**details):
    row=dict(case=case,passed=passed,monotonic_ns=time.monotonic_ns(),**details)
    records.append(row);print(json.dumps(row),flush=True)
def request(path,body=None):
    data=None if body is None else json.dumps(body).encode()
    r=urllib.request.Request('http://127.0.0.1:8080'+path,data=data,headers={
        'Authorization':'Bearer '+token,'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(r,timeout=1.5) as f:return json.load(f)
    except urllib.error.HTTPError as ex:
        try:reason=json.load(ex).get('error','request_rejected')
        except Exception:reason='request_rejected'
        raise RuntimeError(str(ex.code)+':'+str(reason)) from None
def state():return request('/api/state')
def control(op,**fields):
    global sequence
    with lock:
        sequence+=1
        return request('/api/control',dict(op=op,session=session,seq=sequence,**fields))
def nav(op,**fields):
    global nav_sequence
    nav_sequence+=1
    return request('/api/navigation',dict(op=op,session=session,nav_seq=nav_sequence,**fields))
def wait_for(predicate,seconds,label):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        value=state()
        if predicate(value):return value
        if value.get('navigation',{}).get('state')=='ERROR':
            raise RuntimeError('navigation_error:'+value['navigation'].get('reason','unknown'))
        time.sleep(.1)
    raise RuntimeError('timeout:'+label)
def arm():
    global hb
    control('configure',left_mask=10,invert_mask=15)
    wait_for(lambda s:s.get('motor',{}).get('link_ok'),4,'actual_motor_ACK')
    control('arm');hb=True
    wait_for(lambda s:s.get('logical_armed',s.get('armed')),2,'logical_ARM')
def stop():
    global hb
    hb=False
    for op in ('stop','disarm'):
        try:control(op)
        except Exception:pass
def heartbeats():
    while not done:
        if hb:
            try:control('idle')
            except Exception:pass
        time.sleep(.1)
signal.signal(signal.SIGTERM,lambda *_:sys.exit(143))
thread=threading.Thread(target=heartbeats,daemon=True);thread.start()
try:
    before=state();record('startup_idle_no_auto_ARM',not before.get('logical_armed',before.get('armed',False)),
        source=before.get('navigation',{}).get('execution_source'))
    session=request('/api/connect',{})['session']
    arm();record('explicit_ARM_after_real_ACK',True)
    value=wait_for(lambda s:s.get('navigation',{}).get('supports_exploration_mapping'),5,'mapping_adapter')
    nav('mapping-start');record('phone_MAPPING_START_accepted',True)
    value=wait_for(lambda s:s.get('navigation',{}).get('mapping',{}).get('state') in ('EXPLORING','ERROR'),30,'new_SLAM_map')
    mapping=value['navigation']['mapping']
    if mapping['state']=='ERROR':raise RuntimeError('mapping:'+mapping.get('reason','error'))
    # Lifted wheels do not validate exploration quality. End immediately after
    # the first real SLAM map is available; do not fake odometry or clear gates.
    stop();nav('mapping-stop');record('end_mapping_STOP_then_save',True)
    value=wait_for(lambda s:s.get('navigation',{}).get('mapping',{}).get('state') in ('LOADED','ERROR'),65,'save_localize_COMMIT')
    mapping=value['navigation']['mapping']
    if mapping['state']!='LOADED':raise RuntimeError('map_finalize:'+mapping.get('reason','error'))
    record('actual_COMMIT_banner',mapping.get('map_commit_confirmed',False),crc=mapping.get('loaded_crc32'))
    arm()
    view=request('/api/navigation/map');x,y,_=view['pose'];ox,oy=view['origin'];res=view['resolution_m']
    cx,cy=math.floor((x-ox)/res),math.floor((y-oy)/res)
    if not (0<=cx<64 and 0<=cy<64 and view['navigable'][cy*64+cx]=='.'):
        raise RuntimeError('current_real_map_cell_blocked:no_synthetic_map_substituted')
    gx,gy=ox+(cx+.5)*res,oy+(cy+.5)*res
    if math.hypot(gx-x,gy-y)>.08:raise RuntimeError('lifted_test_goal_outside_80mm')
    nav('goal',x=gx,y=gy,map_session=view['session_id'],map_revision=view['map_revision'])
    wait_for(lambda s:s.get('navigation',{}).get('can_start',False),6,'real_FPGA_plan_preview')
    record('goal_preview_only',not state()['navigation'].get('active',False),goal_distance_mm=round(math.hypot(gx-x,gy-y)*1000,3))
    nav('start');record('explicit_START_accepted',True)
    value=wait_for(lambda s:s.get('navigation',{}).get('fpga_state') in ('ARRIVED','CANCELLED'),6,'real_FPGA_terminal')
    arrived=value['navigation'].get('fpga_state')=='ARRIVED'
    record('same_cell_FPGA_arrival',arrived,fpga_state=value['navigation'].get('fpga_state'))
    if not arrived:raise RuntimeError('FPGA_cancelled_before_arrival')
    stop();record('explicit_END',True)
except Exception as ex:
    record('flow_error',False,reason=str(ex))
    try:record('failure_navigation_snapshot',False,navigation=state().get('navigation',{}))
    except Exception:pass
finally:
    if session:stop()
    done=True;thread.join(2)
    print(json.dumps({'summary':{'passed':all(r['passed'] for r in records),
        'cases':len(records),'FPGA_changed':False,'lifted_wheels':True,
        'actual_ground_tracking_or_avoidance':'NOT_RUN'}}),flush=True)
    sys.exit(0 if records and all(r['passed'] for r in records) else 1)
