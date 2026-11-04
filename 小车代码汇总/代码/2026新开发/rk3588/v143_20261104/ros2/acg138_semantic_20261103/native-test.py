"""Native ROS integration with synthetic sensors/YOLO and NO hardware bridge.

Run only in ROS_DOMAIN_ID=187; starts no platform/SLAM/FPGA/network-motor node.
Actual explore_lite, action adapter and production MappingExplorer run unchanged.
"""
import base64,json,math,os,threading,time,urllib.request
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
assert os.environ.get('ROS_DOMAIN_ID')=='187'
TOKEN='semantic-offline-fixture';ROOT=Path('/tmp/semantic-native-fixture');ROOT.mkdir(exist_ok=True)
(ROOT/'token').write_text(TOKEN);Path('/data/logs').mkdir(parents=True,exist_ok=True)
shared={'target':False,'move':False,'quit':False,'seq':0,'post':None}
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*a):pass
 def do_GET(self):
  if self.headers.get('Authorization')!='Bearer '+TOKEN:self.send_error(403);return
  if self.path=='/api/state':
   body=json.dumps({'motor':{'ack_age_ms':0,'ack_left':.3 if shared['move'] else 0.,'ack_right':.3 if shared['move'] else 0.}}).encode()
   self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body);return
  if self.path!='/api/video.mjpg':self.send_error(404);return
  self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=robotframe');self.end_headers()
  try:
   while not shared['quit']:
    shared['seq']+=1;stamp=int(time.monotonic()*1000);d=[]
    if shared['target']:
     d=[{'label':'Person','eye':0,'score':.95,'box':[100,50,300,550]}, {'label':'NO-Mask','eye':0,'score':.95,'box':[150,80,200,130]}]
    m=dict(demo=False,control_eligible=True,detections_pair_status='PAIR_COMPLETE',detection_timestamp_ms=stamp,
      detections_frame_id=shared['seq'],source_epoch=1,width=1000,height=600,detections=d)
    header=b'--robotframe\r\nContent-Length: 1\r\nX-Frame-Meta: '+base64.b64encode(json.dumps(m).encode())+b'\r\n\r\nX\r\n'
    self.wfile.write(header);self.wfile.flush();time.sleep(.066)
  except (BrokenPipeError,ConnectionResetError):pass
server=ThreadingHTTPServer(('127.0.0.1',8082),Handler);server.daemon_threads=True
threading.Thread(target=server.serve_forever,daemon=True).start()
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from acg138_nav_interfaces.srv import Execution
from acg138_semantic.node import SemanticSearch
from acg138_frontier.mapping_explorer import MappingExplorer
rclpy.init(args=['--ros-args','-p','dry_run:=false','-p','token_file:='+str(ROOT/'token')])
manager=SemanticSearch();explorer=MappingExplorer()
class Fixture(Node):
 def __init__(self):
  super().__init__('synthetic_semantic_fixture');self.t0=time.monotonic();self.last=self.t0
  self.x=self.y=self.yaw=0.;self.drive={};self.states=[];self.goals=[];self.nonzero=0;self.found_at=None;self.bad_after_found=0;self.trace=[];self.start_sent=False;self.start_future=None
  qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.mp=self.create_publisher(OccupancyGrid,'/map',qos);self.sp=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data);self.op=self.create_publisher(Odometry,'/odom',10)
  self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
  a=TransformStamped();a.header.frame_id='map';a.child_frame_id='odom';a.transform.rotation.w=1.0
  b=TransformStamped();b.header.frame_id='base_link';b.child_frame_id='laser';b.transform.rotation.w=1.0
  self.static.sendTransform([a,b]);self.start=self.create_client(Execution,'/exploration/manual_start')
  self.create_subscription(String,'/semantic/state',self.state,10);self.create_subscription(String,'/exploration/state',self.exploration,10)
  self.create_subscription(String,'/semantic/goal',lambda m:self.goals.append(json.loads(m.data)),10)
  self.create_timer(.05,self.tick)
 def state(self,m):
  v=json.loads(m.data);self.states.append(v)
  if v['state']=='FOUND' and self.found_at is None:self.found_at=time.monotonic()
 def exploration(self,m):
  v=json.loads(m.data);self.drive=v['drive'];shared['move']=v['drive']['motion_requested']
  trace=(v['state'],v['reason'],v.get('external_status'),v.get('path_points'))
  if not self.trace or self.trace[-1]!=trace:self.trace.append(trace)
  if shared['move']:
   self.nonzero+=1
   if self.found_at is not None:self.bad_after_found+=1
 def post(self):
  try:
   command=dict(task='synthetic-task',target='NO-Mask',gateway_generation='12',produced_mono_ns=str(time.monotonic_ns()),valid_for_ms=500)
   req=urllib.request.Request('http://127.0.0.1:8088/task',json.dumps(command).encode(),{'Authorization':'Bearer '+TOKEN,'Content-Type':'application/json'},method='POST')
   with urllib.request.urlopen(req,timeout=1) as res:shared['post']=json.loads(res.read())
  except Exception as ex:shared['post']={'error':type(ex).__name__}
 def tick(self):
  now=time.monotonic();dt=min(.1,now-self.last);self.last=now
  d=self.drive;v=w=0.
  if shared['move']:
   if d.get('op')=='rotate':w=.6 if d.get('direction')=='ccw' else -.6
   else:v=d.get('throttle',0)*.5;w=-d.get('steer',0)*2
  self.x+=v*math.cos(self.yaw)*dt;self.y+=v*math.sin(self.yaw)*dt;self.yaw+=w*dt
  stamp=self.get_clock().now().to_msg()
  t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom';t.child_frame_id='base_link';t.transform.translation.x=self.x;t.transform.translation.y=self.y;t.transform.rotation.z=math.sin(self.yaw/2);t.transform.rotation.w=math.cos(self.yaw/2);self.tf.sendTransform(t)
  od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link';od.pose.pose.position.x=self.x;od.pose.pose.position.y=self.y;od.pose.pose.orientation=t.transform.rotation;od.twist.twist.linear.x=v;od.twist.twist.angular.z=w;self.op.publish(od)
  scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser';scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.range_min=.05;scan.range_max=12.;scan.ranges=[float('inf')]*360;self.sp.publish(scan)
  if now-self.t0>.5:
   m=OccupancyGrid();m.header.stamp=stamp;m.header.frame_id='map';m.info.width=m.info.height=100;m.info.resolution=.05;m.info.origin.position.x=m.info.origin.position.y=-2.5;m.info.origin.orientation.w=1.
   m.data=[0 if 20<=i%100<80 and 20<=i//100<80 else -1 for i in range(10000)];self.mp.publish(m)
  if now-self.t0>1 and not self.start_sent:
   self.start_sent=True;threading.Thread(target=self.post,daemon=True).start()
  if shared['post'] and shared['post'].get('ok') and self.start_future is None and self.start.service_is_ready():
   q=Execution.Request();q.request_id=time.monotonic_ns();q.run_generation=12;q.created_monotonic_ns=time.monotonic_ns();q.valid_for_ms=500;self.start_future=self.start.call_async(q)
  if self.nonzero>=3:shared['target']=True
fixture=Fixture();executor=SingleThreadedExecutor()
for n in (manager,explorer,fixture):executor.add_node(n)
try:
 deadline=time.monotonic()+35
 while time.monotonic()<deadline:
  executor.spin_once(timeout_sec=.02)
  if fixture.found_at is not None and time.monotonic()-fixture.found_at>1:break
 report=dict(scope='actual explore_lite and production ROS nodes; synthetic camera/TF/map/odom; NO hardware bridge or motion packets',
    admission=shared['post'],start_reply=str(fixture.start_future.result()) if fixture.start_future and fixture.start_future.done() else None,
    states=list(dict.fromkeys(x['state'] for x in fixture.states)),external_goals=[g for g in fixture.goals if g.get('op')!='cancel'],
    exploration_trace=fixture.trace[-40:],simulated_nonzero_intents=fixture.nonzero,found=fixture.found_at is not None,nonzero_after_found=fixture.bad_after_found,
    final=fixture.states[-1] if fixture.states else {})
 (ROOT/'result.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report,ensure_ascii=False))
 assert report['found'] and report['external_goals'] and fixture.nonzero>0 and fixture.bad_after_found==0,report
finally:
 shared['quit']=True
 for n in (fixture,explorer,manager):executor.remove_node(n);n.destroy_node()
 executor.shutdown();rclpy.shutdown();server.shutdown();server.server_close()
