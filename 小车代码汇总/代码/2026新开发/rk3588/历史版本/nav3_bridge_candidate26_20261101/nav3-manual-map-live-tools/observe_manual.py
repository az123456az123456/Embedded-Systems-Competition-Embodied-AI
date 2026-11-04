"""Line A: choose ONE reachable real-map goal; never publish cmd_vel/START/permit."""
import collections,json,math,time,zlib
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from slam_toolbox.srv import Pause
rclpy.init();node=Node('nav3_manual_map_observer');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
result={'goal':None,'path':None,'metadata':None,'local_map':None,'events':[],'metadata_history':[],'path_events':[],'source_maps':[]};local=None
pub=node.create_publisher(PoseStamped,'/fpga/goal',10)
def map_cb(m):
 global local
 local=m
 result['local_map']={'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y],'data':list(m.data)}
def metadata_cb(m):
 result['metadata']=json.loads(m.data)
 result['metadata_history'].append(dict(result['metadata'],observed_ns=node.get_clock().now().nanoseconds))
def path_cb(m):
 result['path_events'].append({'count':len(m.poses),'observed_ns':node.get_clock().now().nanoseconds})
 if m.poses:result['path']={'frame':m.header.frame_id,'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]}
subs=[node.create_subscription(OccupancyGrid,'/fpga/local_costmap',map_cb,q),node.create_subscription(String,'/fpga/map_metadata',metadata_cb,q),node.create_subscription(Path,'/fpga/planned_path',path_cb,q)]
def choose_goal():
 if phase!='loaded':return
 if result['goal'] or local is None:return
 m=result['metadata']
 if not m or m.get('map_load_state')!='FROZEN' or not m.get('uploaded') or not m.get('state_crc_verified'):return
 if len(local.data)!=4096:return
 bitmap=bytearray(512)
 for i,v in enumerate(local.data):
  if v!=0:bitmap[i//8]|=1<<(i%8)
 if zlib.crc32(bitmap)!=m['bitmap_crc32']:return
 x,y=m['pose_m_rad'][:2];ox,oy=m['origin_m'];res=m['resolution_m']
 start=int(math.floor((y-oy)/res))*64+int(math.floor((x-ox)/res))
 if start<0 or start>=4096 or local.data[start]!=0:return
 queue=collections.deque([(start,0)]);seen={start};target=None
 while queue:
  cell,dist=queue.popleft()
  if dist>=3:target=cell;break
  for dx,dy in [(1,0),(0,1),(-1,0),(0,-1)]:
   cx,cy=cell%64+dx,cell//64+dy
   if not(0<=cx<64 and 0<=cy<64):continue
   nxt=cy*64+cx
   if nxt not in seen and local.data[nxt]==0:seen.add(nxt);queue.append((nxt,dist+1))
 if target is None:return
 goal=PoseStamped();goal.header.frame_id=local.header.frame_id;goal.header.stamp=node.get_clock().now().to_msg();goal.pose.orientation.w=1.
 goal.pose.position.x=ox+(target%64+.5)*res;goal.pose.position.y=oy+(target//64+.5)*res
 result['goal']={'x':goal.pose.position.x,'y':goal.pose.position.y,'start_cell':start,'goal_cell':target,'map_revision':m['map_revision'],'crc32':m['bitmap_crc32'],'stamp_ns':node.get_clock().now().nanoseconds}
 result['goal_map']=result['local_map'];result['goal_metadata']=m.copy()
 pub.publish(goal);print(json.dumps({'goal_sent':result['goal']}),flush=True)
pause=node.create_client(Pause,'/slam_toolbox/pause_new_measurements')
load=node.create_client(Trigger,'/fpga/load_map')
phase='preview';future=None;pause_at=None;load_sent=False;repeat_sent=False

def event(name,**values):
 row=dict(event=name,ros_ns=node.get_clock().now().nanoseconds,elapsed_s=time.monotonic()-started_at,**values)
 result['events'].append(row);print(json.dumps(row),flush=True)

def raw_map(m):
 result['source_maps'].append({'stamp_ns':m.header.stamp.sec*1000000000+m.header.stamp.nanosec,'observed_ns':node.get_clock().now().nanoseconds,'cells_crc32':zlib.crc32(bytes((v&255 for v in m.data))), 'width':m.info.width,'height':m.info.height})
subs.append(node.create_subscription(OccupancyGrid,'/map',raw_map,q))
started_at=time.monotonic();last=started_at
try:
 while rclpy.ok() and time.monotonic()-started_at<44:
  rclpy.spin_once(node,timeout_sec=.1)
  elapsed=time.monotonic()-started_at
  if phase=='preview' and elapsed>=18 and result['metadata'] and local is not None:
   if not pause.service_is_ready() or not load.service_is_ready():continue
   event('mapping_preview_complete',metadata=result['metadata'].copy(),preview_free_cells=sum(v==0 for v in local.data))
   future=pause.call_async(Pause.Request());phase='pausing';event('pause_request')
  elif phase=='pausing' and future.done():
   reply=future.result();event('pause_response',status=bool(reply.status))
   if not reply.status:raise RuntimeError('Pause did not confirm true; no map load')
   phase='settling';pause_at=time.monotonic()
  elif phase=='settling' and time.monotonic()-pause_at>=2:
   event('load_request');future=load.call_async(Trigger.Request());phase='loading';load_sent=True
  elif phase=='loading' and future.done():
   reply=future.result();event('load_response',success=reply.success,message=reply.message)
   if not reply.success:raise RuntimeError('Explicit map request rejected')
   phase='loaded'
  choose_goal()
  if phase=='loaded' and result['path'] and not repeat_sent:
   event('second_load_request');future=load.call_async(Trigger.Request());repeat_sent=True;phase='repeat_pending'
  elif phase=='repeat_pending' and future.done():
   reply=future.result();event('second_load_response',success=reply.success,message=reply.message)
   if reply.success:raise RuntimeError('Second map request incorrectly admitted')
   phase='done'
  if time.monotonic()-last>5:
   last=time.monotonic();print(json.dumps({'elapsed_s':elapsed,'phase':phase,'goal':result['goal'],'path_points':len(result['path']['points']) if result['path'] else 0,'metadata':result['metadata']}),flush=True)
except Exception as e:
 result['exception']=repr(e);raise
finally:
 result['final_phase']=phase
 with open('/data/logs/nav3-manual-map-observer-result.json','w') as f:json.dump(result,f)
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
