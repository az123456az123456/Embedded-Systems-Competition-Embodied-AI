"""Line A: choose ONE reachable real-map goal; never publish cmd_vel/START/permit."""
import collections,json,math,time,zlib
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
rclpy.init();node=Node('nav3_stage3_plan_observer');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
result={'goal':None,'path':None,'metadata':None,'local_map':None};local=None
pub=node.create_publisher(PoseStamped,'/fpga/goal',10)
def map_cb(m):
 global local
 local=m
 result['local_map']={'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y],'data':list(m.data)}
def metadata_cb(m):
 result['metadata']=json.loads(m.data)
def path_cb(m):
 if m.poses:result['path']={'frame':m.header.frame_id,'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]}
subs=[node.create_subscription(OccupancyGrid,'/fpga/local_costmap',map_cb,q),node.create_subscription(String,'/fpga/map_metadata',metadata_cb,q),node.create_subscription(Path,'/fpga/planned_path',path_cb,q)]
def choose_goal():
 if result['goal'] or local is None:return
 m=result['metadata']
 if not m or not m.get('uploaded') or not m.get('state_crc_verified'):return
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
 result['goal']={'x':goal.pose.position.x,'y':goal.pose.position.y,'start_cell':start,'goal_cell':target,'map_revision':m['map_revision'],'crc32':m['bitmap_crc32'],'stamp_ns':node.get_clock().now().nanoseconds()}
 pub.publish(goal);print(json.dumps({'goal_sent':result['goal']}),flush=True)
start=time.monotonic();last=start
try:
 while rclpy.ok() and time.monotonic()-start<32:
  rclpy.spin_once(node,timeout_sec=.1);choose_goal()
  if time.monotonic()-last>5:
   last=time.monotonic();print(json.dumps({'elapsed_s':last-start,'goal':result['goal'],'path_points':len(result['path']['points']) if result['path'] else 0,'metadata':result['metadata']}),flush=True)
finally:
 with open('/data/logs/nav3-stage3-observer-result.json','w') as f:json.dump(result,f)
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
