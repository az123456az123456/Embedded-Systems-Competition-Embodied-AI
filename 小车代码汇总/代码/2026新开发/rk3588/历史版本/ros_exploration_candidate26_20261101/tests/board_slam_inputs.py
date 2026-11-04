"""Read only ROS SLAM/TF input freshness; no publishers or service calls."""
import json,time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from tf2_ros import Buffer,TransformListener
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init();n=Node('readonly_slam_input_audit');b=Buffer();listener=TransformListener(b,n);values={};counts={}
def received(name,m):
    counts[name]=counts.get(name,0)+1;values[name]=rclpy.time.Time.from_msg(m.header.stamp).nanoseconds
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
subs=[n.create_subscription(OccupancyGrid,t,lambda m,t=t:received(t,m),q) for t in ('/slam/map','/map')]
subs += [n.create_subscription(PoseWithCovarianceStamped,'/pose',lambda m:received('/pose',m),10),n.create_subscription(Odometry,'/odom',lambda m:received('/odom',m),10)]
end=time.monotonic()+5
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
now=n.get_clock().now().nanoseconds;result={'counts':counts,'header_age_ms':{k:(now-v)/1e6 for k,v in values.items()}}
try:
    t=b.lookup_transform('map','base_link',Time());stamp=rclpy.time.Time.from_msg(t.header.stamp).nanoseconds
    result['map_base_TF']={'available':True,'age_ms':(now-stamp)/1e6,'x':t.transform.translation.x,'y':t.transform.translation.y}
except Exception as ex:result['map_base_TF']={'available':False,'reason':str(ex)}
result['publishers']={t:[i.node_name for i in n.get_publishers_info_by_topic(t)] for t in ('/slam/map','/map','/pose','/odom')}
print(json.dumps(result),flush=True);n.destroy_node();rclpy.shutdown()
