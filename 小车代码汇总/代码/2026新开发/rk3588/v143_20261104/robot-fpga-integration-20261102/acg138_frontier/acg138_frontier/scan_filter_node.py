"""Filter only SLAM/dynamic map input, leaving raw safety data untouched."""
import copy,json,math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from .scan_filter import MappingScanFilter

class MappingScanFilterNode(Node):
    def __init__(self):
        super().__init__('acg138_mapping_scan_filter')
        self.filter=MappingScanFilter();self.pending=None;self.last_clock=0;self.published=0
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(LaserScan,'/scan_mapping',qos_profile_sensor_data)
        self.diag=self.create_publisher(String,'/mapping/scan_filter_state',1)
        self.create_subscription(LaserScan,'/scan',self.receive,qos_profile_sensor_data)
        self.create_timer(.01,self.tick)

    def receive(self,m):self.pending=m

    def tick(self):
        now=self.get_clock().now().nanoseconds
        if now<self.last_clock:self.filter=MappingScanFilter();self.pending=None
        self.last_clock=now;m=self.pending
        if m is None:return
        stamp=Time.from_msg(m.header.stamp);source=stamp.nanoseconds
        if not 0<=now-source<=250000000:self.pending=None;return
        try:
            t=self.tf.lookup_transform('odom',m.header.frame_id,stamp)
            p=t.transform.translation;q=t.transform.rotation
            pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        except Exception:return # Bounded by the original scan timestamp, not now().
        self.pending=None
        try:values=self.filter.process(source,now,pose,m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max)
        except ValueError:return
        out=copy.deepcopy(m);out.ranges=values;self.pub.publish(out);self.published+=1
        self.diag.publish(String(data=json.dumps({'published':self.published,'source_stamp_ns':source,
            'raw_safety_unchanged':True,'per_beam_deskew':False,**self.filter.stats},separators=(',',':'))))

def main():
    rclpy.init();n=MappingScanFilterNode()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
