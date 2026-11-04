"""Map-free lidar observations for the existing authenticated phone controller.

This worker cannot arm, drive, renew leases or start a follow. The controller
accepts observations only for the current selected identity and source times.
"""
import json
import math
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .follow_lidar import associate,associate_centered,RangeConfirmation


def yaw(q):
    norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    if not math.isfinite(norm) or abs(norm-1) > .02:
        raise ValueError('invalid_follow_orientation')
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class FollowLidar(Node):
    def __init__(self):
        super().__init__('acg138_follow_lidar')
        self.offset=float(self.declare_parameter('navigation_heading_offset_rad',math.pi).value)
        self.token=Path(str(self.declare_parameter('token_file','/run/acg138-semantic/token').value)).read_text().strip()
        self.url='http://127.0.0.1:8082'
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.scan=None;self.odom=None;self.future=None;self.previous=None;self.key=None;self.previous_scan=0
        self.confirm=RangeConfirmation()
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
        self.create_timer(.08,self.tick)

    def scan_cb(self,msg): self.scan=msg
    def odom_cb(self,msg): self.odom=msg

    def request(self,path,value=None):
        data=None if value is None else json.dumps(value,allow_nan=False,separators=(',',':')).encode()
        req=urllib.request.Request(self.url+path,data=data,headers={
            'Authorization':'Bearer '+self.token,'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=.18) as response:
            raw=response.read(262145)
        if len(raw)>262144:raise ValueError('follow_state_too_large')
        return json.loads(raw)

    def tick(self):
        if self.future is None or self.future.done():
            # Exceptions fail closed by allowing the original observation to
            # expire. Neither scan nor pose ages are rebased to the worker tick.
            self.future=self.pool.submit(self.observe)

    def observe(self):
        try:
            state=self.request('/api/state')
            target=state.get('selected_target')
            if not target:return
            calibrated=state.get('follow_geometry',{}).get('configured') is True
            if calibrated and target.get('ray_bounds_valid') is not True:return
            if not calibrated and target.get('image_valid') is not True:return
            scan,odom=self.scan,self.odom
            if scan is None or odom is None or odom.header.frame_id!='odom':return
            ros_now=self.get_clock().now().nanoseconds
            mono_now=time.monotonic_ns()
            scan_ns=Time.from_msg(scan.header.stamp).nanoseconds
            pose_ns=Time.from_msg(odom.header.stamp).nanoseconds
            if not 0<=ros_now-scan_ns<=220000000 or not 0<=ros_now-pose_ns<=220000000:return
            # Transform the old scan in odom, then into the latest physical body
            # axes. Rotation of the chassis between scan and camera is retained.
            transform=self.tf.lookup_transform('odom',scan.header.frame_id,Time.from_msg(scan.header.stamp))
            translation=transform.transform.translation
            laser_pose=(translation.x,translation.y,yaw(transform.transform.rotation))
            p=odom.pose.pose.position
            body_pose=(p.x,p.y,yaw(odom.pose.pose.orientation)+self.offset)
            key=(int(target['id']),int(target['selection_mono_ms']))
            if self.key!=key:
                self.previous=None;self.previous_scan=0;self.key=key
            previous=None
            if self.previous and 0<=scan_ns-self.previous_scan<=600000000:
                # Store the hypothesis in odom, so robot motion does not look like
                # a target identity jump in successive body-coordinate scans.
                dx=self.previous[0]-body_pose[0];dy=self.previous[1]-body_pose[1]
                c,s=math.cos(body_pose[2]),math.sin(body_pose[2])
                previous=(c*dx+s*dy,-s*dx+c*dy)
            args=(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,laser_pose,body_pose)
            if calibrated:
                value=associate(*args,target['ray_left_deg'],target['ray_right_deg'],target['camera_lateral_right_m'],previous)
            else:
                value=associate_centered(*args,target['image_error'],target['yaw_since_capture_deg'],
                    target['camera_lateral_right_m'],previous)
            if value['matched']:
                c,s=math.cos(body_pose[2]),math.sin(body_pose[2])
                x,y=value['x_body_m'],value['y_body_m']
                self.previous=(body_pose[0]+c*x-s*y,body_pose[1]+s*x+c*y)
                self.previous_scan=scan_ns
            value['matched']=self.confirm.update(key,scan_ns,value['matched'])
            value.pop('reason')
            value.update(target_id=key[0],selection_ms=key[1],
                vision_ms=int(target['vision_mono_ms']),
                scan_ms=(mono_now-(ros_now-scan_ns))//1000000,
                pose_ms=(mono_now-(ros_now-pose_ns))//1000000)
            self.request('/api/navigation/follow-observation',value)
        except Exception:
            # No authority is acquired on network/TF/scan failure.
            return

    def destroy_node(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init();node=FollowLidar()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
