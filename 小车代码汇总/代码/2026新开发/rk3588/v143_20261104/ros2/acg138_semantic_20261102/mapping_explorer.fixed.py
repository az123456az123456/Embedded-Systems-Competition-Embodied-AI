"""Frontier exploration via the existing phone manual DRIVE/ROTATE path.

No FPGA map/GOAL/START/NVA messages. Gateway alone owns the authenticated
manual backend, phone generation, authorization timeout and stale-command stop.
"""
import json,math,time
from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener
from acg138_nav_interfaces.srv import Execution
from .mapping_core import RequestLedger
from .exploration_motion import plan_frontier,on_safe_grid

class MappingExplorer(Node):
    def __init__(self):
        super().__init__('acg138_mapping_explorer')
        self.dry_run=bool(self.declare_parameter('dry_run',True).value)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.grid=None;self.grid_seq=0;self.scan=None;self.active=False
        self.generation=time.monotonic_ns();self.gateway_generation=0;self.sequence=0
        self.state='IDLE';self.reason='press_mapping_start';self.visited=[];self.plan=None;self.cursor=0
        self.future=None;self.planning_generation=0;self.planned_map=0;self.plan_at=0
        self.planner_pool=ThreadPoolExecutor(max_workers=1);self.ledger=RequestLedger()
        self.started=0;self.last_progress=0;self.progress_pose=None;self.wait_since=0
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',self.map_cb,qos)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(String,'/exploration/manual_takeover',self.takeover,8)
        self.create_service(Execution,'/exploration/manual_start',self.start_cb)
        self.create_service(Trigger,'/exploration/stop',self.stop_cb)
        self.pub=self.create_publisher(String,'/exploration/state',1)
        self.timer=self.create_timer(.05,self.tick)
    def map_cb(self,m):
        q=m.info.origin.orientation
        if m.header.frame_id!='map' or len(m.data)!=m.info.width*m.info.height or len(m.data)>1000000:return
        self.grid=(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,2*math.atan2(q.z,q.w),tuple(m.data));self.grid_seq+=1
    def scan_cb(self,m):self.scan=m
    def start_cb(self,q,r):
        try:
            old=self.ledger.begin('manual-start',q.request_id,q.run_generation,q.created_monotonic_ns,q.valid_for_ms,time.monotonic_ns())
            if old is not None:r.success,r.status,r.message=old;return r
            if self.dry_run:raise ValueError('dry_run_no_motion')
            if self.active:raise ValueError('exploration_already_active')
            self.generation+=1;self.gateway_generation=q.run_generation;self.sequence=0;self.active=True
            self.started=self.last_progress=time.monotonic();self.progress_pose=None;self.wait_since=0
            self.state='SELECTING';self.reason='selecting_reachable_frontier';self.visited=[];self.plan=None;self.plan_at=0
            r.success=True;r.message=json.dumps({'operation':'exploration-started','generation':self.generation})
        except Exception as ex:r.status=4;r.message=str(ex)
        if q.request_id in self.ledger.cache:self.ledger.finish(q.request_id,r.success,r.status,r.message)
        return r
    def stop_cb(self,q,r):
        self.active=False;self.state='IDLE';self.reason='explicit_stop';self.plan=None
        r.success=True;r.message='exploration stopped';return r
    def takeover(self,m):
        try:
            v=json.loads(m.data)
            if int(v.get('gateway_generation',0))>=self.gateway_generation:
                self.active=False;self.state='IDLE';self.reason='manual_takeover';self.plan=None
        except (ValueError,TypeError):pass
    def pose(self):
        t=self.tf.lookup_transform('map','base_link',Time());age=self.get_clock().now().nanoseconds-Time.from_msg(t.header.stamp).nanoseconds
        if not 0<=age<=250000000:raise ValueError('waiting_fresh_pose')
        p=t.transform.translation;q=t.transform.rotation
        result=(p.x,p.y,2*math.atan2(q.z,q.w))
        if not all(math.isfinite(v) for v in result):raise ValueError('invalid_pose')
        return result,age
    def clearance(self):
        m=self.scan
        if m is None:raise ValueError('waiting_lidar_scan')
        age=self.get_clock().now().nanoseconds-Time.from_msg(m.header.stamp).nanoseconds
        if not 0<=age<=250000000 or len(m.ranges)<180 or not math.isfinite(m.angle_increment) or m.angle_increment==0:
            raise ValueError('waiting_fresh_full_scan')
        t=self.tf.lookup_transform('base_link',m.header.frame_id,Time());q=t.transform.rotation
        yaw=2*math.atan2(q.z,q.w);tx,ty=t.transform.translation.x,t.transform.translation.y
        front=float('inf');circle=float('inf');seen=0;valid=0;front_valid=0
        for i,r in enumerate(m.ranges):
            a=m.angle_min+i*m.angle_increment+yaw
            ahead=abs(math.atan2(math.sin(a),math.cos(a)))<.65
            if ahead:seen+=1
            good=(math.isfinite(r) and m.range_min<=r<=m.range_max) or r==float('inf')
            if not good:continue
            valid+=1
            if ahead:front_valid+=1
            if not math.isfinite(r):continue
            x,y=tx+r*math.cos(a),ty+r*math.sin(a);circle=min(circle,math.hypot(x,y))
            if x>0 and abs(y)<.22:front=min(front,x)
        if valid<len(m.ranges)*.6 or seen<5 or front_valid<seen*.6 or abs(m.angle_increment)*len(m.ranges)<6.0:
            raise ValueError('lidar_coverage_incomplete')
        return front,circle,age
    def tick(self):
        now=time.monotonic();stamp=time.monotonic_ns();pose_age=scan_age=None
        op='drive';throttle=steer=speed=0.;direction='cw';moving=False
        try:
            if self.active:
                pose,pose_age=self.pose();front,circle,scan_age=self.clearance()
                if self.grid is None:raise ValueError('waiting_map')
                if now-self.last_progress>300:
                    self.active=False;self.state='ERROR';self.reason='exploration_progress_timeout'
                if self.active and self.future is not None and self.future.done():
                    f=self.future;self.future=None;result=f.result()
                    if self.planning_generation==self.generation:
                        self.plan=result;self.cursor=1;self.plan_at=now
                        self.state='NAVIGATING' if result['path'] else 'NO_FRONTIER';self.reason=result['reason']
                if self.active and self.future is None and (self.plan is None or (self.grid_seq!=self.planned_map and now-self.plan_at>=1)):
                    self.planning_generation=self.generation;self.planned_map=self.grid_seq
                    self.future=self.planner_pool.submit(plan_frontier,self.grid,pose,tuple(self.visited),.20)
                if self.active and self.plan is None:
                    self.state='SELECTING';self.reason='computing_frontier_route'
                # Replanning does not pause an existing route; fresh lidar still
                # stops every motion tick, and a completed plan replaces it atomically.
                if self.active and self.plan is not None and self.plan['path']:
                    path=self.plan['path'];goal=path[-1]
                    if math.hypot(goal[0]-pose[0],goal[1]-pose[1])<=.12:
                        self.visited=(self.visited+[goal])[-128:];self.plan=None;self.state='ARRIVED';self.reason='frontier_reached'
                    else:
                        # Advance at most one corner per tick; never look through a wall.
                        if self.cursor<len(path)-1 and math.hypot(path[self.cursor][0]-pose[0],path[self.cursor][1]-pose[1])<.08:self.cursor+=1
                        target=path[self.cursor];dx,dy=target[0]-pose[0],target[1]-pose[1]
                        if not on_safe_grid(self.plan,*pose[:2]) or any(not on_safe_grid(self.plan,pose[0]+dx*k/10,pose[1]+dy*k/10) for k in range(11)):
                            self.plan=None;raise ValueError('route_no_longer_clear')
                        error=math.atan2(math.sin(math.atan2(dy,dx)-pose[2]),math.cos(math.atan2(dy,dx)-pose[2]))
                        if abs(error)>.35:
                            if circle<=.20:raise ValueError('rotation_clearance_blocked')
                            op='rotate';direction='ccw' if error>0 else 'cw';speed=.22;moving=True
                        else:
                            if front<=.30: self.plan=None;raise ValueError('front_obstacle_replan')
                            throttle=.30;steer=max(-.15,min(.15,-error*.35));moving=True
                        self.state='NAVIGATING';self.reason='following_frontier_route'
                        if self.progress_pose is None or math.hypot(pose[0]-self.progress_pose[0],pose[1]-self.progress_pose[1])>.10:
                            self.last_progress=now;self.progress_pose=pose
                # No frontier is observable; keep mapping and wait for map changes.
                if self.active and self.state=='NO_FRONTIER':self.reason='no_reachable_frontier_waiting_map'
                self.wait_since=0
        except Exception as ex:
            moving=False;op='drive';throttle=steer=speed=0.
            self.state='WAITING' if self.active else self.state;self.reason=str(ex)
            if not self.wait_since:self.wait_since=now
            if self.active and now-self.wait_since>30:
                self.active=False;self.state='ERROR';self.reason='exploration_wait_timeout:'+str(ex)
        self.sequence+=1
        # Even a zero command has a bounded original production timestamp.
        drive={'gateway_generation':str(self.gateway_generation),'generation':str(self.generation),'sequence':str(self.sequence),
               'produced_mono_ns':str(stamp),'valid_for_ms':200,'op':op,'throttle':throttle,'steer':steer,'speed':speed,'direction':direction,
               'pose_age_ns':pose_age,'scan_age_ns':scan_age,'motion_requested':moving}
        value={'state':self.state,'reason':self.reason,'generation':self.generation,'gateway_generation':str(self.gateway_generation),
               'published_mono_ns':stamp,'active':self.active,'strategy':'frontier','execution_source':'RK_manual_channel',
               'drive':drive,'goal':self.plan.get('goal') if self.plan else None,'path_points':len(self.plan['path']) if self.plan else 0}
        self.pub.publish(String(data=json.dumps(value,separators=(',',':'))))
    def destroy_node(self):
        self.active=False;self.planner_pool.shutdown(wait=False,cancel_futures=True);return super().destroy_node()

def main():
    rclpy.init();node=MappingExplorer()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
