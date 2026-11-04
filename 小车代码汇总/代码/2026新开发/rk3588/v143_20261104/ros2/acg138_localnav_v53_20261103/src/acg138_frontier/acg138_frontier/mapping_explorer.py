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
from .exploration_motion import plan_frontier,scan_clearance,route_clear_latest,body_motion_block,tracking_target
from .recovery import Retreat,rear_scan_clear,reverse_block_detail
from .body_heading import physical_pose,sensor_in_body
from .startup_escape import StartupEscape,startup_admission

class MappingExplorer(Node):
    def __init__(self):
        super().__init__('acg138_mapping_explorer')
        self.dry_run=bool(self.declare_parameter('dry_run',True).value)
        self.inflation_radius=float(self.declare_parameter('inflation_radius',.15).value)
        self.body_width=float(self.declare_parameter('body_width',.30).value)
        self.body_front=float(self.declare_parameter('body_front',.1589).value)
        self.body_rear=float(self.declare_parameter('body_rear',.1942).value)
        self.heading_offset=float(self.declare_parameter('navigation_heading_offset_rad',0.).value)
        if not math.isfinite(self.heading_offset):raise ValueError('invalid_body_heading')
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.grid=None;self.grid_seq=0;self.scan=None;self.active=False
        self.generation=time.monotonic_ns();self.gateway_generation=0;self.sequence=0
        self.state='IDLE';self.reason='press_mapping_start';self.visited=[];self.plan=None;self.cursor=0
        self.future=None;self.planning_generation=0;self.planned_map=0;self.plan_at=0
        self.route_epoch=0;self.planning_epoch=0;self.checked_map=0
        self.blocked_grid=None;self.blocked_pose=None;self.blocked_reason='';self.retry_at=0.
        self.block_detail=None
        self.recovery_detail=None
        self.startup=StartupEscape(time.monotonic());self.startup_origin=None
        self.retreat=Retreat();self.recovery_reason='';self.forward_progress_pose=None
        # Node.executor belongs to rclpy; assigning a Python worker pool invokes
        # ROS's executor setter and crashes before the service is advertised.
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
            self.route_epoch+=1;self.checked_map=0
            self.blocked_grid=None;self.blocked_pose=None;self.blocked_reason='';self.retry_at=0.
            self.block_detail=None
            self.recovery_detail=None
            self.startup=StartupEscape(time.monotonic());self.startup_origin=None
            self.retreat=Retreat();self.recovery_reason='';self.forward_progress_pose=None
            r.success=True;r.message=json.dumps({'operation':'exploration-started','generation':self.generation})
        except Exception as ex:r.status=4;r.message=str(ex)
        if q.request_id in self.ledger.cache:self.ledger.finish(q.request_id,r.success,r.status,r.message)
        return r
    def stop_cb(self,q,r):
        self.active=False;self.state='IDLE';self.reason='explicit_stop';self.plan=None
        self.retreat.cancel()
        self.route_epoch+=1
        r.success=True;r.message='exploration stopped';return r
    def takeover(self,m):
        try:
            v=json.loads(m.data)
            if int(v.get('gateway_generation',0))>=self.gateway_generation:
                self.active=False;self.state='IDLE';self.reason='manual_takeover';self.plan=None
                self.retreat.cancel()
                self.route_epoch+=1
        except (ValueError,TypeError):pass
    def pose(self):
        t=self.tf.lookup_transform('map','base_link',Time());age=self.get_clock().now().nanoseconds-Time.from_msg(t.header.stamp).nanoseconds
        if not 0<=age<=250000000:raise ValueError('waiting_fresh_pose')
        p=t.transform.translation;q=t.transform.rotation
        result=(p.x,p.y,2*math.atan2(q.z,q.w))
        if not all(math.isfinite(v) for v in result):raise ValueError('invalid_pose')
        return physical_pose(result,self.heading_offset),age
    def clearance(self):
        m=self.scan
        if m is None:raise ValueError('waiting_lidar_scan')
        age=self.get_clock().now().nanoseconds-Time.from_msg(m.header.stamp).nanoseconds
        if not 0<=age<=250000000 or len(m.ranges)<180 or not math.isfinite(m.angle_increment) or m.angle_increment==0:
            raise ValueError('waiting_fresh_full_scan')
        t=self.tf.lookup_transform('base_link',m.header.frame_id,Time());q=t.transform.rotation
        yaw,tx,ty=sensor_in_body(2*math.atan2(q.z,q.w),t.transform.translation.x,
                                t.transform.translation.y,self.heading_offset)
        front,circle=scan_clearance(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,yaw,tx,ty)
        return front,circle,age
    def block_route(self,reason,pose,now,detail=None):
        self.plan=None;self.route_epoch+=1
        self.blocked_grid=self.grid;self.blocked_pose=pose;self.blocked_reason=reason;self.retry_at=now+.5
        self.block_detail=detail
        raise ValueError(reason)
    def odom_pose(self):
        # Odom is continuous across map->odom corrections: SLAM jumps cannot
        # pretend that the robot physically backed up 15 cm.
        t=self.tf.lookup_transform('odom','base_link',Time())
        age=self.get_clock().now().nanoseconds-Time.from_msg(t.header.stamp).nanoseconds
        if not 0<=age<=250000000:raise ValueError('reverse_waiting_fresh_odom')
        p=t.transform.translation;q=t.transform.rotation
        odom=(p.x,p.y,2*math.atan2(q.z,q.w))
        if not all(math.isfinite(v) for v in odom):raise ValueError('reverse_invalid_odom')
        return physical_pose(odom,self.heading_offset)
    def recovery_step(self,pose,now):
        self.recovery_detail=None
        odom=self.odom_pose()
        m=self.scan;t=self.tf.lookup_transform('base_link',m.header.frame_id,Time());q=t.transform.rotation
        yaw,tx,ty=sensor_in_body(2*math.atan2(q.z,q.w),t.transform.translation.x,
                                t.transform.translation.y,self.heading_offset)
        rear_ok=rear_scan_clear(m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,
            yaw,tx,ty,self.body_width,self.body_rear)
        if not self.retreat.active:
            if not rear_ok:return False,'reverse_lidar_blocked_or_unobserved'
            block=reverse_block_detail(self.grid,pose,self.body_width,self.body_front,self.body_rear)
            if block:
                self.recovery_detail=dict(block,map_seq=self.grid_seq)
                return False,block['reason']
            if not self.retreat.begin(odom,now):return False,'reverse_attempt_limit_stop'
            self.forward_progress_pose=None
            self.plan=None;self.route_epoch+=1;self.checked_map=0
        moving,reason=self.retreat.update(self.grid,pose,odom,now,rear_ok,self.body_width,self.body_front,self.body_rear)
        if reason=='reverse_complete_replan':
            self.blocked_grid=None;self.blocked_pose=None;self.blocked_reason='';self.retry_at=0.;self.plan=None
        return moving,reason
    def startup_step(self,pose,now,front,circle):
        startup=self.startup;odom=self.odom_pose()
        if startup.active:
            done=startup.progress(odom,now)
            if done:
                self.plan=None;self.blocked_grid=None;self.blocked_pose=None;self.route_epoch+=1
                return False,done
        if front<=.45 or circle<=.28:
            if startup.active:startup.finish()
            return False,'startup_live_clearance_blocked'
        m=self.scan;t=self.tf.lookup_transform('base_link',m.header.frame_id,Time());q=t.transform.rotation
        yaw,tx,ty=sensor_in_body(2*math.atan2(q.z,q.w),t.transform.translation.x,
                                t.transform.translation.y,self.heading_offset)
        ok,detail=startup_admission(self.grid,pose,m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,
                                   yaw,tx,ty,self.body_width,self.body_front,self.body_rear)
        startup.detail=detail
        if not ok:
            if startup.active:startup.finish()
            return False,detail['reason']
        if not startup.active:
            if not startup.begin(odom,now):return False,'startup_window_closed'
            self.plan=None;self.route_epoch+=1;self.checked_map=0
            self.blocked_grid=None;self.blocked_pose=None;self.retreat.cancel()
        return True,'startup_lidar_verified_step'
    def wait_for_changed_evidence(self,pose,now):
        if self.blocked_grid is None:return
        previous=self.blocked_pose
        displaced=math.hypot(pose[0]-previous[0],pose[1]-previous[1])>=.05
        turned=abs(math.atan2(math.sin(pose[2]-previous[2]),math.cos(pose[2]-previous[2])))>=math.radians(5)
        # A new message/sequence carrying identical cells is not new evidence.
        # Do not continuously discard and select the same blocked route.
        if now<self.retry_at or (self.grid==self.blocked_grid and not displaced and not turned):
            raise ValueError(self.blocked_reason)
        self.blocked_grid=None;self.blocked_pose=None;self.blocked_reason=''
    def tick(self):
        now=time.monotonic();stamp=time.monotonic_ns();pose_age=scan_age=None
        op='drive';throttle=steer=speed=0.;direction='cw';moving=False
        try:
            if self.active:
                pose,pose_age=self.pose();front,circle,scan_age=self.clearance()
                if self.grid is None:raise ValueError('waiting_map')
                if getattr(self,'startup',None) and self.startup.active:raise ValueError('startup_escape_active')
                if self.retreat.active:raise ValueError('reverse_recovery_active')
                if now-self.last_progress>300:
                    self.active=False;self.state='ERROR';self.reason='exploration_progress_timeout'
                if self.active:self.wait_for_changed_evidence(pose,now)
                if self.active and self.future is not None and self.future.done():
                    f=self.future;self.future=None
                    if self.planning_generation==self.generation and self.planning_epoch==self.route_epoch:
                        try:result=f.result()
                        except ValueError as ex:
                            self.block_route(str(ex),pose,now,{'kind':'planning_rejected','reason':str(ex)})
                        self.plan=result;self.cursor=1;self.plan_at=now
                        self.checked_map=0
                        self.state='NAVIGATING' if result['path'] else 'NO_FRONTIER';self.reason=result['reason']
                        blocks=result.get('initial_motion_blocks',[])
                        self.block_detail=(dict(blocks[0],map_seq=self.planned_map) if not result['path'] and blocks else None)
                # Keep a valid frontier and cursor until arrival. A map refresh
                # must not continuously replace a good route with another goal.
                if self.active and self.future is None and (self.plan is None or (not self.plan['path'] and self.grid_seq!=self.planned_map and now-self.plan_at>=1)):
                    self.planning_generation=self.generation;self.planned_map=self.grid_seq
                    self.planning_epoch=self.route_epoch
                    self.future=self.planner_pool.submit(plan_frontier,self.grid,pose,tuple(self.visited),
                        self.inflation_radius,self.body_width,self.body_front,self.body_rear)
                if self.active and self.plan is None:
                    self.state='SELECTING';self.reason='computing_frontier_route'
                # Every new map is checked BEFORE any nonzero command. A result
                # computed on an older map passes the same check on adoption.
                if self.active and self.plan is not None and self.plan['path']:
                    path=self.plan['path'];goal=path[-1]
                    if self.checked_map!=self.grid_seq:
                        if not route_clear_latest(self.grid,pose,path,self.cursor,self.inflation_radius,self.body_width,self.body_front,self.body_rear):
                            self.block_route('latest_map_blocks_route_stop_replan',pose,now)
                        self.checked_map=self.grid_seq
                    if math.hypot(goal[0]-pose[0],goal[1]-pose[1])<=.12:
                        self.visited=(self.visited+[goal])[-128:];self.plan=None;self.state='ARRIVED';self.reason='frontier_reached'
                        self.route_epoch+=1
                    else:
                        # Advance at most one corner per tick; never look through a wall.
                        if self.cursor<len(path)-1 and math.hypot(path[self.cursor][0]-pose[0],path[self.cursor][1]-pose[1])<.08:self.cursor+=1
                        target=tracking_target(path,self.cursor,pose);dx,dy=target[0]-pose[0],target[1]-pose[1]
                        error=math.atan2(math.sin(math.atan2(dy,dx)-pose[2]),math.cos(math.atan2(dy,dx)-pose[2]))
                        obstruction=body_motion_block(self.grid,pose,target,abs(error)>.35,self.body_width,self.body_front,self.body_rear)
                        if obstruction is not None:
                            obstruction.update(pose=list(pose),target=list(target),map_seq=self.grid_seq)
                            self.block_route('latest_map_blocks_body_stop_replan',pose,now,obstruction)
                        if abs(error)>.35:
                            if circle<=.20:raise ValueError('rotation_clearance_blocked')
                            op='rotate';direction='ccw' if error>0 else 'cw';speed=.22;moving=True
                        else:
                            if front<=.30: self.plan=None;self.route_epoch+=1;raise ValueError('front_obstacle_replan')
                            throttle=.30;steer=max(-.15,min(.15,-error*.35));moving=True
                        self.state='NAVIGATING';self.reason='following_frontier_route';self.block_detail=None
                        startup=getattr(self,'startup',None)
                        if startup and startup.available(now) and op=='drive' and throttle>0:
                            odom=self.odom_pose()
                            if self.startup_origin is None:self.startup_origin=odom
                            elif math.hypot(odom[0]-self.startup_origin[0],odom[1]-self.startup_origin[1])>=.08:startup.used=True
                        if op=='drive' and throttle>0 and self.retreat.attempts:
                            forward_odom=self.odom_pose()
                            if self.forward_progress_pose is None:self.forward_progress_pose=forward_odom
                            elif math.hypot(forward_odom[0]-self.forward_progress_pose[0],forward_odom[1]-self.forward_progress_pose[1])>.20:
                                self.retreat.attempts=0;self.forward_progress_pose=forward_odom
                        if self.progress_pose is None or math.hypot(pose[0]-self.progress_pose[0],pose[1]-self.progress_pose[1])>.10:
                            self.last_progress=now;self.progress_pose=pose
                # No frontier is observable; keep mapping and wait for map changes.
                if self.active and self.plan is not None and not self.plan['path']:
                    self.state='NO_FRONTIER'
                    self.reason=('frontiers_require_blocked_initial_motion' if self.plan and
                        self.plan.get('reason')=='frontiers_require_blocked_initial_motion' else 'no_reachable_frontier_waiting_map')
                    if self.reason=='frontiers_require_blocked_initial_motion':raise ValueError(self.reason)
                # Planning an unchanged blocked route is not recovery. Preserve
                # the existing 30s wait deadline until a command is admissible.
                if moving or self.state in ('ARRIVED','NO_FRONTIER'):self.wait_since=0
        except Exception as ex:
            moving=False;op='drive';throttle=steer=speed=0.
            self.state='WAITING' if self.active else self.state;self.reason=str(ex)
            recoverable=('reverse_recovery_active','frontiers_require_blocked_initial_motion',
                         'latest_map_blocks_body_stop_replan','rotation_clearance_blocked','front_obstacle_replan',
                         'latest_map_blocks_route_stop_replan','startup_escape_active')
            startup=getattr(self,'startup',None);startup_handled=False
            if self.active and startup and (startup.active or (startup.available(now) and str(ex) in (
                    'frontiers_require_blocked_initial_motion','latest_map_blocks_body_stop_replan','latest_map_blocks_route_stop_replan'))):
                try:
                    moving,startup_reason=self.startup_step(pose,now,front,circle)
                    startup_handled=moving or str(ex)=='startup_escape_active'
                    if startup_handled:
                        self.reason=startup_reason;self.recovery_reason=''
                        if moving:throttle=.12;self.state='NAVIGATING';self.wait_since=0
                        else:self.state='SELECTING'
                except Exception:
                    startup.finish();moving=False
            if self.active and str(ex) in recoverable and not startup_handled:
                try:
                    # First blocked tick remains a stop. Retreat starts on a
                    # later fresh cycle, and never bypasses scan/pose checks.
                    if self.recovery_reason==str(ex) or self.retreat.active:
                        moving,self.reason=self.recovery_step(pose,now)
                        if moving:throttle=-.18;self.state='NAVIGATING';self.wait_since=0
                    self.recovery_reason=str(ex)
                except Exception as recovery_error:
                    self.retreat.cancel();self.reason=str(recovery_error)
            else:
                self.retreat.cancel();self.recovery_reason=''
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
               'inflation_radius_m':self.inflation_radius,'body_width_m':self.body_width,'self_footprint_exemption':True,
               'latest_map_seq':self.grid_seq,'route_checked_map_seq':self.checked_map,'route_epoch':self.route_epoch,
               'body_block':self.block_detail,
               'recovery_block':self.recovery_detail,
               'navigation_heading_offset_rad':self.heading_offset,
               'recovery_active':self.retreat.active,'recovery_attempts':self.retreat.attempts,
               'startup_escape':({'active':self.startup.active,'used':self.startup.used,'detail':self.startup.detail}
                                 if getattr(self,'startup',None) else None),
               'initial_motion_rejections':self.plan.get('initial_motion_rejections',0) if self.plan else 0,
               'self_unknown_cells':self.plan.get('self_unknown_cells',0) if self.plan else 0,
               'initial_motion_blocks':self.plan.get('initial_motion_blocks',[]) if self.plan else [],
               'drive':drive,'goal':self.plan.get('goal') if self.plan else None,'path_points':len(self.plan['path']) if self.plan else 0}
        self.pub.publish(String(data=json.dumps(value,separators=(',',':'))))
    def destroy_node(self):
        self.active=False;self.planner_pool.shutdown(wait=False,cancel_futures=True);return super().destroy_node()

def main():
    rclpy.init();node=MappingExplorer()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
