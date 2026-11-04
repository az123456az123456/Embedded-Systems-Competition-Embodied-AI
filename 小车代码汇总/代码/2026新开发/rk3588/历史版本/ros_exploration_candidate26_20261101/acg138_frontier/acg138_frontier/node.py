"""ROS adapter: goals and lifecycle only. NEVER publishes cmd_vel/MOT/NVA1.
Gateway supplies a confirmed permit ticket on /exploration/authorized_start;
absence of this source is a visible blocker, not implicit authorization.
"""
import json
import math
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from acg138_nav_interfaces.srv import Execution
from .core import Grid, Explorer, Mode, Identity, select_frontier
from .validation import local_from_ros, metadata_matches, parse_identity

class FrontierNode(Node):
    def __init__(self):
        super().__init__('acg138_frontier')
        self.dry_run=bool(self.declare_parameter('dry_run',True).value)
        self.e=Explorer();self.global_map=None;self.local=None;self.meta={};self.fpga={}
        self.observed_cells=0;self.wait_stop_confirmation=False;self.state_received=0;self.last_map_request_revision=None;self.action_request=None
        self.target_sent_ns=0;self.work=None;self.pool=ThreadPoolExecutor(max_workers=1)
        self.next_request=time.monotonic_ns();self.finish_requested=False
        self.last_status_key=None;self.phase_started_ms=self.now_ms();self.phase_key=None
        self.early_commands=deque(maxlen=8)
        self.pub_goal=self.create_publisher(PoseStamped,'/fpga/goal',1)
        self.pub_status=self.create_publisher(String,'/exploration/state',1)
        # Gateway must match identity+generation and mint NVA permission. This is
        # a request for authorization, NOT a speed or an execution command.
        self.pub_start_request=self.create_publisher(String,'/exploration/start_request',1)
        self.pub_final=self.create_publisher(String,'/exploration/finalize_request',1)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',self.map_cb,qos)
        self.create_subscription(OccupancyGrid,'/fpga/local_costmap',self.local_cb,qos)
        self.create_subscription(String,'/fpga/map_metadata',self.meta_cb,qos)
        self.create_subscription(String,'/fpga/state',self.state_cb,qos)
        self.create_subscription(String,'/fpga/plan_result',self.plan_cb,qos)
        self.create_subscription(String,'/fpga/command_result',self.command_cb,8)
        self.create_subscription(String,'/exploration/authorized_start',self.authorized_cb,1)
        self.create_subscription(String,'/exploration/gateway_start',self.gateway_start_cb,8)
        self.create_subscription(String,'/exploration/manual_takeover',lambda m:self.stop('MANUAL_TAKEOVER'),1)
        self.create_service(Trigger,'/exploration/start',self.start_service)
        self.create_service(Trigger,'/exploration/stop',self.stop_service)
        self.load=self.create_client(Trigger,'/fpga/load_map')
        self.start=self.create_client(Execution,'/fpga/start')
        self.cancel=self.create_client(Execution,'/fpga/cancel')
        self.timer=self.create_timer(.05,self.tick)

    @staticmethod
    def now_ms():return time.monotonic_ns()//1000000
    def emit(self,publisher,value):publisher.publish(String(data=json.dumps(value,separators=(',',':'),allow_nan=False)))
    def decode(self,msg,max_bytes=16384):
        if len(msg.data)>max_bytes:raise ValueError('oversized status')
        value=json.loads(msg.data)
        if not isinstance(value,dict):raise ValueError('expected object')
        return value
    def map_cb(self,m):
        try:
            q=m.info.origin.orientation
            if abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.z*q.z+q.w*q.w-1)>.001:raise ValueError('non-planar map')
            g=Grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,tuple(m.data),2*math.atan2(q.z,q.w),m.header.frame_id);g.validate();self.global_map=g;self.observed_cells=sum(v>=0 for v in g.cells)
        except Exception as ex:self.get_logger().warning(str(ex))
    def local_cb(self,m):
        try:self.local=local_from_ros(m)
        except Exception as ex:self.local=None;self.get_logger().warning(str(ex))
    def meta_cb(self,m):
        try:self.meta=self.decode(m)
        except Exception:self.meta={}
    def state_cb(self,m):
        try:
            v=self.decode(m);self.fpga=v;self.state_received=self.now_ms()
            if not v.get('available') or v.get('state_stale') or v.get('stream_fault'):return
            action=self.e.observe(parse_identity(v),v['state'],v['reason'],v['pose_seq'],self.now_ms(),v['pose_age_ms']+v['host_receipt_age_ms'])
            if action=='STOP':self.stop(self.e.reason)
            elif action=='CANCEL_AND_CONFIRM_STOP':self.send_cancel()
        except (KeyError,ValueError,TypeError):return
    def stopped(self):
        v=self.fpga
        return (self.now_ms()-self.state_received<=250 and v.get('available') and not v.get('state_stale')
          and not v.get('stream_fault') and v.get('owner')==0 and not (v.get('flags',1)&1)
          and v.get('state') in ('IDLE','CANCELLED','ARRIVED'))
    def start_service(self,req,res):
        if not self.global_map:res.message='ROS map unavailable';return res
        if self.e.mode not in (Mode.IDLE,Mode.ERROR,Mode.NO_FRONTIER):res.message='already exploring';return res
        if not self.stopped():res.message='fresh stopped FPGA state required';return res
        # Starting exploration authorizes selection only. It does not arm or
        # grant NVA; Gateway separately validates the current phone authority.
        self.e.begin(self.now_ms(),True);self.last_map_request_revision=None;self.action_request=None;self.finish_requested=False
        # Coordinator binds status to this exact exploration generation, rather
        # than interpreting an old transient status as this new mapping task.
        res.success=True;res.message=json.dumps({'operation':'exploration-started','generation':self.e.generation});return res
    def stop_service(self,req,res):
        self.finish_requested=True;self.stop('USER_STOP');res.success=True;res.message='cancel requested; save/load waits for confirmed stop';return res
    def execution_request(self,identity):
        q=Execution.Request();self.next_request+=1;q.request_id=self.next_request
        q.session_id=identity.session;q.run_generation=identity.run;q.goal_id=list(bytes.fromhex(identity.goal))
        q.map_epoch=identity.map_epoch;q.map_revision=identity.revision;q.localization_epoch=identity.localization_epoch
        q.plan_id=identity.plan;q.nav_epoch=identity.nav_epoch;q.created_monotonic_ns=time.monotonic_ns();q.valid_for_ms=500
        return q
    def send_cancel(self,identity=None):
        try:
            identity=identity or parse_identity(self.fpga)
            if not identity.run:return
            if self.dry_run:return
            if not self.cancel.service_is_ready():raise ValueError('CANCEL service unavailable')
            q=self.execution_request(identity);q.reason=4
            self.action_request=('cancel',q.request_id,self.e.generation)
            self.cancel.call_async(q).add_done_callback(lambda f,g=self.e.generation:self.queue_result(f,'cancel',g))
        except Exception as ex:self.e.reason=str(ex)
    def stop(self,reason):
        self.e.stop(reason);self.send_cancel()
        # Gateway receives this and immediately stops NVA renewal independently
        # of whether a ROS CANCEL ACK arrives. Manual MOT retains hard priority.
        self.emit(self.pub_start_request,{'operation':'stop','generation':self.e.generation,'reason':reason})
    def queue_result(self,future,operation="start",generation=None):
        if generation is not None and generation!=self.e.generation:return
        try:
            response=future.result()
            if not response.success:
                if operation=='cancel':self.e.reason='CANCEL_QUEUE_REJECTED:'+response.message
                else:self.stop('CONTROL_QUEUE_REJECTED:'+response.message)
        except Exception as ex:
            if operation=='cancel':self.e.reason='CANCEL_SERVICE_ERROR:'+str(ex)
            else:self.stop('CONTROL_SERVICE_ERROR:'+str(ex))
    def plan_cb(self,m):
        try:
            v=self.decode(m,32768)
            if self.e.phase!='WAIT_PLAN' or self.e.target is None:return
            if (abs(float(v['goal_x'])-self.e.target.x)>1e-6 or abs(float(v['goal_y'])-self.e.target.y)>1e-6
                or v.get('goal_source_stamp_ns')!=self.target_sent_ns):return
            ident=parse_identity(v)
            if ident.revision!=self.meta.get('map_revision') or ident.session!=self.fpga.get('session_id'):return
            if not self.e.planned(self.e.generation,ident,v['status']==0 and v['path_count']>0):
                self.send_cancel(ident);self.last_map_request_revision=None;return
            self.e.reason='WAIT_GATEWAY_NVA_ACK'
            self.emit(self.pub_start_request,{'operation':'authorize_start','generation':self.e.generation,'identity':v,'dry_run':self.dry_run})
        except (KeyError,TypeError,ValueError):return
    def authorized_cb(self,m):
        try:
            v=self.decode(m);ident=parse_identity(v)
            if self.dry_run or self.e.phase!='WAIT_START' or v['generation']!=self.e.generation or ident!=self.e.identity:return
            # Preserve the original Gateway monotonic timestamp, not receipt time.
            produced=int(v['created_monotonic_ns']);ttl=int(v['valid_for_ms']);age=time.monotonic_ns()-produced
            if ttl<=0 or ttl>500 or age<0 or age>=ttl*1000000:raise ValueError('expired authority ticket')
            if not v.get('permit_ack_verified'):raise ValueError('NVA ACK not verified')
            if not self.start.service_is_ready():raise ValueError('START service unavailable')
            q=self.execution_request(ident);q.created_monotonic_ns=produced;q.valid_for_ms=ttl
            q.permit_seq=int(v['permit_seq']);q.auth_epoch=int(v['auth_epoch'])
            self.action_request=('start',q.request_id,self.e.generation);self.start.call_async(q).add_done_callback(lambda f,g=self.e.generation:self.queue_result(f,'start',g))
        except (KeyError,TypeError,ValueError) as ex:self.stop(str(ex))
    def command_cb(self,m):
        try:
            v=self.decode(m)
            if not self.action_request or (v['operation'],v['request_id'],self.e.generation)!=self.action_request:
                self.early_commands.append((self.now_ms(),m));return
            self.action_request=None
            if v['status']!=0 or not v['fpga_ack']:self.stop('FPGA_COMMAND_REJECTED');return
            if v['operation']=='start':self.e.started(self.e.generation,parse_identity(v),self.now_ms(),True)
            elif self.e.mode in (Mode.ARRIVED,Mode.ERROR):self.wait_stop_confirmation=True
        except (KeyError,TypeError,ValueError):return
    def gateway_start_cb(self,m):
        try:
            v=self.decode(m);ident=parse_identity({**v,**{k:int(v[k]) for k in ('session_id','run_generation','plan_id','map_epoch','map_revision','localization_epoch','nav_epoch')}})
            if self.e.phase!='WAIT_START' or ident!=self.e.identity or int(v['generation'])!=self.e.generation:return
            produced=int(v['produced_mono_ns']);ttl=int(v['valid_for_ms']);age=time.monotonic_ns()-produced
            if not 0<ttl<=500 or not 0<=age<ttl*1000000:return
            self.action_request=('start',int(v['request_id']),self.e.generation)
            old=list(self.early_commands);self.early_commands.clear()
            for received,command in old:
                if 0<=self.now_ms()-received<=500:self.command_cb(command)
        except (KeyError,TypeError,ValueError):return
    def tick(self):
        observed=self.observed_cells
        if self.wait_stop_confirmation and self.stopped():
            self.e.stopped(self.e.generation,True);self.last_map_request_revision=None;self.wait_stop_confirmation=False
        if self.e.tick(self.now_ms(),observed)=='STOP':self.stop(self.e.reason)
        if self.finish_requested and self.stopped() and self.action_request is None:
            self.finish_requested=False
            self.emit(self.pub_final,{'operation':'save_and_load_fixed_map','generation':self.e.generation,'stopped_confirmed':True})
        if self.e.mode==Mode.SELECTING and self.e.phase=='WAIT_STOPPED_MAP' and self.global_map and self.stopped():
            if self.last_map_request_revision is None:
                if self.dry_run:self.e.reason='DRY_RUN: waiting for recorded committed local map'
                elif self.load.service_is_ready():
                    self.last_map_request_revision=int(self.meta.get('map_load_generation',0))
                    self.load.call_async(Trigger.Request()).add_done_callback(lambda f,g=self.e.generation:self.load_result(f,g))
            changed=self.dry_run or (self.last_map_request_revision is not None and self.meta.get('map_load_generation',0)>self.last_map_request_revision)
            if changed and self.meta.get('map_load_state')=='FAILED':
                self.e.mode=Mode.ERROR;self.e.reason='MAP_LOAD_FAILED:'+str(self.meta.get('upload_error','FPGA map was not committed'))
            elif changed and self.local and metadata_matches(self.local,self.meta,self.fpga) and self.work is None:
                pose=self.fpga.get('pose_mm_urad');gen=self.e.generation
                if not isinstance(pose,list) or len(pose)!=3 or any(isinstance(x,bool) or not isinstance(x,int) for x in pose):
                    self.e.mode=Mode.ERROR;self.e.reason='POSE_MISSING';return
                self.work=(gen,self.pool.submit(select_frontier,self.global_map,self.local,(pose[0]/1000,pose[1]/1000),tuple(self.e.excluded)))
        if self.work and self.work[1].done():
            gen,future=self.work;self.work=None
            try:
                selection=future.result()
                if self.e.map_ready(gen,selection,self.stopped(),metadata_matches(self.local,self.meta,self.fpga)):
                    if self.dry_run:self.e.reason='DRY_RUN target preview only'
                    else:
                        msg=PoseStamped();msg.header.frame_id=self.global_map.frame;msg.header.stamp=self.get_clock().now().to_msg()
                        self.target_sent_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
                        msg.pose.position.x=selection.target.x;msg.pose.position.y=selection.target.y;msg.pose.orientation.w=1.;self.pub_goal.publish(msg)
            except Exception as ex:self.stop('SELECT_ERROR:'+str(ex))
        phase_key=(self.e.generation,self.e.mode,self.e.phase)
        if phase_key!=self.phase_key:self.phase_key=phase_key;self.phase_started_ms=self.now_ms()
        if self.e.mode==Mode.SELECTING:
            budget=15000 if self.e.phase=='WAIT_STOPPED_MAP' else 5000
            if self.now_ms()-self.phase_started_ms>=budget:self.stop('EXPLORATION_STAGE_TIMEOUT:'+self.e.phase)
        target=None if self.e.target is None else {'x':self.e.target.x,'y':self.e.target.y,'frontier_size':self.e.target.frontier_size}
        value={'state':self.e.mode.value,'phase':self.e.phase,'reason':self.e.reason,'generation':self.e.generation,
            'published_mono_ns':str(time.monotonic_ns()),'strategy':'frontier','target':target,'dry_run':self.dry_run,
            'map_loaded_in_fpga':metadata_matches(self.local,self.meta,self.fpga) if self.local else False}
        key=(value['state'],value['phase'],value['reason'],value['generation'],None if target is None else (target['x'],target['y']))
        if key!=self.last_status_key:self.get_logger().info('EXPLORATION_STATUS '+json.dumps(value,separators=(',',':')));self.last_status_key=key
        self.emit(self.pub_status,value)
    def load_result(self,future,generation):
        if generation!=self.e.generation:return
        try:
            if not future.result().success:self.e.mode=Mode.ERROR;self.e.reason='MAP_LOAD_REJECTED:'+future.result().message
        except Exception as ex:self.stop('MAP_LOAD_SERVICE_ERROR:'+str(ex))
    def destroy_node(self):
        self.pool.shutdown(wait=False,cancel_futures=True);super().destroy_node()

def main():
    rclpy.init();node=FrontierNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
