"""Button-started SLAM; RK frontier exploration; stopped-only FPGA map load."""
import copy,json,math,os,signal,subprocess,time,uuid
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from slam_toolbox.srv import Pause,SerializePoseGraph
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from lifecycle_msgs.srv import GetState,ChangeState
from tf2_ros import Buffer,TransformListener
from acg138_nav_interfaces.srv import Execution
from .mapping_core import MappingSession,save_grid,RequestLedger,slam_parameters,current_tf_pose,mapping_start_ready
from .core import Grid
from .slam_lifecycle import SlamLifecycle

class MappingCoordinator(Node):
    def __init__(self):
        super().__init__('acg138_mapping_coordinator')
        self.dry_run=bool(self.declare_parameter('dry_run',True).value)
        self.interface=str(self.declare_parameter('interface','eth0').value)
        self.root=Path(str(self.declare_parameter('data_directory','/data/maps').value))
        self.root.mkdir(parents=True,exist_ok=True)
        self.runtime=Path('/run/acg138-mapping');self.runtime.mkdir(mode=0o700,exist_ok=True)
        self.session=MappingSession();self.requests=RequestLedger();self.slam=None;self.bridge=None;self.logs=[]
        self.fpga={};self.state_received=0;self.metadata={};self.latest_map=None;self.fixed=None
        self.slam_start_ros_ns=0;self.pose=None;self.pose_stamp=0;self.phase='';self.future=None;self.future_generation=0
        self.map_save_directory=None;self.paused_at=0;self.loaded_request_at=0;self.started_frontier=False
        self.exploration={};self.exploration_received_ns=0;self.exploration_generation=0;self.last_status_key=None
        self.gateway_generation=0;self.slam_mode=None
        self.tf=Buffer();self.tf_listener=TransformListener(self.tf,self)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map=self.create_publisher(OccupancyGrid,'/map',qos)
        self.pub_state=self.create_publisher(String,'/mapping/state',qos)
        # The dynamic worker corrects historical SLAM obstacles with fresh rays.
        # Both UI and exploration consume this same /map; saved maps freeze it.
        self.create_subscription(OccupancyGrid,'/mapping/live_map',self.map_cb,qos)
        self.create_subscription(PoseWithCovarianceStamped,'/pose',self.pose_cb,10)
        self.create_subscription(String,'/fpga/state',self.state_cb,10)
        self.create_subscription(String,'/fpga/map_metadata',self.meta_cb,qos)
        self.create_subscription(String,'/exploration/finalize_request',self.finalize_cb,1)
        self.create_subscription(String,'/exploration/state',self.exploration_cb,1)
        self.create_subscription(String,'/exploration/manual_takeover',self.takeover_cb,8)
        self.create_service(Execution,'/mapping/start',self.start_service)
        self.create_service(Execution,'/mapping/stop',self.stop_service)
        self.frontier_start=self.create_client(Execution,'/exploration/manual_start');self.frontier_stop=self.create_client(Trigger,'/exploration/stop')
        self.pause=self.create_client(Pause,'/slam_toolbox/pause_new_measurements')
        self.parameters=self.create_client(GetParameters,'/slam_toolbox/get_parameters')
        self.serialize=self.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
        self.load=self.create_client(Trigger,'/fpga/load_map')
        self.slam_lifecycle=SlamLifecycle(self.create_client(GetState,'/slam_toolbox/get_state'),
            self.create_client(ChangeState,'/slam_toolbox/change_state'),GetState.Request,ChangeState.Request)
        self.timer=self.create_timer(.05,self.tick)
        # Handshake and state observation only; no map, GOAL, START or NVA here.
        if not self.dry_run:
            self.spawn_bridge()

    @staticmethod
    def ms():return time.monotonic_ns()//1000000
    def admit(self,op,q,r):
        try:
            cached=self.requests.begin(op,q.request_id,q.run_generation,q.created_monotonic_ns,q.valid_for_ms,time.monotonic_ns())
            if cached is not None:r.success,r.status,r.message=cached;return False
            return True
        except ValueError as ex:r.status=4;r.message=str(ex);return False
    def replied(self,q,r):
        self.requests.finish(q.request_id,r.success,r.status,r.message)
        return r
    def state_cb(self,m):
        try:self.fpga=json.loads(m.data);self.state_received=self.ms()
        except (ValueError,TypeError):self.fpga={}
    def takeover_cb(self,m):
        try:
            value=json.loads(m.data)
            if int(value.get('gateway_generation',0))>=self.gateway_generation and self.session.state in ('STARTING','EXPLORING'):
                if self.frontier_stop.service_is_ready():self.frontier_stop.call_async(Trigger.Request())
                self.session.generation+=1;self.session.state='IDLE';self.session.reason='manual_takeover';self.session.deadline_ms=0
                self.phase='';self.future=None;self.started_frontier=False
        except (TypeError,ValueError):pass
    def meta_cb(self,m):
        try:self.metadata=json.loads(m.data)
        except (ValueError,TypeError):self.metadata={}
    def exploration_cb(self,m):
        try:
            if len(m.data)>16384:return
            value=json.loads(m.data);produced=int(value['published_mono_ns']);now=time.monotonic_ns()
            if not 0<=now-produced<=250000000:return
            if not isinstance(value.get('generation'),int) or isinstance(value['generation'],bool):return
            self.exploration=value;self.exploration_received_ns=now
        except (ValueError,TypeError,KeyError):pass
    def map_cb(self,m):
        if m.header.frame_id!='map' or m.info.width*m.info.height>1000000:return
        stamp=Time.from_msg(m.header.stamp).nanoseconds
        if stamp<self.slam_start_ros_ns:return
        self.latest_map=m
        if self.slam_mode=='mapping':self.pub_map.publish(m)
    def pose_cb(self,m):
        stamp=Time.from_msg(m.header.stamp).nanoseconds
        if m.header.frame_id=='map' and stamp>=self.slam_start_ros_ns:self.pose=m;self.pose_stamp=stamp
    def stopped(self):
        s=self.fpga
        return (self.ms()-self.state_received<=250 and s.get('available') and not s.get('state_stale')
                and not s.get('stream_fault') and s.get('owner')==0 and not (s.get('flags',1)&1)
                and s.get('state') in ('IDLE','ARRIVED','CANCELLED'))
    def fresh_tf(self):
        transform=self.tf.lookup_transform('map','base_link',Time())
        age_ns=(self.get_clock().now()-Time.from_msg(transform.header.stamp)).nanoseconds
        p=transform.transform.translation;q=transform.transform.rotation
        self.live_pose=current_tf_pose(p.x,p.y,2*math.atan2(q.z,q.w),age_ns,time.monotonic_ns())
        # /pose is the last scan-matching observation and may stay unchanged
        # while stationary. The composed TF includes current real odometry.
        return age_ns/1000000
    def log_process(self,name,args):
        log=open(self.runtime/(name+'.log'),'ab',buffering=0);self.logs.append(log)
        return subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
            env={**os.environ,'RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','ROS_AUTOMATIC_DISCOVERY_RANGE':'SUBNET'})
    def stop_owned(self,process):
        if process is None:return
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(2)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(1)
    def spawn_bridge(self):
        self.stop_owned(self.bridge);self.fpga={};self.metadata={};self.state_received=0
        self.bridge=self.log_process('map-bridge',['ros2','run','acg138_fpga_map_bridge','fpga_map_bridge','--ros-args',
            '-p','protocol:=nav3','-p','dry_run:=false','-p','enable_execution:=true','-p','enable_plan:=true',
            '-p','exclusive_nav2_owner:=true','-p','map_timeout_s:=0.0','-p','pose_timeout_s:=0.25',
            # User confirmed the displayed arrow equals the physical front.
            # The existing arrow uses TF yaw + pi; NAV3 must use the same axis.
            '-p','navigation_heading_offset_rad:=3.141592653589793',
            '-p','upload_period_s:=0.5','-p','footprint_radius:=0.15','-p','safety_margin:=0.0',
            '-p','body_width:=0.30','-p','body_front:=0.1589','-p','body_rear:=0.1942',
            '-p','self_footprint_exemption:=true',
            '-p','interface:='+self.interface])
    def spawn_slam(self,mode):
        self.slam_mode=mode
        self.stop_owned(self.slam);self.pose=None;self.pose_stamp=0
        self.slam_lifecycle.reset()
        profile=yaml.safe_load((Path(get_package_share_directory('acg138_frontier'))/'config/slam.yaml').read_text())
        if mode=='localization':
            if self.fixed is None or self.pose_at_stop is None:raise ValueError('saved map/pose missing')
        profile=slam_parameters(profile,mode,self.map_save_directory/'graph' if mode=='localization' else None,
                                self.pose_at_stop if mode=='localization' else None)
        parameters=self.runtime/(mode+'.yaml');parameters.write_text(yaml.safe_dump(profile))
        self.slam_start_ros_ns=self.get_clock().now().nanoseconds
        self.slam=self.log_process('slam-'+mode,['ros2','launch','acg138_frontier','slam_session.launch.py',
            'mode:='+mode,'parameters:='+str(parameters)])
    def start_service(self,q,r):
        if not self.admit('start',q,r):return r
        if self.dry_run:r.message='dry_run: selection/SLAM not started';return self.replied(q,r)
        try:
            # Gateway admits only fresh zero-duty manual ACKs; map ownership is
            # irrelevant to RK manual exploration and must not gate its start.
            self.session.start(self.ms(),True);self.fixed=None;self.future=None;self.started_frontier=False
            self.gateway_generation=q.run_generation
            self.phase='SPAWN_MAPPING'
            self.exploration={};self.exploration_generation=0;self.exploration_received_ns=0
            # Acknowledge bounded admission before slow child shutdown/start.
            # The timer owns lifecycle work; this reply grants no NVA or START.
            r.success=True;r.message='frontier selection accepted; RK manual channel; motion not yet confirmed'
        except Exception as ex:self.session.fail(str(ex));r.status=16;r.message=str(ex)
        return self.replied(q,r)
    def stop_service(self,q,r):
        if not self.admit('stop',q,r):return r
        # Phone endMapping sends emergency STOP on an independent lane first.
        # That STOP may already have moved this lifecycle to IDLE. A subsequent
        # explicit mapping/stop still saves the live map; takeover alone never
        # enters saving, and this branch grants no movement authorization.
        live_map=self.slam_mode=='mapping' and self.latest_map is not None and self.slam is not None and self.slam.poll() is None
        if self.session.state=='LOADED' or (self.session.state=='IDLE' and not live_map):
            r.success=True;r.message='no active mapping task';return self.replied(q,r)
        if (self.session.state=='ERROR' and self.fixed is not None
                and self.map_save_directory is not None and self.slam_mode=='localization'):
            # Explicit retry uses the already saved immutable map. Do not ask
            # SLAM to pause a localization node or serialize a second graph.
            self.session.generation+=1;self.session.state='LOCALIZING'
            self.session.expected_session=0;self.session.reason='retry_saved_map_after_stop'
            self.session.deadline_ms=self.ms()+30000
            self.phase='RETRY_BRIDGE';self.future=None
            r.success=True;r.message='retry saved map; still requires fresh stopped FPGA and COMMIT/CRC'
            return self.replied(q,r)
        self.session.stop(self.ms());self.phase='';self.future=None
        if self.frontier_stop.service_is_ready():self.frontier_stop.call_async(Trigger.Request())
        r.success=True;r.message='cancel requested; save/load waits for stopped FPGA';return self.replied(q,r)
    def finalize_cb(self,m):
        # A completion notification cannot resurrect a cancelled mapping generation.
        try:
            v=json.loads(m.data)
            if v.get('operation')=='save_and_load_fixed_map' and self.session.state=='EXPLORING':
                self.session.stop(self.ms());self.phase=''
        except (ValueError,TypeError):pass
    def pending(self,future,phase):self.future=future;self.future_generation=self.session.generation;self.phase=phase
    def future_result(self):
        if self.future is None or not self.future.done():return None
        f=self.future;self.future=None
        if self.future_generation!=self.session.generation:return None
        return f.result()
    def save_after_pause(self):
        if self.latest_map is None:return
        # Pause has been confirmed. A stationary SLAM map need not get a new
        # timestamp after pause; freeze the last actually received map.
        self.fresh_tf();self.fixed=copy.deepcopy(self.latest_map)
        self.pose_at_stop=tuple(self.live_pose['xyyaw'])
        m=self.fixed;q=m.info.origin.orientation
        g=Grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,tuple(m.data),2*math.atan2(q.z,q.w),m.header.frame_id)
        name='site-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]
        self.map_save_directory=save_grid(self.root/name,g)
        if not self.serialize.service_is_ready():raise ValueError('serialize pose graph service unavailable')
        request=SerializePoseGraph.Request();request.filename=str(self.map_save_directory/'graph')
        self.pending(self.serialize.call_async(request),'SERIALIZING')
    def tick(self):
        try:
            if not self.dry_run and self.slam is not None and self.slam.poll() is None:
                self.slam_lifecycle.tick(time.monotonic())
            self.session.tick(self.ms());s=self.session.state
            if s=='STARTING' and self.phase=='SPAWN_MAPPING':
                self.latest_map=None;self.spawn_slam('mapping');self.phase=''
            if self.slam and self.slam.poll() is not None and s not in ('IDLE','ERROR'):raise ValueError('owned SLAM process exited')
            if self.bridge and self.bridge.poll() is not None and s!='ERROR':raise ValueError('owned FPGA bridge process exited')
            if s=='STARTING' and self.slam_lifecycle.ready and self.latest_map is not None:
                if self.session.ready(self.session.generation,self.ms(),True,self.fresh_tf()):self.phase='START_FRONTIER'
            if self.session.state=='EXPLORING' and self.phase=='START_FRONTIER' and self.frontier_start.service_is_ready():
                request=Execution.Request();request.request_id=time.monotonic_ns();request.run_generation=self.gateway_generation
                request.created_monotonic_ns=time.monotonic_ns();request.valid_for_ms=500
                self.pending(self.frontier_start.call_async(request),'WAIT_FRONTIER')
            elif s=='EXPLORING' and self.phase=='WAIT_FRONTIER':
                result=self.future_result()
                if result is not None:
                    if not result.success:raise ValueError('frontier start rejected: '+result.message)
                    receipt=json.loads(result.message)
                    if receipt.get('operation')!='exploration-started' or type(receipt.get('generation')) is not int or receipt['generation']<=0:
                        raise ValueError('frontier generation acknowledgement missing')
                    self.exploration_generation=receipt['generation']
                    self.started_frontier=True;self.phase='RUNNING'
            elif s=='STOPPING' and self.stopped():
                if self.latest_map is None:
                    self.stop_owned(self.slam);self.slam=None;self.session.state='IDLE';self.session.reason='cancelled_before_a_map_was_available';self.session.deadline_ms=0
                elif self.session.stopped(self.session.generation,self.ms(),True):self.phase='PAUSE'
            elif s=='SAVING':
                if self.phase=='PAUSE' and self.pause.service_is_ready():self.pending(self.pause.call_async(Pause.Request()),'WAIT_PAUSE')
                elif self.phase=='WAIT_PAUSE':
                    result=self.future_result()
                    if result is not None:
                        if not result.status:raise ValueError('SLAM pause call failed')
                        request=GetParameters.Request();request.names=['paused_new_measurements'];self.pending(self.parameters.call_async(request),'CHECK_PAUSED')
                elif self.phase=='CHECK_PAUSED':
                    result=self.future_result()
                    if result is not None:
                        if len(result.values)!=1 or result.values[0].type!=ParameterType.PARAMETER_BOOL or not result.values[0].bool_value:raise ValueError('SLAM is not confirmed paused')
                        self.paused_at=self.get_clock().now().nanoseconds;self.phase='WAIT_FINAL_MAP'
                elif self.phase=='WAIT_FINAL_MAP':self.save_after_pause()
                elif self.phase=='SERIALIZING':
                    result=self.future_result()
                    if result is not None:
                        if result.result!=0:raise ValueError('pose graph serialization failed')
                        for ext in ('.posegraph','.data'):
                            if not Path(str(self.map_save_directory/'graph')+ext).is_file() or Path(str(self.map_save_directory/'graph')+ext).stat().st_size==0:raise ValueError('saved pose graph missing/empty')
                        self.fixed.header.stamp=self.get_clock().now().to_msg();stamp=Time.from_msg(self.fixed.header.stamp).nanoseconds
                        self.session.saved(self.session.generation,self.ms(),stamp,True,True)
                        self.spawn_slam('localization');self.spawn_bridge();self.phase='WAIT_LOCALIZATION'
            elif s=='LOCALIZING':
                self.pub_map.publish(self.fixed)
                if self.phase=='RETRY_BRIDGE':
                    self.spawn_bridge();self.phase='WAIT_LOCALIZATION'
                sid=self.fpga.get('session_id',0)
                if not self.metadata.get('handshake') or self.metadata.get('session_id')!=sid:sid=0
                if self.session.localized(self.session.generation,self.ms(),sid,self.fresh_tf(),self.stopped()):self.phase='REQUEST_LOAD'
            elif s=='LOADING':
                self.pub_map.publish(self.fixed)
                if self.phase=='REQUEST_LOAD' and self.load.service_is_ready():self.pending(self.load.call_async(Trigger.Request()),'WAIT_LOAD_QUEUE')
                elif self.phase=='WAIT_LOAD_QUEUE':
                    result=self.future_result()
                    if result is not None:
                        if not result.success:raise ValueError('fixed map load rejected: '+result.message)
                        self.phase='WAIT_COMMIT'
                elif self.phase=='WAIT_COMMIT':
                    if (self.metadata.get('session_id')==self.session.expected_session
                            and self.metadata.get('source_map_stamp_ns')==self.session.fixed_stamp_ns
                            and self.metadata.get('map_load_state')=='FAILED'):
                        raise ValueError('FPGA_map_load_failed: '+str(self.metadata.get('upload_error') or 'map transaction failed'))
                    if self.session.committed(self.session.generation,self.metadata,self.fpga):
                        (self.root/'current-map.json.tmp').write_text(json.dumps({'directory':str(self.map_save_directory),'session':self.session.loaded_session,'revision':self.session.loaded_revision,'crc32':self.session.loaded_crc32,'pose':list(self.pose_at_stop)}))
                        os.replace(self.root/'current-map.json.tmp',self.root/'current-map.json');self.phase='DONE'
            elif s=='LOADED' or (s=='ERROR' and self.fixed is not None and self.slam_mode=='localization'):
                self.pub_map.publish(self.fixed)
            # Keep logs bounded without changing open file identities.
            for f in self.logs:
                if os.fstat(f.fileno()).st_size>4*1024*1024:os.ftruncate(f.fileno(),0)
        except Exception as ex:
            # Missing TF is a wait, never a zero pose or an implicit success.
            if 'fresh actual SLAM' in str(ex) or type(ex).__name__ in ('LookupException','ConnectivityException','ExtrapolationException'):pass
            else:self.session.fail(str(ex))
        live_pose=None
        try:self.fresh_tf();live_pose=self.live_pose
        except Exception:pass
        # The UI must not say 'exploring' merely because SLAM is running.
        # Project only current-generation, original-source-aged frontier status.
        display_state=self.session.state;display_reason=self.session.reason
        if self.session.state=='IDLE' and self.slam is None:
            display_reason='press_mapping_start'
        elif self.session.state in ('IDLE','STARTING') and not self.slam_lifecycle.ready:
            display_reason=self.slam_lifecycle.reason
        elif self.session.state in ('IDLE','STARTING') and self.latest_map is None:
            display_reason='waiting_for_SLAM_map'
        elif self.session.state=='STARTING' and live_pose is None:
            display_reason='waiting_for_fresh_map_to_base_TF'
        if self.session.state=='EXPLORING' and self.started_frontier:
            now_ns=time.monotonic_ns();v=self.exploration;generation=v.get('generation',0)
            fresh=0<=now_ns-self.exploration_received_ns<=250000000 and 0<=now_ns-int(v.get('published_mono_ns',0))<=250000000
            if fresh and generation==self.exploration_generation:
                if v.get('state')=='ERROR':
                    self.session.fail('frontier:'+str(v.get('reason','exploration failed')))
                    display_state='ERROR';display_reason=self.session.reason
                elif v.get('state') in ('SELECTING','NAVIGATING','ARRIVED','NO_FRONTIER','WAITING','IDLE'):
                    display_state=v['state'];display_reason=str(v.get('reason',''))
            elif fresh and generation==self.exploration_generation+1 and v.get('state')=='IDLE':
                self.session.fail('frontier stopped:'+str(v.get('reason','unknown')))
                display_state='ERROR';display_reason=self.session.reason
        value={'state':display_state,'lifecycle_state':self.session.state,'phase':self.phase,'reason':display_reason,'live_pose':live_pose,
            'generation':self.session.generation,'gateway_generation':str(self.gateway_generation),'published_mono_ns':time.monotonic_ns(),
            # Admission readiness must not require the map which this button creates.
            # Execution remains gated separately by fresh map/TF in STARTING.
            'ready':mapping_start_ready(self.dry_run,self.frontier_start.service_is_ready(),self.bridge is not None and self.bridge.poll() is None,self.session.state),
            'slam_ready':self.slam is not None and self.slam.poll() is None and self.slam_lifecycle.ready and self.latest_map is not None and live_pose is not None,
            'active':self.session.state in ('STARTING','EXPLORING','STOPPING','SAVING','LOCALIZING','LOADING'),
            'slam_lifecycle_state':self.slam_lifecycle.state,'slam_lifecycle_reason':self.slam_lifecycle.reason,'slam_lifecycle_timeouts':self.slam_lifecycle.timeouts,'map_received':self.latest_map is not None,
            'slam_session_start_ns':str(self.slam_start_ros_ns),'slam_mode':self.slam_mode,
            'map_commit_confirmed':self.session.state=='LOADED','loaded_map_session':str(self.session.loaded_session),
            'saved_map_available':self.fixed is not None and self.map_save_directory is not None,
            'loaded_map_revision':self.session.loaded_revision,'loaded_crc32':self.session.loaded_crc32,'dry_run':self.dry_run,'sends_velocity':self.session.state=='EXPLORING','execution_source':'RK_manual_channel',
            'exploration':self.exploration,'strategy':'frontier'}
        key=(display_state,self.phase,display_reason,self.session.generation)
        if key!=self.last_status_key:self.get_logger().info('MAPPING_STATUS '+json.dumps(value,separators=(',',':')));self.last_status_key=key
        self.pub_state.publish(String(data=json.dumps(value,separators=(',',':'))))
    def destroy_node(self):
        self.stop_owned(self.slam);self.stop_owned(self.bridge)
        for f in self.logs:f.close()
        return super().destroy_node()

def main():
    rclpy.init();node=MappingCoordinator()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
