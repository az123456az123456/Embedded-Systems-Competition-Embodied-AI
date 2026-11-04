"""Independent live occupancy worker: never starts mapping or sends motion."""
import copy,json,math,time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from .core import Grid
from .dynamic_map import DynamicMap
from .scan_pair import ClearingScanCache


class DynamicMapNode(Node):
    def __init__(self):
        super().__init__('acg138_dynamic_map')
        self.layer=DynamicMap(int(self.declare_parameter('clear_confirmations',3).value),
                              float(self.declare_parameter('raytrace_range',3.).value))
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.session=0;self.mode=None;self.raw=None;self.pending_map=None;self.scan=None
        self.clearing=ClearingScanCache();self.scan_received_mono=0.
        self.reference=None;self.reference_stamp=0;self.dirty=False;self.publish_at=0.;self.clock=0
        self.status='waiting_mapping_session';self.process_ms=0.;self.last_process_ms=0.
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(OccupancyGrid,'/mapping/live_map',qos)
        self.diag=self.create_publisher(String,'/mapping/dynamic_map_state',qos)
        self.create_subscription(OccupancyGrid,'/slam/map',self.map_cb,qos)
        self.create_subscription(LaserScan,'/scan_mapping',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan_clearing',self.clearing.put,qos_profile_sensor_data)
        self.create_subscription(String,'/mapping/state',self.session_cb,qos)
        self.timer=self.create_timer(.02,self.tick)

    @staticmethod
    def pose(t):
        p=t.transform.translation;q=t.transform.rotation
        return (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def session_cb(self,m):
        try:
            v=json.loads(m.data);session=int(v['slam_session_start_ns']);mode=v['slam_mode']
            if (session,mode)==(self.session,self.mode):return
            self.session=session;self.mode=mode;self.raw=None;self.scan=None;self.reference=None;self.reference_stamp=0
            self.clearing.clear()
            self.layer.grid=None;self.layer.reset('mapping_session_changed');self.dirty=False;self.status='waiting_session_map'
            if self.pending_map is not None:self.map_cb(self.pending_map)
        except (KeyError,ValueError,TypeError):return

    def map_cb(self,m):
        self.pending_map=m
        if self.mode!='mapping' or self.session<=0 or Time.from_msg(m.header.stamp).nanoseconds<self.session:return
        q=m.info.origin.orientation
        try:
            g=Grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,
                   tuple(m.data),2*math.atan2(q.z,q.w),m.header.frame_id)
            before=self.layer.resets
            self.layer.bind(g);self.raw=m;self.dirty=True
            if self.layer.resets>before:
                self.get_logger().info('MAP_HISTORY_RESET reason='+self.layer.last_reset_reason)
        except ValueError as ex:
            self.status=str(ex);self.get_logger().warning('MAP_GEOMETRY_REJECT '+str(ex))

    def scan_cb(self,m):
        self.scan=m;self.scan_received_mono=time.monotonic()

    def tick(self):
        now=self.get_clock().now().nanoseconds
        if self.clock and now<self.clock:
            self.layer.reset('source_clock_reversal');self.scan=None;self.reference=None;self.reference_stamp=0;self.dirty=self.raw is not None
            self.clearing.clear()
        self.clock=now
        if self.mode!='mapping' or self.raw is None:return
        m=self.scan
        if m is not None:
            stamp=Time.from_msg(m.header.stamp);source=stamp.nanoseconds
            if not 0<=now-source<=250000000 or source<self.session:
                self.scan=None;self.status='stale_or_future_scan'
            else:
                try:
                    # Exact acquisition timestamp, not latest TF. The C1 driver
                    # has time_increment=0: no per-ray deskew is claimed.
                    laser=self.pose(self.tf.lookup_transform('map',m.header.frame_id,stamp))
                    ref=self.pose(self.tf.lookup_transform('map','odom',stamp))
                    if not all(math.isfinite(v) for v in (*laser,*ref)):raise ValueError('nonfinite_TF')
                except Exception:
                    self.status='waiting_scan_timestamp_TF'
                else:
                    # Both chains retain original stamps. Bounded 50ms pairing
                    # wait, within the unchanged 250ms source-age check. Missing
                    # far evidence falls back to the original near-only path.
                    far=self.clearing.take(m)
                    if far is None and time.monotonic()-self.scan_received_mono<.05:return
                    self.scan=None
                    # Old scans must not reset the reference or get a new age.
                    if source<=self.reference_stamp:
                        self.status='duplicate_or_out_of_order_scan'
                    else:
                        if self.reference is not None:
                            dx,dy=ref[0]-self.reference[0],ref[1]-self.reference[1]
                            da=math.atan2(math.sin(ref[2]-self.reference[2]),math.cos(ref[2]-self.reference[2]))
                            if math.hypot(dx,dy)>.10 or abs(da)>.0523598776:
                                self.layer.reference_correction();self.dirty=True;self.reference=ref
                                d=self.layer.history.diagnostics()
                                self.get_logger().info('MAP_HISTORY_RETAIN reason=map_odom_correction dx=%.4f dy=%.4f yaw=%.4f stable_free=%d stable_occupied=%d'%(dx,dy,da,d['stable_free_cells'],d['stable_occupied_cells']))
                        if self.reference is None:self.reference=ref
                        self.reference_stamp=source
                        started=time.monotonic()
                        ok=self.layer.observe(source,now,laser,m.ranges,m.angle_min,m.angle_increment,m.range_min,m.range_max,clearing_ranges=far)
                        self.last_process_ms=(time.monotonic()-started)*1000
                        self.process_ms=max(self.process_ms,self.last_process_ms)
                        self.status='live_raycast_updates' if ok else self.layer.last_rejection
                        if ok:self.dirty=True
        mono=time.monotonic()
        if self.dirty and mono>=self.publish_at:
            result=copy.deepcopy(self.raw);result.data=self.layer.cells()
            # The stable lattice may differ from SLAM's fractional bounding box.
            # Publish matching metadata and cells as one occupancy-grid message.
            g=self.layer.grid
            result.info.width=g.width;result.info.height=g.height;result.info.resolution=g.resolution
            result.info.origin.position.x=g.origin_x;result.info.origin.position.y=g.origin_y
            result.info.origin.orientation.x=0.;result.info.origin.orientation.y=0.
            result.info.origin.orientation.z=math.sin(g.yaw/2);result.info.origin.orientation.w=math.cos(g.yaw/2)
            # Original evidence time, never re-age an old map/scan as fresh.
            source=max(Time.from_msg(self.raw.header.stamp).nanoseconds,self.layer.last_stamp)
            result.header.stamp=Time(nanoseconds=source).to_msg()
            self.pub.publish(result);self.dirty=False;self.publish_at=mono+.2
            d=self.layer.diagnostics();d.update(status=self.status,session_start_ns=self.session,
                last_process_ms=self.last_process_ms,max_process_ms=self.process_ms,motion_commands_sent=0)
            self.diag.publish(String(data=json.dumps(d,separators=(',',':'))))

def main():
    rclpy.init();node=DynamicMapNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
