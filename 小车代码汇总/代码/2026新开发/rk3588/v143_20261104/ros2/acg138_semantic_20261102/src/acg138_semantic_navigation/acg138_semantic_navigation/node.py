"""ROS2 adapter. The FPGA gateway continues to own execution authorization."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import socket
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from .core import (Grid, Pose, Rejected, direction_answer, presence_answer,
                   short_goal, target_from_instruction, wrap)
from .camera import backend_image

IPC = Path('/run/acg138-ros-bridge/semantic')


def infer(image, prompt, left_eye=False):
    request = {'id': uuid.uuid4().hex, 'image': str(image), 'prompt': prompt, 'left_eye': left_eye}
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(15)
        client.connect(str(IPC/'model.sock'))
        client.sendall(json.dumps(request).encode()+b'\n')
        answer = bytearray()
        while b'\n' not in answer and len(answer) <= 65536:
            block = client.recv(4096)
            if not block:
                break
            answer.extend(block)
    if not answer.endswith(b'\n') or len(answer) > 65536:
        raise Rejected('INVALID_MODEL_REPLY')
    result = json.loads(answer)
    if 'error' in result:
        raise Rejected(result['error'])
    if result.get('id') != request['id']:
        raise Rejected('MODEL_REPLY_ID_MISMATCH')
    return result


class SemanticNode(Node):
    def __init__(self):
        super().__init__('semantic_navigation')
        config = self.declare_parameter('config', '/opt/acg138_semantic_20261102/config.json').value
        self.cfg = json.loads(Path(config).read_text())
        self.publish_enabled = self.declare_parameter('publish_goals', False).value
        self.fixture_mode = self.declare_parameter('fixture_mode', False).value
        self.grid = Grid(self.cfg['map_yaml'])
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.state, self.state_at, self.meta, self.meta_at = {}, 0., {}, 0.
        self.map_matches = False
        self.frame = None
        self.result_pub = self.create_publisher(String, '/semantic/result', 10)
        self.preview_pub = self.create_publisher(PoseStamped, '/semantic/goal_preview', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/fpga/goal', 1)
        self.create_subscription(String, '/semantic/command', self.command, 1)
        self.create_subscription(String, '/fpga/state', self.state_received, 1)
        self.create_subscription(String, '/fpga/map_metadata', self.meta_received, 1)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, map_qos)
        image_topic = self.declare_parameter('image_topic', '/camera/left/image_raw').value
        self.create_subscription(Image, image_topic, self.image_received, qos_profile_sensor_data)
        self.create_timer(.05, self.complete)
        self.report({'status': 'READY', 'backend': 'SmolVLM2-500M', 'input_size': 384,
                     'publish_goals': self.publish_enabled, 'fixture_mode': self.fixture_mode})

    def report(self, result):
        result['sends_start'] = False
        result['fixture_mode'] = self.fixture_mode
        self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        self.get_logger().info(json.dumps(result, ensure_ascii=False))

    def state_received(self, message):
        try:
            self.state = json.loads(message.data)
            self.state_at = time.monotonic()
        except (ValueError, TypeError):
            self.state = {}

    def meta_received(self, message):
        try:
            self.meta = json.loads(message.data)
            self.meta_at = time.monotonic()
        except (ValueError, TypeError):
            self.meta = {}

    def map_received(self, message):
        q = message.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.map_matches = (
            message.header.frame_id == self.cfg.get('map_frame', 'map') and
            message.info.width == self.grid.width and message.info.height == self.grid.height and
            abs(message.info.resolution-self.grid.resolution) < 1e-6 and
            abs(message.info.origin.position.x-self.grid.origin[0]) < 1e-6 and
            abs(message.info.origin.position.y-self.grid.origin[1]) < 1e-6 and
            abs(wrap(yaw-self.grid.origin[2])) < 1e-6 and list(message.data) == self.grid.cells)

    def image_received(self, message):
        age = (self.get_clock().now().nanoseconds-Time.from_msg(message.header.stamp).nanoseconds)/1e9
        if not 0 <= age <= 1. or not 0 < message.width*message.height <= 4096*4096:
            return
        if message.encoding not in ('rgb8', 'bgr8') or message.step < message.width*3:
            return
        if len(message.data) != message.height*message.step:
            return
        self.frame = (message, time.monotonic())

    def pose(self):
        transform = self.tf.lookup_transform(self.cfg.get('map_frame', 'map'), 'base_link', Time())
        age = (self.get_clock().now().nanoseconds-Time.from_msg(transform.header.stamp).nanoseconds)/1e9
        if not 0 <= age <= .25:
            raise Rejected('STALE_LOCALIZATION')
        t, q = transform.transform.translation, transform.transform.rotation
        return Pose(t.x, t.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def identity(self):
        return tuple(self.state.get(k) for k in ('session_id', 'run_generation', 'map_epoch',
                                                 'map_revision', 'localization_epoch'))

    def publication_check(self):
        if self.fixture_mode:
            raise Rejected('FIXTURE_MODE_NEVER_PUBLISHES_FPGA_GOALS')
        if self.cfg.get('validated_for_navigation') is not True:
            raise Rejected('MODEL_NOT_VALIDATED_FOR_NAVIGATION')
        if not self.map_matches:
            raise Rejected('LIVE_MAP_DIFFERS_FROM_SURVEYED_MAP')
        s, m, now = self.state, self.meta, time.monotonic()
        if (now-self.state_at > .25 or not s.get('available') or s.get('state_stale', True)
                or s.get('stream_fault', True) or s.get('state') not in ('IDLE', 'ARRIVED', 'CANCELLED')
                or s.get('owner') != 0 or s.get('flags', 1) & 1
                or s.get('pose_age_ms', 9999) > 250 or s.get('permit_remaining_ms', 1) != 0):
            raise Rejected('FPGA_NOT_FRESH_AND_IDLE')
        if (now-self.meta_at > 1. or m.get('map_load_state') != 'FROZEN'
                or not m.get('state_crc_verified') or m.get('session_id') != s.get('session_id')
                or m.get('map_revision') != s.get('map_revision')):
            raise Rejected('FPGA_MAP_NOT_VERIFIED_AND_FROZEN')

    def command(self, message):
        if self.future is not None:
            self.report({'status': 'BUSY', 'goal_published': False})
            return
        try:
            data = json.loads(message.data) if message.data.lstrip().startswith('{') else {'instruction': message.data}
            if not isinstance(data, dict) or set(data)-{'instruction', 'image', 'pose'}:
                raise Rejected('INVALID_COMMAND_FIELDS')
            target = target_from_instruction(data['instruction'])
            if self.publish_enabled:
                self.publication_check()
            if self.fixture_mode:
                image = Path(data['image']).resolve()
                if not image.is_relative_to(IPC.resolve()) or not image.is_file():
                    raise Rejected('FIXTURE_OUTSIDE_SHARED_IPC')
                pose = Pose(*data['pose']) if 'pose' in data else None
                left_eye = False
            else:
                if 'image' in data or 'pose' in data:
                    raise Rejected('FIXTURE_FIELDS_NOT_ALLOWED_IN_LIVE_MODE')
                try:
                    pose = self.pose()
                except Exception:
                    if self.publish_enabled:
                        raise
                    pose = None
                image = None
                left_eye = self.cfg.get('http_stereo_left_eye', True)
            frame = self.frame
            self.job = (target, pose, self.identity(), time.monotonic())
            self.future = self.pool.submit(self.analyze, target, image, frame, left_eye)
            self.report({'status': 'INFERENCING', 'target': target, 'goal_published': False})
        except Exception as error:
            self.report({'status': 'REJECTED', 'reason': str(error), 'goal_published': False})

    def analyze(self, target, image, frame, left_eye):
        if image is None:
            image = IPC/'frame.jpg'
            if frame and time.monotonic()-frame[1] <= .5:
                message = frame[0]
                pixels = b''.join(bytes(message.data[r*message.step:r*message.step+message.width*3])
                                  for r in range(message.height))
                if message.encoding == 'bgr8':
                    rgb = bytearray(pixels)
                    rgb[0::3], rgb[2::3] = pixels[2::3], pixels[0::3]
                    pixels = bytes(rgb)
                image = IPC/'frame.ppm'
                payload = f'P6\n{message.width} {message.height}\n255\n'.encode()+pixels
                left_eye = False  # Image topic is explicitly one eye.
            else:
                payload = backend_image(self.cfg)
            temp = image.with_suffix('.new')
            temp.write_bytes(payload)
            temp.replace(image)
        present = infer(image, f'Is a {target} visible in this image? Answer only YES or NO.', left_eye)
        result = {'presence': present, 'goal_published': False}
        try:
            presence_answer(present['text'])
            located = infer(image, f'Locate the {target} in the image. If it is absent answer NOT_FOUND. '
                            'If visible answer exactly LEFT, CENTER, or RIGHT. Do not guess.', left_eye)
            result['location'] = located
            result['direction'] = direction_answer(located['text'])
        except Rejected as error:
            result['reason'] = str(error)
        return result

    def complete(self):
        if self.future is None or not self.future.done():
            return
        future, self.future = self.future, None
        target, observed_pose, identity, started = self.job
        result = {'target': target, 'goal_published': False}
        try:
            result.update(future.result())
            if 'reason' in result:
                raise Rejected(result['reason'])
            if observed_pose is None:
                raise Rejected('NEEDS_MAP_TO_CAMERA_POSE')
            if (self.cfg.get('camera', {}).get('calibrated') and
                    result['presence'].get('source_width_px') != self.cfg['camera']['width_px']):
                raise Rejected('IMAGE_WIDTH_DIFFERS_FROM_CALIBRATION')
            goal = short_goal(target, result['direction'], observed_pose, self.cfg, self.grid)
            result['goal'] = goal
            message = PoseStamped()
            message.header.frame_id = self.cfg.get('map_frame', 'map')
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x, message.pose.position.y = goal['x'], goal['y']
            message.pose.orientation.z, message.pose.orientation.w = math.sin(goal['yaw']/2), math.cos(goal['yaw']/2)
            self.preview_pub.publish(message)
            if self.publish_enabled:
                self.publication_check()
                current = self.pose()
                if (identity != self.identity() or time.monotonic()-started > 10.
                        or math.hypot(current.x-observed_pose.x, current.y-observed_pose.y) > .05
                        or abs(wrap(current.yaw-observed_pose.yaw)) > .05):
                    raise Rejected('SCENE_OR_NAVIGATION_SESSION_CHANGED')
                self.goal_pub.publish(message)
                result['goal_published'] = True
            result['status'] = 'GOAL_SUBMITTED_FOR_PLAN' if result['goal_published'] else 'PREVIEW'
        except Exception as error:
            result.update(status='REJECTED', reason=str(error))
        result['elapsed_ms'] = (time.monotonic()-started)*1000
        self.report(result)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
