"""Authenticated camera-only semantic search coordinator.

Semantic search does not build or consume a map.  The exploration node owns
the lidar/odometry random walk; this node only admits a task, supplies fresh
camera evidence, and revokes motion when the executor or stop acknowledgement
is missing.
"""
import hmac
import json
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .camera import frames
from .core import SearchSession


class SemanticSearch(Node):
    def __init__(self):
        super().__init__('acg138_semantic_search')
        self.dry_run = bool(self.declare_parameter('dry_run', True).value)
        token_file = str(self.declare_parameter('token_file', '/run/acg138-semantic/token').value)
        self.token = Path(token_file).read_text().strip()
        if not self.token or '\n' in self.token or '\r' in self.token:
            raise ValueError('invalid_local_pairing_token')

        self.session = SearchSession()
        self.session_lock = threading.RLock()
        self.quit = threading.Event()
        self.lock = threading.Lock()
        self.latest_meta = None
        self.backend = {}
        self.backend_received = 0.0
        self.exploration = {}
        self.exploration_received = 0.0
        self.odom = None
        self.stop_requested_task = ''
        self.logged_state = None
        self.public_status = self.session.status(time.monotonic(), not self.dry_run)

        gate_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gate_pub = self.create_publisher(String, '/semantic/gate', gate_qos)
        self.state_pub = self.create_publisher(String, '/semantic/state', 1)
        self.stop_client = self.create_client(Trigger, '/exploration/stop')
        self.create_subscription(String, '/exploration/state', self.exploration_cb, 1)
        self.create_subscription(String, '/exploration/manual_takeover', self.takeover_cb, 8)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def reply(self, code, value):
                body = json.dumps(value, ensure_ascii=False).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                if self.path != '/task' or not hmac.compare_digest(
                        self.headers.get('Authorization', ''), 'Bearer ' + outer.token):
                    self.reply(403, {'ok': False, 'error': 'not_authorized'})
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 2048 or self.headers.get('Transfer-Encoding'):
                        raise ValueError('invalid_length')
                    self.connection.settimeout(.4)
                    command = json.loads(self.rfile.read(length))
                    if not isinstance(command, dict):
                        raise ValueError('invalid_command')
                    result = outer.admit(command)
                    self.reply(200 if result.get('ok') else 409, result)
                except (ValueError, OSError, TypeError, json.JSONDecodeError) as ex:
                    self.reply(400, {'ok': False, 'error': str(ex) or 'semantic_command_rejected'})

        self.http = ThreadingHTTPServer(('127.0.0.1', 8088), Handler)
        self.http.daemon_threads = True
        threading.Thread(target=self.http.serve_forever, daemon=True).start()

        self.cfg = {'url': 'http://127.0.0.1:8082/api/video.mjpg', 'token_file': token_file}
        threading.Thread(target=self.camera_worker, daemon=True).start()
        threading.Thread(target=self.backend_worker, daemon=True).start()
        self.create_timer(.05, self.tick)

    def admit(self, command):
        """Start synchronously so manual_start cannot race a missing gate."""
        now = time.monotonic()
        with self.session_lock:
            if self.dry_run:
                return {'ok': False, 'error': 'semantic_dry_run'}
            try:
                self.session.start(command['target'], command['task'],
                                   int(command['gateway_generation']),
                                   int(command['produced_mono_ns']),
                                   int(command['valid_for_ms']), now)
            except (ValueError, TypeError, KeyError) as ex:
                return {'ok': False, 'error': str(ex)}
            self.stop_requested_task = ''
            gate = self.session.gate(now)
        self.gate_pub.publish(String(data=json.dumps(gate, separators=(',', ':'))))
        return {'ok': True, 'task': self.session.task}

    def camera_worker(self):
        while not self.quit.is_set():
            try:
                for meta in frames(self.cfg, self.quit):
                    with self.lock:
                        self.latest_meta = meta
            except Exception:
                self.quit.wait(.25)

    def backend_worker(self):
        while not self.quit.is_set():
            try:
                req = urllib.request.Request(
                    'http://127.0.0.1:8082/api/state',
                    headers={'Authorization': 'Bearer ' + self.token})
                with urllib.request.urlopen(req, timeout=.3) as response:
                    raw = response.read(262145)
                if len(raw) > 262144:
                    raise ValueError('state_too_large')
                value = json.loads(raw)
                with self.lock:
                    self.backend, self.backend_received = value, time.monotonic()
            except Exception:
                pass
            self.quit.wait(.1)

    def exploration_cb(self, msg):
        try:
            value = json.loads(msg.data)
            stamp = int(value['published_mono_ns'])
            if not 0 <= time.monotonic_ns() - stamp <= 250000000:
                return
            with self.lock:
                self.exploration, self.exploration_received = value, time.monotonic()
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            pass

    def takeover_cb(self, msg):
        try:
            value = json.loads(msg.data)
            with self.session_lock:
                if int(value['gateway_generation']) >= self.session.generation:
                    self.session.stop('CANCELLED', 'manual_takeover', time.monotonic())
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            pass

    def odom_cb(self, msg):
        self.odom = msg

    def stopped(self, now):
        with self.lock:
            state, received = self.backend, self.backend_received
        motor = state.get('motor', {})
        try:
            ack = (now - received <= .25 and 0 <= motor['ack_age_ms'] <= 250 and
                   abs(motor['ack_left']) <= .001 and abs(motor['ack_right']) <= .001)
            odom = self.odom
            age = (self.get_clock().now().nanoseconds -
                   Time.from_msg(odom.header.stamp).nanoseconds) / 1e9
            velocity = odom.twist.twist
            return (ack and 0 <= age <= .25 and abs(velocity.linear.x) < .02 and
                    abs(velocity.angular.z) < .05)
        except (AttributeError, KeyError, TypeError, ValueError):
            return False

    def request_exploration_stop(self, task):
        if not task or task == self.stop_requested_task:
            return
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
            self.stop_requested_task = task

    def tick(self):
        now = time.monotonic()
        with self.lock:
            meta, self.latest_meta = self.latest_meta, None
            exploration = dict(self.exploration)
            exploration_received = self.exploration_received
        with self.session_lock:
            if meta is not None:
                self.session.observe(meta, now)
            executor_ready = (
                str(exploration.get('gateway_generation')) == str(self.session.generation)
                and now - exploration_received <= .25
                and exploration.get('active') is True
                and exploration.get('semantic_external') is True
            )
            self.session.tick(now, executor_ready, self.stopped(now))
            if self.session.state == 'SEARCHING' and not executor_ready:
                self.session.stop('ERROR', 'exploration_owner_or_heartbeat_lost', now)
            task = self.session.task
            state = self.session.state
            gate = self.session.gate(now)
            # `ready` means the coordinator is alive.  Executor readiness is
            # reported separately because it only becomes true after the
            # Gateway has admitted this task and started the lidar executor.
            status = self.session.status(now, not self.dry_run)
            status['explore_process_running'] = False
            status['executor_ready'] = executor_ready
        if state not in ('IDLE', 'WAIT_EXECUTOR', 'SEARCHING'):
            self.request_exploration_stop(task)
        self.gate_pub.publish(String(data=json.dumps(gate, separators=(',', ':'))))
        self.state_pub.publish(String(data=json.dumps(status, ensure_ascii=False,
                                                      separators=(',', ':'))))
        transition = (status['task'], status['state'], status['reason'])
        if transition != self.logged_state:
            self.get_logger().info('SEMANTIC_STATUS ' + json.dumps(status, ensure_ascii=False))
            self.logged_state = transition
        with self.lock:
            self.public_status = status

    def destroy_node(self):
        self.quit.set()
        self.http.shutdown()
        self.http.server_close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = SemanticSearch()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
