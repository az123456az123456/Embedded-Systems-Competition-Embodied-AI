"""Real board VLM + ROS2 test. Domain 137 has no FPGA bridge or motor node."""
import copy
import json
import math
import os
from pathlib import Path
import time

os.environ['ROS_DOMAIN_ID'] = '137'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from acg138_semantic_navigation.node import SemanticNode
from acg138_semantic_navigation.core import Grid, Pose, short_goal


def main():
    rclpy.init(args=['--ros-args', '-p', 'fixture_mode:=true'])
    semantic = SemanticNode()
    probe = Node('semantic_smoke_probe')
    results, goals = [], []
    probe.create_subscription(String, '/semantic/result', lambda m: results.append(json.loads(m.data)), 10)
    probe.create_subscription(PoseStamped, '/fpga/goal', lambda m: goals.append(m), 10)
    command = probe.create_publisher(String, '/semantic/command', 1)
    publisher = probe.create_publisher(PoseStamped, '/fpga/goal', 1)
    executor = SingleThreadedExecutor()
    executor.add_node(semantic)
    executor.add_node(probe)

    def until(condition, timeout=20):
        deadline = time.monotonic()+timeout
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert condition(), 'ROS2 test deadline expired'

    try:
        until(lambda: command.get_subscription_count() > 0 and publisher.get_subscription_count() > 0)
        for instruction, image, expected in [
            ('找到红色瓶子并靠近', 'camera-fpn-left.png', 'TARGET_NOT_FOUND'),
            ('person', 'ppe-h002.png', 'NEEDS_CAMERA_CALIBRATION')]:
            before = len(results)
            command.publish(String(data=json.dumps({'instruction': instruction,
                'image': '/run/acg138-ros-bridge/semantic/'+image, 'pose': [0., 0., 0.]})))
            until(lambda: any(r.get('status') == 'REJECTED' for r in results[before:]))
            final = next(r for r in reversed(results[before:]) if r.get('status') == 'REJECTED')
            assert final['reason'] == expected, final
            assert not goals and not final['goal_published'], 'Unexpected real goal publication'
        names = {name for name, namespace in probe.get_node_names_and_namespaces()}
        assert names <= {'semantic_navigation', 'semantic_smoke_probe'}, names

        # Transport/geometry test uses an explicitly synthetic map and surveyed fixture.
        # It does NOT claim that the real photographs have map poses or surveyed targets.
        grid = copy.deepcopy(semantic.grid)
        grid.width = grid.height = 120
        grid.resolution, grid.origin = .05, (-3., -3., 0.)
        grid.cells, grid.sha256 = [0]*14400, 'SYNTHETIC_TEST_MAP'
        cfg = {'map_sha256': grid.sha256, 'camera': {'calibrated': True,
            'calibration_source': 'SYNTHETIC_TEST_ONLY', 'fx_px': 400, 'cx_px': 480,
            'width_px': 960, 'base_x_m': 0., 'base_y_m': 0., 'base_yaw_rad': 0.},
            'landmarks': [{'id': 'SYNTHETIC_TEST_TARGET', 'targets': ['red bottle'],
                           'surveyed': True, 'x_m': 2., 'y_m': 0.}]}
        goal = short_goal('red bottle', 'CENTER', Pose(0., 0., 0.), cfg, grid)
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = probe.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = goal['x'], goal['y']
        message.pose.orientation.w = 1.
        publisher.publish(message)
        until(lambda: len(goals) == 1)
        assert math.isclose(goals[0].pose.position.x, .75)
        print('ROS2_SMOKE_RESULT '+json.dumps({'passed': True, 'domain': 137,
            'real_vlm_results': [r for r in results if r.get('status') == 'REJECTED'],
            'synthetic_transport_goal': goal, 'hardware_nodes': [],
            'real_navigation_goals': 0, 'sends_start': False}), flush=True)
    finally:
        semantic.pool.shutdown(wait=True, cancel_futures=True)
        semantic.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
