"""Regression for rclpy Node.executor collision; isolated domain, no hardware nodes."""
import json
import os
import time
os.environ['ROS_DOMAIN_ID'] = '137'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from acg138_frontier.mapping_explorer import MappingExplorer


def main():
    rclpy.init()
    explorer, probe = MappingExplorer(), Node('explorer_idle_regression_probe')
    samples = []
    probe.create_subscription(String, '/exploration/state', lambda m: samples.append(json.loads(m.data)), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(explorer)
    executor.add_node(probe)
    try:
        deadline = time.monotonic()+5.
        while len(samples) < 3 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert len(samples) >= 3
        for value in samples:
            assert value['state'] == 'IDLE' and value['active'] is False
            assert value['drive']['motion_requested'] is False
            assert all(value['drive'][k] == 0 for k in ('throttle', 'steer', 'speed'))
        print('EXPLORER_IDLE_CHECK '+json.dumps({'passed': True, 'samples': len(samples),
              'domain': 137, 'state': samples[-1]['state'], 'motion_requested': False}), flush=True)
    finally:
        executor.remove_node(explorer)
        executor.remove_node(probe)
        explorer.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
