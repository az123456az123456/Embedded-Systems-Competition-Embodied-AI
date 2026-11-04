#!/usr/bin/env python3
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--output', default='/data/logs/ros-map-smoke.json')
    args = parser.parse_args()
    rclpy.init()
    node = Node('map_acceptance_check')
    result = {}
    def receive(msg):
        if msg.info.width < 2 or msg.info.height < 2 or len(msg.data) != msg.info.width * msg.info.height:
            return
        free = sum(v == 0 for v in msg.data)
        occupied = sum(v >= 65 for v in msg.data)
        if free and occupied:
            result.update(result='PASS', frame=msg.header.frame_id, width=msg.info.width, height=msg.info.height,
                          resolution_m=msg.info.resolution, free_cells=free, occupied_cells=occupied,
                          scope='Actual ROS2 OccupancyGrid; simulated scan and pose; not hardware SLAM')
    subscription = node.create_subscription(OccupancyGrid, '/map', receive,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and not result and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    if not result:
        result = {'result': 'FAIL', 'reason': 'no_valid_occupancy_map_before_deadline'}
    with open(args.output, 'w', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(0 if result['result'] == 'PASS' else 1)
if __name__ == '__main__':
    main()
