#!/usr/bin/env python3
"""Explicit headless AMCL initial pose; never publishes vehicle commands."""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True, help='Map x in metres')
    parser.add_argument('--y', type=float, required=True, help='Map y in metres')
    parser.add_argument('--yaw-deg', type=float, required=True, help='Counterclockwise map heading')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.yaw_deg)):
        parser.error('Pose must contain finite numbers')
    rclpy.init(args=[])
    node = Node('acg138_initial_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    try:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            raise RuntimeError('No /initialpose subscriber: activate AMCL first')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = args.x
        msg.pose.pose.position.y = args.y
        angle = math.radians(args.yaw_deg)
        msg.pose.pose.orientation.z = math.sin(angle / 2.0)
        msg.pose.pose.orientation.w = math.cos(angle / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = math.radians(10.0) ** 2
        for _ in range(3):
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.15)
        print('Initial pose sent; confirm map -> base_link before planning.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
