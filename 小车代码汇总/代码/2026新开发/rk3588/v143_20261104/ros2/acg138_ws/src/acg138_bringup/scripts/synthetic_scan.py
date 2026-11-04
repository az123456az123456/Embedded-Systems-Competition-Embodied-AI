#!/usr/bin/env python3
"""Deterministic headless SLAM fixture. Simulated ranges/odom, no hardware or motor IO."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Fixture(Node):
    def __init__(self):
        super().__init__('synthetic_c1_fixture')
        self.scan = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.start = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        now = self.get_clock().now()
        t = max(0., (now - self.start).nanoseconds * 1e-9 - 2.)
        x, y, yaw = .9 * math.sin(t / 18.), .5 * (1. - math.cos(t / 18.)), .3 * math.sin(t / 25.)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2.), math.cos(yaw / 2.)
        self.odom.publish(odom)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = odom.header, 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = now.to_msg(), 'laser'
        scan.angle_min, scan.angle_increment = -math.pi, 2. * math.pi / 500.
        scan.angle_max = scan.angle_min + 499 * scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time, scan.time_increment = .05, 12., .1, 0.
        ranges = []
        for i in range(500):
            a = yaw + scan.angle_min + i * scan.angle_increment
            dx, dy = math.cos(a), math.sin(a)
            candidates = []
            if abs(dx) > 1e-12:
                candidates.extend([(3. - x) / dx, (-3. - x) / dx])
            if abs(dy) > 1e-12:
                candidates.extend([(2. - y) / dy, (-2. - y) / dy])
            # Circular pillar at (1.8,1.1), radius0.25 gives distinct geometry.
            qx, qy = x - 1.8, y - 1.1
            b = qx * dx + qy * dy
            disc = b * b - (qx * qx + qy * qy - .25 * .25)
            if disc >= 0:
                candidates.extend([-b - math.sqrt(disc), -b + math.sqrt(disc)])
            ranges.append(min(d for d in candidates if d > .05))
        scan.ranges, scan.intensities = ranges, [30.] * 500
        self.scan.publish(scan)

def main():
    rclpy.init()
    node = Fixture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
if __name__ == '__main__':
    main()
