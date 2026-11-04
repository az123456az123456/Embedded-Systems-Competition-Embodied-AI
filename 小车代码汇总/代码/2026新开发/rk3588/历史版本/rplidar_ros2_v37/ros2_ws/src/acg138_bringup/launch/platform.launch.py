"""C1 + measured laser mount + read-only encoder/IMU odometry. No motor controller."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, EmitEvent
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def _start(context):
    cfg = context.launch_configurations
    use_ekf = cfg['use_ekf'].lower() == 'true'
    with_base = cfg['with_base'].lower() == 'true'
    here = get_package_share_directory('acg138_bringup')
    actions = [Node(package='acg138_rplidar', executable='c1_node', name='acg138_rplidar', output='screen',
                    on_exit=[EmitEvent(event=Shutdown(reason='C1 driver exited; restart complete mapping session'))],
                    parameters=[os.path.join(get_package_share_directory('acg138_rplidar'), 'config', 'c1.yaml'),
                                {'serial_port': cfg['serial_port'], 'baudrate': 460800, 'frame_id': 'laser'}]),
               Node(package='tf2_ros', executable='static_transform_publisher', name='laser_mount',
                    arguments=['--x', cfg['laser_x'], '--y', cfg['laser_y'], '--z', cfg['laser_z'],
                               '--roll', '0', '--pitch', '0', '--yaw', cfg['laser_yaw'],
                               '--frame-id', 'base_link', '--child-frame-id', 'laser'])]
    if with_base:
        actions += [Node(package='acg138_base_telemetry', executable='base_telemetry_node', name='base_telemetry', output='screen',
                         on_exit=[EmitEvent(event=Shutdown(reason='base telemetry exited; do not map with stale odometry'))],
                         parameters=[os.path.join(get_package_share_directory('acg138_base_telemetry'), 'config', 'base.yaml'),
                                     {'interface': cfg['interface'], 'publish_direct_odom': not use_ekf}]),
                    Node(package='tf2_ros', executable='static_transform_publisher', name='imu_mount',
                         arguments=['--x', '0.081417', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                                    '--frame-id', 'base_link', '--child-frame-id', 'imu_link'])]
        if use_ekf:
            actions += [Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
                             parameters=[os.path.join(here, 'config', 'ekf.yaml')]),
                        Node(package='acg138_base_telemetry', executable='odom_health_gate', name='odom_health_gate', output='screen')]
    return actions

def generate_launch_description():
    values = {'serial_port': '/dev/rplidar_c1', 'interface': 'eth0', 'with_base': 'true', 'use_ekf': 'false',
              'laser_x': '0.0', 'laser_y': '0.0', 'laser_z': '0.175', 'laser_yaw': '0.0'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in values.items()] + [OpaqueFunction(function=_start)])
