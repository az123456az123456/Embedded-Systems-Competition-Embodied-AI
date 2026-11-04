"""No hardware simulation entrypoint for validating the complete ROS image."""
import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    here = get_package_share_directory('acg138_bringup')
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=lambda event, context: [EmitEvent(event=Shutdown(reason="critical ROS process failed"))] if event.returncode else [])),
        Node(package='acg138_bringup', executable='synthetic_scan.py', output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='laser_mount',
             arguments=['--x', '0', '--y', '0', '--z', '.175', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'laser']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'false', 'autostart': 'true', 'use_lifecycle_manager': 'false', 'slam_params_file': os.path.join(here, 'config', 'slam.yaml')}.items()),
        Node(package='acg138_fpga_map_bridge', executable='fpga_map_bridge', name='acg138_fpga_map_bridge', output='screen',
             parameters=[os.path.join(get_package_share_directory('acg138_fpga_map_bridge'), 'config', 'map_bridge.yaml'), {'dry_run': True}])])
