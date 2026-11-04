"""Idle at startup. Only explicit phone tasks authorize FPGA execution."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dry = ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)
    interface = LaunchConfiguration('interface')
    platform = os.path.join(get_package_share_directory('acg138_bringup'), 'launch', 'platform.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('interface', default_value='eth0'),
        DeclareLaunchArgument('record_lidar', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar_c1'),
        DeclareLaunchArgument('laser_z', default_value='0.214'),
        # Confirmed: a vehicle-front return appeared at +Y (left).
        # Rotate the native laser frame clockwise 90 degrees into base_link.
        DeclareLaunchArgument('laser_yaw', default_value='-1.5707963267948966'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(platform), launch_arguments={
            'interface': interface, 'serial_port': LaunchConfiguration('serial_port'),
            'laser_x': '0.0', 'laser_y': '0.0', 'laser_z': LaunchConfiguration('laser_z'), 'laser_yaw': LaunchConfiguration('laser_yaw'),
            'with_base': 'true', 'use_ekf': 'false'}.items()),
        Node(package='acg138_frontier', executable='mapping_explorer', output='screen',
             # Same physical front as NAV3 and the user-verified phone arrow.
             # Re-express both exploration poses and scan checks; never edit TF.
             parameters=[{'dry_run': dry,'navigation_heading_offset_rad': 3.141592653589793}]),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain', output='screen',
             parameters=[os.path.join(get_package_share_directory('acg138_frontier'), 'config', 'laser_filters.yaml')],
             remappings=[('scan', '/scan'), ('scan_filtered', '/scan_mapping')]),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain', name='scan_clearing_filter', output='screen',
             parameters=[os.path.join(get_package_share_directory('acg138_frontier'), 'config', 'laser_clearing_filters.yaml')],
             remappings=[('scan', '/scan'), ('scan_filtered', '/scan_clearing')]),
        Node(package='acg138_frontier', executable='dynamic_map', output='screen'),
        ExecuteProcess(cmd=['ros2', 'run', 'acg138_frontier', 'lidar_recorder',
                            '--qos-file', os.path.join(get_package_share_directory('acg138_frontier'), 'config', 'lidar_record_qos.yaml')],
                       condition=IfCondition(LaunchConfiguration('record_lidar')), output='screen',
                       sigterm_timeout='12', sigkill_timeout='5'),
        Node(package='acg138_frontier', executable='mapping_coordinator', output='screen',
             parameters=[{'dry_run': dry, 'interface': interface, 'data_directory': '/data/maps'}]),
        Node(package='acg138_ros_bridge', executable='ros_bridge_node', output='screen',
             parameters=[{'ipc_directory': '/run/acg138-ros-bridge'}]),
    ])
