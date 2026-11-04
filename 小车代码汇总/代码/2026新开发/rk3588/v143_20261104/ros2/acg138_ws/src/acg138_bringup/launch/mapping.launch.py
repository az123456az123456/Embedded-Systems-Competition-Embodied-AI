"""Headless live mapping. Local map bridge previews by default; no vehicle motion output."""
import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def _bridge(context):
    cfg = context.launch_configurations
    if cfg['enable_fpga_bridge'].lower() != 'true':
        return []
    return [Node(package='acg138_fpga_map_bridge', executable='fpga_map_bridge', name='acg138_fpga_map_bridge', output='screen',
                 parameters=[os.path.join(get_package_share_directory('acg138_fpga_map_bridge'), 'config', 'map_bridge.yaml'),
                             {'interface': cfg['interface'], 'dry_run': cfg['dry_run'].lower() == 'true',
                              'exclusive_nav2_owner': cfg['exclusive_nav2_owner'].lower() == 'true',
                              'enable_plan': cfg['enable_plan'].lower() == 'true',
                              'host_proc_root': os.environ.get('ACG138_HOST_PROC_ROOT', '/proc')}])]

def generate_launch_description():
    here = get_package_share_directory('acg138_bringup')
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=lambda event, context: [EmitEvent(event=Shutdown(reason="critical ROS process failed"))] if event.returncode else [])),
        DeclareLaunchArgument('interface', default_value='eth0'),
        DeclareLaunchArgument('enable_fpga_bridge', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('exclusive_nav2_owner', default_value='false'),
        DeclareLaunchArgument('enable_plan', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'platform.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
                                 launch_arguments={'use_sim_time': 'false', 'autostart': 'true', 'use_lifecycle_manager': 'false',
                                                   'slam_params_file': os.path.join(here, 'config', 'slam.yaml')}.items()),
        OpaqueFunction(function=_bridge)])
