"""Load a saved occupancy map with AMCL. Wait for an explicit /initialpose; no motor output."""
import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def _start(context):
    cfg = context.launch_configurations
    path = cfg['map']
    if not os.path.isabs(path) or not os.path.isfile(path):
        raise RuntimeError('map:= requires an existing absolute .yaml path inside the ROS userspace')
    params = os.path.join(get_package_share_directory('acg138_bringup'), 'config', 'amcl.yaml')
    nodes = [Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                  parameters=[params, {'yaml_filename': path}]),
             Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', parameters=[params]),
             Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_localization_lifecycle', output='screen',
                  parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}])]
    if cfg['enable_fpga_bridge'].lower() == 'true':
        nodes.append(Node(package='acg138_fpga_map_bridge', executable='fpga_map_bridge', name='acg138_fpga_map_bridge', output='screen',
                          parameters=[os.path.join(get_package_share_directory('acg138_fpga_map_bridge'), 'config', 'map_bridge.yaml'),
                                      {'map_timeout_s': 0.0, 'interface': cfg['interface'], 'dry_run': cfg['dry_run'].lower() == 'true',
                                       'exclusive_nav2_owner': cfg['exclusive_nav2_owner'].lower() == 'true',
                                       'enable_plan': cfg['enable_plan'].lower() == 'true',
                                       'host_proc_root': os.environ.get('ACG138_HOST_PROC_ROOT', '/proc')}]))
    return nodes

def generate_launch_description():
    here = get_package_share_directory('acg138_bringup')
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=lambda event, context: [EmitEvent(event=Shutdown(reason="critical ROS process failed"))] if event.returncode else [])),
        DeclareLaunchArgument('map'), DeclareLaunchArgument('interface', default_value='eth0'),
        DeclareLaunchArgument('enable_fpga_bridge', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='true'), DeclareLaunchArgument('exclusive_nav2_owner', default_value='false'),
        DeclareLaunchArgument('enable_plan', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'platform.launch.py'))),
        OpaqueFunction(function=_start)])
