from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('acg138_fpga_map_bridge'), 'config', 'map_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='eth0'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('exclusive_nav2_owner', default_value='false'),
        DeclareLaunchArgument('enable_plan', default_value='false'),
        DeclareLaunchArgument('host_proc_root', default_value=EnvironmentVariable('ACG138_HOST_PROC_ROOT', default_value='/proc')),
        Node(package='acg138_fpga_map_bridge', executable='fpga_map_bridge', output='screen', parameters=[
            config,
            {'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
             'exclusive_nav2_owner': ParameterValue(LaunchConfiguration('exclusive_nav2_owner'), value_type=bool),
             'enable_plan': ParameterValue(LaunchConfiguration('enable_plan'), value_type=bool)},
            {'host_proc_root': LaunchConfiguration('host_proc_root')},
            {'interface': LaunchConfiguration('interface')},
        ]),
    ])
