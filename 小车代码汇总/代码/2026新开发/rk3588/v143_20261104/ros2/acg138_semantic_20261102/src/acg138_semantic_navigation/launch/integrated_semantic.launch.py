import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    existing = os.path.join(get_package_share_directory('acg138_frontier'), 'launch', 'integrated.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(existing),
                                 launch_arguments={'dry_run': LaunchConfiguration('dry_run')}.items()),
        Node(package='acg138_semantic_navigation', executable='semantic_node', output='screen',
             parameters=[{'publish_goals': False}]),
    ])
