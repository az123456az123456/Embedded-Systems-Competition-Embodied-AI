from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package='acg138_frontier',executable='frontier_node',parameters=[{'dry_run':True}],output='screen')])
