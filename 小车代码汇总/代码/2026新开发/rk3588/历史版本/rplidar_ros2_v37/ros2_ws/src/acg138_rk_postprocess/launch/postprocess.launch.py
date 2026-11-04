from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="acg138_rk_postprocess",
            executable="rk_postprocess_node",
            name="acg138_rk_postprocess",
            output="screen",
        ),
    ])
