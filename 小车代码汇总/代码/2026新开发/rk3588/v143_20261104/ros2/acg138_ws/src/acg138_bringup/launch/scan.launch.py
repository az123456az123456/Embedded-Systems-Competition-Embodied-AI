"""MVP stage1: only /scan and the measured rigid laser mount, no fake odometry."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('acg138_bringup'), 'launch', 'platform.launch.py')),
        launch_arguments={'with_base': 'false'}.items())])
