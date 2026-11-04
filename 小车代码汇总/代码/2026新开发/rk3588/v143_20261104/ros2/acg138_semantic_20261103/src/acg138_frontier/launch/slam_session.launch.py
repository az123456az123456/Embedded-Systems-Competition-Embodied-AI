"""Standard slam_toolbox lifecycle; mapping algorithm is unchanged."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node

def start(context):
    mode=context.launch_configurations['mode']
    if mode not in ('mapping','localization'):raise ValueError('invalid SLAM mode')
    executable='async_slam_toolbox_node' if mode=='mapping' else 'localization_slam_toolbox_node'
    # The coordinator is the single lifecycle owner. It polls GetState after
    # every transition, including lost replies, instead of relying on one event.
    node=Node(package='slam_toolbox',executable=executable,name='slam_toolbox',namespace='',output='screen',
                       parameters=[context.launch_configurations['parameters'],{'use_sim_time':False,'use_lifecycle_manager':False}])
    return [node]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode',default_value='mapping'),DeclareLaunchArgument('parameters'),OpaqueFunction(function=start)])
