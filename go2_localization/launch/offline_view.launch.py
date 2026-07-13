"""Offline visualization of localization against the prior map.

Starts, in one shot:
  - map_matcher_node  : localization node; also publishes the prior map latched on
                        /localization/map (transient-local) so RViz shows it immediately
  - ros2 bag play     : replays a Point-LIO output bag (/registered_scan + /state_estimation)
  - rviz2             : grey prior map + yellow live aligned scan + red robot arrow + blue trail + TF

Example:
  ros2 launch go2_localization offline_view.launch.py \
       bag:=~/maps/loc_test_2_1_plio start_offset:=0 rate:=1.0

The bag path accepts ~ and relative paths (expanded here). Note: the map matcher is
tracking-only, so the robot must start near the map origin; otherwise set the start pose
in RViz with the "2D Pose Estimate" tool (publishes /initialpose).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('go2_localization')
    cfg = os.path.join(pkg, 'config', 'localization.yaml')
    rviz = os.path.join(pkg, 'rviz', 'localization.rviz')

    # Resolve the bag path at runtime so ~ and relative paths work (launch does not
    # expand them by itself).
    bag = os.path.abspath(os.path.expanduser(LaunchConfiguration('bag').perform(context)))
    rate = LaunchConfiguration('rate').perform(context)
    start_offset = LaunchConfiguration('start_offset').perform(context)

    matcher = Node(
        package='go2_localization', executable='map_matcher_node', name='map_matcher_node',
        parameters=[cfg], output='screen',
    )
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz],
    )
    # Give the matcher a moment to load the map before the bag starts streaming.
    bag_play = TimerAction(period=3.0, actions=[
        ExecuteProcess(cmd=[
            'ros2', 'bag', 'play', bag, '--rate', rate, '--start-offset', start_offset,
        ], output='screen'),
    ])
    return [matcher, rviz_node, bag_play]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='/home/dmitriyb51/autonomy_stack_go2/plio_out'),
        DeclareLaunchArgument('start_offset', default_value='0'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        OpaqueFunction(function=launch_setup),
    ])
