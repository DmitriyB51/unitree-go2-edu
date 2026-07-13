import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_localization')
    default_cfg = os.path.join(pkg, 'config', 'localization.yaml')

    config_arg = DeclareLaunchArgument('config', default_value=default_cfg,
                                       description='Path to localization.yaml')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false',
                                     description='Launch RViz')

    matcher = Node(
        package='go2_localization',
        executable='map_matcher_node',
        name='map_matcher_node',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg, 'rviz', 'localization.rviz')],
    )

    return LaunchDescription([config_arg, rviz_arg, matcher, rviz])
