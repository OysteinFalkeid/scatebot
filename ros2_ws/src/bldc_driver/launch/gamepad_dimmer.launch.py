#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable

def generate_launch_description():
    
    bldc_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('bldc_driver'), 'launch', 'bldc_interface.launch.py'])
        )
    )

    gamepad_twist_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('joy_teleop'), 'launch', 'gamepad_to_twist.launch.py'])
        )
    )

    return LaunchDescription([
        bldc_node,
        gamepad_twist_node,
    ])