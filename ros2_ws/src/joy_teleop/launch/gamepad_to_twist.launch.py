#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable

def generate_launch_description():
    
    teleop_node = Node(
        namespace="teleop",
        package="joy_teleop",
        executable="joy_teleop",
        output="screen",
        remappings=[
            ("/teleop/joy", "/gamepad_joy/joy")
        ],
    )

    gamepad_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('gamepad_joy'), 'launch', 'joy.launch.py'])
        )
    )

    return LaunchDescription([
        teleop_node,
        gamepad_node,
    ])