#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    virtual_gamepad_node = launch_ros.actions.Node(
        namespace="gamepad_joy",
        package="gamepad_joy",
        executable="gamepad_joy",
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable('SDL_AUDIODRIVER', 'dummy'),
        virtual_gamepad_node
    ])