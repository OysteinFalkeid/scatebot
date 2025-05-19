#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    virtual_gamepad_node = launch_ros.actions.Node(
        namespace="gamepad_joy",
        package="gamepad_joy",
        executable="gamepad_joy",
        output="screen",
    )

    return LaunchDescription([
        virtual_gamepad_node
    ])