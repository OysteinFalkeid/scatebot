#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    UART_node = Node(
        namespace="bldc",
        package="bldc_driver",
        executable="uart_interface",
        output="screen",
        remappings=[
            ("/cmd_vel", "/teleop/joy_twist"),
        ],
    )

    return LaunchDescription([
        UART_node
    ])