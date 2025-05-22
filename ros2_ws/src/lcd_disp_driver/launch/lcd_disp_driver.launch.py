#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable

def generate_launch_description():
    
    lcd_disp_driver_node = Node(
        namespace="lcd_disp_driver",
        package="lcd_disp_driver",
        executable="lcd_disp_driver",
        output="screen",
        remappings=[
            ("/twist", "/teleop/joy_twist")
        ],
    )

    return LaunchDescription([
        lcd_disp_driver_node,
    ])