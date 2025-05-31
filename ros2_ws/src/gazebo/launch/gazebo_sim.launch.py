#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown

def generate_launch_description():

    bot_name = LaunchConfiguration('bot_name', default='imrt_bot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_share_dir = get_package_share_directory('scatebot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ros2_control = LaunchConfiguration('ros2_control', default='false')
    on_exit_shutdown = LaunchConfiguration('on_exit_shutdown', default='true')

    default_world = os.path.join(
        robot_share_dir,
        'worlds',
        'empty_world.sdf'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': on_exit_shutdown}.items()
    )

    kill_gz = ExecuteProcess(
        cmd=['pkill', '-f', 'gz'],  # or use 'gzserver' depending on how it's launched
        shell=True
    )

    node_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share_dir, 'launch', 'description.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'ros2_control': ros2_control}.items()
    )

    
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', bot_name,
            '-z', '0.5',
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        robot_share_dir,
        'config',
        'bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        # RegisterEventHandler(
        #     OnShutdown(
        #         on_shutdown=[
        #             LogInfo(msg="Shutting down Gazebo manually..."),
        #             kill_gz
        #         ]
        #     )
        # ),
            
        world_arg,
        gazebo,
        node_robot_state_publisher,
        start_gazebo_ros_spawner_cmd,
        start_gazebo_ros_bridge_cmd,
    ])