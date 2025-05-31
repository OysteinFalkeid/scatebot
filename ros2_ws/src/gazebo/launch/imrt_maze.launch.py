import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition

from launch.event_handlers import OnProcessExit

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ros2_control = LaunchConfiguration('ros2_control', default='true')
    robot_share_dir = get_package_share_directory('imrt_bot')

    world_name = LaunchConfiguration('world_name')

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty_world.sdf',
        description='world name to load from imrt bot package'
        )

    print(world_name)
    default_world = PathJoinSubstitution(
        (
        robot_share_dir,
        'worlds',
        world_name
        )
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    gazebo_robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('imrt_bot'), 
                'launch', 
                'maze.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time, 
            'on_exit_shutdown': 'true', 
            'ros2_control': ros2_control, 
            'world': world
        }.items()
    )
    


    imrt_switch_joy_node = Node(
        name='imrt_switch_joy',
        package="imrt_virtual_joy",
        executable="gamepad_switch_talker",
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'bot_name',
            default_value='imrt_bot',
            description='bot to load'
        ),        
        RegisterEventHandler(
            OnProcessExit(
                target_action=imrt_switch_joy_node,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            'closed the turtlesim window')),
                    EmitEvent(event=Shutdown(
                        reason='home button pressed'))
                ]
            )
        ),
        world_name_arg,
        world_arg,
        gazebo_robot_sim,
        # imrt_switch_joy_node,
    ])
