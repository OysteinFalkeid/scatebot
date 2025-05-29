#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/oystein/scatebot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch scatebot bringup.launch.py 
