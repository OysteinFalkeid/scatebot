import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/oystein/scatebot/ros2_ws/install/gamepad_joy'
