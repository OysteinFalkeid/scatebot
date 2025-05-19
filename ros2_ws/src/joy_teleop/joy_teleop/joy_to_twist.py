#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

class TeleopTalker(Node):
    def __init__(self):
        super().__init__("imrt_teleop")
        self.subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            QoSProfile(depth=10)
        )
        self.publisher_ = self.create_publisher(Twist, 'joy_twist', 10)

    def joy_callback(self, msg):
        forward = 0.0

        if msg.axes[3] > 0.15:
            forward = (msg.axes[3]**2 + msg.axes[2]**2)**0.5*0.7-abs(msg.axes[2])*0.4
        elif msg.axes[3] < -0.15:
            forward = -(msg.axes[3]**2 + msg.axes[2]**2)**0.5*0.7-abs(msg.axes[2])*0.4
        rotation = -msg.axes[2]
        
        message = Twist()
        message.angular = Vector3(x=0.0, y=0.0, z=rotation)
        message.linear = Vector3(x=forward, y=0.0, z=0.0)
        # print(message)
        self.publisher_.publish(message)

def main(args=None):
    rclpy.init() 
    teleop_talker = TeleopTalker()  # Pass gamepad instance
    rclpy.spin(teleop_talker)
    rclpy.shutdown()

if __name__ == '__main__':
    main()