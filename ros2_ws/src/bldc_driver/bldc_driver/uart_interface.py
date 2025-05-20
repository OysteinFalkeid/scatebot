#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Vector3
import serial

class UART_interface(Node):
    def __init__(self):
        super().__init__("bldc_driver")
        self.subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            QoSProfile(depth=10)
        )
        self.serial = serial.Serial('/dev/ttyACM0', timeout=1)

    def twist_callback(self, msg):
        angular: Vector3 = msg.angular
        linear: Vector3 = msg.linear
        angular_list = [angular.x, angular.y, angular.z]
        linear_list = [linear.x, linear.y, linear.z]
        self.get_logger().info(str(abs(linear_list[0])*254))
        self.serial.write(bytes([int(abs(linear_list[0])*254)]))

def main(args=None):
    rclpy.init() 
    uart_interface = UART_interface()  # Pass gamepad instance
    rclpy.spin(uart_interface)
    rclpy.shutdown()

if __name__ == '__main__':
    main()