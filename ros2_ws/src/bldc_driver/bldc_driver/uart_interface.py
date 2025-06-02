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

        byte_val = 120 - 120 * abs(linear_list[0])
        if abs(linear_list[0]) < 0.001:
            byte_val = 0.0

        self.get_logger().info(str(byte_val))

        self.serial.write(bytes([int(255)]))
        self.serial.write(bytes([int(byte_val)]))
        self.serial.write(bytes([int(byte_val)]))
        self.serial.write(bytes([int(0)]))

        bytes_read = self.serial.read_all()
        self.get_logger().info(str(bytes_read))

def main(args=None):
    rclpy.init() 
    uart_interface = UART_interface()  # Pass gamepad instance
    rclpy.spin(uart_interface)
    rclpy.shutdown()

if __name__ == '__main__':
    main()