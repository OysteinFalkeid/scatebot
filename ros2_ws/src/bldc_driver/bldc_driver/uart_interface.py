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
        self.serial_read_timer = self.create_timer(0.01, self.serial_read_callback)
    
    def serial_read_callback(self):

        while self.serial.in_waiting:
            bytes_read = self.serial.read(1)
            # self.get_logger().info(format(ord(bytes_read), '08b'))
            self.get.logger().info(str(bytes_read))

    def twist_callback(self, msg):
        angular: Vector3 = msg.angular
        linear: Vector3 = msg.linear

        linear_speed = linear.x
        angular_speed = angular.z
        
        byte_val_motor_0 =101 - 100 * (linear_speed)+25
        if abs(linear_speed) < 0.0001:
            byte_val_motor_0 = 0.0

        byte_val_motor_1 =101 - 100 * (angular_speed)+25
        if abs(angular_speed) < 0.0001:
            byte_val_motor_1 = 0.0

        # self.get_logger().info(str(int(byte_val_motor_0)) + " " + str(int(byte_val_motor_1)))

        self.serial.write(bytes([int(255)]))
        self.serial.write(bytes([int(byte_val_motor_0)]))
        self.serial.write(bytes([int(byte_val_motor_1)]))
        self.serial.write(bytes([int(0)]))




def main(args=None):
    rclpy.init() 
    uart_interface = UART_interface()  # Pass gamepad instance
    rclpy.spin(uart_interface)
    rclpy.shutdown()

if __name__ == '__main__':
    main()