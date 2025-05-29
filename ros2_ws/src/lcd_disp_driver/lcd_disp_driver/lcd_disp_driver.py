#!/usr/bin/env python3

import smbus2
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3



class LCS_DISP_DRIVER(Node):
    def __init__(self, node_name, *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False, enable_logger_service = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides, enable_logger_service=enable_logger_service)

        # LCD Address (from your i2cdetect): 0x27
        self.LCD_ADDR = 0x27

        self.bus = smbus2.SMBus(1)

        # Commands and flags (based on HD44780 and PCF8574)
        self.LCD_CHR = 1  # Mode - Sending data
        self.LCD_CMD = 0  # Mode - Sending command

        self.LCD_BACKLIGHT = 0x08  # On
        self.ENABLE = 0b00000100   # Enable bit

        self.subscriber = self.create_subscription(
            Twist,
            '/twist',
            self.twist_callback,
            QoSProfile(depth=10)
        )

        self.lcd_init()
        self.lcd_string("Scatebot", 1)

    def lcd_strobe(self, data):
        self.bus.write_byte(self.LCD_ADDR, data | self.ENABLE | self.LCD_BACKLIGHT)
        self.bus.write_byte(self.LCD_ADDR, (data & ~self.ENABLE) | self.LCD_BACKLIGHT)
        time.sleep(0.001)

    def lcd_write_byte(self, bits, mode):
        high_bits = mode | (bits & 0xF0) | self.LCD_BACKLIGHT
        low_bits = mode | ((bits << 4) & 0xF0) | self.LCD_BACKLIGHT

        self.bus.write_byte(self.LCD_ADDR, high_bits)
        self.lcd_strobe(high_bits)

        self.bus.write_byte(self.LCD_ADDR, low_bits)
        self.lcd_strobe(low_bits)

    def lcd_init(self):
        self.lcd_write_byte(0x33, self.LCD_CMD)  # Initialize
        self.lcd_write_byte(0x32, self.LCD_CMD)  # Set to 4-bit mode
        self.lcd_write_byte(0x28, self.LCD_CMD)  # 2 line, 5x8 font
        self.lcd_write_byte(0x0C, self.LCD_CMD)  # Display ON, cursor OFF, blink OFF
        self.lcd_write_byte(0x06, self.LCD_CMD)  # Entry mode set
        self.lcd_write_byte(0x01, self.LCD_CMD)  # Clear display
        time.sleep(0.0005)

    def lcd_string(self, message, line):
        if line == 1:
            self.lcd_write_byte(0x80, self.LCD_CMD)
        elif line == 2:
            self.lcd_write_byte(0xC0, self.LCD_CMD)

        for char in message.ljust(16):
            self.lcd_write_byte(ord(char), self.LCD_CHR)

    def twist_callback(self, msg):
        message: Twist = msg
        linear: Vector3 = message.linear
        angular: Vector3 = message.angular
        x_string = ("X:" + str(round(linear.x, 2))).ljust(7, " ")
        yaw_string = str(round(angular.z, 2))
        self.lcd_string((x_string + "YAW:" + yaw_string), 2)


def main(ars=Node):
    rclpy.init() 
    lcd_disp_driver = LCS_DISP_DRIVER("lcd_disp_driver")  # Pass gamepad instance
    rclpy.spin(lcd_disp_driver)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
