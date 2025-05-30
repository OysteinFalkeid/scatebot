#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
from pygame.locals import *

class GamepadTalker(Node):
    def __init__(self):
        super().__init__('gamepad_talker')

        pygame.init()
        self.reconector = self.create_timer(0.8, self.reconect)
        self.gamepad_connected = pygame.joystick.get_count() > 0
        if self.gamepad_connected:
            self.get_logger().info("gamepad found")
            self.gamepad = pygame.joystick.Joystick(0)
        else:
            self.get_logger().warning("waiting for gamepad")

        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        timer_period = 0.02 
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def reconect(self):
        if not self.gamepad_connected:
            pygame.joystick.quit()
            pygame.joystick.init()
            self.gamepad_connected = pygame.joystick.get_count() > 0
            if self.gamepad_connected:
                self.get_logger().info("gamepad found")
                self.gamepad = pygame.joystick.Joystick(0)

    def timer_callback(self):
        self.gamepad_connected = pygame.joystick.get_count() > 0
        if self.gamepad_connected:
            for event in pygame.event.get(): # get the events (update the joystick)
                pass
            
            # if self.gamepad.get_button(11):
            #     exit()

            dpad = self.gamepad.get_hat(0)

            DPAD_RIGHT = 0
            DPAD_LEFT = 0
            DPAD_UP = 0
            DPAD_DOWN = 0

            if dpad[0] == 1:
                DPAD_RIGHT = 1
            elif dpad[0] == -1:
                DPAD_LEFT = 1
            if dpad[1] == 1:
                DPAD_UP = 1
            elif dpad[1] == -1:
                DPAD_DOWN = 1

            axes = [
                self.gamepad.get_axis(0),
                -self.gamepad.get_axis(1),
                self.gamepad.get_axis(3),
                -self.gamepad.get_axis(4),
                self.gamepad.get_axis(2),
                self.gamepad.get_axis(5),
            ]

            # for i, ax in enumerate(axes):
            #     if abs(ax) < 0.05:
            #         axes[i] = 0.0

            
            message = Joy()
            message.axes = [
                axes[0], # LEFTX
                axes[1], # LEFTY inverse
                axes[2], # RIGHTX
                axes[3], # RIGHTY inverse
                axes[4],
                axes[5],
            
            ]
            message.buttons = [
                bool(self.gamepad.get_button(0)), # A
                bool(self.gamepad.get_button(1)), # B
                bool(self.gamepad.get_button(2)), # X
                bool(self.gamepad.get_button(3)), # Y
                bool(self.gamepad.get_button(4)), # L
                bool(self.gamepad.get_button(5)), # R
                bool(self.gamepad.get_button(6)), # -
                bool(self.gamepad.get_button(7)), # +
                bool(self.gamepad.get_button(8)), # Home
                bool(self.gamepad.get_button(9)), # LEFTSTICK
                bool(self.gamepad.get_button(10)), # RIGHTSTICK
                bool(DPAD_UP),
                bool(DPAD_DOWN),
                bool(DPAD_LEFT),
                bool(DPAD_RIGHT),
                # bool(self.gamepad.get_button(4)), # Screanshot,
                # False,
                # False,
                # False,
                # False,
            ]
            
            self.publisher_.publish(message)



def main(args=None):
    rclpy.init()
    gamepad_to_joy = GamepadTalker()  # Pass gamepad instance
    rclpy.spin(gamepad_to_joy)
    pygame.quit()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
