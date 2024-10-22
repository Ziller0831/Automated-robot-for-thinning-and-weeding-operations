#!/usr/bin/python3
import rclpy
import threading
from rclpy.node import Node
from customize_interface.msg import JoyMotionCommand
from sensor_msgs.msg import Joy

joyMotionCommand = JoyMotionCommand()


class joyController(Node):
    def __init__(self):
        super().__init__("joy_controller")

        self.turning_mode = 0
        self.button1_flag = True
        self.vel_direction = True

        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        self.joy_publisher = self.create_publisher(
            JoyMotionCommand, "joy_command", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def joy_callback(self, joy_data):
        # * 轉彎模態切換
        if joy_data.buttons[0] == 1 and self.button1_flag == True:
            self.turning_mode = not (self.turning_mode)
            self.button1_flag = False
        elif joy_data.buttons[0] == 0:
            self.button1_flag = True

        joyMotionCommand.turning_mode = self.turning_mode
        linear_x = linear_mapping(joy_data.axes[4], 1, -1, 0, 2)

        # * Ackerman
        if self.turning_mode == 0:

            # * 轉速與正反轉處理
            if joy_data.buttons[6] == 1 and linear_x == 0:
                self.vel_direction = False
            elif joy_data.buttons[7] == 1 and linear_x == 0:
                self.vel_direction = True

            if self.vel_direction == True:
                joyMotionCommand.linear_x = linear_x
            elif self.vel_direction == False:
                joyMotionCommand.linear_x = -linear_x

            # * 轉彎功能
            joyMotionCommand.center_rotate_angle = linear_mapping(
                joy_data.axes[0], -1, 1, 40, -40)

        elif self.turning_mode == 1:
            joyMotionCommand.linear_x = 0.0
            joyMotionCommand.center_rotate_angle = linear_mapping(
                joy_data.axes[0], -1, 1, 2, -2)

    def timer_callback(self):
        global joyMotionCommand
        self.joy_publisher.publish(joyMotionCommand)


def main(args=None):
    rclpy.init(args=args)
    joy_controller = joyController()
    rclpy.spin(joy_controller)
    joy_controller.destroy_node()
    rclpy.shutdown()


def linear_mapping(value, in_min, in_max, out_min, out_max):
    return round((value-in_min) * (out_max-out_min) / (in_max-in_min) + out_min, 2)
