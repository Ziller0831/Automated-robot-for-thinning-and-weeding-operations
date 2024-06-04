#!/usr/bin/python3
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from customize_interface.msg import JoyMotionCommand
from sensor_msgs.msg import Joy

joyMotionCommand = JoyMotionCommand()

VelOriginalRange = np.array([1, -1])
VelTargetRange = np.array([0, 5])

AngleOriginalRange = np.array([1, -1])
AngleTargetRange = np.array([-30, 30])

vel_direction = True
Button1_flag = 1


class Joy_pub(Node):
    def __init__(self):
        super().__init__("joy_pub")
        timer_period = 0.5
        self.publisher_ = self.create_publisher(
            JoyMotionCommand, "joy_command", 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global joyMotionCommand
        self.publisher_.publish(joyMotionCommand)


class Joy_sub(Node):
    def __init__(self):
        super().__init__("joy_sub")
        self.subscriber_ = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)

    def listener_callback(self, JoyData):
        global joyMotionCommand
        # global Button1_flag

        linear_x = np.interp(JoyData.axes[6], VelOriginalRange, VelTargetRange)

        if JoyData.axes[4] == 1:  # 後退
            vel_direction = False
        elif JoyData.axes[5] == 1:  # 前進
            vel_direction = True

        if vel_direction == True:
            joyMotionCommand.linear_x = linear_x
        elif vel_direction == False:
            joyMotionCommand.linear_x = -linear_x

        joyMotionCommand.center_rotate_angle = np.interp(
            JoyData.axes[0], AngleOriginalRange, AngleTargetRange)

        if JoyData.buttons[0] == 1 and Button1_flag == 1:
            joyMotionCommand.turning_distance = 1
            Button1_flag = 0
        elif JoyData.button[0] == 0 and Button1_flag != 1:
            Button1_flag = 1


def main(args=None):
    rclpy.init(args=args)

    joy_sub = Joy_sub()
    joy_pub = Joy_pub()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joy_sub)
    executor.add_node(joy_pub)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = joy_pub.create_rate(2)

    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()
