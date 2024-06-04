#!/usr/bin/python3
import rclpy
import threading
from rclpy.node import Node
from customize_interface.msg import JoyMotionCommand
from sensor_msgs.msg import Joy

joyMotionCommand = JoyMotionCommand()


class Joy_pub(Node):
    def __init__(self):
        super().__init__("joy_pub")
        timer_period = 0.1
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
        self.turning_mode = 0
        self.button1_flag = True
        self.vel_direction = True

    def listener_callback(self, JoyData):

        linear_x = linear_mapping(JoyData.axes[4], 1, -1, 0, 2)

        if JoyData.axes[4] == 1:
            self.vel_direction = False
        elif JoyData.axes[5] == 1:
            self.vel_direction = True

        if self.vel_direction == True:
            joyMotionCommand.linear_x = round(linear_x, 2)
        elif self.vel_direction == False:
            joyMotionCommand.linear_x = round(-linear_x, 2)

        joyMotionCommand.center_rotate_angle = linear_mapping(
            round(JoyData.axes[0], 2), -1, 1, -45, 45)

        if JoyData.buttons[0] == 1 and self.button1_flag == True:
            self.turning_mode = not (self.turning_mode)
            self.button1_flag = False
        elif JoyData.buttons[0] == 0:
            self.button1_flag = True

        joyMotionCommand.turning_mode = self.turning_mode


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


def linear_mapping(value, in_min, in_max, out_min, out_max):
    return (value-in_min) * (out_max-out_min) / (in_max-in_min) + out_min
