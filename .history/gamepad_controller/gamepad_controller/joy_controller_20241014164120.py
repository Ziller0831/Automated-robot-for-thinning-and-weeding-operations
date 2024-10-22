#!/usr/bin/python3
import rclpy
import threading
from rclpy.node import Node
from customize_interface.msg import JoyMotionCommand
from sensor_msgs.msg import Joy

joyMotionCommand = JoyMotionCommand()


class JoyBasePublisher(Node):
    def __init__(self):
        super().__init__("joy_base_publisher")
        timer_period = 0.1
        self.publisher_ = self.create_publisher(
            JoyMotionCommand, "joy_command", 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global joyMotionCommand
        self.publisher_.publish(joyMotionCommand)


class JoySubscriber(Node):
    def __init__(self):
        super().__init__("joy_subscriber")
        self.subscriber_ = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.turning_mode = 0
        self.button1_flag = True
        self.vel_direction = True

    def listener_callback(self, joy_data):
        # * 轉速與正反轉處理
        linear_x = linear_mapping(joy_data.axes[4], 1, -1, 0, 2)

        if joy_data.buttons[6] == 1 and linear_x == 0:
            self.vel_direction = False
        elif joy_data.buttons[7] == 1 and linear_x == 0:
            self.vel_direction = True

        if self.vel_direction == True:
            joyMotionCommand.linear_x = round(linear_x, 2)
        elif self.vel_direction == False:
            joyMotionCommand.linear_x = round(-linear_x, 2)

        # * 轉彎功能
        joyMotionCommand.center_rotate_angle = linear_mapping(
            round(joy_data.axes[0], 2), -1, 1, 40, -40)

        # * 轉彎模態切換
        if joy_data.buttons[0] == 1 and self.button1_flag == True:
            self.turning_mode = not (self.turning_mode)
            self.button1_flag = False
        elif joy_data.buttons[0] == 0:
            self.button1_flag = True

        joyMotionCommand.turning_mode = self.turning_mode


def main(args=None):
    rclpy.init(args=args)

    joy_sub = JoySubscriber()
    joy_base_pub = JoyBasePublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joy_sub)
    executor.add_node(joy_base_pub)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = joy_base_pub.create_rate(2)

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
