#!/usr/bin/python3
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from customize_interface.msg import MotionCommand
from sensor_msgs.msg import Joy

Motion_command = MotionCommand()

VelOriginalRange = np.array([1, -1])
VelTargetRange = np.array([0, 5])

AngleOriginalRange = np.array([1, -1])
AngleTargetRange = np.array([-30, 30])

Button1_flag = False
vel_direction = True

class Joy_vel_pub(Node):
    def __init__(self):
        super().__init__("Joy_vel_pub")
        timer_period = 0.02
        self.vel_pub = self.create_publisher(MotionCommand, "Motion_command", 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global Vel_msg
        self.vel_pub.publish(Vel_msg)



class Joy_controller(Node):
    def __init__(self):
        super().__init__("Joy_sub")

        self.Joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
    
    def listener_callback(self, JoyData):
        global Motion_command

        if JoyData.axes[4] == 1:
            vel_direction = False
        elif JoyData.axes[5] == 1:
            vel_direction = True

        Motion_command.linear_x = np.interp(JoyData.axes[6], VelOriginalRange, VelTargetRange)

        Motion_command.center_rotation_rad = np.interp(JoyData.axes[0], AngleOriginalRange, AngleTargetRange)

        if JoyData.buttons[0] == 1 and Button1_flag == True:
            Motion_command.turning_distance = 1
            Button1_flag = False
        elif JoyData.button[0] == 0 and Button1_flag != True:
            Button1_flag = True



def main(args = None):
    rclpy.init(args = args)

    joy_controller = Joy_controller()
    joy_vel_pub = Joy_vel_pub()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joy_controller)
    executor.add_node(joy_vel_pub)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = joy_vel_pub.create_rate(2)
    
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()



if __name__ == '__main__':
    main()