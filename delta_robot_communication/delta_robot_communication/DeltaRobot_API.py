import rclpy
from rclpy.node import Node
import serial
from time import sleep

ser = serial.Serial('/dev/MEGA2560', 115200, timeout=1)  # open serial port
sleep(2)


class CoordinateSub(Node):
    def __init__(self):
        super().__init__("CoordinateSub")
        # self.subscriber_ = self.create_subscription(
        #     Joy,
        #     'joy',
        #     self.listener_callback,
        #     10)


def Delta_communication(gcodes):
    for gcode in gcodes:
        ser.write((gcode + '\n').encode())
        while 1:
            response = ser.readline()
            print(response)
            if (response.find('Ok'.encode()) > -1):
                break
            elif (response.find('Unknown'.encode()) > -1):
                print("Unknown: Limit Exceeded")
                break


def main(args=None):
    rclpy.init(args=args)

    gcodes = []
    gcodes.append('G28')
    gcodes.append('G01 Z-570')


ser.close()             # close port
