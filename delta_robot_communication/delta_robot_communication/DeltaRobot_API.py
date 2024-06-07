import rclpy
from rclpy.node import Node
import serial
from time import sleep

ser = serial.Serial('/dev/MEGA2560', 115200, timeout=1)  # open serial port
sleep(2)

gcodes = []
gcodes.append('G28')
gcodes.append('G01 Z-570')


class DeltaPub(Node):
    def __init__(self):
        super().__init__("DeltaPub")


def main():
    for gcode in gcodes:
        print(gcode)
        ser.write((gcode + '\n').encode())
        while 1:
            response = ser.readline()
            print(response)
            if (response.find('Ok'.encode()) > -1):
                break
            elif (response.find('Unknown'.encode()) > -1):
                break


ser.close()             # close port