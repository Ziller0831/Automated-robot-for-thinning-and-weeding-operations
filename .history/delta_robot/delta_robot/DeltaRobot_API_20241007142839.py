import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from time import sleep

coordinate = Float32MultiArray()

ser = serial.Serial('/dev/MEGA2560', 115200, timeout=1)  # open serial port
sleep(2)


class CoordinateSubscriber(Node):
    def __init__(self):
        super().__init__("coordinate_sub")
        self.subscriber_ = self.create_subscription(
            Float32MultiArray, 'delta_coordinate', self.listener_callback, 10)

    def listener_callback(self, coordinate):
        groups_num = coordinate.layout.dim[0].size
        coord_num = coordinate.layout.dim[1].size

        gcodes = []
        for i in range(groups_num):
            group = coordinate.data[i*coord_num:(i+1)*coord_num]
            gcodes.append(self.__coordinate_to_gcode(group))

        Delta_communication(gcodes)

        self.get_logger().info(f'I heard: {gcodes}')

    def __coordinate_to_gcode(self, delta_cord):
        if delta_cord[2] == 0:
            gcode = 'G28'
            return gcode

        x = round(delta_cord[0], 2)
        y = round(delta_cord[1], 2)
        z = round(delta_cord[2], 2)
        
        gcode = 'G0 X' + str(x) + ' Y' + str(y) + ' Z' + str(z)

        return gcode


def Delta_communication(gcodes):
    for gcode in gcodes:
        ser.write((gcode + '\n').encode())
        while 1:
            response = ser.readline()
            # print(response)
            if (response.find('Ok'.encode()) > -1):
                break
            elif (response.find('Unknown'.encode()) > -1):
                print("Unknown: Limit Exceeded")
                break


def main(args=None):
    rclpy.init(args=args)
    coordinate_sub = CoordinateSubscriber()
    rclpy.spin(coordinate_sub)
    coordinate_sub.destroy_node()
    rclpy.shutdown()


# ser.close()             # close port
