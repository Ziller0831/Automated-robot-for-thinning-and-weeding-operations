import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from time import sleep

coordinate = Float32MultiArray()

ser = serial.Serial('/dev/MEGA2560', 115200, timeout=1)  # open serial port
sleep(2)


class DeltaCordSub(Node):
    def __init__(self, node_name):
        super().__init__("node_name")
        self.cord_subscriber = self.create_subscription(
            Float32MultiArray, 'delta_cord', self.delta_callback, 10)

    def delta_callback(self, msg):
        # groups與cord分別是msg的第一與第二維度
        groups_size = msg.layout.dim[0].size
        cord_size = msg.layout.dim[1].size

        # 將msg的資料解壓縮成gcode
        gcodes = []
        for i in range(groups_size):
            group = msg.data[i*cord_size:(i+1)*cord_size]
            gcodes.append(self.__coordinate_to_gcode(group))

        # 與delta機器人序列埠溝通
        Delta_communication(gcodes)

        self.get_logger().info(f'Delta cord: {gcodes}')

    def __coordinate_to_gcode(self, delta_cord):
        # 如果z軸深度為0，判斷要歸零
        if delta_cord[2] == 0:
            gcode = 'G28'
            return gcode

        x = round(delta_cord[0], 2)
        y = round(delta_cord[1], 2)
        z = round(delta_cord[2], 2)

        gcode = 'G0 X' + str(x) + ' Y' + str(y) + ' Z' + str(z)

        return gcode

# * 序列埠通訊


def Delta_communication(gcodes):
    for gcode in gcodes:
        ser.write((gcode + '\n').encode())
        while 1:
            response = ser.readline()
            if (response.find('Ok'.encode()) > -1):
                break
            elif (response.find('Unknown'.encode()) > -1):
                print("Unknown: Limit Exceeded")
                break


def main(args=None):
    rclpy.init(args=args)
    delta_cord_sub = DeltaCordSub('delta_sub')
    rclpy.spin(delta_cord_sub)
    delta_cord_sub.destroy_node()
    rclpy.shutdown()


# ser.close()             # close port
