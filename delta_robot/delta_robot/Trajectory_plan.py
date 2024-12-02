import rclpy
import threading
from pandas import read_csv
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# csv_path = "/home/ced/Image_recognition_ws/output/keypoints_3d.csv"


class TrajectoryPlanNode(Node):
    def __init__(self):
        super().__init__("trajectory_planer")

        # Create a subscriber to the plant_cord topic
        self.cord_subscriber = self.create_subscription(
            Float32MultiArray, 'plant_cord', self.traject_callback, 10)

        # Create a publisher to the delta_cord topic
        self.cord_publisher = self.create_publisher(
            Float32MultiArray, 'delta_cord', 10)

        self.delta_working_level = [-500, -625]
        self.plant_cord = [[]]

    def traject_callback(self, msg):
        self.get_logger().info(f'Plant cord: {msg.data}')
        self.planed_cord = self.__trajectory_plan(msg.data)

        # 建構Float32MultiArray消息
        cord_array = Float32MultiArray()

        cord_array.layout.dim.append(MultiArrayDimension())

        cord_array.layout.dim[0].label = "group"
        cord_array.layout.dim[1].label = "coordinate"
        cord_array.layout.dim[0].size = len(self.planed_cord)
        cord_array.layout.dim[1].size = len(self.planed_cord[0])
        cord_array.layout.dim[0].stride = len(
            self.planed_cord) * len(self.planed_cord[0])
        cord_array.layout.dim[1].stride = len(self.planed_cord[0])

        cord_array.layout.data_offset = 0

        flat_array = [
            float(item) for sublist in self.planed_cord for item in sublist]
        cord_array.data = flat_array

        self.cord_publisher.publish(cord_array)

    def __trajectory_plan(self, raw_cord):

        planed_cord = [[0, 0, 0], [0, 0, -450]]

        for row in raw_cord:
            for z in self.delta_working_level:
                planing_cord = row[:2] + [z]
                planed_cord.append(planing_cord)

        planed_cord.append([0, 0, -450])

        return planed_cord


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
