import rclpy
import threading
from pandas import read_csv
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# csv_path = "/home/ced/Image_recognition_ws/output/keypoints_3d.csv"


class TrajectoryPlanNode(Node):
    def __init__(self):
        super().__init__("trajectory_plan_node")

        # Create a subscriber to the plant_cord topic
        self.cord_subscriber = self.create_subscription(
            Float32MultiArray, 'plant_cords', self.traject_callback, 10)

        # Create a publisher to the delta_cord topic
        self.cord_publisher = self.create_publisher(
            Float32MultiArray, 'delta_cords', 10)

        self.delta_working_level = [-500, -625]
        self.plant_cord = [[]]

    def traject_callback(self, msg):
        # Log the received plant coordinates
        self.get_logger().info(f'Plant cord: {msg.data}')

        # Generate the planned coordinates
        self.planed_cord = self.__trajectory_plan(msg.data)

        # Construct Float32MultiArray message
        self.__publish_target_cords(self.planed_cord)

    def __publish_target_cords(self, targets: np.ndarray):
        """
        Publish the target coordinates to the plant_cord topic.
        """
        cord_array = Float32MultiArray()
        num_groups = len(targets)
        num_coords = len(targets[0]) if num_groups > 0 else 0

        # 設定 Layout 的維度
        cord_array.layout.dim = [
            MultiArrayDimension(label="group", size=num_groups,
                                stride=num_groups * num_coords),
            MultiArrayDimension(label="coordinate",
                                size=num_coords, stride=num_coords)
        ]
        cord_array.layout.data_offset = 0

    def __flatten_2d_array(self, array: list[list[float]]) -> list[float]:
        """
        Helper function to flatten a 2D list into a 1D list.

        :param array: A 2D list of floats
        :return: A flattened 1D list
        """
        return [float(item) for sublist in array for item in sublist]

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
