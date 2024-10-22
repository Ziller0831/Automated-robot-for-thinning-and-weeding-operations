import rclpy
import threading
from pandas import read_csv
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

coordinate = Float32MultiArray()

csv_path = "/home/ced/Image_recognition_ws/output/keypoints_3d.csv"


class PlantCordSub(Node):
    def __init__(self):
        super().__init__('plant_cord_sub')
        self.subscriber = self.create_subscription(
            Float32MultiArray, 'plant_detect_cord', self.listener_callback, 10)
        
        global recognition_cord

    def listener_callback(self, msg):
        groups_num = coordinate.layout.dim[0].size
        coord_num = coordinate.layout.dim[1].size
        self.recognition_result = msg.data
        ## TODO: 要先確定上面的訊息已什麼形式怎麼傳
        # self.get_logger().info(f'I heard: {self.recognition_result}')


class PlanCordPublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')

        self.publisher = self.create_publisher(
            Float32MultiArray, 'delta_coordinate', 10)
        
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.delta_working_level = [-500, -625]

    def timer_callback(self):
        planed_cord = self.trajectory_plan()

        cord_array = Float32MultiArray()
        cord_array.layout.dim.append(MultiArrayDimension())
        cord_array.layout.dim.append(MultiArrayDimension())

        cord_array.layout.dim[0].label = "group"
        cord_array.layout.dim[1].label = "coordinate"
        cord_array.layout.dim[0].size = len(planed_cord)
        cord_array.layout.dim[1].size = len(planed_cord[0])
        cord_array.layout.dim[0].stride = len(planed_cord) * len(planed_cord[0])
        cord_array.layout.dim[1].stride = len(planed_cord[0])

        cord_array.layout.data_offset = 0

        flat_array = [
            float(item) for sublist in planed_cord for item in sublist]
        cord_array.data = flat_array

        self.publisher.publish(cord_array)

    def trajectory_plan(self):
        raw_cord = csv_input(csv_path)

        planed_cord = [[0, 0, 0], [0, 0, -450]]

        for row in raw_cord:
            for z in self.delta_working_level:
                planing_cord = row[:2] + [z]
                planed_cord.append(planing_cord)

        planed_cord.append([0, 0, -450])

        # print(planed_cord)

        return planed_cord


def main(args=None):
    rclpy.init(args=args)
    plan_cord_publisher = PlanCordPublisher()
    rclpy.spin(plan_cord_publisher)
    plan_cord_publisher.destroy_node()
    rclpy.shutdown()


