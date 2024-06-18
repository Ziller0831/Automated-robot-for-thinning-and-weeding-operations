import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

test_2d_coordinate = [[0, 0, 0],
                      [0, 0, -450],
                      [100, 100, -550],
                      [100, -100, -550],
                      [-100, -100, -550],
                      [-100, 100, -550],
                      [100, 100, -550],
                      [0, 0, -550],
                      [0, 0, -450]]


# class ImageRecognitionCoordSub(Node):
#     def __init__(self):
#         super().__init__('image_recognition_coord_sub')
#         self.subscription = self.create_subscription(
#             Float32MultiArray, 'image_recognition_coordinate', self.listener_callback, 10)
#         self.recognition_result = None

#     def listener_callback(self, msg):
#         self.recognition_result = msg.data
#         # self.get_logger().info(f'I heard: {self.recognition_result}')


class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 'Delta_coordinate', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        coord_array = Float32MultiArray()
        coord_array.layout.dim.append(MultiArrayDimension())
        coord_array.layout.dim.append(MultiArrayDimension())
        coord_array.layout.dim[0].label = "group"
        coord_array.layout.dim[0].size = len(test_2d_coordinate)
        coord_array.layout.dim[0].stride = len(
            test_2d_coordinate) * len(test_2d_coordinate[0])
        coord_array.layout.dim[1].label = "coordinate"
        coord_array.layout.dim[1].size = len(test_2d_coordinate[0])
        coord_array.layout.dim[1].stride = len(test_2d_coordinate[0])
        coord_array.layout.data_offset = 0

        flat_array = [
            float(item) for sublist in test_2d_coordinate for item in sublist]
        coord_array.data = flat_array

        self.publisher_.publish(coord_array)
        # self.get_logger().info(f'Publishing: {coord_array.data}')

    # def trajectory_plan(self):
    #     pass


def main(args=None):
    rclpy.init(args=args)
    coordinate_publisher = CoordinatePublisher()
    rclpy.spin(coordinate_publisher)
    coordinate_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
