"""
建立一個節點，用來發布圖
形測試用的
發布Topic: /image_raw
"""
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

img_path = '/home/ced/ros2_ws/src/plant_detection/test_photo/IMG_4102.jpg'


class photoPubNode(Node):
    def __init__(self):
        super().__init__('photo_pub_node')
        timer_period = 0.5
        self.bridge = CvBridge()
        self.img_publisher = self.create_publisher(
            Image, 'image_raw', 10)
        self.timer = self.create_timer(timer_period, self.image_callback)

    def image_callback(self):
        img = cv2.imread(img_path)
        self.img_publisher.publish(
            self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))
        # self.get_logger().info(f"Publish image")


def main():
    rclpy.init()
    node = photoPubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
