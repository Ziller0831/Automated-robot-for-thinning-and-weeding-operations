import os
import shutil
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from plantcv import plantcv as pcv
import csv

import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


MODEL_PATH = "/home/ced/Image_recognition_ws/datasets/model/train134/weights/best.pt"


class PlantDetectNode(Node):
    def __init__(self):
        super().__init__('plant_detect_node')

        self.bridge = CvBridge()
        self.detect_model = YOLO(MODEL_PATH)

        self.img_subscriber_ = self.create_subscription(
            Image, '/agric_robot/D455/color/image_raw', self.image_callback, 10)

        self.cord_publisher_ = self.create_publisher(
            Float32MultiArray, 'plant_cord', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        


    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        predictions = self.detect_model(image, save=True, stream=True)
        obj_cords = self.__extract_obj_cords(predictions)


    def timer_callback(self):
        plant_cord = self.get_cord()

        cord_array = Float32MultiArray()
        cord_array.layout.dim.append(MultiArrayDimension())
        cord_array.layout.dim.append(MultiArrayDimension())

        cord_array.layout.dim[0].label = "group"
        cord_array.layout.dim[0].size = len(plant_cord)
        cord_array.layout.dim[0].stride = len(
            plant_cord) * len(plant_cord[0])
        
        cord_array.layout.dim[1].label = "coordinate"
        cord_array.layout.dim[1].size = len(plant_cord[0])
        cord_array.layout.dim[1].stride = len(plant_cord[0])
        
        cord_array.layout.data_offset = 0

        flat_array = [
            float(item) for sublist in plant_cord for item in sublist]
        cord_array.data = flat_array

        self.publisher_.publish(cord_array)

    def get_cord(self):
        plant_cord = [[100, 100, -550]]
        return plant_cord

    def __extract_obj_cords(self, predictions):
        obj_cords = []
        for prediction in predictions:
            if prediction:
                for idx, box in enumerate(prediction.boxes):
                    x, y, w, h = box.xywh[0].tolist()

                    cls = int(box.cls[0].tolist())

                    obj_name = f"object_{idx + 1}"
                    centroid_x = int(x + w / 2)
                    centroid_y = int(y + h / 2)

                    obj_cords[obj_name] = {
                        "x": x, "y": y, "w": w, "h": h,
                        "class": cls, 
                        "centroid_x": centroid_x, 
                        "centroid_y": centroid_y
                    }
        return obj_cords

    


def main(args=None):
    rclpy.init(args=args)
    node = PlantDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
