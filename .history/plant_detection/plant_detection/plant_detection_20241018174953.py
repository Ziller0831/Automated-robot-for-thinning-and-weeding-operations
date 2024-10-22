import cv2
import numpy as np
from ultralytics import YOLO
# from plantcv import plantcv as pcv

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
        # self.timer = self.create_timer(0.5, self.timer_callback)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        predictions = self.detect_model(image, save=True, stream=True)
        self.get_logger().info(f"{predictions}")
        yolo_obj_cords = self.__extract_obj_cords(predictions)

        if not yolo_obj_cords:
            self.get_logger().info(f"未檢測到任何植物")
            return

        obj_list = self.__cords_convert_list(yolo_obj_cords)
        new_points = self.__segment_objects(image, yolo_obj_cords)
        obj_list.extend(new_points)
        obj_list.sort(key=lambda obj: (obj['class'] != 0, -obj['y']))

        self.get_logger().info(f"Object list: {obj_list}")

    # def timer_callback(self):
    #     plant_cord = self.get_cord()

    #     cord_array = Float32MultiArray()
    #     cord_array.layout.dim.append(MultiArrayDimension())
    #     cord_array.layout.dim.append(MultiArrayDimension())

    #     cord_array.layout.dim[0].label = "group"
    #     cord_array.layout.dim[0].size = len(plant_cord)
    #     cord_array.layout.dim[0].stride = len(
    #         plant_cord) * len(plant_cord[0])

    #     cord_array.layout.dim[1].label = "coordinate"
    #     cord_array.layout.dim[1].size = len(plant_cord[0])
    #     cord_array.layout.dim[1].stride = len(plant_cord[0])

    #     cord_array.layout.data_offset = 0

    #     flat_array = [
    #         float(item) for sublist in plant_cord for item in sublist]
    #     cord_array.data = flat_array

    #     self.publisher_.publish(cord_array)

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

    def __cords_convert_list(self, obj_cords):
        obj_list = []
        for obj_name, obj_cord in obj_cords.items():
            obj_list.append({
                "Object name": obj_name,
                "x": obj_cord["x"],
                "y": obj_cord["y"],
                "class": obj_cord["class"],
            })
        return obj_list

    def __segment_objects(self, img, yolo_obj_cord):
        segment_obj_points = []
        color = (0, 255, 0)
        for obj_name, obj_cord in yolo_obj_cord.items():
            x = int(obj_cord["x"])
            y = int(obj_cord["y"])
            w = int(obj_cord["w"])
            h = int(obj_cord["h"])

            x1 = int(x - w / 2)
            y1 = int(y - h / 2)
            x2 = int(x + w / 2)
            y2 = int(y + h / 2)

            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(img.shape[1], x2)
            y2 = min(img.shape[0], y2)

            # * 從原始影像中切割出植物的部分
            crop_img = img[y1:y2, x1:x2]

            # * La*b* 二值化
            # @ 使用plantCV
            # alpha_channel_img  = pcv.rgb2gray_lab(rgb_img=crop_img, channel='a')
            # binary_img = pcv.threshold.binary(
            #     gray_img=alpha_channel_img , threshold=123, object_type='dark')
            # fill_img = pcv.fill.holes(bin_img=binary_img)
            # mask_img = pcv.apply_mask(
            #     img=crop_img, mask=fill_img, mask_color='black')

            # @ 使用OpenCV
            lab_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2LAB)
            alpha_channel_img = lab_img[:, :, 1]

            _, binary_img = cv2.threshold(
                alpha_channel_img, 123, 255, cv2.THRESH_BINARY_INV)

            kernel = np.ones((3, 3), np.uint8)
            fill_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)

            mask_img = cv2.bitwise_and(crop_img, crop_img, mask=fill_img)
            mask_inv = cv2.bitwise_not(fill_img)
            black_background_img = cv2.add(
                crop_img, np.zeros_like(crop_img), mask=mask_inv)

            contours, _ = cv2.findContours(
                fill_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                moment = cv2.moments(max_contour)
                if moment["m00"] != 0:
                    cX = int(moment["m10"] / moment["m00"])
                    cY = int(moment["m01"] / moment["m00"])
                    # cv2.circle(mask_img, (cX, cY), 5, color, -1)
                    # cv2.circle(img, (x1 + cX, y1 + cY), 5, color, -1)
                    segment_obj_points.append({
                        'Object Name': f"{obj_name}_contour",
                        'x': x1 + cX,
                        'y': y1 + cY,
                        'class': 0
                    })

        return segment_obj_points

    # def __filter_duplicate_points(self, obj_list):


def main(args=None):
    rclpy.init(args=args)
    node = PlantDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
