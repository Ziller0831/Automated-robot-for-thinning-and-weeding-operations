"""
植栽辨識與疏苗算法主程式

Architecture:
class PlantDetectNode
|
+-- def __init__(self)
|
+-- def image_callback(self, msg)
|
+-- def __create_multiarray_layout(self, groups: int, coords_per_group: int)
|
+-- def __flatten_2d_array(self, array: list[list[float]]) -> list[float]
|
+-- def __generate_colors(self) -> list
|
+-- def __extract_obj_cords(self, yolo_results: list) -> dict
|
+-- def __segment_objects(self, img: np.ndarray, obj_cords: list) -> list
|
+-- def __thinning_algorithm(self, segment_objs: list, radius: float)
|
+-- def __result_display(self, img: np.ndarray, obj_cords: list, saved_plants: list, removed_plants: list, weeds: list)
"""

import cv2
import numpy as np
from ultralytics import YOLO
# from plantcv import plantcv as pcv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MODEL_PATH = "/home/ced/Image_recognition_ws/datasets/model/train134/weights/best.pt"


class PlantDetectNode(Node):
    def __init__(self):
        super().__init__('plant_detect_node')

        self.thinning_radius = 150
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)
        self.colors = self.__generate_colors()

        # self.img_subscriber = self.create_subscription(
        #     Image, '/agric_robot/D455/color/image_raw', self.image_callback, 10)

        # ! 測試用
        self.img_subscriber = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)

        self.cord_publisher = self.create_publisher(
            Float32MultiArray, 'plant_cord', 10)
        self.img_publisher = self.create_publisher(
            Image, 'plant_segmentation', 10)

    def image_callback(self, msg):
        self.get_logger().info(f"Processing...")
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        yolo_results = self.model.predict(image, save=False, stream=False)

        # @ 輸出YOLO辨識框
        # for result in yolo_results:
        #     for box in result.boxes:
        #         # 解析檢測框數據
        #         x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        #         cls = int(box.cls[0].tolist())
        #         conf = box.conf[0].tolist()
        #         label = f"{cls}: {conf:.2f}"

        #         # 繪製檢測框和標籤
        #         cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #         cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
        #                     0.5, (0, 255, 0), 2)
        # self.img_publisher.publish(
        #     self.bridge.cv2_to_imgmsg(image, "bgr8"))

        # @ 學彥整套流程
        obj_cords = self.__extract_obj_cords(yolo_results)
        if not obj_cords:
            self.get_logger().info(f"未檢測到任何植物")
            return

        segment_objs = self.__segment_objects(image, obj_cords)
        saved_plants, removed_plants, weeds = self.__thinning_algorithm(
            segment_objs, self.thinning_radius)

        result_img = self.__result_display(
            image, obj_cords, saved_plants, removed_plants, weeds)
        self.img_publisher.publish(
            self.bridge.cv2_to_imgmsg(result_img, "bgr8"))

        removed_targets = np.concatenate((weeds, removed_plants))

        print(removed_targets)

        # cord_array = Float32MultiArray()
        # cord_array.layout = self.__create_multiarray_layout(
        #     len(removed_plants),
        #     len(removed_plants[0])
        # )

    def __create_multiarray_layout(self, groups: int, coords_per_group: int):
        """
        Helper function to create the layout for Float32MultiArray.

        :param groups: Number of groups (outer dimension)
        :param coords_per_group: Number of coordinates per group (inner dimension)
        :return: A populated layout object
        """
        layout = Float32MultiArray._layout_type()
        layout.dim.append(MultiArrayDimension(label="group",
                                              size=groups,
                                              stride=groups * coords_per_group))
        layout.dim.append(MultiArrayDimension(label="coordinate",
                                              size=coords_per_group,
                                              stride=coords_per_group))
        layout.data_offset = 0
        return layout

    def __flatten_2d_array(self, array: list[list[float]]) -> list[float]:
        """
        Helper function to flatten a 2D list into a 1D list.

        :param array: A 2D list of floats
        :return: A flattened 1D list
        """
        return [float(item) for sublist in array for item in sublist]

    def __generate_colors(self) -> list:
        """
        Generate a list of colors for drawing bounding boxes.

        :return: A list of colors
        """
        colors = [
            (0, 255, 0),    # Green
            (0, 0, 255),    # Red
            (255, 0, 0)     # Blue
        ]
        return colors

    def __extract_obj_cords(self, yolo_results: list) -> dict:
        """
        Extract object coordinates from YOLO results.

        :param yolo_results: A list of YOLO results
        :return: A dictionary of object coordinates
        """
        obj_cords = {}
        for result in yolo_results:
            if result:
                for idx, box in enumerate(result.boxes):
                    x, y, w, h = box.xywh[0].tolist()
                    bx1, by1, bx2, by2 = box.xyxy[0].tolist()
                    cls = int(box.cls[0].tolist())

                    obj_name = f"object_{idx + 1}"
                    # centroid_x = int(x + w / 2)
                    # centroid_y = int(y + h / 2)

                    obj_cords[obj_name] = {
                        "x": x,
                        "y": y,
                        "w": w,
                        "h": h,
                        "bx1": bx1,
                        "by1": by1,
                        "bx2": bx2,
                        "by2": by2,
                        "conf": box.conf[0],
                        "class": cls
                    }
        return obj_cords

    def __segment_objects(self, img: np.ndarray, obj_cords: list) -> list:
        """
        Segment objects from the image.

        :param img: The input image
        :param obj_cords: A list of object coordinates
        :return: A list of segmented objects
        """
        segment_objs = []

        for obj_name, obj_cord in obj_cords.items():
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

            cls = obj_cord['class']

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
            # mask_img = cv2.bitwise_and(crop_img, crop_img, mask=fill_img)
            # mask_inv = cv2.bitwise_not(fill_img)
            # black_background_img = cv2.add(
            #     crop_img, np.zeros_like(crop_img), mask=mask_inv)

            contours, _ = cv2.findContours(
                fill_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

            if contours:
                max_contour = max(contours, key=cv2.contourArea)
                moment = cv2.moments(max_contour)
                if moment["m00"] != 0:
                    center_x = int(moment["m10"] / moment["m00"])
                    center_y = int(moment["m01"] / moment["m00"])
                    segment_objs.append({
                        'class': cls,
                        'x': x1 + center_x,
                        'y': y1 + center_y
                    })

        return segment_objs

    def __thinning_algorithm(self, segment_objs: list, radius: float):
        """
        Thinning algorithm to remove overlapped objects.

        :param segment_objs: A list of segmented objects
        :param radius: The radius for thinning
        :return: A tuple of saved plants, removed plants, and weeds
        """
        plants = []
        weeds = []
        for segment_obj in segment_objs:
            if segment_obj['class'] == 0:
                plants.append(segment_obj)
            elif segment_obj['class'] == 1:
                weeds.append([segment_obj['x'], segment_obj['y']])

        cx_list = []
        cy_list = []
        for plant in plants:
            cx_list.append(plant['x'])
            cy_list.append(plant['y'])

        random_points = np.column_stack((cx_list, cy_list))

        sorting_methods = [
            lambda point: (point[0], point[1]),
            lambda point: (-point[0], point[1]),
            lambda point: (point[1], point[0]),
            lambda point: (-point[1], -point[1])
        ]
        max_points = 0
        best_saved_plants = None
        best_removed_plants = None

        for sorting_method in sorting_methods:
            sorted_random_points = sorted(random_points, key=sorting_method)
            saved_plants = []
            removed_plants = []
            for point in sorted_random_points:
                if all(np.linalg.norm(point - other_point) >= radius for other_point in saved_plants):
                    saved_plants.append(point)
                else:
                    removed_plants.append(point)
            if len(saved_plants) > max_points:
                max_points = len(saved_plants)
                best_saved_plants = saved_plants
                best_removed_plants = removed_plants

            return np.array(best_saved_plants), np.array(best_removed_plants), np.array(weeds)

    def __result_display(self, img: np.ndarray, obj_cords: list, saved_plants: list, removed_plants: list, weeds: list):
        """
        Display the results on the image.

        :param img: The input image
        :param obj_cords: A list of object coordinates
        :param saved_plants: A list of saved plant coordinates
        :param removed_plants: A list of removed plant coordinates
        :param weeds: A list of weed coordinates
        :return: The image with results
        """
        for _, obj_cord in obj_cords.items():
            x = int(obj_cord['x'])
            y = int(obj_cord['y'])
            bx1 = int(obj_cord['bx1'])
            bx2 = int(obj_cord['bx2'])
            by1 = int(obj_cord['by1'])
            by2 = int(obj_cord['by2'])
            cls = obj_cord['class']

            color = self.colors[cls]

            cv2.rectangle(img, (bx1, by1), (bx2, by2), color, 2)
            if cls == 0:
                cv2.putText(img, f"C {obj_cord['conf']*100:.1f}%", (bx1, by1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
            elif cls == 1:
                cv2.putText(img, f"W {obj_cord['conf']*100:.1f}%",
                            (bx1, by1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

        for plant in saved_plants:
            cv2.circle(img, (plant[0], plant[1]), 5, self.colors[0], -1)
        for plant in removed_plants:
            cv2.circle(img, (plant[0], plant[1]), 5, self.colors[1], -1)
        for weed in weeds:
            cv2.circle(img, (weed[0], weed[1]), 5, self.colors[2], -1)

        return img


def main(args=None):
    rclpy.init(args=args)
    node = PlantDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
