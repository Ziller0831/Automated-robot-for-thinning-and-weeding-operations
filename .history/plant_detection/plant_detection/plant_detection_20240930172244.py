import os
import shutil
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from plantcv import plantcv as pcv
import csv


class ObjectDetector:
    def __init__(self, model_path, output_directory='/home/ced/Image_recognition_ws/output1', unlabeled_directory='/home/ced/Image_recognition_ws/output1'):
        self.model = YOLO(model_path)  # Load YOLO model
        self.output_directory = output_directory  # Output directory
        self.unlabeled_directory = unlabeled_directory  # Unlabeled directory
        os.makedirs(self.output_directory, exist_ok=True)
        os.makedirs(self.unlabeled_directory, exist_ok=True)
        self.colors = self._generate_colors()  # Generate colors for different labels

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 5)
        self.config.enable_stream(
            rs.stream.color, 1280, 720, rs.format.bgr8, 5)
        self.pipeline_profile = self.pipeline.start(self.config)

        # Get device
        device = self.pipeline_profile.get_device()
        self.depth_sensor = device.first_depth_sensor()
        self.color_sensor = device.first_color_sensor()

        # Set default exposure and laser power
        self.adjust_laser_power(360)
        self.adjust_exposure(25)

        # Get stream profile and camera intrinsics
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(
            profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = depth_profile.get_intrinsics()

        self.depth_fx = 638.028
        self.depth_fy = 638.028
        self.depth_cx = 644.003
        self.depth_cy = 361.52

    def adjust_exposure(self, exposure_value):
        """Set the exposure value for the camera."""
        if self.color_sensor.supports(rs.option.exposure):
            self.color_sensor.set_option(
                rs.option.enable_auto_exposure, 0)  # Disable auto exposure
            self.color_sensor.set_option(rs.option.exposure, exposure_value)
            print(f"Exposure set to: {exposure_value} microseconds")

    def adjust_laser_power(self, laser_power_value):
        """Set the laser power for the camera."""
        if self.depth_sensor.supports(rs.option.laser_power):
            self.depth_sensor.set_option(
                rs.option.laser_power, laser_power_value)
            print(f"Laser power set to: {laser_power_value}")

    def _generate_colors(self):
        # Predefine colors for different labels
        colors = [
            (255, 0, 0),    # Blue
            (0, 255, 0),    # Green
            (0, 0, 255),    # Red
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Yellow
        ]
        return colors

    def create_backup(self, image_path):
        backup_directory = "backup"
        try:
            shutil.copy(image_path, backup_directory)
            print(f"Backup created successfully: {backup_directory}")
        except Exception as e:
            print(f"Error creating backup: {e}")

    def detect_objects(self):
        # 从RealSense相机捕获一帧
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            print("无法从RealSense相机捕获帧。")
            return

        # 将彩色帧转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())

        # 保存捕获的帧作为备份
        image_path = os.path.join(
            self.unlabeled_directory, 'realsense_capture1.jpg')
        cv2.imwrite(image_path, color_image)
        print(f"捕获的图像已保存：{image_path}")

        self.create_backup(image_path)
        prediction = self.model.predict(image_path, save=True, stream=True)
        object_coordinates = self._extract_object_coordinates(prediction)
        if not object_coordinates:
            print(f"无法为 {image_path} 添加标注。未检测到任何物体。")
            return

        object_list = self._convert_to_list(object_coordinates)
        image = cv2.imread(image_path)
        new_points = self._extract_yolo_points(object_coordinates)
        object_list.extend(new_points)
        object_list.sort(key=lambda obj: (obj['class'] != 0, -obj['y']))
        csv_filename = self._save_to_csv(image_path, object_list)
        centroid_x_list, centroid_y_list, class_list = self._load_coordinates_from_csv(
            csv_filename)
        filtered_points, removed_points = self._filter_points(
            centroid_x_list, centroid_y_list)
        self._display_points(image, filtered_points, removed_points)

        # 将移除的点保存到CSV文件中
        removed_csv_filename = os.path.join(
            self.output_directory, "removed_points1.csv")
        self._save_removed_points_to_csv(removed_points, removed_csv_filename)

    def _extract_object_coordinates(self, prediction):
        object_coordinates = {}
        for r in prediction:
            if r:
                for idx, box in enumerate(r.boxes):
                    x, y, w, h = box.xywh[0].tolist()
                    cls = int(box.cls[0].tolist())
                    object_name = f"object_{idx + 1}"
                    centroid_x = int(x + w / 2)
                    centroid_y = int(y + h / 2)
                    object_coordinates[object_name] = {
                        "x": x, "y": y, "w": w, "h": h,
                        "class": cls, "centroid_x": centroid_x, "centroid_y": centroid_y
                    }
        return object_coordinates

    def _convert_to_list(self, object_coordinates):
        object_list = []
        for obj_name, obj_coord in object_coordinates.items():
            object_list.append({
                'Object Name': obj_name,
                'x': obj_coord['centroid_x'],
                'y': obj_coord['centroid_y'],
                'class': obj_coord['class']
            })
        return object_list

    def _extract_yolo_points(self, object_coordinates):
        new_points = []
        for obj_name, obj_coord in object_coordinates.items():
            new_points.append({
                'Object Name': obj_name,
                'x': obj_coord['centroid_x'],
                'y': obj_coord['centroid_y'],
                'class': obj_coord['class']
            })
        return new_points

    def _save_to_csv(self, image_path, object_list):
        csv_filename = os.path.join(self.output_directory, os.path.splitext(
            os.path.basename(image_path))[0] + '.csv')
        with open(csv_filename, 'w', newline='') as csvfile:
            fieldnames = ['Object Name', 'x', 'y', 'class']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            seen_points = set()  # Set to track seen points
            for obj in object_list:
                point = (obj['x'], obj['y'])
                if point not in seen_points:
                    writer.writerow(obj)
                    seen_points.add(point)
                else:
                    print(f"Duplicate point found and removed: {obj}")

        print(f"CSV file saved: {csv_filename}")
        return csv_filename

    def _load_coordinates_from_csv(self, csv_filename):
        centroid_x_list = []
        centroid_y_list = []
        class_list = []
        with open(csv_filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                centroid_x_list.append(float(row['x']))
                centroid_y_list.append(float(row['y']))
                class_list.append(int(row['class']))
        return centroid_x_list, centroid_y_list, class_list

    def _filter_points(self, centroid_x_list, centroid_y_list, radius=93):  # 158,93
        random_points = np.column_stack((centroid_x_list, centroid_y_list))
        sorting_methods = [
            lambda point: (point[0], point[1]),
            lambda point: (-point[0], point[1]),
            lambda point: (point[1], point[0]),
            lambda point: (-point[1], -point[1])
        ]
        max_points = 0
        best_filtered_points = None
        best_removed_points = None
        for sorting_method in sorting_methods:
            sorted_random_points = sorted(random_points, key=sorting_method)
            filtered_points = []
            removed_points = []
            for point in sorted_random_points:
                if all(np.linalg.norm(point - other_point) >= radius for other_point in filtered_points):
                    filtered_points.append(point)
                else:
                    removed_points.append(point)
            if len(filtered_points) > max_points:
                max_points = len(filtered_points)
                best_filtered_points = filtered_points
                print(filtered_points)
                best_removed_points = removed_points
                print(removed_points)
        return np.array(best_filtered_points), np.array(best_removed_points)

    def _is_valid_point(self, x, y):
        # Example criteria for filtering points
        return y > 100  # Keep points with y-coordinate greater than 100

    def _display_points(self, image, filtered_points, removed_points):
        for (x, y) in filtered_points:
            # Green for filtered points
            cv2.circle(image, (int(x), int(y)), 5, (0, 255, 0), -1)
        for (x, y) in removed_points:
            # Red for removed points
            cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)

        # Save the image with points to the output directory
        output_image_path = os.path.join(
            self.output_directory, 'points_display1.jpg')
        cv2.imwrite(output_image_path, image)
        print(f"Image with points saved: {output_image_path}")

        # Optionally display the image
        # cv2.imshow("Points Display1", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def _save_removed_points_to_csv(self, removed_points, removed_csv_filename):
        with open(removed_csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y'])
            writer.writerows(removed_points)
        print(f"Removed points saved to CSV file: {removed_csv_filename}")


# Create an instance of the ObjectDetector class
detector = ObjectDetector(
    model_path="/home/ced/Image_recognition_ws/datasets/model/train134/weights/best.pt")

# Run object detection and annotation with RealSense capture
detector.detect_objects()

# # Stop streaming and close the pipeline
# detector.stop_pipeline()
