from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',
        namespace='agriculture_robot',
        executable='realsense2_camera_node'
        name='D455')

    return LaunchDescription([realsense_node])
