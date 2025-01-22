from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    publish_image_node = Node(
        package='plant_detection',
        executable='photo_pub',
        output='screen'
    )

    plant_detection_node = Node(
        package='plant_detection',
        executable='plant_detector',
        output='screen'
    )

    launch_description = LaunchDescription([
        publish_image_node,
        plant_detection_node
    ])

    return launch_description
