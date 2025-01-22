from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # realsense_node = Node(
    #     package='realsense2_camera',
    #     namespace='agric_robot',
    #     name='D455',
    #     executable='realsense2_camera_node',
    #     output='screen'
    # )
    publish_image_node = Node(
        package='plant_detection',
        executable='photo_pub',
        output='screen'
    )

    plant_detection_node = Node(
        package='plant_detection',
        executable='plant_detection',
        output='screen'
    )

    launch_description = LaunchDescription([
        publish_image_node,
        plant_detection_node
    ])

    return launch_description
