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
        executable='publish_image',
        output='screen'
    )

    return LaunchDescription([])
