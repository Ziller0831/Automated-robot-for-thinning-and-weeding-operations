from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',
        namespace='agric_robot',
        executable='realsense2_camera_node',
        name='D455',
        output='screen'

    )

    return LaunchDescription([realsense_node])
