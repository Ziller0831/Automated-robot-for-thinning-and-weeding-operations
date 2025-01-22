from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    plant_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('plant_detection'),
             '/launch/plant_detection_launch.py'])
    )
    delta_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('delta_robot'),
             '/launch/delta_robot_launch.py'])
    )
    # realsense_node = Node(
    #     package='realsense2_camera',
    #     namespace='agric_robot',
    #     name='D455',
    #     executable='realsense2_camera_node',
    #     output='screen'
    # )

    launch_description = LaunchDescription([
        plant_detection_launch,
        delta_robot_launch
    ])

    return launch_description
