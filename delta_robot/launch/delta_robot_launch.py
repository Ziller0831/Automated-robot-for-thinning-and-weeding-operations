from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    delta_robot_api = Node(
        package='delta_robot',
        executable='delta_robot_api',
        output='screen'
    )

    trajectory_plan_node = Node(
        package='delta_robot',
        executable='trajectory_plan',
        output='screen'
    )

    launch_description = LaunchDescription([
        delta_robot_api,
        trajectory_plan_node
    ])

    return launch_description
