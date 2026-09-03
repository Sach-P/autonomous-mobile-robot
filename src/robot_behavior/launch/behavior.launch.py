from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("robot_behavior")

    stop_service = Node(
        package='robot_behavior',
        executable= 'stop_service',
        name='stop_service',
        output='screen'
    )

    keyboard_stop_client = Node(
        package='robot_behavior',
        executable= 'keyboard_stop_client',
        name='keyboard_stop_client',
        output='screen'
    )

    goal_client = Node(
        package='robot_behavior',
        executable= 'goal_client',
        name='goal_client',
        output='screen',
        arguments=['goals.txt']
    )

    return LaunchDescription(
        [
            stop_service,
            keyboard_stop_client,
            goal_client
        ]
    )