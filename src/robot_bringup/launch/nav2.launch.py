import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_desc = get_package_share_directory('rc_car_description')
    pkg_bringup = get_package_share_directory('robot_bringup')

    # -------------------------
    # Launch arguments
    # -------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'slam.launch.py')
        ),
    )

    # -------------------------
    # Nav2 bringup
    # -------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(
            pkg_bringup,
            'config',
            'nav2_params.yaml')
        }.items()
    )

 



    return LaunchDescription([
        declare_use_sim_time,

        slam,

        nav2,
    ])