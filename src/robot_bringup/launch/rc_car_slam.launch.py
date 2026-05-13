import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'rc_car_sim.launch.py')
        ),
        launch_arguments={
            'rviz': rviz,
            'use_slam': 'true',
            'world': world,
            'x': x,
            'y': y,
            'z': z,
        }.items(),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(pkg_slam, 'config', 'mapper_params_online_async.yaml'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
            description='Launch RViz2'),
        DeclareLaunchArgument('world', default_value='',
            description='Path to SDF world'),
        DeclareLaunchArgument('x', default_value='-2.74',
            description='Spawn X'),
        DeclareLaunchArgument('y', default_value='-0.46',
            description='Spawn Y'),
        DeclareLaunchArgument('z', default_value='0.1',
            description='Spawn Z'),
        include_sim,
        slam_toolbox,
    ])
