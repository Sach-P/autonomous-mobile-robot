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

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_bringup, 'maps', 'map.yaml')
    )

    # -------------------------
    # Robot description
    # -------------------------
    xacro_file = os.path.join(pkg_desc, 'urdf', 'rc_car.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # -------------------------
    # Gazebo
    # -------------------------
    world = os.path.join(pkg_desc, 'worlds', 'cat_robotics.world')

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        output='screen'
    )

    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'rc_car',
                '-x', '-2.74', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )]
    )

    # -------------------------
    # Bridges
    # -------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/ackermann_steering_controller/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/world/cat_robotics_world/pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V",

            # 2D LiDAR — LaserScan
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # Camera image
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # -------------------------
    # Controllers
    # -------------------------
    joint_state_broadcaster = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
            output='screen'
        )]
    )

    ackermann_controller = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'ackermann_steering_controller',
                '--controller-manager', '/controller_manager',
                '--ros-args',
                '--remap',
                'ackermann_steering_controller/reference_unstamped:=/cmd_vel'
            ],
            output='screen'
        )]
    )

    # -------------------------
    # EKF
    # -------------------------
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            'odom0': '/ackermann_steering_controller/odometry',

            'odom0_config': [True, True, False,
                             False, False, True,
                             False, False, False,
                             False, False, True,
                             False, False, False],

            'publish_tf': True,

            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'odom',

            'two_d_mode': True,
        }]
    )

    # -------------------------
    # Map Server
    # -------------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map': map_yaml
        }]
    )

    # -------------------------
    # AMCL
    # -------------------------
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map'
        }]
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
            'map': map_yaml
        }.items()
    )

    # -------------------------
    # Static map->odom (temporary)
    # -------------------------
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # -------------------------
    # RViz
    # -------------------------
    # rviz_config = os.path.join(pkg_desc, "rviz", "rc_car.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # arguments=["-d", rviz_config],
        output="screen",
    )

    fake_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,

        robot_state_publisher,
        gazebo,
        spawn,
        bridge,

        joint_state_broadcaster,
        ackermann_controller,
        

        ekf,

        map_server,
        amcl,
        static_map_odom,
        fake_odom_tf,

        nav2,

        rviz
    ])