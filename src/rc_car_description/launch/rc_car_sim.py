import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction,
    OpaqueFunction, ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("rc_car_description")

    use_rviz  = LaunchConfiguration("rviz").perform(context)
    world_arg = LaunchConfiguration("world").perform(context)
    x_pos     = LaunchConfiguration("x").perform(context)
    y_pos     = LaunchConfiguration("y").perform(context)
    z_pos     = LaunchConfiguration("z").perform(context)

    world_path = world_arg if world_arg else \
        os.path.join(pkg, "worlds", "cat_robotics.world")

    xacro_file = os.path.join(pkg, "urdf", "rc_car.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )

    # ── 1. Robot State Publisher ───────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # ── 2. Gazebo ──────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "--verbose", "-r", world_path],
        output="screen",
    )

    # ── 3. Spawn robot (delayed 3s for Gazebo to start) ────────
    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", "robot_description",
                "-name",  "rc_car",
                "-x", x_pos,
                "-y", y_pos,
                "-z", z_pos,
            ],
            output="screen",
        )],
    )

    # ── 4. ROS <-> Gazebo bridge ───────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/ackermann_steering_controller/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/world/cat_robotics_world/pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V",

            # 3D LiDAR — PointCloud2
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # Camera image
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── 5. Controllers ─────────────────────────────────────────
    joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        )],
    )

    ackermann_controller = TimerAction(
        period=8.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["ackermann_steering_controller",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        )],
    )

    # ── 6. Static TF: map -> odom ──────────────────────────────
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,

            # INPUT
            'odom0': '/ackermann_steering_controller/odometry',

            'odom0_config': [
                True,  True,  False,   # x, y, z
                False, False, True,    # roll, pitch, yaw
                True,  True,  False,   # vx, vy, vz
                False, False, True,    # vroll, vpitch, vyaw
                False, False, False
            ],

            # FRAMES
            'base_link_frame': 'base_footprint',
            'odom_frame': 'odom',
            'world_frame': 'odom',

            # OUTPUT
            'publish_tf': True,

            # SETTINGS
            'two_d_mode': True,
            'frequency': 50.0,
        }],
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=[
            '/cmd_vel',
            '/ackermann_steering_controller/reference_unstamped'
        ],
    )

    # ── 8. RViz ────────────────────────────────────────────────
    rviz_config = os.path.join(pkg, "rviz", "rc_car.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    nodes = [
        robot_state_publisher,
        gazebo,
        spawn,
        bridge,
        joint_state_broadcaster,
        ackermann_controller,
        ekf_node,
        map_to_odom,
        cmd_vel_relay,
    ]
    if use_rviz == "true":
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz",  default_value="true",
            description="Launch RViz2"),
        DeclareLaunchArgument("world", default_value="",
            description="Path to SDF world"),
        DeclareLaunchArgument("x",     default_value="-2.74",
            description="Spawn X"),
        DeclareLaunchArgument("y",     default_value="-0.46",
            description="Spawn Y"),
        DeclareLaunchArgument("z",     default_value="0.1",
            description="Spawn Z"),
        OpaqueFunction(function=launch_setup),
    ])
