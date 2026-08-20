import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction,
    OpaqueFunction, ExecuteProcess
)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("rc_car_description")

    use_rviz  = LaunchConfiguration("rviz").perform(context)
    use_slam_config = LaunchConfiguration("use_slam")
    use_slam  = use_slam_config.perform(context)
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
            # keep Gazebo truth odom on its own topic to avoid colliding with controller odom
            "/model/rc_car/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/model/rc_car/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            # model pose mapping for direct rc_car chassis pose
            "/model/rc_car/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
            # map Ignition dynamic_pose (per-entity poses) into ROS PoseArray for reliable chassis sampling
            "/world/cat_robotics_world/dynamic_pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V",
            "/world/cat_robotics_world/pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V",

            # 2D LiDAR — LaserScan
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # Camera image
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',


            '/depth_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/depth_camera/depth_image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
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

    # ── 6. Static TF: map -> odom (only when SLAM is disabled) ──
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam_config),
    )

    world_frame = 'odom'

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,

            # INPUT
            'odom0': '/model/rc_car/odometry',

            'odom0_config': [
                True,  True,  False,   # x, y, z
                False, False, True,    # roll, pitch, yaw
                True,  True,  False,   # vx, vy, vz
                False, False, True,    # vroll, vpitch, vyaw
                False, False, False
            ],
            'odom0_queue_size': 10,
            'odom0_differential': False,

            'imu0': '/imu/data',
            'imu0_config': [
                False, False, False,   # x, y, z
                True,  True,  True,    # roll, pitch, yaw
                False, False, False,   # vx, vy, vz
                False, False, False,   # vroll, vpitch, vyaw
                False, False, False
            ],
            'imu0_queue_size': 10,
            'imu0_differential': False,
            'imu0_remove_gravitational_acceleration': True,

            # FRAMES
            'base_link_frame': 'base_footprint',
            'odom_frame': 'odom',
            'world_frame': world_frame,

            # OUTPUT
            'publish_tf': False,
            'publish_acceleration': False,

            # SETTINGS
            'two_d_mode': True,
            'frequency': 50.0,
            'sensor_timeout': 0.1,
        }],
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=[
            '/cmd_vel',
            '/model/rc_car/cmd_vel'
        ],
        parameters=[{'use_sim_time': True}],
    )

    odom_tf_broadcaster = Node(
        package='robot_bringup',
        executable='broadcast_odom_tf.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    pose_odom_logger = Node(
        package='robot_bringup',
        executable='log_pose_odom.py',
        name='pose_odom_logger',
        output='screen',
        arguments=['--output', '/tmp/odom_pose_sync.csv'],
        parameters=[{'use_sim_time': True}],
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
        ekf_node,
        map_to_odom,
        cmd_vel_relay,
        odom_tf_broadcaster,
    ]
    if use_rviz == "true":
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz",  default_value="true",
            description="Launch RViz2"),
        DeclareLaunchArgument("use_slam", default_value="false",
            description="Disable static map->odom when using SLAM"),
        DeclareLaunchArgument("world", default_value="",
            description="Path to SDF world"),
        DeclareLaunchArgument("x",     default_value="-2.74",
            description="Spawn X"),
        DeclareLaunchArgument("y",     default_value="-0.46",
            description="Spawn Y"),
        DeclareLaunchArgument("z",     default_value="0",
            description="Spawn Z"),
        OpaqueFunction(function=launch_setup),
    ])
