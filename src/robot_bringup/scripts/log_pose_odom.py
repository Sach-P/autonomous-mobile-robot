#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import rclpy
import traceback
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class PoseOdomLogger(Node):
    def __init__(self, output_path: str, pose_topic: str = '/model/rc_car/pose', pose_stamped_topic: str = '/model/rc_car/pose_stamped', odom_topic: str = '/model/rc_car/odometry'):
        super().__init__('pose_odom_logger')
        self.output_path = Path(output_path)
        self.odom_topic = odom_topic
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.last_odom = None
        self.last_pose = None
        self.latest_pose_stamp = None
        self.latest_odom_stamp = None

        # subscriptions will be created after parsing CLI args to avoid
        # create/destroy races with rclpy wait sets during startup.
        self.odom_sub = None
        self.pose_sub = None
        self.pose_stamped_sub = None

        # We also write a CSV row when we have both packets; this keeps the file compact and timestamped.
        self.timer = self.create_timer(0.05, self.flush)
        self.write_handle = open(self.output_path, 'w', newline='')
        self.writer = csv.writer(self.write_handle)
        self.writer.writerow([
            'odom_stamp_ns',
            'odom_x',
            'odom_y',
            'odom_yaw',
            'odom_az',
            'pose_stamp_ns',
            'pose_x',
            'pose_y',
            'pose_yaw',
            'dt_s',
            'yaw_err_rad',
        ])

    def _stamp_ns(self, msg):
        if hasattr(msg, 'header') and msg.header.stamp:
            t = msg.header.stamp
            return int(t.sec) * 1_000_000_000 + int(t.nanosec)
        return int(self.get_clock().now().nanoseconds)

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg
        self.latest_odom_stamp = self._stamp_ns(msg)

    def pose_cb(self, msg: Pose):
        self.last_pose = msg
        self.latest_pose_stamp = self._stamp_ns(msg)

    def pose_stamped_cb(self, msg: PoseStamped):
        self.last_pose = msg.pose
        self.latest_pose_stamp = self._stamp_ns(msg)

    def flush(self):
        if self.last_odom is None or self.last_pose is None:
            return

        odom = self.last_odom
        pose = self.last_pose
        odom_yaw = quat_to_yaw(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        pose_yaw = quat_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        odom_t_ns = self.latest_odom_stamp or self._stamp_ns(odom)
        pose_t_ns = self.latest_pose_stamp or self._stamp_ns(pose)
        dt_s = (pose_t_ns - odom_t_ns) / 1e9
        yaw_err = math.atan2(math.sin(pose_yaw - odom_yaw), math.cos(pose_yaw - odom_yaw))

        self.writer.writerow([
            odom_t_ns,
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom_yaw,
            odom.twist.twist.angular.z,
            pose_t_ns,
            pose.position.x,
            pose.position.y,
            pose_yaw,
            dt_s,
            yaw_err,
        ])
        self.write_handle.flush()

        # keep only most recent values for continuous logging.
        self.last_odom = None
        self.last_pose = None

    def shutdown(self):
        self.write_handle.close()


def main():
    parser = argparse.ArgumentParser(description='Log odom and rc_car pose with ROS timestamps to a CSV file.', add_help=True)
    parser.add_argument('--output', type=str, default='odom_pose_sync.csv', help='Output CSV file path')
    parser.add_argument('--pose-topic', type=str, default='/model/rc_car/pose', help='Pose topic (Pose)')
    parser.add_argument('--pose-stamped-topic', type=str, default='/model/rc_car/pose_stamped', help='PoseStamped topic')
    parser.add_argument('--odom-topic', type=str, default='/model/rc_car/odometry', help='Odometry topic')
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = PoseOdomLogger(args.output, pose_topic=args.pose_topic, pose_stamped_topic=args.pose_stamped_topic, odom_topic=args.odom_topic)
    # create subscriptions using provided CLI topics
    node.odom_sub = node.create_subscription(
        Odometry,
        args.odom_topic,
        node.odom_cb,
        qos_profile_sensor_data,
    )
    node.pose_sub = node.create_subscription(
        Pose,
        args.pose_topic,
        node.pose_cb,
        qos_profile_sensor_data,
    )
    node.pose_stamped_sub = node.create_subscription(
        PoseStamped,
        args.pose_stamped_topic,
        node.pose_stamped_cb,
        qos_profile_sensor_data,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # write full traceback for debugging
        with open('/tmp/logger.err', 'w') as f:
            f.write('Exception in log_pose_odom.py\n')
            traceback.print_exc(file=f)
        raise
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # ignore already-shutdown / double-shutdown races
            pass


if __name__ == '__main__':
    main()
