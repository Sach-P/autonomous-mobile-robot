#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped

class GoalPosePublisher(Node):

    def __init__(self, poses: list[PoseStamped]):
        super().__init__('goal_pose_publisher')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_status = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_status_callback, 10)
        self.poses = poses
        self.send_goal(self.poses[0])
        self.idx = 1

    def send_goal(self, pose: PoseStamped):
        self.goal_pub.publish(pose)
        self.get_logger().info("New Goal Published")


    def goal_status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            return

        status_code = msg.status_list[-1].status

        if status_code == 1:
            return
        if status_code == 4:
            self.get_logger().info("Reached Next Goal")
            self.send_goal(self.poses[self.idx])
            self.idx += 1
            if self.idx >= len(self.poses):
                self.idx = 0
        if status_code == 5 or status_code == 6:
            self.get_logger().warn('Navigation Stopped')



def main():
    poses = []
    x = 0.0
    y = 0.0
    z = 0.0
    ox = 0.0
    oy = 0.0
    oz = 0.0
    ow = 0.0
    with open('goals.txt', 'r') as fh:
        while True:
            line = fh.readline()
            if not line:
                break
            line = line.rstrip('\n')
            if line == '  position:':
                line = fh.readline()
                x = float(line[7:])
                line = fh.readline()
                y = float(line[7:])
                line = fh.readline()
                z = float(line[7:])
                    
            if line == '  orientation:':
                line = fh.readline()
                ox = float(line[7:])
                line = fh.readline()
                oy = float(line[7:])
                line = fh.readline()
                oz = float(line[7:])
                line = fh.readline()
                ow = float(line[7:])                

            if line == '---':
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.x = ox
                pose.pose.orientation.y = oy
                pose.pose.orientation.z = oz
                pose.pose.orientation.w = ow
                poses.append(pose)

    rclpy.init()
    node = GoalPosePublisher(poses)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()