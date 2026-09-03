import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import sys

class GoalClient(Node):

    def __init__(self):
        super().__init__('goal_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')


    def send_goal(self, poses: list[PoseStamped]):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f} m | ', throttle_duration_sec=1.0)
            # f'Number of recovery executions: {feedback.number_of_recoveries}'

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal Rejected')
            return

        self.get_logger().info('Goal Accetped')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Reached all poses!')
        else:
            self.get_logger().warn(f'Navigation ended with status code {status}')




def main(args=None):

    file = 'goals.txt'
    if len(sys.argv) > 1:
        file = sys.argv[1]
    poses = []
    x = 0.0
    y = 0.0
    z = 0.0
    ox = 0.0
    oy = 0.0
    oz = 0.0
    ow = 0.0
    with open(file, 'r') as fh:
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
    node = GoalClient()
    node.send_goal(poses)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()