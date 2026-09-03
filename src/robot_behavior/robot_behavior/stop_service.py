import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses

class StopServer(Node):
    def __init__(self):
        super().__init__('stop_server')

        self._srv = self.create_service(Trigger, 'stop_robot', self.stop_robot_callback)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._nav_to_pose_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._nav_through_poses_client = self.create_client(CancelGoal, '/navigate_through_poses/_action/cancel_goal')


    def stop_robot_callback(self, request, response):
        self.get_logger().warn('STOP REQUEST RECIEVED')

        cancel_msg = CancelGoal.Request()

        if self._nav_to_pose_client.service_is_ready():
            self._nav_to_pose_client.call_async(cancel_msg)

        if self._nav_through_poses_client.service_is_ready():
            self._nav_through_poses_client.call_async(cancel_msg)


        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        self._cmd_vel_pub.publish(stop_msg)


        response.success = True
        response.message = 'Robot stopped and all Nav2 goals were cancelled'
        return response


def main():
    rclpy.init()
    node = StopServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()