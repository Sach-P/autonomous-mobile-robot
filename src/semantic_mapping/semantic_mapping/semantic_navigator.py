import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
import sys
import json
from pathlib import Path
import math
from geometry_msgs.msg import PoseStamped
import json
import requests


OLLAMA_URL = 'http://localhost:11434/api/chat'
MODEL = 'qwen2.5:3b-instruct'


SYSTEM = '''
        You are a guide robot. You get a user request and a JSON list of 
        known objects in the environment, each with an id and a description. Reply with JSON only: 
        
        
        {
            "observation_id": "<id from the list>",
            "response": "<your response here>"
            
        } 
        
        or 
        
        {
            "observation_id": null,
            "response": "<your response here>"
        } 
        
        if nothing fits. 

        Infer what the user means. If there are multiple possible matches, choose the most relevant one.
        Never invent ids.
        
        '''


class SemanticNavigator(Node):

    def __init__(self):
        super().__init__('semantic_navigator')
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
                
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._test_timer = self.create_timer(2.0, self._test_goal)

    def _test_goal(self):
        self._test_timer.cancel()     
        observation_id = self.select_object('Take me to something green')
        if observation_id is None:
            self.get_logger().warn("No valid observation selected")
            return
        record = self.load_observation(observation_id)
        self.get_logger().info(f'Loaded observation: {record}')
        goal = self.make_approach_pose(record['position'])
        if goal is None:
            self.get_logger().warn('TF not ready')
            return
        self.send_goal([goal])


    def select_object(self, request):

        path = Path.home() / '.ros' / 'semantic_observations.jsonl'
        
        observations = []
        with path.open(encoding='utf-8') as file:
            for line in file:
                record = json.loads(line)
                observations.append(record)  

        if not observations:
            raise ValueError('No observations found')

        listing = '\n'.join(f'{o["observation_id"]}: {o["description"]}' for o in observations)
        
        response = requests.post(OLLAMA_URL, json={
            'model': MODEL,
            'stream': False,
            'format': 'json',            
            'options': {'temperature': 0},
            'messages': [
                {'role': 'system', 'content': SYSTEM},
                {'role': 'user', 'content': f'Objects:\n{listing}\n\nRequest: {request}'},
            ],
        }, timeout=60)

        response.raise_for_status()
        chosen = json.loads(response.json()['message']['content']).get('observation_id')
        self.get_logger().info(f'Observation: {json.loads(response.json()["message"]["content"])}')
        valid = {o['observation_id'] for o in observations}
        return chosen if chosen in valid else None  


    def load_observation(self, observation_id):
        path = Path.home() / '.ros' / 'semantic_observations.jsonl'

        matches = []
        with path.open(encoding='utf-8') as file:
            for line in file:
                record = json.loads(line)
                if record.get('observation_id') == observation_id:
                    matches.append(record)

        if len(matches) != 1:
            raise ValueError(
                f'Expected one observation with ID {observation_id}, found {len(matches)}'
            )

        record = matches[0]
        if record.get('map_id') != 'my_world' or record.get('frame') != 'map':
            raise ValueError('Observation is not in the active map frame')

        return record

    def make_approach_pose(self, object_position):
        object_x, object_y = object_position[:2]

        try:
            tf_robot_map = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            self.get_logger().warning(f'Cannot get robot transform')
            return

        
        robot_x = tf_robot_map.transform.translation.x
        robot_y = tf_robot_map.transform.translation.y

        dx = robot_x - object_x
        dy = robot_y - object_y
        distance = math.hypot(dx, dy)

        if distance < 1e-3:
            raise ValueError('Robot is too close to the object to choose an approach side')

        # distance away from object
        standoff = 0.8
        goal_x = object_x + standoff * dx / distance
        goal_y = object_y + standoff * dy / distance

        # Face the object from the approach pose.
        yaw = math.atan2(object_y - goal_y, object_x - goal_x)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal


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

        self.get_logger().info('Goal Accepted')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Reached destination!')
        else:
            self.get_logger().warn(f'Navigation ended with status code {status}')



def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SemanticNavigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown_worker()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()