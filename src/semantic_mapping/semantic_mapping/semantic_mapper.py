import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data
from message_filters import ApproximateTimeSynchronizer, Subscriber
import torch
from transformers import (
    AutoProcessor,
    BitsAndBytesConfig,
    Qwen3VLForConditionalGeneration,
)
from PIL import Image as PILImage
from concurrent.futures import ThreadPoolExecutor
import time
import json
import numpy as np
from uuid import uuid4
from pathlib import Path


MODEL = "Qwen/Qwen3-VL-2B-Instruct"

PROMPT = """
Identify up to 3 distinct stationary objects visible in this image that a
person might want to navigate to.

Return only a JSON array. For each object, provide a short description
and a bounding box in this format:

[
  {
    "description": "description of object",
    "bbox": [xmin, ymin, xmax, ymax]
  }
]

Use integer coordinates on a 0 to 1000 scale, where (0, 0) is the
top-left and (1000, 1000) is the bottom-right of the image.
Return [] if there are no identifiable objects.
"""

class SemanticMapper(Node):
    def __init__(self):
        super().__init__('semantic_mapper')

        self._image_sub = Subscriber(self, Image, '/camera/image_raw', qos_profile_sensor_data)
        self._depth_sub = Subscriber(self, PointCloud2, '/depth_camera/depth_image/points', qos_profile_sensor_data)

        self._sync = ApproximateTimeSynchronizer(
            [self._image_sub, self._depth_sub],
            queue_size=10,
            slop=0.05,
        )

        self._sync.registerCallback(self.image_pair_callback)

        self._image_pub = self.create_publisher(Image, '/semantic_mapper/debug_image', 10)
        
        
        self._processor = AutoProcessor.from_pretrained(MODEL)
        quantization_config = BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16,
        )
        self._model = Qwen3VLForConditionalGeneration.from_pretrained(
            MODEL,
            dtype=torch.float16,
            quantization_config=quantization_config,
            device_map="auto",
        )
        self._model.eval()
        # self.get_logger().info(f'Device map: {self._model.hf_device_map}')
        devices = {str(param.device) for param in self._model.parameters()}
        self.get_logger().info(f'Model parameter devices: {devices}')
        self._cv_bridge = CvBridge()

        self._latest_image = None
        self._latest_cloud = None
        self._future = None
        self._inference_image_msg = None
        self._inference_points = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._worker = ThreadPoolExecutor(max_workers=1)

        self._timer = self.create_timer(0.25, self.process_image)

        self._map_id = 'my_world'

        self._observations_path = Path.home() / '.ros' / 'semantic_observations.jsonl'
        self._observations_path.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Saving observations to {self._observations_path}')


    def image_pair_callback(self, image_msg: Image, depth_msg: PointCloud2):

        self._latest_image = image_msg
        self._latest_cloud = depth_msg

    def process_image(self):
        if self._future is not None:
            if not self._future.done():
                return

            try:
                description = self._future.result()
                self.get_logger().info(f'Objects:\n{description}')
                self.draw_detections(self._inference_image_msg, self._inference_points, description)
            except Exception as exc:
                self.get_logger().error(f'Inference failed: {exc}')
            finally:
                self._future = None
                self._inference_image_msg = None
                self._inference_points = None


        if self._latest_image is None or self._latest_cloud is None:
            return


        image_msg = self._latest_image
        depth_msg = self._latest_cloud
        self._latest_image = None
        self._latest_cloud = None

        try:
            rgb_image = self._cv_bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='rgb8',
            )
        except CvBridgeError as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        image = PILImage.fromarray(rgb_image)

        try:
            tf_camera_map = self._tf_buffer.lookup_transform('map', depth_msg.header.frame_id, rclpy.time.Time.from_msg(depth_msg.header.stamp))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            self.get_logger().warning(f'Cannot get camera transform')
            return


        points = point_cloud2.read_points_numpy(depth_msg, ['x', 'y', 'z'], skip_nans=False)
        points = points.reshape(depth_msg.height, depth_msg.width, 3)

        if len(points) <= 0:
            self.get_logger().warning(f'No depth points')
            return

        self._inference_points = self.transform_points(points, tf_camera_map)
        self._inference_image_msg = image_msg
        self._future = self._worker.submit(self.describe_image, image)

    def describe_image(self, image):
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": PROMPT},
                ],
            }
        ]

        inputs = self._processor.apply_chat_template(
            messages,
            tokenize=True,
            add_generation_prompt=True,
            return_dict=True,
            return_tensors="pt",
        )
        inputs = inputs.to(self._model.device)
        self.get_logger().info('Gennerating response')
        with torch.inference_mode():
            output_ids = self._model.generate(
                **inputs,
                max_new_tokens=256,
            )

        self.get_logger().info('Generation finished')
        generated_ids = output_ids[:, inputs.input_ids.shape[1]:]

        return self._processor.batch_decode(
            generated_ids,
            skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )[0]

    def draw_detections(self, image_msg, depth_points, response):
        try:
            detections = json.loads(response)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f'Qwen did not return valid JSON: {exc}. Response: {response}'
            )
            return

        if not isinstance(detections, list):
            self.get_logger().error('Expected Qwen response to be a JSON array')
            return

        try:
            image = self._cv_bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8',
            ).copy()
        except CvBridgeError as exc:
            self.get_logger().error(f'Could not convert debug image: {exc}')
            return

        height, width = image.shape[:2]

        for detection in detections:
            if not isinstance(detection, dict):
                self.get_logger().warning('Skipping malformed detection')
                continue

            description = detection.get('description')
            box = detection.get('bbox')

            if (
                not isinstance(description, str)
                or not isinstance(box, list)
                or len(box) != 4
                or not all(type(value) in (int, float) for value in box)
                or not all(np.isfinite(value) for value in box)
            ):
                self.get_logger().warning(f'Skipping invalid detection: {detection}')
                continue

            xmin, ymin, xmax, ymax = box

            if not (0 <= xmin < xmax <= 1000 and 0 <= ymin < ymax <= 1000):
                self.get_logger().warning(f'Skipping invalid bounding box: {box}')
                continue

            # Convert to real pixel locations
            x1 = round(xmin * (width - 1) / 1000)
            y1 = round(ymin * (height - 1) / 1000)
            x2 = round(xmax * (width - 1) / 1000)
            y2 = round(ymax * (height - 1) / 1000)

            region = depth_points[y1:y2, x1:x2]
            valid_mask = np.isfinite(region).all(axis=2)
            valid_points = region[valid_mask]

            if valid_points.size == 0:
                self.get_logger().warning(f'No valid cloud points for {description}')
                continue

            position = np.median(valid_points, axis=0)
            pos_str = "[" + str(round(position[0], 2)) + ", " + str(round(position[1], 2)) + ", " + str(round(position[2], 2)) + "]" 
            
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            self.put_text_with_background(
                image,
                description,
                (x1, max(20, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
            self.put_text_with_background(
                image, 
                pos_str,
                ((x1 + x2) // 2, (y1 + y2) // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )

            self.save_observations(description, position, image_msg)

        debug_msg = self._cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug_msg.header = image_msg.header
        self._image_pub.publish(debug_msg)

    def save_observations(self, description, position, image_msg):
        record = {
            'observation_id': uuid4().hex,
            'description': description,
            'position': [float(value) for value in position],
            'frame': 'map',
            'map_id': self._map_id,
            'image_stamp': {'sec' : image_msg.header.stamp.sec, 'nanosec': image_msg.header.stamp.nanosec,},
        }

        try:
            with self._observations_path.open('a', encoding='utf-8') as file:
                file.write(json.dumps(record, allow_nan=False) + '\n')
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Could not save observation: {exc}')
            return


    def put_text_with_background(
        self, image, text, origin, font, font_scale, text_color, thickness
    ):
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, font_scale, thickness
        )
        padding = 4
        left = max(0, origin[0] - padding)
        top = max(0, origin[1] - text_height - padding)
        right = min(image.shape[1], origin[0] + text_width + padding)
        bottom = min(image.shape[0], origin[1] + baseline + padding)

        background = image[top:bottom, left:right].copy()
        cv2.rectangle(
            background,
            (0, 0),
            (right - left, bottom - top),
            (0, 0, 0),
            cv2.FILLED,
        )
        cv2.addWeighted(
            background,
            0.45,
            image[top:bottom, left:right],
            0.55,
            0,
            image[top:bottom, left:right],
        )
        cv2.putText(image, text, origin, font, font_scale, text_color, thickness)


    def transform_points(self, points, tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        rotation_matrix = self.quaternion_to_matrix([q.x, q.y, q.z, q.w])
        translation_vector = np.array([t.x, t.y, t.z])
        return np.dot(points, rotation_matrix.T) + translation_vector


    def quaternion_to_matrix(self, q):
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x**2 + y**2)]                                                                                                                                                                                                                                                                                                                                                                                                    
        ])

    def shutdown_worker(self):
        self._worker.shutdown(wait=True)

def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SemanticMapper()
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