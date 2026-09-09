import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
import tf2_sensor_msgs
import numpy as np


class PitDetector(Node):
    def __init__(self):
        super().__init__('pit_detector')

        self._depth_sub = self.create_subscription(PointCloud2, '/depth_camera/depth_image/points', self.detection_callback, 10)

        self._cloud_pub = self.create_publisher(PointCloud2, '/pit_detector/points', 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.floor_height_tolerance = 0.025
        self.edge_jump_threshold = 0.15
        self.max_lookahead = 3
        self.max_lateral = 1
        self.grid_resolution = 0.025
        self.min_edge_length = 0.05 / self.grid_resolution

    def detection_callback(self, msg: PointCloud2):

        try:
            tf_camera_base = self._tf_buffer.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time())
            # tf_base_odom = self._tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return

        points = point_cloud2.read_points_numpy(msg, ['x', 'y', 'z'], skip_nans=False)
        points = points.reshape(msg.height, msg.width, 3)

        if len(points) <= 0:
                return

        valid = np.isfinite(points).all(axis=2)

        if not valid.any():
            return

        

        t = tf_camera_base.transform.translation
        q = tf_camera_base.transform.rotation
        rotation_matrix = self.quaternion_to_matrix([q.x, q.y, q.z, q.w])
        translation_vector = np.array([t.x, t.y, t.z])

        points_robot = np.dot(points, rotation_matrix.T) + translation_vector
    
        # filter out points above the floor
        # floor_points = points_robot[points_robot[:, 2] < 0]

        # # filter out points far away where pit detection is not relevant

        # floor_points = floor_points[floor_points[:, 0] < 3]

        edge_points = self.find_pit_edges(points_robot, valid)

        if edge_points.size == 0:
            return

        # t = tf_base_odom.transform.translation
        # q = tf_base_odom.transform.rotation
        # rotation_matrix = self.quaternion_to_matrix([q.x, q.y, q.z, q.w])
        # translation_vector = np.array([t.x, t.y, t.z])

        # odom_points = np.dot(edge_points, rotation_matrix.T) + translation_vector

        

        edge_cloud = point_cloud2.create_cloud_xyz32(
            msg.header,
            edge_points.tolist(),
        )

        edge_cloud.header.frame_id = 'base_footprint'
        edge_cloud.header.stamp = msg.header.stamp

        self._cloud_pub.publish(edge_cloud)

    def find_pit_edges(self, points, valid):
        height, width, _ = points.shape

        #compare each pixel to the pixel underneath
        upper = points[:-1, :, :]
        lower = points[1:, :, :]

        #make sure both pixels are valid
        pair_valid = valid[:-1, :] & valid[1:, :]

        #check how far the jump between pixels is by the x value
        forward_jump = upper[:, :, 0] - lower[:, :, 0]

        #make sure lower pixel is part of the floor
        lower_is_floor = np.abs(lower[:, :, 2]) < 0.075

        jump_mask = pair_valid & (
            forward_jump > self.edge_jump_threshold
        )

        range_mask = (
            (lower[:, :, 0] < self.max_lookahead) &
            (np.abs(lower[:, :, 1]) < self.max_lateral)
        )

        candidate = (
            jump_mask |
            (valid[1:, :] & ~valid[:-1, :])
        ) & range_mask & lower_is_floor

        self.get_logger().info(
            f'valid={valid.sum()} '
            f'pair_valid={pair_valid.sum()} '
            f'jump={jump_mask.sum()} '
            f'range={range_mask.sum()} '
            f'floor={lower_is_floor.sum()} '
            f'candidate={candidate.sum()}',
            throttle_duration_sec=1.0,
        )

        

        rows, columns = np.where(candidate)

        if len(rows) == 0:
            return np.empty((0, 3), dtype=np.float32)

        edge_points = lower[rows, columns]

        cells = set()

        

        for point in edge_points:
            cells.add((int(np.floor(point[0] / self.grid_resolution)), int(np.floor(point[1] / self.grid_resolution))))

        if len(cells) < 4:
            return np.empty((0, 3), dtype=np.float32)

        

        accepted_cells = set()

        cells = sorted(cells, key=lambda y: y[1])

        l = r = 0

        self.get_logger().info(f"e: {len(edge_points)}")

        while r < len(cells):
            while r + 1 < len(cells) and cells[r + 1][1] == cells[r][1] + 1:
                r += 1
            if r - l + 1 >= self.min_edge_length:
                accepted_cells.update(cells[l:r])
            l = r + 1
            r = l

        accepted_points = []

        for x, y in accepted_cells:
            accepted_points.append([(x + 0.5) * self.grid_resolution, (y + 0.5) * self.grid_resolution, 0.20])

        return np.asarray(accepted_points, dtype=np.float32)


    def quaternion_to_matrix(self, q):
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])



def main():

    rclpy.init()
    node = PitDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt as e:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
        