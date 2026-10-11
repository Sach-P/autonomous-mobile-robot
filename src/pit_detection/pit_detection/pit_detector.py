import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros
import tf2_sensor_msgs
import numpy as np


class PitDetector(Node):
    def __init__(self):
        super().__init__('pit_detector')

        self._depth_sub = self.create_subscription(PointCloud2, '/depth_camera/depth_image/points', self.detection_callback, 10)

        self._cloud_pub = self.create_publisher(PointCloud2, '/pit_detector/points', 10)

        qos = QoSProfile(depth=1)
        qos.durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE
        self._grid_pub = self.create_publisher(OccupancyGrid, '/pit_detector/grid', qos)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)


        self.floor_height_tolerance = 0.075
        self.edge_jump_threshold = 0.15
        self.max_lookahead = 3
        self.max_lateral = 1.5
        self.grid_resolution = 0.025
        self.min_edge_length = 0.05 // self.grid_resolution


        self.grid_occ_res = 0.1
        self.grid_size_m = 20.0
        self.grid_dim = int(self.grid_size_m / self.grid_occ_res)
        self.grid_origin_x = -self.grid_size_m / 2
        self.grid_origin_y = -self.grid_size_m / 2
        self.occupancy = np.full((self.grid_dim, self.grid_dim), -1, dtype=np.int8)

    def detection_callback(self, msg: PointCloud2):

        try:
            tf_camera_base = self._tf_buffer.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time())
            tf_base_map = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return

        points = point_cloud2.read_points_numpy(msg, ['x', 'y', 'z'], skip_nans=False)
        points = points.reshape(msg.height, msg.width, 3)

        if len(points) <= 0:
                return

        valid = np.isfinite(points).all(axis=2)

        if not valid.any():
            return


        points_robot = self.transform_points(points, tf_camera_base)

        edge_points = self.find_pit_edges(points_robot, valid)

        if edge_points.size == 0:
            self.publish_occupancy(msg.header.stamp)
            return
        
        map_points = self.transform_points(edge_points, tf_base_map)

        # edge_cloud = point_cloud2.create_cloud_xyz32(
        #     msg.header,
        #     edge_points.tolist(),
        # )

        # edge_cloud.header.frame_id = 'base_footprint'
        # edge_cloud.header.stamp = msg.header.stamp

        self.mark_grid(map_points)
        self.publish_occupancy(msg.header.stamp)
        self.publish_remmebered_points(msg.header.stamp)

    def find_pit_edges(self, points, valid):
        height, width, _ = points.shape

        #compare each pixel to the pixel underneath
        upper = points[:-1, :, :]
        lower = points[1:, :, :]

        #make sure both pixels are valid
        pair_valid = valid[:-1, :] & valid[1:, :]

        #check how far the jump between pixels is by the x value
        forward_jump = upper[:, :, 0] - lower[:, :, 0]

        vertical_jump = upper[:, :, 2] - lower[:, :, 2]

        #make sure lower pixel is part of the floor
        lower_is_floor = np.abs(lower[:, :, 2]) < self.floor_height_tolerance

        forward_jump_mask = pair_valid & lower_is_floor & (
            forward_jump > self.edge_jump_threshold
        )

        range_mask = (
            (lower[:, :, 0] < self.max_lookahead) &
            (np.abs(lower[:, :, 1]) < self.max_lateral)
        )

        candidate = (
            forward_jump_mask |
            (valid[1:, :] & ~valid[:-1, :])
        ) & range_mask

        self.get_logger().info(
            f'valid={valid.sum()} '
            f'pair_valid={pair_valid.sum()} '
            f'front={forward_jump_mask.sum()} '
            f'range={range_mask.sum()} '
            f'floor={lower_is_floor.sum()} '
            f'candidate={candidate.sum()}',
            throttle_duration_sec=1.0,
        )

        

        rows, columns = np.where(candidate)

        if len(rows) == 0:
            return np.empty((0, 3), dtype=np.float32)

        edge_points = lower[rows, columns]
        edge_points = np.vstack([edge_points, upper[rows, columns]])

        cells = set()

        

        for point in edge_points:
            cells.add((int(np.floor(point[0] / self.grid_resolution)), int(np.floor(point[1] / self.grid_resolution))))

        if len(cells) < 4:
            return np.empty((0, 3), dtype=np.float32)

        

        accepted_cells = set()

        cells = sorted(cells, key=lambda y: y[1])

        l = r = 0

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


    def mark_grid(self, points):
        gx = ((points[:, 0] - self.grid_origin_x) / self.grid_occ_res).astype(int)
        gy = ((points[:, 1] - self.grid_origin_y) / self.grid_occ_res).astype(int)

        valid = (gx >= 0) & (gx < self.grid_dim) & (gy >= 0) & (gy < self.grid_dim)
        self.occupancy[gy[valid], gx[valid]] = 100

    def publish_occupancy(self, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.info.resolution = self.grid_occ_res
        msg.info.width = self.grid_dim
        msg.info.height = self.grid_dim
        msg.info.origin.position.x = self.grid_origin_x
        msg.info.origin.position.y = self.grid_origin_y
        msg.data = self.occupancy.flatten().tolist()
        self._grid_pub.publish(msg)

    def publish_remmebered_points(self, stamp):
        rows, cols = np.where(self.occupancy == 100)

        if rows.size == 0:
            return

        xs = self.grid_origin_x + (cols + 0.5) * self.grid_occ_res
        ys = self.grid_origin_y + (rows + 0.5) * self.grid_occ_res
        zs = np.full_like(xs, 0.20)

        points = np.column_stack([xs, ys, zs])

        header = Header()
        header.stamp = stamp
        header.frame_id = "map"

        cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self._cloud_pub.publish(cloud)


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
        