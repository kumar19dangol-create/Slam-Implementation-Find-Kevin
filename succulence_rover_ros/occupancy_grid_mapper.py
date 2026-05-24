"""
Bayesian Occupancy Grid Mapping (Week 6)
"""

import array
import numpy as np
from typing import Tuple
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation


def probability_to_log_odds(p: float) -> float:
    p = np.clip(p, 1e-10, 1 - 1e-10)
    return np.log(p / (1 - p))


def log_odds_to_probability(l: float) -> float:
    return 1.0 / (1.0 + np.exp(-l))


class OccupancyGrid:

    def __init__(self,
                 resolution: float,
                 width: int,
                 height: int,
                 origin_x: float,
                 origin_y: float,
                 log_odds_occupied: float,
                 log_odds_free: float,
                 log_odds_max: float,
                 log_odds_min: float,
                 max_range: float,
                 min_range: float,
                 lidar_x_offset: float,
                 lidar_y_offset: float,
                 lidar_yaw_offset: float):

        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.log_odds_occ = probability_to_log_odds(log_odds_occupied)
        self.log_odds_free = probability_to_log_odds(log_odds_free)
        self.log_odds_max = log_odds_max
        self.log_odds_min = log_odds_min

        self.max_range = max_range
        self.min_range = min_range

        self.lidar_x_offset = lidar_x_offset
        self.lidar_y_offset = lidar_y_offset
        self.lidar_yaw_offset = lidar_yaw_offset

        self.grid = np.zeros((height, width), dtype=np.float32)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def is_valid_cell(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def _ray_trace(self, start: Tuple[int, int], end: Tuple[int, int]) -> list:
        row0, col0 = start
        row1, col1 = end

        cells = []

        drow = abs(row1 - row0)
        dcol = abs(col1 - col0)

        srow = 1 if row1 > row0 else -1
        scol = 1 if col1 > col0 else -1

        err = dcol - drow

        row, col = row0, col0

        while True:
            if row == row1 and col == col1:
                break

            cells.append((row, col))

            e2 = 2 * err

            if e2 > -drow:
                err -= drow
                col += scol

            if e2 < dcol:
                err += dcol
                row += srow

        return cells

    def update(self, pose: np.ndarray, ranges: np.ndarray,
               angle_min: float, angle_increment: float):

        robot_x, robot_y, robot_theta = pose

        c_r = np.cos(robot_theta)
        s_r = np.sin(robot_theta)

        lidar_x = robot_x + c_r * self.lidar_x_offset - s_r * self.lidar_y_offset
        lidar_y = robot_y + s_r * self.lidar_x_offset + c_r * self.lidar_y_offset

        robot_row, robot_col = self.world_to_grid(lidar_x, lidar_y)

        if not self.is_valid_cell(robot_row, robot_col):
            return

        for i, r in enumerate(ranges):

            if np.isnan(r) or r < self.min_range or r > self.max_range:
                continue

            beam_angle = robot_theta + self.lidar_yaw_offset + (angle_min + i * angle_increment)

            end_x = lidar_x + r * np.cos(beam_angle)
            end_y = lidar_y + r * np.sin(beam_angle)

            end_row, end_col = self.world_to_grid(end_x, end_y)

            if not self.is_valid_cell(end_row, end_col):
                continue

            free_cells = self._ray_trace((robot_row, robot_col), (end_row, end_col))

            for (row, col) in free_cells:
                if self.is_valid_cell(row, col):
                    self.grid[row, col] += self.log_odds_free
                    self.grid[row, col] = max(self.grid[row, col], self.log_odds_min)

            self.grid[end_row, end_col] += self.log_odds_occ
            self.grid[end_row, end_col] = min(self.grid[end_row, end_col], self.log_odds_max)

    def get_ros_occupancy_grid(self) -> np.ndarray:
        occupancy = np.zeros_like(self.grid, dtype=np.int8)
        unknown_mask = np.abs(self.grid) < 0.1
        occupancy[unknown_mask] = -1
        known_mask = ~unknown_mask
        prob = 1.0 / (1.0 + np.exp(-self.grid[known_mask]))
        occupancy[known_mask] = (prob * 100).astype(np.int8)
        return occupancy

    def to_ros_message(self, frame_id='map', timestamp=None) -> OccupancyGridMsg:
        msg = OccupancyGridMsg()
        msg.header = Header()
        msg.header.frame_id = frame_id

        if timestamp:
            msg.header.stamp = timestamp

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        occupancy = self.get_ros_occupancy_grid()
        msg.data = array.array('b', occupancy.ravel().tobytes())

        return msg


class OccupancyGridMapperNode(Node):

    def __init__(self):
        super().__init__('occupancy_grid_mapper')

        self.declare_parameter('scan_topic')
        self.declare_parameter('odom_topic')
        self.declare_parameter('map_topic')
        self.declare_parameter('map_publish_rate')

        self.declare_parameter('occupancy_grid.resolution')
        self.declare_parameter('occupancy_grid.width')
        self.declare_parameter('occupancy_grid.height')
        self.declare_parameter('occupancy_grid.origin_x')
        self.declare_parameter('occupancy_grid.origin_y')
        self.declare_parameter('occupancy_grid.log_odds_occupied')
        self.declare_parameter('occupancy_grid.log_odds_free')
        self.declare_parameter('occupancy_grid.log_odds_max')
        self.declare_parameter('occupancy_grid.log_odds_min')
        self.declare_parameter('occupancy_grid.max_range')
        self.declare_parameter('occupancy_grid.min_range')

        self.declare_parameter('lidar.x_offset')
        self.declare_parameter('lidar.y_offset')
        self.declare_parameter('lidar.yaw_offset')

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        map_topic = self.get_parameter('map_topic').value
        publish_rate = self.get_parameter('map_publish_rate').value

        self.occupancy_grid = OccupancyGrid(
            resolution=self.get_parameter('occupancy_grid.resolution').value,
            width=self.get_parameter('occupancy_grid.width').value,
            height=self.get_parameter('occupancy_grid.height').value,
            origin_x=self.get_parameter('occupancy_grid.origin_x').value,
            origin_y=self.get_parameter('occupancy_grid.origin_y').value,
            log_odds_occupied=self.get_parameter('occupancy_grid.log_odds_occupied').value,
            log_odds_free=self.get_parameter('occupancy_grid.log_odds_free').value,
            log_odds_max=self.get_parameter('occupancy_grid.log_odds_max').value,
            log_odds_min=self.get_parameter('occupancy_grid.log_odds_min').value,
            max_range=self.get_parameter('occupancy_grid.max_range').value,
            min_range=self.get_parameter('occupancy_grid.min_range').value,
            lidar_x_offset=self.get_parameter('lidar.x_offset').value,
            lidar_y_offset=self.get_parameter('lidar.y_offset').value,
            lidar_yaw_offset=self.get_parameter('lidar.yaw_offset').value,
        )

        self.current_pose = None
        self.scan_count = 0

        self.map_pub = self.create_publisher(OccupancyGridMsg, map_topic, 10)

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        self.map_timer = self.create_timer(1.0 / publish_rate, self.publish_map)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        theta = rotation.as_euler('xyz')[2]
        self.current_pose = np.array([x, y, theta])

    def scan_callback(self, msg: LaserScan):
        if self.current_pose is None:
            return

        ranges = np.array(msg.ranges)

        self.occupancy_grid.update(
            pose=self.current_pose,
            ranges=ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment
        )

        self.scan_count += 1

    def publish_map(self):
        if self.scan_count == 0:
            return

        msg = self.occupancy_grid.to_ros_message(
            frame_id='map',
            timestamp=self.get_clock().now().to_msg()
        )

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
