"""
A* Planner ROS2 Node

Subscribes to the SLAM occupancy grid and the SLAM odometry. On a timer,
runs A* from the robot's current cell to the hardcoded goal cell and
publishes a nav_msgs/Path in the map frame.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg, Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from scipy import ndimage
from scipy.spatial.transform import Rotation

from .astar import astar_search, inflate_obstacles


class PlannerNode(Node):
    """ROS2 wrapper around A*. Replans at a fixed rate against the latest SLAM map."""

    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('map_topic')
        self.declare_parameter('odom_topic')
        self.declare_parameter('plan_topic')
        self.declare_parameter('frames.map_frame')
        self.declare_parameter('planning.replan_period')
        self.declare_parameter('planning.occupancy_threshold')
        self.declare_parameter('planning.treat_unknown_as_obstacle')
        self.declare_parameter('planning.inflation_radius_cells')
        self.declare_parameter('goal.x')
        self.declare_parameter('goal.y')

        map_topic = self.get_parameter('map_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        plan_topic = self.get_parameter('plan_topic').value

        self.map_frame = self.get_parameter('frames.map_frame').value
        self.replan_period = self.get_parameter('planning.replan_period').value
        self.occ_threshold = int(self.get_parameter('planning.occupancy_threshold').value)
        self.unknown_as_obstacle = bool(
            self.get_parameter('planning.treat_unknown_as_obstacle').value)
        self.inflation_radius = int(
            self.get_parameter('planning.inflation_radius_cells').value)

        self.goal_x = float(self.get_parameter('goal.x').value)
        self.goal_y = float(self.get_parameter('goal.y').value)

        self.latest_map: OccupancyGridMsg | None = None
        self.robot_xy: tuple[float, float] | None = None
        self.consecutive_failures = 0

        self.path_pub = self.create_publisher(Path, plan_topic, 10)
        # Debug topic: publishes the inflated `blocked` grid as an OccupancyGrid
        # so you can see in RViz exactly what A* sees. Inflated cells show as
        # occupied (black), unknown cells remain unknown (dark gray), free cells
        # stay free (light). Add this topic as a separate Map display in RViz.
        self.inflated_pub = self.create_publisher(
            OccupancyGridMsg, plan_topic + '/inflated', 10)
        # Debug topic: publishes the set of cells reachable from the rover
        # under A*'s own movement rules (8-connectivity + anti-corner-cutting).
        # Only published when A* fails with "graph disconnected" — the boundary
        # of the reachable region tells you exactly where A* loses connectivity.
        self.reachable_pub = self.create_publisher(
            OccupancyGridMsg, plan_topic + '/reachable', 10)
        self.create_subscription(OccupancyGridMsg, map_topic, self._map_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_timer(self.replan_period, self._replan)

        self.get_logger().info(
            f'PlannerNode started — map: {map_topic}, odom: {odom_topic}, '
            f'goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def _map_cb(self, msg: OccupancyGridMsg):
        self.latest_map = msg

    def _odom_cb(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _world_to_cell(self, x: float, y: float, info) -> tuple[int, int]:
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        return row, col

    def _cell_to_world(self, row: int, col: int, info) -> tuple[float, float]:
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    def _clear_halo(self, blocked: np.ndarray, grid: np.ndarray,
                    center: tuple[int, int]):
        # Clear the inflation halo around `center` (start or goal). Anything
        # near `center` that's blocked purely because of inflation gets freed;
        # actually-occupied cells (real walls) stay blocked. Used to let A*
        # plan from a rover stuck inside a halo and to plan TO a goal whose
        # halo has been overrun as the map fills in.
        radius = self.inflation_radius
        r0 = max(0, center[0] - radius)
        r1 = min(blocked.shape[0], center[0] + radius + 1)
        c0 = max(0, center[1] - radius)
        c1 = min(blocked.shape[1], center[1] + radius + 1)
        sub_grid = grid[r0:r1, c0:c1]
        sub = sub_grid >= self.occ_threshold
        if self.unknown_as_obstacle:
            sub |= sub_grid < 0
        blocked[r0:r1, c0:c1] = sub

    @staticmethod
    def _has_free_neighbour(blocked: np.ndarray, cell: tuple[int, int]) -> bool:
        r, c = cell
        h, w = blocked.shape
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and not blocked[nr, nc]:
                    return True
        return False

    def _replan(self):
        if self.latest_map is None or self.robot_xy is None:
            return

        info = self.latest_map.info
        grid = np.frombuffer(bytes(self.latest_map.data), dtype=np.int8).reshape(
            info.height, info.width).copy()

        blocked = inflate_obstacles(
            grid, self.inflation_radius,
            self.occ_threshold, self.unknown_as_obstacle)

        self._publish_inflated_debug(blocked, grid, info)

        start = self._world_to_cell(*self.robot_xy, info)
        goal = self._world_to_cell(self.goal_x, self.goal_y, info)

        if not (0 <= start[0] < info.height and 0 <= start[1] < info.width):
            self._log_failure('robot outside map bounds')
            self._publish_empty_path()
            return
        if not (0 <= goal[0] < info.height and 0 <= goal[1] < info.width):
            self._log_failure('goal outside map bounds')
            self._publish_empty_path()
            return

        # Clear inflation halos around both endpoints. The rover is physically
        # at `start` so inflation around it is safety buffer we're already
        # inside; clearing it lets A* plan an escape. Same for `goal`: if the
        # map fills in so that walls inflate over the goal cell, the rover
        # still needs to be able to plan TO the goal. Real walls in both
        # halos stay blocked.
        self._clear_halo(blocked, grid, start)
        self._clear_halo(blocked, grid, goal)

        # After clearing halos, these snapshots reflect the REAL degenerate
        # cases: goal in an actual wall, or start with no escape neighbours.
        goal_blocked = bool(blocked[goal])
        start_caged = not self._has_free_neighbour(blocked, start)

        # Publish reachable-from-start every tick so RViz always shows the
        # current connectivity. The boundary between the reachable region
        # (light) and the stranded region (gray) is where A* loses
        # connectivity. If goal lies in the gray region, that's the disconnect.
        self._publish_reachable_debug(blocked, start, info)

        path_cells = astar_search(blocked, start, goal)
        if path_cells is None:
            if goal_blocked:
                gx, gy = self._cell_to_world(goal[0], goal[1], info)
                reason = (f'goal cell blocked after inflation '
                          f'(cell={goal}, world=({gx:.2f}, {gy:.2f}))')
            elif start_caged:
                sx, sy = self._cell_to_world(start[0], start[1], info)
                reason = (f'start neighbourhood fully blocked '
                          f'(cell={start}, world=({sx:.2f}, {sy:.2f}))')
            else:
                sx, sy = self._cell_to_world(start[0], start[1], info)
                gx, gy = self._cell_to_world(goal[0], goal[1], info)
                # Compare A*'s view with scipy's pure 8-connectivity view.
                # If scipy says SAME component but A* still says no path,
                # the disconnect is from anti-corner-cutting refusing a
                # 1-cell diagonal squeeze somewhere (likely a jagged
                # inflation corner). If scipy also says DIFFERENT, the
                # disconnect is real geometric.
                labels, _ = ndimage.label(
                    ~blocked, structure=np.ones((3, 3), dtype=bool))
                start_lbl = int(labels[start])
                goal_lbl = int(labels[goal])
                if start_lbl != 0 and start_lbl == goal_lbl:
                    scipy_view = ('SAME 8-conn component — anti-corner-cutting '
                                  'is refusing a diagonal squeeze (likely a '
                                  'jagged inflation corner)')
                else:
                    scipy_view = ('DIFFERENT 8-conn components — real '
                                  f'geometric disconnect (start_lbl={start_lbl}, '
                                  f'goal_lbl={goal_lbl})')
                reason = (f'no connecting path between start={start} '
                          f'world=({sx:.2f}, {sy:.2f}) and goal={goal} '
                          f'world=({gx:.2f}, {gy:.2f}) (graph disconnected). '
                          f'scipy: {scipy_view}')
            self._log_failure(reason)
            self._publish_empty_path()
            return

        self.consecutive_failures = 0
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame

        # Identity orientation for all waypoints — the controller handles heading.
        q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        for (r, c) in path_cells:
            wx, wy = self._cell_to_world(r, c, info)
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation = q
            path_msg.poses.append(ps)

        # Replace the final waypoint with the exact goal so the controller
        # converges to the requested position rather than the cell centre.
        if path_msg.poses:
            path_msg.poses[-1].pose.position.x = self.goal_x
            path_msg.poses[-1].pose.position.y = self.goal_y

        self.path_pub.publish(path_msg)

    def _publish_reachable_debug(self, blocked: np.ndarray,
                                 start: tuple[int, int], info):
        # Compute the 8-connected component containing `start`. This ignores
        # anti-corner-cutting (A*'s extra restriction), so if A* says
        # disconnected but this visualization shows start and goal in the
        # same component, the issue is corner-cutting. If they're in
        # different components here too, the disconnect is real geometric.
        h, w = blocked.shape
        if not (0 <= start[0] < h and 0 <= start[1] < w) or blocked[start]:
            return  # nothing to flood from

        labels, _ = ndimage.label(~blocked, structure=np.ones((3, 3), dtype=bool))
        start_label = int(labels[start])
        reachable = (labels == start_label) if start_label != 0 else np.zeros_like(blocked)

        # Encode as OccupancyGrid: reachable -> 0 (light), blocked -> 100
        # (dark), non-reachable non-blocked -> 50 (medium gray, the "stranded"
        # region A* can't get to from the rover).
        debug = np.full_like(blocked, 50, dtype=np.int8)
        debug[reachable] = 0
        debug[blocked] = 100

        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info = info
        msg.data = debug.flatten().tolist()
        self.reachable_pub.publish(msg)

    def _publish_inflated_debug(self, blocked: np.ndarray, grid: np.ndarray, info):
        # Build a debug OccupancyGrid showing what A* actually sees: anything
        # blocked (originally occupied OR inflated) becomes 100, originally
        # unknown cells that didn't get blocked stay -1, the rest stay 0.
        debug = np.zeros_like(grid, dtype=np.int8)
        debug[grid < 0] = -1
        debug[blocked] = 100

        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info = info
        msg.data = debug.flatten().tolist()
        self.inflated_pub.publish(msg)

    def _publish_empty_path(self):
        empty = Path()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = self.map_frame
        self.path_pub.publish(empty)

    def _log_failure(self, reason: str):
        self.consecutive_failures += 1
        # Log the first failure and then once every ~10 to avoid spam.
        if self.consecutive_failures == 1 or self.consecutive_failures % 10 == 0:
            self.get_logger().warn(
                f'Planner: {reason} (failure #{self.consecutive_failures})')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
