"""
Probabilistic Motion Model for Odometry Processing (Week 5)
...
"""

import numpy as np
from typing import List, Tuple, Optional
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, PoseStamped
from scipy.spatial.transform import Rotation

from . import utils


# ============================================================================
# STUDENT TODO: Implement this function
# ============================================================================

def compute_motion_covariance(relative_pose: np.ndarray,
                              alpha1: float, alpha2: float,
                              alpha3: float, alpha4: float) -> np.ndarray:
    """
    Compute covariance matrix for a relative motion using the alpha noise model.
    """

    # TODO: YOUR CODE HERE (~7 lines)

    dx, dy, dtheta = relative_pose

    delta_trans = np.sqrt(dx**2 + dy**2)
    delta_rot = np.abs(dtheta)

    var_x = alpha1 * delta_trans**2 + alpha2 * delta_rot**2
    var_y = alpha1 * delta_trans**2 + alpha2 * delta_rot**2
    var_theta = alpha3 * delta_trans**2 + alpha4 * delta_rot**2

    cov = np.diag([var_x, var_y, var_theta])

    return cov


# ============================================================================
# Everything below is provided — you do not need to modify it.
# ============================================================================

class OdometryProcessor(Node):

    def __init__(self):
        super().__init__('odometry_processor')

        self.declare_parameter('odom_topic')
        self.declare_parameter('dead_reckoning.odometry_topic')
        self.declare_parameter('dead_reckoning.path_topic')
        self.declare_parameter('frames.odom_frame')
        self.declare_parameter('frames.base_link_frame')
        self.declare_parameter('frames.map_frame')
        self.declare_parameter('motion_model.alpha1')
        self.declare_parameter('motion_model.alpha2')
        self.declare_parameter('motion_model.alpha3')
        self.declare_parameter('motion_model.alpha4')
        self.declare_parameter('motion_model.max_trajectory_length')

        odom_topic = self.get_parameter('odom_topic').value
        odometry_out = self.get_parameter('dead_reckoning.odometry_topic').value
        path_out = self.get_parameter('dead_reckoning.path_topic').value

        self.odom_frame = self.get_parameter('frames.odom_frame').value
        self.base_link_frame = self.get_parameter('frames.base_link_frame').value
        self.map_frame = self.get_parameter('frames.map_frame').value

        self.alpha1 = self.get_parameter('motion_model.alpha1').value
        self.alpha2 = self.get_parameter('motion_model.alpha2').value
        self.alpha3 = self.get_parameter('motion_model.alpha3').value
        self.alpha4 = self.get_parameter('motion_model.alpha4').value
        self.max_trajectory_length = self.get_parameter(
            'motion_model.max_trajectory_length').value

        self.trajectory: List[Tuple[float, np.ndarray, np.ndarray]] = []
        self.prev_odom_pose: Optional[np.ndarray] = None
        self.current_pose = np.array([0.0, 0.0, 0.0])
        self.current_cov = np.zeros((3, 3))

        self.odom_pub = self.create_publisher(Odometry, odometry_out, 10)
        self.path_pub = self.create_publisher(Path, path_out, 10)

        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(f'OdometryProcessor started — listening on {odom_topic}')
        self.get_logger().info(
            f'Alpha params: a1={self.alpha1}, a2={self.alpha2}, '
            f'a3={self.alpha3}, a4={self.alpha4}')

    def odom_callback(self, msg: Odometry):

        odom_pose = self._odom_msg_to_pose(msg)

        if self.prev_odom_pose is None:
            self.prev_odom_pose = odom_pose
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.trajectory.append(
                (timestamp, self.current_pose.copy(), self.current_cov.copy()))
            return

        relative_odom = utils.pose_difference(self.prev_odom_pose, odom_pose)

        motion_cov = compute_motion_covariance(
            relative_odom, self.alpha1, self.alpha2, self.alpha3, self.alpha4)

        if motion_cov is None:
            motion_cov = np.zeros((3, 3))

        J1, J2 = utils.pose_compose_jacobians(self.current_pose, relative_odom)
        self.current_cov = utils.covariance_propagate(
            self.current_cov, motion_cov, J1, J2)

        self.current_pose = odom_pose

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.trajectory.append(
            (timestamp, self.current_pose.copy(), self.current_cov.copy()))

        if len(self.trajectory) > self.max_trajectory_length:
            self.trajectory = self.trajectory[-self.max_trajectory_length:]

        self.prev_odom_pose = odom_pose

        self._publish_odometry(msg.header.stamp)
        if len(self.trajectory) % 10 == 0:
            self._publish_path()

        if len(self.trajectory) % 100 == 0:
            self.get_logger().info(
                f'Poses: {len(self.trajectory)}, '
                f'Position: [{self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}], '
                f'Cov trace: {np.trace(self.current_cov):.4f}')

    def _odom_msg_to_pose(self, msg: Odometry) -> np.ndarray:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        theta = rotation.as_euler('xyz', degrees=False)[2]
        return np.array([x, y, theta])

    def _publish_odometry(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_link_frame
        msg.pose.pose.position.x = self.current_pose[0]
        msg.pose.pose.position.y = self.current_pose[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._yaw_to_quaternion(self.current_pose[2])
        msg.pose.covariance = self._3x3_to_6x6_covariance(self.current_cov)
        self.odom_pub.publish(msg)

    def _publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame

        max_points = 500
        recent = self.trajectory[-max_points:]
        for _, pose, _ in recent:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = pose[0]
            ps.pose.position.y = pose[1]
            ps.pose.position.z = 0.0
            ps.pose.orientation = self._yaw_to_quaternion(pose[2])
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Rotation.from_euler('z', yaw).as_quat()
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def _3x3_to_6x6_covariance(self, cov_3x3: np.ndarray) -> list:
        cov = [0.0] * 36
        cov[0] = cov_3x3[0, 0]
        cov[1] = cov_3x3[0, 1]
        cov[6] = cov_3x3[1, 0]
        cov[7] = cov_3x3[1, 1]
        cov[35] = cov_3x3[2, 2]
        return cov


def main(args=None):
    rclpy.init(args=args)
    node = OdometryProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
