#!/usr/bin/env python3
"""Adapter — /motor_rpm (Float32) → /wheel_odom (nav_msgs/Odometry).

robot_localization's ekf_localization_node only fuses
nav_msgs/Odometry-shaped measurements; the bridge publishes motor
RPM as a bare std_msgs/Float32 because the wire format upstream
predates the autonomy stack. This node wraps it:

    vx_body = rpm * RPM_TO_MS
    twist.linear.x = vx_body
    everything else = 0 with large covariance

So the EKF treats /wheel_odom as a vx-only measurement (huge variance
on all other axes) and only updates the state's vx through it.

`RPM_TO_MS = 0.00821` matches the constant the existing
OdometryFilter uses (re-derived 2026-05-10 under #380; see
`pipeline/odometry_filter/include/odometry_filter/odometry_filter.hpp`).
Keep them in sync.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


# Matches OdometryFilter and cone_graph_slam — single source of truth
# at the wire-format level, not duplicated meaning.
RPM_TO_MS = 0.00821

# Covariance on vx_body measurement. 0.05 m/s 1σ is the noise floor
# from the rear-axle wheel encoder model — small enough that the EKF
# trusts the measurement during steady driving, large enough to not
# blow up the Kalman gain numerically.
SIGMA_VX = 0.05

# Covariance on every other field of /wheel_odom we emit. Huge
# so the EKF ignores anything but vx — pose, vy, vz, angular all get
# rejected by the Kalman gain.
SIGMA_HUGE = 1e6


class RpmToOdomNode(Node):
    NODE_NAME = "rpm_to_odom"

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        # /motor_rpm is BEST_EFFORT on the publisher side.
        rpm_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._sub = self.create_subscription(
            Float32, "/motor_rpm", self._on_rpm, rpm_qos)
        self._pub = self.create_publisher(Odometry, "/wheel_odom", 10)

        # Pre-fill the covariance pattern; only vx (index 7) gets the
        # tight value, everything else huge. Indexing: 6×6 row-major,
        # diagonal is 0, 7, 14, 21, 28, 35 → x, y, z, roll, pitch, yaw
        # for pose, and same for twist.
        self._twist_cov = [0.0] * 36
        for diag in (0, 7, 14, 21, 28, 35):
            self._twist_cov[diag] = SIGMA_HUGE ** 2
        self._twist_cov[0] = SIGMA_VX ** 2  # vx — the trustworthy one
        # Pose isn't measured at all here; mark every diagonal huge so
        # the EKF doesn't try to use it.
        self._pose_cov = [0.0] * 36
        for diag in (0, 7, 14, 21, 28, 35):
            self._pose_cov[diag] = SIGMA_HUGE ** 2

        self.get_logger().info(
            f"rpm_to_odom started — vx_body = rpm × {RPM_TO_MS}, "
            f"σ_vx = {SIGMA_VX} m/s, publishing /wheel_odom")

    def _on_rpm(self, msg: Float32) -> None:
        vx = float(msg.data) * RPM_TO_MS

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        # robot_localization needs the child frame as base_link so it
        # knows the twist is body-frame. parent doesn't matter when
        # twist-only — pose isn't measured.
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link"
        out.twist.twist.linear.x = vx
        out.twist.covariance = self._twist_cov
        out.pose.covariance = self._pose_cov
        self._pub.publish(out)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = RpmToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
