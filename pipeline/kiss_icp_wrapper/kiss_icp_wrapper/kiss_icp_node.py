"""ROS 2 LifecycleNode wrapping the upstream `kiss-icp` Python pipeline.

Consumes /lidar/Lidar1 (sensor_msgs/PointCloud2) and publishes the
KISS-ICP-integrated pose on /odom_lidar (nav_msgs/Odometry) at scan rate
(~10 Hz). The pose is the global, *drift-monotonic* localiser output —
no IMU fusion, no loop closure, just adaptive-threshold point-to-point
ICP against a rolling voxel-hash map.

Used by cone_slam as a global-rotation anchor (a `PriorFactorPose3` on
`X(k)`) to fix the cone-only DA cascade documented in
docs/archive/2026-05-06_slam-phase2-cascade-frozen.md — see issue #485
and PR for the full integration plan (Pattern A).

Frame choices
-------------
KISS-ICP's pose is reported relative to the car's start pose (it
integrates motion from t=0 in its own internal frame). We publish it
with:

  header.frame_id  = "kiss_odom"    (KISS-ICP's start-pose frame)
  child_frame_id   = "base_link"    (the chassis frame)

The bridge publishes `base_link → fsds/Lidar` as identity
(ifssim_ros_wrapper.cpp:1370), so the LiDAR-frame pose KISS-ICP
estimates IS the base_link pose — no static-TF composition needed for
this stack. If the lidar mount ever moves off-origin, this assumption
breaks and we'd need to compose by the inverse static transform.

No TF is broadcast. cone_slam still owns map→odom→base_link; this node
is a measurement source, not a frame definition.

Lifecycle layout (mirrors cone_detection_node):

  on_configure   create publisher, instantiate KissICP with FS-tuned
                 config. Cheap (<1 s).
  on_activate    create LiDAR subscription, super().on_activate().
  on_deactivate  drop subscription so a parked node burns no CPU.
  on_cleanup     destroy publisher, drop KissICP state.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


# Match cone_detection_node's QoS for /fsds/lidar/Lidar1 — BEST_EFFORT,
# KEEP_LAST(1). The bridge publishes the LiDAR as BEST_EFFORT and any
# RELIABLE subscriber gets dropped from discovery.
QOS_LATEST = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


# ---------------------------------------------------------------------
# KISS-ICP config — FS-scale defaults
# ---------------------------------------------------------------------
# max_range: FS tracks are ~30 m wide; cones beyond ~30 m are sparse
# enough that ICP correspondences degrade. 50 m gives margin without
# pulling in noisy long-range returns.
#
# min_range: filter out the chassis self-returns at the lidar mount.
# The bridge already filters z < 0 in some configurations; 0.5 m is
# safe regardless.
#
# voxel_size: KISS-ICP downsamples the rolling voxel-hash map at this
# resolution. 0.5 m is sized for FS cone spacing (~3 m) — fine enough
# to capture cone geometry without exploding the voxel-hash memory at
# FS-track scales.
#
# initial_threshold: starting ICP correspondence-distance threshold (m).
# The adaptive-threshold loop in KISS-ICP tightens this based on
# observed motion; this is just the seed value. 2 m matches the
# upstream default for ground vehicles.
KISS_MAX_RANGE_M = 50.0
KISS_MIN_RANGE_M = 0.5
KISS_VOXEL_SIZE_M = 0.5
KISS_INITIAL_THRESHOLD_M = 2.0


def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to (qx, qy, qz, qw) — ROS convention.

    Uses Shepperd's method (numerically stable for any rotation).
    Avoiding a scipy.spatial.transform.Rotation dependency keeps this
    file's import surface minimal — handy when the bridge container
    boots with rcl-only.
    """
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


class KissIcpNode(LifecycleNode):
    """LifecycleNode publishing /odom_lidar at LiDAR scan rate."""

    NODE_NAME = "kiss_icp_node"

    # Frame names — see module docstring.
    FRAME_PARENT = "kiss_odom"
    FRAME_CHILD = "base_link"

    def __init__(self):
        super().__init__(self.NODE_NAME)
        # All I/O created in on_configure / on_activate. Side-effect-free __init__
        # so a freshly constructed but unconfigured node holds no resources.
        self._pub_odom = None
        self._sub_lidar = None
        self._kiss = None
        # Diagnostic counters reset on activate.
        self._n_scans = 0
        self._n_failures = 0
        self._sum_register_ms = 0.0
        self._last_log_t = 0.0

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"on_configure: creating /odom_lidar publisher + KISS-ICP "
            f"(max_range={KISS_MAX_RANGE_M} m, voxel={KISS_VOXEL_SIZE_M} m)"
        )
        self._pub_odom = self.create_lifecycle_publisher(
            Odometry, "/odom_lidar", 10,
        )

        # Defer imports until on_configure so an unconfigured node doesn't
        # pay the kiss_icp module-load cost, and so a kiss-icp install
        # failure shows up as a clean lifecycle transition failure rather
        # than a crash at node spawn time.
        try:
            from kiss_icp.kiss_icp import KissICP
            from kiss_icp.config import KISSConfig
        except ImportError as e:
            self.get_logger().error(
                f"on_configure: kiss-icp not importable ({e}). "
                f"Install with `pip install kiss-icp` in the bridge container."
            )
            return TransitionCallbackReturn.FAILURE

        config = KISSConfig()
        # KISS-ICP's config is a pydantic model with nested .data,
        # .mapping, .adaptive_threshold sub-configs. The schema has
        # been stable across recent releases but defensive attribute
        # access keeps a future field rename from blowing up the
        # whole pipeline.
        try:
            config.data.max_range = KISS_MAX_RANGE_M
            config.data.min_range = KISS_MIN_RANGE_M
            config.mapping.voxel_size = KISS_VOXEL_SIZE_M
            config.adaptive_threshold.initial_threshold = KISS_INITIAL_THRESHOLD_M
        except AttributeError as e:
            self.get_logger().warning(
                f"on_configure: KISS-ICP config schema differs from expected "
                f"({e}). Falling back to upstream defaults."
            )

        self._kiss = KissICP(config)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate: subscribing to /lidar/Lidar1")
        self._reset_diag()
        self._sub_lidar = self.create_subscription(
            PointCloud2, "/lidar/Lidar1", self._on_scan, QOS_LATEST,
        )
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate: dropping LiDAR subscription")
        if self._sub_lidar is not None:
            self.destroy_subscription(self._sub_lidar)
            self._sub_lidar = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_cleanup: destroying publisher + KISS state")
        if self._sub_lidar is not None:
            self.destroy_subscription(self._sub_lidar)
            self._sub_lidar = None
        if self._pub_odom is not None:
            self.destroy_publisher(self._pub_odom)
            self._pub_odom = None
        self._kiss = None
        self._reset_diag()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown")
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Per-scan callback
    # ------------------------------------------------------------------
    def _on_scan(self, msg: PointCloud2) -> None:
        # Defensive: shouldn't fire if subscription destroyed first in
        # on_deactivate, but a multi-threaded executor can't guarantee
        # mid-callback teardown safety.
        if self._pub_odom is None or self._kiss is None:
            return

        # Parse PointCloud2 — same format-agnostic approach as
        # cone_detection_node.listener_callback (cone_detection_node.py:151-155).
        floats_per_point = msg.point_step // 4
        num_points = msg.width * msg.height
        if num_points == 0:
            return

        try:
            raw = np.frombuffer(msg.data, dtype=np.float32).reshape(
                num_points, floats_per_point
            )
        except ValueError:
            # Malformed cloud — point_step doesn't divide len(data).
            # Skip the scan rather than crash the node.
            self.get_logger().warning(
                f"_on_scan: malformed PointCloud2 ({num_points} pts × "
                f"{floats_per_point} floats vs. {len(msg.data)} bytes)"
            )
            return

        # KISS-ICP wants float64 xyz. Per-point timestamps are not
        # available in the FSDS-format clouds we ship today, so we
        # skip the deskewing step (passing None to register_frame).
        # At 10 Hz / <10 m/s the intra-scan motion is < 1 m which is
        # below the FS-scale voxel resolution.
        points = raw[:, :3].astype(np.float64)

        t0 = time.perf_counter()
        try:
            # Upstream API: register_frame(points, timestamps) -> (frame, keypoints).
            # Return value discarded; we only need kiss.last_pose afterwards.
            self._kiss.register_frame(points, None)
        except Exception as e:
            self._n_failures += 1
            if self._n_failures <= 5 or self._n_failures % 50 == 0:
                self.get_logger().error(
                    f"_on_scan: KISS-ICP register_frame failed ({e}); "
                    f"total failures={self._n_failures}"
                )
            return
        register_ms = (time.perf_counter() - t0) * 1000.0
        self._sum_register_ms += register_ms
        self._n_scans += 1

        T = self._kiss.last_pose  # 4x4 SE(3)
        if T is None or T.shape != (4, 4):
            return

        # Publish as Odometry message in base_link frame.
        # base_link ↔ fsds/Lidar is identity per
        # ifssim_ros_wrapper.cpp:1370 — no static-TF composition needed.
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.FRAME_PARENT
        odom.child_frame_id = self.FRAME_CHILD
        odom.pose.pose.position.x = float(T[0, 3])
        odom.pose.pose.position.y = float(T[1, 3])
        odom.pose.pose.position.z = float(T[2, 3])
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(T[:3, :3])
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Pose covariance: KISS-ICP does not expose a per-scan covariance
        # estimate. Downstream consumers (cone_slam's PriorFactorPose3)
        # apply their own constant sigma; leave the 6x6 block as zeros
        # so a future consumer can distinguish "no estimate" from a
        # real diagonal.
        # Twist: KISS-ICP doesn't estimate velocity; leave zero.

        self._pub_odom.publish(odom)

        self._maybe_log_diag()

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    def _reset_diag(self) -> None:
        self._n_scans = 0
        self._n_failures = 0
        self._sum_register_ms = 0.0
        self._last_log_t = time.monotonic()

    def _maybe_log_diag(self) -> None:
        now = time.monotonic()
        if now - self._last_log_t < 5.0:
            return
        if self._n_scans == 0:
            self._last_log_t = now
            return
        avg_ms = self._sum_register_ms / self._n_scans
        self.get_logger().info(
            f"diag: {self._n_scans} scans in last window, "
            f"{self._n_failures} failures, avg register={avg_ms:.1f} ms"
        )
        self._sum_register_ms = 0.0
        self._n_scans = 0
        # Don't reset _n_failures — running total surfaces persistent issues.
        self._last_log_t = now


def main(args=None):
    rclpy.init(args=args)
    node = KissIcpNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
