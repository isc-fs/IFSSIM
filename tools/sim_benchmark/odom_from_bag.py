"""Reconstruct sim_supervisor /odom offline from sensor topics in a rosbag."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from perception_metrics import msg_time_ns

# Match sim_supervisor_node production settings.
IMU_DECIMATION = 1
ODOM_PUBLISH_INTERVAL_NS = 10_000_000  # 100 Hz

ODOM_SENSOR_TOPICS = ("/imu", "/motor_rpm", "/steering_angle", "/brake_pressure")


def filter_state_to_odometry(state, stamp_ns: int) -> Any:
    """Build nav_msgs/Odometry from OdometryFilter.state."""
    from nav_msgs.msg import Odometry

    half = 0.5 * state.yaw
    msg = Odometry()
    msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = state.x
    msg.pose.pose.position.y = state.y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.w = float(math.cos(half))
    msg.pose.pose.orientation.z = float(math.sin(half))
    msg.twist.twist.linear.x = state.vx
    msg.twist.twist.linear.y = state.vy
    msg.twist.twist.angular.z = state.yaw_rate
    return msg


def synthesize_supervisor_odom(
    buckets: dict[str, list[tuple[int, object]]],
    *,
    steering_units: str = "radians",
    imu_time_scale: float = 1.0,
) -> list[tuple[int, object]]:
    """Replay IMU/RPM/steering/brake through OdometryFilter; return (t_ns, Odometry).

    ``imu_time_scale`` rescales the IMU integration clock onto true-motion time
    (the sim's wall-clock stamps over-count dt; see
    ``slam_metrics.estimate_imu_time_scale``). Published stamps stay on wall
    time so the synthesized /odom still aligns with the rest of the replay.
    """
    if not buckets.get("/imu"):
        return []

    # Match production: /odom comes from the C++ 9-state EKF
    # (odometry_filter_node), ported here as OdometryFilterCpp. The
    # legacy sim_supervisor complementary OdometryFilter integrated
    # centripetal accel-y directly into vy and drifted badly in corners.
    from odometry_filter_cpp import (
        EkfParams,
        OdometryFilterCpp,
        steering_to_road_wheel_rad,
    )

    events: list[tuple[int, str, object]] = []
    for topic in ODOM_SENSOR_TOPICS:
        for bag_t, msg in buckets.get(topic, []):
            events.append((msg_time_ns(bag_t, msg), topic, msg))
    events.sort(key=lambda e: e[0])

    filt = OdometryFilterCpp(EkfParams())
    out: list[tuple[int, object]] = []
    last_pub_ns = -1
    imu_idx = 0

    for t_ns, topic, msg in events:
        t = t_ns * 1e-9
        if topic == "/imu":
            imu_idx += 1
            if imu_idx % IMU_DECIMATION:
                continue
            filt.push_imu(
                t * imu_time_scale,
                np.array(
                    [
                        msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z,
                    ]
                ),
                np.array(
                    [
                        msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z,
                    ]
                ),
            )
        elif topic == "/motor_rpm":
            filt.push_rpm(t, float(msg.data))
        elif topic == "/steering_angle":
            filt.push_steering(
                t,
                steering_to_road_wheel_rad(float(msg.data), units=steering_units),
            )
        # /brake_pressure: the 9-state EKF dropped brake as an input
        # (it added noise, never signal); ignored here for parity.

        if not filt.is_calibrated():
            continue
        if last_pub_ns >= 0 and (t_ns - last_pub_ns) < ODOM_PUBLISH_INTERVAL_NS:
            continue
        out.append((t_ns, filter_state_to_odometry(filt.state, t_ns)))
        last_pub_ns = t_ns

    return out
