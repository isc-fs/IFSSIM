#!/usr/bin/env python3
"""Stream fast_LIMO state vs bridge GT odom side-by-side, after putting both
into the same coordinate frame.

fast_LIMO publishes /fast_limo/state in its OWN odom frame, anchored at the
LiDAR/base_link pose at the time of its first scan. Bridge publishes
/testing_only/odom in absolute ENU world. Comparing raw (x, y) columns is
apples-to-oranges — the frames differ by the car's spawn pose at SLAM
init, plus the constant LiDAR-to-chassis offset.

This script latches the GT pose at the first /fast_limo/state message and
treats that as the anchor for fast_LIMO's odom. Subsequent SLAM poses are
rotated + translated into ENU world before being printed alongside GT, so
the columns share a frame and you can read them as a drift in metres
directly.

Caveat: we don't subtract the LiDAR-to-chassis offset (~0.5 m forward).
At the >50 m total displacement we're tuning against, that's a fixed ~1%
bias in distance comparisons — small enough to ignore for SLAM tuning.

Usage:
  python3 pose_cmp.py [duration-seconds]
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


def yaw_of(o):
    return math.degrees(math.atan2(
        2 * (o.w * o.z + o.x * o.y),
        1 - 2 * (o.y ** 2 + o.z ** 2),
    ))


def wrap_deg(d):
    return (d + 540.0) % 360.0 - 180.0


class PoseCmp(Node):
    def __init__(self, slam_topic: str = "/cone_slam/state"):
        super().__init__("pose_cmp")
        self._slam_topic = slam_topic
        # SLAM in its own odom frame (raw)
        self._slam_raw = None
        # SLAM transformed into ENU world (using anchor)
        self.slam = None
        # GT in ENU world
        self.gt = None
        # Anchor: GT pose snapshotted at the first SLAM message arrival.
        # The SLAM node anchors X(0) at world origin = the car's base_link
        # at the moment SLAM_RUNNING starts; that's this anchor.
        self._anchor = None

        # /cone_slam/state ships RELIABLE (rclpy default).
        # /testing_only/odom ships BEST_EFFORT (bridge sensor_data convention).
        self.create_subscription(Odometry, self._slam_topic,
                                 self._slam_cb, 10)
        self.create_subscription(Odometry, "/testing_only/odom",
                                 self._gt_cb, qos_profile_sensor_data)

    def _slam_cb(self, m):
        p = m.pose.pose.position
        slam_yaw = yaw_of(m.pose.pose.orientation)
        self._slam_raw = (p.x, p.y, slam_yaw)

        # First-arrival anchoring: use whatever GT is right now as the spawn
        # frame of LIMO odom. If GT hasn't arrived yet, defer.
        if self._anchor is None and self.gt is not None:
            self._anchor = self.gt

        if self._anchor is None:
            self.slam = None
            return

        ax, ay, ayaw_deg = self._anchor
        c = math.cos(math.radians(ayaw_deg))
        s = math.sin(math.radians(ayaw_deg))
        # Rotate LIMO odom (x, y) by the anchor's yaw, then translate to anchor's
        # (x, y) in ENU. yaw composes additively.
        enu_x = ax + p.x * c - p.y * s
        enu_y = ay + p.x * s + p.y * c
        enu_yaw = wrap_deg(ayaw_deg + slam_yaw)
        self.slam = (enu_x, enu_y, enu_yaw)

    def _gt_cb(self, m):
        p = m.pose.pose.position
        self.gt = (p.x, p.y, yaw_of(m.pose.pose.orientation))


def fmt(t):
    if t is None:
        return "   --       --       --   "
    return "{:+7.2f} {:+7.2f} {:+7.1f}".format(*t)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("duration", nargs="?", type=int, default=30,
                        help="seconds of comparison to print (1 Hz prints)")
    parser.add_argument("--slam-topic", default="/cone_slam/state",
                        help="topic to read SLAM odometry from")
    args = parser.parse_args()

    rclpy.init()
    node = PoseCmp(slam_topic=args.slam_topic)
    n = args.duration
    print(f"  t  |  SLAM->ENU  x       y      yaw  |   GT  x       y      yaw  | dist_err", flush=True)
    t_start = time.monotonic()
    last_print = -1.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = time.monotonic() - t_start
        if elapsed - last_print >= 1.0:
            last_print = elapsed
            i = int(round(elapsed))
            err = ""
            if node.slam is not None and node.gt is not None:
                dx = node.slam[0] - node.gt[0]
                dy = node.slam[1] - node.gt[1]
                err = " {:+6.2f}m".format(math.hypot(dx, dy))
            print("{:>3d}s | {} | {} |{}".format(
                i, fmt(node.slam), fmt(node.gt), err),
                  flush=True)
            if i >= n:
                break
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
