#!/usr/bin/env python3
"""Pure-pursuit driver that follows a Content/tracks/*.csv layout at low
speed.

Uses /testing_only/odom (ground-truth pose) for control so the drive is
deterministic and bypasses any in-progress SLAM — the point is to feed
SLAM (fast_LIMO, cone_slam) a real motion trajectory, not to close the
control loop on its estimate. Run alongside cone_slam (or fast_LIMO)
and ros2 bag record to capture a reproducible test fixture.

Usage (inside the running dv_pipeline_stack container):

    docker exec -d ifssim-dv_pipeline_stack-1 bash -lc '
        source /opt/ros/humble/setup.bash
        source /dv_pipeline_stack_ws/install/setup.bash
        python3 /repo/tools/track_driver.py /repo/Content/tracks/<file>.csv
    '

(Mount the repo into /repo or copy the CSV in via `docker cp` first.)
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry


THROTTLE_CAP_DEFAULT = 0.15
STEER_CAP_DEFAULT    = 0.35
LOOKAHEAD_M_DEFAULT  = 4.0
WHEELBASE_M          = 1.55      # IFS-08 approx, used in the pure-pursuit law
LOOP_HZ              = 20
MAX_DRIVE_S_DEFAULT  = 90        # safety stop


def load_track(path, rotate_ccw_90: bool = True):
    """Load a Content/tracks/*.csv layout.

    Cones in the CSV files are stored in the UE5 engine frame (X-
    forward, left-handed). The bridge publishes /testing_only/odom in
    ROS REP-103 (right-handed, Y-forward) which is rotated 90° CCW vs
    the engine. We verified this empirically against bag
    `trackA_manual_001602`: applying (x, y) → (-y, x) to the CSV cones
    matched 237/240 of them to the LiDAR observations within 1 m
    (median 0.15 m) — see the cone-alignment analysis run on
    2026-04-28. Without this transform, the loaded centerline lives in
    the wrong frame and a pure-pursuit driver chases waypoints that
    don't match where the car actually is.

    Set `rotate_ccw_90=False` if you ever record a CSV that's already
    in the GT-odometry frame.
    """
    blues, yellows, oranges = [], [], []
    with open(path) as f:
        for row in csv.reader(f):
            if not row:
                continue
            color = row[0].strip().lower()
            x, y = float(row[1]), float(row[2])
            if rotate_ccw_90:
                x, y = -y, x
            if color == "blue":
                blues.append((x, y))
            elif color == "yellow":
                yellows.append((x, y))
            elif color == "big_orange":
                oranges.append((x, y))
    return np.array(blues), np.array(yellows), np.array(oranges)


def centerline_midpoints(blues, yellows):
    """For each blue cone, midpoint with its nearest yellow cone."""
    mids = []
    for b in blues:
        d = np.linalg.norm(yellows - b, axis=1)
        y = yellows[np.argmin(d)]
        mids.append(0.5 * (b + y))
    return np.array(mids)


def order_by_walk(pts, start_idx, max_step_m=8.0):
    """Greedy nearest-neighbor traversal from start_idx; bail when the
    next-nearest jump exceeds max_step_m (handles loops + dead ends).
    """
    remaining = list(range(len(pts)))
    order = [start_idx]
    remaining.remove(start_idx)
    while remaining:
        last = pts[order[-1]]
        d = np.linalg.norm(pts[remaining] - last, axis=1)
        nxt_local = int(np.argmin(d))
        if d[nxt_local] > max_step_m:
            break
        order.append(remaining.pop(nxt_local))
    return order


def yaw_from_quat(q):
    """ROS quaternion (x, y, z, w) → yaw (rad)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class TrackDriver(Node):
    def __init__(self, waypoints, throttle_cap, steer_cap, lookahead_m, max_drive_s):
        super().__init__("track_driver")
        self.waypoints   = waypoints
        self.throttle_cap = throttle_cap
        self.steer_cap   = steer_cap
        self.lookahead_m = lookahead_m
        self.max_drive_s = max_drive_s
        self.idx  = 0
        self.pose = None

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, "/testing_only/odom", self._on_odom, qos)
        self.pub = self.create_publisher(ControlCommand, "/control_command", 10)
        self.timer = self.create_timer(1.0 / LOOP_HZ, self._tick)

        self.t_start = time.time()
        self._last_log_s = -1
        self.get_logger().info(
            f"track_driver loaded {len(waypoints)} waypoints, "
            f"throttle≤{throttle_cap} steer≤{steer_cap} lookahead={lookahead_m} m")

    def _on_odom(self, msg):
        self.pose = msg.pose.pose

    def _tick(self):
        elapsed = time.time() - self.t_start
        if elapsed > self.max_drive_s:
            self._stop("max drive time reached")
            return
        if self.pose is None:
            return

        cx, cy = self.pose.position.x, self.pose.position.y
        yaw    = yaw_from_quat(self.pose.orientation)
        cosY, sinY = math.cos(-yaw), math.sin(-yaw)

        def to_body(wx, wy):
            dx, dy = wx - cx, wy - cy
            return cosY * dx - sinY * dy, sinY * dx + cosY * dy

        # Step 1 — advance `idx` past any waypoints that are now BEHIND
        # the car (body-frame x ≤ 0). This is "progress accounting" and
        # has nothing to do with lookahead distance. Without it the
        # pursuit could keep targeting a waypoint we already drove past.
        for _ in range(len(self.waypoints)):
            wx, wy = self.waypoints[self.idx]
            bx, _ = to_body(wx, wy)
            if bx > 0.0:
                break
            self.idx = (self.idx + 1) % len(self.waypoints)

        # Step 2 — from `idx` forward, pick the first waypoint whose
        # body-frame distance is ≥ lookahead_m. If we never reach
        # lookahead_m within a 60-waypoint forward window (e.g. tight
        # turn where the next waypoint is only 1 m away), pursue the
        # farthest-found forward waypoint — that's a degraded but
        # always-defined pursuit point. Previously this loop bailed out
        # entirely on tight curves, which is why the driver braked at
        # the first turn of the track.
        target_idx = self.idx
        target_bx  = 0.0
        target_by  = 0.0
        target_d   = 0.0
        farthest_d = -1.0
        for offset in range(60):
            j = (self.idx + offset) % len(self.waypoints)
            wx, wy = self.waypoints[j]
            bx, by = to_body(wx, wy)
            if bx <= 0.0:
                continue
            d = math.hypot(bx, by)
            if d >= self.lookahead_m:
                target_idx, target_bx, target_by, target_d = j, bx, by, d
                break
            if d > farthest_d:
                farthest_d = d
                target_idx, target_bx, target_by, target_d = j, bx, by, d
        if target_d <= 0.0:
            self._stop("no forward waypoint in 60-step window (lap done?)")
            return
        bx, by, d = target_bx, target_by, target_d

        # Pure-pursuit steering law: δ = atan2(2 L sin α, d).
        alpha     = math.atan2(by, bx)
        steer_rad = math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), d)
        steer     = max(-self.steer_cap, min(self.steer_cap, steer_rad))

        cmd = ControlCommand()
        cmd.throttle = float(self.throttle_cap)
        cmd.steering = float(steer)
        cmd.brake    = 0.0
        self.pub.publish(cmd)

        # ~1 Hz info log.
        sec = int(elapsed)
        if sec != self._last_log_s:
            self._last_log_s = sec
            self.get_logger().info(
                f"t={elapsed:5.1f}s pose=({cx:+6.1f},{cy:+6.1f},"
                f"yaw={math.degrees(yaw):+5.1f}°) "
                f"wp={self.idx}/{len(self.waypoints)} "
                f"d={d:.1f} steer={steer:+.2f}")

    def _stop(self, reason):
        self.get_logger().info(f"stopping: {reason}")
        for _ in range(5):  # send a few full-brake messages to be safe
            cmd = ControlCommand()
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake    = 1.0
            self.pub.publish(cmd)
            time.sleep(0.05)
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("track_csv", help="path to a Content/tracks/*.csv file")
    p.add_argument("--throttle", type=float, default=THROTTLE_CAP_DEFAULT,
                   help=f"throttle cap (default {THROTTLE_CAP_DEFAULT})")
    p.add_argument("--steer", type=float, default=STEER_CAP_DEFAULT,
                   help=f"steer cap (default {STEER_CAP_DEFAULT})")
    p.add_argument("--lookahead", type=float, default=LOOKAHEAD_M_DEFAULT,
                   help=f"pure-pursuit lookahead m (default {LOOKAHEAD_M_DEFAULT})")
    p.add_argument("--duration", type=float, default=MAX_DRIVE_S_DEFAULT,
                   help=f"max drive seconds (default {MAX_DRIVE_S_DEFAULT})")
    p.add_argument("--no-rotate", action="store_true",
                   help="skip the CSV→world 90° CCW rotation (only set this "
                        "if your CSV is already in the GT-odometry frame)")
    args = p.parse_args()

    if not os.path.isfile(args.track_csv):
        print(f"track file not found: {args.track_csv}", file=sys.stderr)
        sys.exit(2)

    blues, yellows, oranges = load_track(
        args.track_csv, rotate_ccw_90=not args.no_rotate)
    print(f"loaded {len(blues)} blue + {len(yellows)} yellow + "
          f"{len(oranges)} big_orange cones")
    mids = centerline_midpoints(blues, yellows)

    gate = oranges.mean(axis=0) if len(oranges) > 0 else mids[0]
    start = int(np.argmin(np.linalg.norm(mids - gate, axis=1)))
    order = order_by_walk(mids, start)
    waypoints = mids[order]
    print(f"centerline: {len(waypoints)} waypoints, "
          f"start near gate=({gate[0]:.1f},{gate[1]:.1f}) idx={start}")

    rclpy.init()
    node = TrackDriver(waypoints, args.throttle, args.steer,
                       args.lookahead, args.duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    print("driver done")


if __name__ == "__main__":
    main()
