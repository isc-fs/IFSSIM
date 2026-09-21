#!/usr/bin/env python3
"""Pure-pursuit driver that follows a Content/tracks/*.csv layout at low
speed.

Uses /testing_only/odom (ground-truth pose) for control so the drive is
deterministic and bypasses any in-progress SLAM — the point is to feed
SLAM (fast_LIMO, cone_slam) a real motion trajectory, not to close the
control loop on its estimate. Run alongside cone_slam (or fast_LIMO)
and ros2 bag record to capture a reproducible test fixture.

Lateral control uses ``pipeline/control`` Pure Pursuit unchanged. Each
tick builds a short forward path prefix (≤12 m arc, like ``/Path`` from
path_planning) so global nearest-index never snaps to the far side of a
long/loop-shaped CSV polyline.

Usage (inside the running dv_pipeline_stack container):

    docker exec -d ifssim-dv_pipeline_stack-1 bash -lc '
        source /opt/ros/humble/setup.bash
        source /dv_pipeline_stack_ws/install/setup.bash
        python3 /repo/tools/track_driver.py /repo/Content/tracks/<file>.csv
    '

(Mount the repo into /repo or copy the CSV in via ``docker cp`` first.)
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

TARGET_SPEED_DEFAULT = 3.0  # m/s — benchmark control cruise setpoint
THROTTLE_MAX_DEFAULT = 0.6
# Full normalized rack (control_node publishes up to ±1.0 after sign flip).
STEER_CAP_DEFAULT = 1.0
# PI cruise (body-frame vx, same sign convention as control_node).
SPEED_KP = 0.25
SPEED_KI = 0.05
SPEED_DEADBAND = 0.1
SPEED_I_CLAMP = 0.5
# Match pipeline/control defaults (control_node.py).
LOOKAHEAD_MIN_DEFAULT = 1.0
LOOKAHEAD_K_DEFAULT = 0.5
LOOKAHEAD_MAX_DEFAULT = 8.0
WHEELBASE_M = 1.627
MAX_STEER_DEG = 22.4
LOOP_HZ = 20
# 0 = no time limit; benchmark sessions end on lap completion at the gate.
MAX_DRIVE_S_DEFAULT = 0.0
# Lap finish — same idea as control_node stop_latch_min_travel (30 m).
LAP_MIN_TRAVEL_M = 30.0
LAP_FINISH_RADIUS_M = 6.0
# Same cap as path_planning fasttube_adapter._MAX_PATH_ARC_M — controller
# only ever sees a short forward prefix, not the full CSV polyline.
PATH_HORIZON_ARC_M = 12.0
# When the NN walk returns near the gate, cut the tail so Pure Pursuit
# cannot snap to the far side of a nearly-closed polyline.
LOOP_TRIM_MIN_TRAVEL_M = 20.0
LOOP_TRIM_CLOSE_M = 8.0
# Centerline resampling. The ordered cone-midpoint polyline is coarse
# (one point per blue cone, ~3-5 m, uneven), which makes the controller-
# side finite-difference curvature noisy and the Pure Pursuit chase
# target snap between sparse points. The live pipeline feeds /Path
# resampled to ~0.5 m with analytical spline κ — match that here so the
# (unchanged) pipeline Pure Pursuit behaves the same on the GT setup.
CENTERLINE_SPACING_M = 0.5
# splprep smoothing as a per-point positional tolerance (m). The total
# smoothing condition is s = (tol² · n_points); 0.1 m lightly de-noises
# the midpoints without cutting corners.
CENTERLINE_SMOOTH_TOL_M = 0.1


def _control_pkg_candidates() -> list[Path]:
    """Directories that contain the ``control`` Python package."""
    here = Path(__file__).resolve()
    candidates: list[Path] = []
    env = os.environ.get("IFSSIM_CONTROL_PKG", "").strip()
    if env:
        candidates.append(Path(env))
    candidates.append(Path("/control_pkg"))  # mission_control_backend mount
    # Local repo layout: tools/track_driver.py → pipeline/control
    if here.parent.name == "tools":
        candidates.append(here.parent.parent / "pipeline" / "control")
    return candidates


def _ensure_control_importable() -> None:
    """Allow ``from control...`` when not launched from a sourced ROS ws."""
    try:
        import control  # noqa: F401
        return
    except ImportError:
        pass
    for pkg_root in _control_pkg_candidates():
        if not pkg_root.is_dir():
            continue
        root = str(pkg_root.resolve())
        if root not in sys.path:
            sys.path.insert(0, root)
        try:
            import control  # noqa: F401
            return
        except ImportError:
            continue
    raise ImportError(
        "control package not found. In Docker, mount pipeline/control at "
        "/control_pkg and set IFSSIM_CONTROL_PKG=/control_pkg; locally, run "
        "from the IFSSIM repo with pipeline/control present."
    )


_ensure_control_importable()

from control.controllers.pure_pursuit import PurePursuit  # noqa: E402
from control.models.bicycle import KinematicBicycle  # noqa: E402
from control.reference import ReferenceTrajectory  # noqa: E402
from control.state import VehicleState  # noqa: E402


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


def trim_loop_closure(
    pts: np.ndarray,
    gate_xy: np.ndarray,
    *,
    min_travel_m: float = LOOP_TRIM_MIN_TRAVEL_M,
    close_m: float = LOOP_TRIM_CLOSE_M,
) -> np.ndarray:
    """Truncate when the ordered walk comes back near the gate."""
    n = len(pts)
    if n < 3:
        return pts
    travelled = 0.0
    for i in range(1, n):
        travelled += float(np.linalg.norm(pts[i] - pts[i - 1]))
        if travelled < min_travel_m:
            continue
        if float(np.linalg.norm(pts[i] - gate_xy)) < close_m:
            return pts[:i]
    return pts


def _dedup_consecutive(pts: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Drop consecutive duplicate points (splprep rejects zero-length segs)."""
    if len(pts) < 2:
        return pts
    keep = [0]
    for i in range(1, len(pts)):
        if np.linalg.norm(pts[i] - pts[keep[-1]]) > eps:
            keep.append(i)
    return pts[keep]


def _resample_linear(
    pts: np.ndarray, spacing: float,
) -> tuple[list[float], list[float], None]:
    """Fallback resample: piecewise-linear interpolation at uniform arc length.

    No smoothing/curvature — used only when scipy is unavailable. Still
    fixes the sparse-target snapping by giving Pure Pursuit a dense path.
    """
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    if total < spacing:
        return [float(p[0]) for p in pts], [float(p[1]) for p in pts], None
    n_out = max(2, int(round(total / spacing)) + 1)
    s_target = np.linspace(0.0, total, n_out)
    xs = np.interp(s_target, s, pts[:, 0])
    ys = np.interp(s_target, s, pts[:, 1])
    return [float(v) for v in xs], [float(v) for v in ys], None


def resample_centerline(
    waypoints: np.ndarray,
    spacing: float = CENTERLINE_SPACING_M,
    smooth_tol_m: float = CENTERLINE_SMOOTH_TOL_M,
) -> tuple[list[float], list[float], list[float] | None]:
    """Fit a smooth spline through ordered midpoints and resample uniformly.

    Returns (xs, ys, kappa) at ~``spacing`` m arc length, mirroring the live
    pipeline's /Path. ``kappa`` is the analytical spline curvature (signed,
    1/m) — parameterization-invariant, so valid regardless of the spline's
    internal u-parameter; ``None`` from the linear fallback. Falls back to a
    linear resample when scipy is missing or the path is too short to spline.
    """
    pts = _dedup_consecutive(np.asarray(waypoints, dtype=float))
    if len(pts) < 4:
        return (
            [float(p[0]) for p in pts],
            [float(p[1]) for p in pts],
            None,
        )

    try:
        from scipy.interpolate import splev, splprep
    except ImportError:
        return _resample_linear(pts, spacing)

    n = len(pts)
    smoothing = (smooth_tol_m ** 2) * n
    k = min(3, n - 1)
    try:
        tck, _ = splprep([pts[:, 0], pts[:, 1]], s=smoothing, k=k, per=0)
    except Exception:
        return _resample_linear(pts, spacing)

    # Dense eval → arc-length table → invert for uniform-arc sampling.
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    rough_len = float(seg.sum())
    dense_n = max(200, int(rough_len / (spacing * 0.2)))
    ud = np.linspace(0.0, 1.0, dense_n)
    xd, yd = splev(ud, tck)
    sd = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xd), np.diff(yd)))])
    arclen = float(sd[-1])
    if arclen < spacing:
        return _resample_linear(pts, spacing)

    n_out = max(2, int(round(arclen / spacing)) + 1)
    u_target = np.interp(np.linspace(0.0, arclen, n_out), sd, ud)
    xs, ys = splev(u_target, tck)
    dx, dy = splev(u_target, tck, der=1)
    ddx, ddy = splev(u_target, tck, der=2)
    denom = (dx * dx + dy * dy) ** 1.5
    kappa = np.where(denom > 1e-9, (dx * ddy - dy * ddx) / denom, 0.0)
    return (
        [float(v) for v in xs],
        [float(v) for v in ys],
        [float(v) for v in kappa],
    )


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
    def __init__(
        self,
        ref: ReferenceTrajectory,
        gate_xy: tuple[float, float],
        target_speed: float,
        throttle_max: float,
        steer_cap: float,
        lookahead_min: float,
        lookahead_k: float,
        lookahead_max: float,
        max_drive_s: float,
        lap_min_travel_m: float,
        lap_finish_radius_m: float,
    ):
        super().__init__("track_driver")
        self.ref = ref
        self._gate_xy = gate_xy
        # Ordered centerline always starts at the gate (index 0).
        self._anchor_idx = 0
        self._path_oriented = False
        self.target_speed = target_speed
        self.throttle_max = throttle_max
        self.steer_cap = steer_cap
        self.max_drive_s = max_drive_s
        self._lap_min_travel_m = lap_min_travel_m
        self._lap_finish_radius_m = lap_finish_radius_m
        self._travelled = 0.0
        self._last_xy: tuple[float, float] | None = None
        self._speed_i = 0.0
        self.pose = None
        self.twist = None

        # steer_cap applies once at the wire (like control_node); do not also
        # scale max_steer_rad here — that was limiting PP to ~10° at 0.35.
        self._lateral = PurePursuit(
            lookahead_min=lookahead_min,
            lookahead_k=lookahead_k,
            lookahead_max=lookahead_max,
            model=KinematicBicycle(
                wheelbase=WHEELBASE_M,
                max_steer_rad=math.radians(MAX_STEER_DEG),
            ),
        )

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, "/testing_only/odom", self._on_odom, qos)
        self.pub = self.create_publisher(ControlCommand, "/control_command", 10)
        self.timer = self.create_timer(1.0 / LOOP_HZ, self._tick)

        self.t_start = time.time()
        self._last_log_s = -1
        cap_s = f"{max_drive_s:.0f}s" if max_drive_s > 0 else "none"
        self.get_logger().info(
            f"track_driver: {len(ref.x)} ref points, "
            f"PurePursuit Ld=[{lookahead_min}, {lookahead_max}] k={lookahead_k}, "
            f"v_target={target_speed:.1f}m/s throttle≤{throttle_max} steer≤{steer_cap}, "
            f"lap finish gate=({gate_xy[0]:.1f},{gate_xy[1]:.1f}) "
            f"after≥{lap_min_travel_m:.0f}m, time_cap={cap_s}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.pose = msg.pose.pose
        self.twist = msg.twist.twist

    def _vehicle_state(self) -> VehicleState | None:
        if self.pose is None:
            return None
        yaw = yaw_from_quat(self.pose.orientation)
        vx = float(self.twist.linear.x) if self.twist is not None else 0.0
        vy = float(self.twist.linear.y) if self.twist is not None else 0.0
        return VehicleState(
            x=float(self.pose.position.x),
            y=float(self.pose.position.y),
            yaw=yaw,
            vx=vx,
            vy=vy,
        )

    def _ensure_path_faces_vehicle(self, state: VehicleState) -> None:
        """Flip wp[1:] if wp[0]→wp[1] points opposite to heading.

        Gate stays at index 0 (``order_by_walk`` always starts there). A full
        list reverse would move the gate to the end and leave only ~2 horizon
        points — the failure seen as anchor=146/148.
        """
        if self._path_oriented or len(self.ref.x) < 2:
            return
        dx = self.ref.x[1] - self.ref.x[0]
        dy = self.ref.y[1] - self.ref.y[0]
        hx, hy = math.cos(state.yaw), math.sin(state.yaw)
        if dx * hx + dy * hy >= 0.0:
            self._path_oriented = True
            self._seed_anchor(state)
            return
        xs = [self.ref.x[0]] + list(reversed(self.ref.x[1:]))
        ys = [self.ref.y[0]] + list(reversed(self.ref.y[1:]))
        self.ref = ReferenceTrajectory.from_xy(xs, ys)
        self._anchor_idx = 0
        self._path_oriented = True
        self.get_logger().info(
            "centerline tail reversed to match initial vehicle heading"
        )
        self._seed_anchor(state)

    def _seed_anchor(self, state: VehicleState) -> None:
        """Snap anchor near the vehicle on the first path segment only."""
        n = len(self.ref.x)
        if n < 2:
            return
        end = min(n, 25)
        best_i = 0
        best_d2 = float("inf")
        for i in range(end):
            d2 = (self.ref.x[i] - state.x) ** 2 + (self.ref.y[i] - state.y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        self._anchor_idx = best_i

    def _forward_index_end(self, a0: int) -> int:
        """Last index reachable within PATH_HORIZON_ARC_M ahead of ``a0``."""
        n = len(self.ref.x)
        s0 = self.ref.s[a0]
        for i in range(a0, n):
            if self.ref.s[i] - s0 >= PATH_HORIZON_ARC_M:
                return min(i + 1, n)
        return n

    def _advance_anchor(self, state: VehicleState) -> None:
        """Project pose onto the polyline, searching forward from anchor only."""
        xs, ys = self.ref.x, self.ref.y
        n = len(xs)
        a0 = self._anchor_idx
        if a0 >= n - 1:
            return
        i_end = self._forward_index_end(a0)
        best_i = a0
        best_d2 = float("inf")
        for j in range(a0, min(i_end, n - 1)):
            px, py = xs[j], ys[j]
            nx, ny = xs[j + 1], ys[j + 1]
            vx, vy = nx - px, ny - py
            seg2 = vx * vx + vy * vy
            if seg2 < 1e-8:
                continue
            along = ((state.x - px) * vx + (state.y - py) * vy) / seg2
            t = max(0.0, min(1.0, along))
            cx = px + t * vx
            cy = py + t * vy
            d2 = (state.x - cx) ** 2 + (state.y - cy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = min(j + 1, n - 1) if along > 1.0 else j
        self._anchor_idx = max(a0, best_i)

    def _path_horizon(self, state: VehicleState) -> ReferenceTrajectory:
        """Forward path prefix like /Path — what Pure Pursuit is written for."""
        self._advance_anchor(state)
        xs, ys, s_arr = self.ref.x, self.ref.y, self.ref.s
        n = len(xs)
        i0 = self._anchor_idx
        if i0 >= n - 1:
            return ReferenceTrajectory()
        s0 = s_arr[i0]
        if s_arr[-1] - s0 < 0.5:
            return ReferenceTrajectory()
        i1 = n
        for i in range(i0, n):
            if s_arr[i] - s0 > PATH_HORIZON_ARC_M:
                i1 = max(i0 + 2, i)
                break
        return ReferenceTrajectory.from_xy(xs[i0:i1], ys[i0:i1])

    def _throttle_brake(self, state: VehicleState) -> tuple[float, float]:
        """PI on longitudinal vx toward ``target_speed`` (m/s)."""
        err = self.target_speed - state.vx
        if abs(err) < SPEED_DEADBAND:
            err = 0.0
        dt = 1.0 / LOOP_HZ
        self._speed_i += err * dt
        self._speed_i = max(-SPEED_I_CLAMP, min(SPEED_I_CLAMP, self._speed_i))
        u = SPEED_KP * err + SPEED_KI * self._speed_i
        if u > SPEED_DEADBAND:
            return min(self.throttle_max, u), 0.0
        if u < -SPEED_DEADBAND:
            return 0.0, min(self.throttle_max, -u)
        return 0.0, 0.0

    def _accumulate_travel(self, state: VehicleState) -> None:
        xy = (state.x, state.y)
        if self._last_xy is not None:
            self._travelled += math.hypot(
                xy[0] - self._last_xy[0], xy[1] - self._last_xy[1],
            )
        self._last_xy = xy

    def _lap_complete(self, state: VehicleState) -> bool:
        if self._travelled < self._lap_min_travel_m:
            return False
        gx, gy = self._gate_xy
        return (
            math.hypot(state.x - gx, state.y - gy) < self._lap_finish_radius_m
        )

    def _tick(self) -> None:
        elapsed = time.time() - self.t_start
        if self.max_drive_s > 0 and elapsed > self.max_drive_s:
            self._stop("max drive time reached")
            return

        state = self._vehicle_state()
        if state is None:
            return

        self._accumulate_travel(state)
        if self._lap_complete(state):
            self._stop(
                f"lap complete ({self._travelled:.0f}m travelled, "
                f"{elapsed:.0f}s)"
            )
            return

        self._ensure_path_faces_vehicle(state)
        ref = self._path_horizon(state)
        if ref.empty:
            self._stop(
                f"reached end of centerline "
                f"(anchor={self._anchor_idx}/{len(self.ref.x)})"
            )
            return

        steer_norm = self._lateral.compute(state, ref)
        # Same sign flip as control_node: strategy positive=left, UE5 positive=right.
        steer = max(-self.steer_cap, min(self.steer_cap, -steer_norm))
        throttle, brake = self._throttle_brake(state)

        cmd = ControlCommand()
        cmd.throttle = float(throttle)
        cmd.steering = float(steer)
        cmd.brake = float(brake)
        self.pub.publish(cmd)

        sec = int(elapsed)
        if sec != self._last_log_s:
            self._last_log_s = sec
            px, py = self.ref.x[self._anchor_idx], self.ref.y[self._anchor_idx]
            cte = math.hypot(px - state.x, py - state.y)
            self.get_logger().info(
                f"t={elapsed:5.1f}s pose=({state.x:+6.1f},{state.y:+6.1f},"
                f"yaw={math.degrees(state.yaw):+5.1f}°) "
                f"v={state.vx:+.2f}m/s tgt={self.target_speed:.1f} "
                f"dist={self._travelled:.0f}m "
                f"anchor={self._anchor_idx}/{len(self.ref.x)} cte={cte:.1f}m "
                f"horizon={len(ref.x)}pts steer={steer:+.2f}"
            )

    def _stop(self, reason: str) -> None:
        self.get_logger().info(f"stopping: {reason}")
        for _ in range(5):
            cmd = ControlCommand()
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake = 1.0
            self.pub.publish(cmd)
            time.sleep(0.05)
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("track_csv", help="path to a Content/tracks/*.csv file")
    p.add_argument("--target-speed", type=float, default=TARGET_SPEED_DEFAULT,
                   help=f"cruise speed in m/s (default {TARGET_SPEED_DEFAULT})")
    p.add_argument("--throttle-max", type=float, default=THROTTLE_MAX_DEFAULT,
                   help=f"max throttle command (default {THROTTLE_MAX_DEFAULT})")
    p.add_argument("--steer", type=float, default=STEER_CAP_DEFAULT,
                   help=f"normalized steer cap (default {STEER_CAP_DEFAULT})")
    p.add_argument("--lookahead", type=float, default=LOOKAHEAD_MIN_DEFAULT,
                   help=f"Pure Pursuit Ld_min in m (default {LOOKAHEAD_MIN_DEFAULT})")
    p.add_argument("--lookahead-k", type=float, default=LOOKAHEAD_K_DEFAULT,
                   help=f"Ld speed gain (default {LOOKAHEAD_K_DEFAULT})")
    p.add_argument("--lookahead-max", type=float, default=LOOKAHEAD_MAX_DEFAULT,
                   help=f"Ld max in m (default {LOOKAHEAD_MAX_DEFAULT})")
    p.add_argument("--duration", type=float, default=MAX_DRIVE_S_DEFAULT,
                   help="safety time cap in seconds, 0=disabled (default)")
    p.add_argument("--lap-min-travel", type=float, default=LAP_MIN_TRAVEL_M,
                   help=f"min metres before gate counts as finish (default {LAP_MIN_TRAVEL_M})")
    p.add_argument("--lap-finish-radius", type=float, default=LAP_FINISH_RADIUS_M,
                   help=f"metres to start gate to end lap (default {LAP_FINISH_RADIUS_M})")
    p.add_argument("--centerline-spacing", type=float, default=CENTERLINE_SPACING_M,
                   help=f"resampled centerline arc-length spacing in m "
                        f"(default {CENTERLINE_SPACING_M}, matches live /Path)")
    p.add_argument("--centerline-smooth", type=float, default=CENTERLINE_SMOOTH_TOL_M,
                   help=f"spline smoothing tolerance in m (default "
                        f"{CENTERLINE_SMOOTH_TOL_M}; 0 = interpolate exactly)")
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
    trimmed = trim_loop_closure(waypoints, gate)
    if len(trimmed) < len(waypoints):
        print(
            f"centerline: trimmed loop tail {len(waypoints)} → {len(trimmed)}"
        )
    waypoints = trimmed
    print(f"centerline: {len(waypoints)} waypoints, "
          f"start near gate=({gate[0]:.1f},{gate[1]:.1f}) idx={start}")

    xs, ys, kappa = resample_centerline(
        waypoints,
        spacing=args.centerline_spacing,
        smooth_tol_m=args.centerline_smooth,
    )
    print(
        f"centerline: resampled {len(waypoints)} → {len(xs)} pts "
        f"@ {args.centerline_spacing:.2f} m spacing "
        f"({'spline κ' if kappa is not None else 'linear, no κ'})"
    )
    ref = ReferenceTrajectory.from_xy(xs, ys, kappa=kappa)
    if ref.empty:
        print("centerline too short after ordering", file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    gate_xy = (float(waypoints[0][0]), float(waypoints[0][1]))
    node = TrackDriver(
        ref,
        gate_xy,
        args.target_speed,
        args.throttle_max,
        args.steer,
        args.lookahead,
        args.lookahead_k,
        args.lookahead_max,
        args.duration,
        args.lap_min_travel,
        args.lap_finish_radius,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    print("driver done")


if __name__ == "__main__":
    main()
