from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass, field
from statistics import median
from typing import Any

from perception_metrics import (
    Cone2D,
    WorldCone,
    header_at_ns,
    latch_track_layout,
    msg_time_ns,
    odom_at_time,
    odom_for_lidar_scan,
    world_cones_to_body,
    yaw_from_odom,
)

BASE_TOPICS = {"/imu", "/motor_rpm", "/odom", "/testing_only/odom"}
GT_CONE_TRIGGER_TOPIC = "/lidar/Lidar1"


@dataclass
class SlamSample:
    t_s: float
    step: float
    slam_x: float
    slam_y: float
    slam_yaw: float
    gt_x: float
    gt_y: float
    gt_yaw: float
    err_m: float
    yaw_err_rad: float


@dataclass
class TrajectoryPoint:
    t_s: float
    gt_x: float
    gt_y: float
    slam_x: float | None
    slam_y: float | None
    err_m: float | None


@dataclass
class PoseStepSample:
    """Per-sensor-step pose vs GT (aligned frame) for detailed reporting."""

    t_s: float
    event: str  # imu | rpm | supervisor_odom | gt_odom | cone
    gt_x: float
    gt_y: float
    gt_yaw: float
    filter_x: float | None = None
    filter_y: float | None = None
    filter_yaw: float | None = None
    filter_err_m: float | None = None
    filter_yaw_err_rad: float | None = None
    filter_err_delta_m: float | None = None
    filter_vx: float | None = None
    filter_vy: float | None = None
    filter_yaw_rate: float | None = None
    imu_only_x: float | None = None
    imu_only_y: float | None = None
    imu_only_yaw: float | None = None
    imu_only_err_m: float | None = None
    imu_only_yaw_err_rad: float | None = None
    imu_only_err_delta_m: float | None = None
    imu_only_vx: float | None = None
    imu_only_vy: float | None = None
    wheel_x: float | None = None
    wheel_y: float | None = None
    wheel_yaw: float | None = None
    wheel_err_m: float | None = None
    wheel_yaw_err_rad: float | None = None
    wheel_vx: float | None = None
    wheel_yaw_rate: float | None = None
    supervisor_x: float | None = None
    supervisor_y: float | None = None
    supervisor_yaw: float | None = None
    supervisor_err_m: float | None = None
    supervisor_yaw_err_rad: float | None = None
    slam_x: float | None = None
    slam_y: float | None = None
    slam_yaw: float | None = None
    slam_err_m: float | None = None
    slam_yaw_err_rad: float | None = None
    slam_committed: bool = False


@dataclass
class MapPoint:
    x: float
    y: float
    n_obs: int = 0
    landmark_id: int | None = None
    color: int | None = None


@dataclass
class MapMatchStats:
    gt_cones: int = 0
    slam_landmarks: int = 0
    matched: int = 0
    false_positive: int = 0
    false_negative: int = 0
    mean_match_err_m: float | None = None
    max_match_err_m: float | None = None


@dataclass
class ReplayResult:
    samples: list[SlamSample] = field(default_factory=list)
    trajectory: list[TrajectoryPoint] = field(default_factory=list)
    pose_steps: list[PoseStepSample] = field(default_factory=list)
    gt_map: list[MapPoint] = field(default_factory=list)
    slam_map: list[MapPoint] = field(default_factory=list)
    map_stats: MapMatchStats | None = None
    n_imu: int = 0
    n_rpm: int = 0
    n_supervisor_odom: int = 0
    n_odom: int = 0
    n_cone_updates: int = 0
    wheel_steer_units: str = "radians"
    imu_time_scale: float = 1.0


def estimate_imu_time_scale(odom_msgs: list[tuple[int, object]]) -> tuple[float, dict]:
    """Factor to rescale the IMU integration clock onto true-motion time.

    The bridge stamps every sensor with ``node_->now()`` (wall clock), but UE5
    runs slower than real-time under load, so the wall stamps span ~30% more
    time than the car actually moved. GT odom carries both an integrated
    position and an instantaneous twist on that same clock, so:

        scale = path_length / integral(speed * dt_wall)

    is the ratio of true motion time to wall time. Feeding ``t * scale`` to the
    EKF compresses every ``dt`` back onto sim time, which removes the inflated
    distance (``x += v * dt``). Returns ``(scale, diagnostics)``; ``1.0`` when
    GT is unusable.
    """
    rows = sorted((msg_time_ns(b, m), m) for b, m in odom_msgs)
    if len(rows) < 2:
        return 1.0, {}
    path = 0.0
    integ = 0.0
    for i in range(1, len(rows)):
        ta, ma = rows[i - 1]
        tb, mb = rows[i]
        dt = (tb - ta) * 1e-9
        if dt <= 0:
            continue
        pa, pb = ma.pose.pose.position, mb.pose.pose.position
        path += math.hypot(pb.x - pa.x, pb.y - pa.y)
        va, vb = ma.twist.twist.linear, mb.twist.twist.linear
        integ += 0.5 * (math.hypot(va.x, va.y) + math.hypot(vb.x, vb.y)) * dt
    if integ <= 1e-6 or path <= 1e-6:
        return 1.0, {}
    scale = path / integ
    return scale, {
        "imu_time_scale": scale,
        "gt_path_m": path,
        "gt_twist_integral_m": integ,
        "clock_stretch_pct": (1.0 / scale - 1.0) * 100.0,
    }


def odom_to_pose3(msg):
    import gtsam
    import numpy as np

    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return gtsam.Pose3(
        gtsam.Rot3.Quaternion(q.w, q.x, q.y, q.z),
        np.array([p.x, p.y, p.z]),
    )


def yaw_err(slam_yaw: float, gt_yaw: float) -> float:
    import numpy as np

    d = slam_yaw - gt_yaw
    return float(np.arctan2(np.sin(d), np.cos(d)))


def pose_err_m(ax: float, ay: float, bx: float, by: float) -> float:
    return float(math.hypot(ax - bx, ay - by))


def _scalar_series_from_bucket(
    buckets: dict[str, list[tuple[int, object]]],
    topic: str,
) -> list[tuple[int, float]]:
    """Build a (header_stamp_ns, value) series sorted by time."""
    from perception_metrics import msg_time_ns

    series: list[tuple[int, float]] = []
    for bag_t, msg in buckets.get(topic, []):
        series.append((msg_time_ns(bag_t, msg), float(msg.data)))
    series.sort(key=lambda row: row[0])
    return series


def infer_steering_angle_units(
    events: list[tuple[int, str, object]],
    rpm_series: list[tuple[int, float]],
    steer_series: list[tuple[int, float]],
    *,
    imu_decimation: int = 1,
    wheelbase_m: float = 1.570,
    rpm_to_ms: float = 0.00821,
    max_steer_rad: float = 0.5,
    max_samples: int = 3000,
) -> str:
    """Pick ``'radians'`` vs ``'normalized'`` for ``/steering_angle`` bag samples.

    Bridge contract is road-wheel radians, but some captures still store Chaos
    normalized [-1, 1]. Compare kinematic ω against IMU gyro_z on early samples.
    """
    err_rad = 0.0
    err_norm = 0.0
    n = 0
    imu_idx = 0
    for _bag_t_ns, topic, msg in events:
        if topic != "/imu":
            continue
        imu_idx += 1
        if imu_idx % imu_decimation != 0:
            continue
        event_t_ns = msg_time_ns(_bag_t_ns, msg)
        wz = float(msg.angular_velocity.z)
        steer = _sample_latest_before(steer_series, event_t_ns)
        rpm = _sample_latest_before(rpm_series, event_t_ns)
        if steer is None or rpm is None:
            continue
        vx = max(0.0, rpm * rpm_to_ms)
        if abs(steer) < 0.03 or vx < 2.0 or abs(wz) < 0.03:
            continue
        tan_s = math.tan(steer)
        tan_norm = math.tan(steer * max_steer_rad)
        wk_rad = -(vx / wheelbase_m) * tan_s
        wk_norm = -(vx / wheelbase_m) * tan_norm
        err_rad += (wz - wk_rad) ** 2
        err_norm += (wz - wk_norm) ** 2
        n += 1
        if n >= max_samples:
            break
    if n < 50:
        return "radians"
    # Bridge #462 publishes road-wheel radians on /steering_angle; only pick
    # normalized when it clearly fits the IMU gyro better.
    if err_norm < 0.25 * err_rad:
        return "normalized"
    return "radians"


def _sample_latest_before(
    series: list[tuple[int, float]],
    t_ns: int,
) -> float | None:
    """Last sample with stamp <= ``t_ns`` (holds through brief gaps)."""
    if not series:
        return None
    stamps = [row[0] for row in series]
    idx = bisect_right(stamps, t_ns) - 1
    if idx < 0:
        return None
    return series[idx][1]


def world_pose_to_aligned(
    gt_init_pose,
    x: float,
    y: float,
    yaw: float,
) -> tuple[float, float, float]:
    """Map an ENU ``/testing_only/odom`` pose into the SLAM/GT-aligned frame."""
    import gtsam
    import numpy as np

    p = gtsam.Pose3(gtsam.Rot3.Yaw(yaw), np.array([x, y, 0.0]))
    a = gt_init_pose.inverse().compose(p)
    return float(a.x()), float(a.y()), float(a.rotation().yaw())


def _state_to_pose3(x: float, y: float, yaw: float):
    import gtsam
    import numpy as np

    return gtsam.Pose3(gtsam.Rot3.Yaw(yaw), np.array([x, y, 0.0]))


def odom_frame_to_gt_aligned(
    sync_pose,
    x: float,
    y: float,
    yaw: float,
) -> tuple[float, float, float]:
    """Map sim_supervisor ``/odom`` (filter odom frame) into the GT-aligned frame.

    ``/odom`` integrates from (0,0,0) at filter calibration; SLAM/GT alignment uses
    the GT pose at SLAM calibration. ``sync_pose`` is the filter pose at that instant.
    """
    rel = sync_pose.inverse().compose(_state_to_pose3(x, y, yaw))
    return float(rel.x()), float(rel.y()), float(rel.rotation().yaw())


def aligned_gt_at_time(gt_init_pose, odom_msgs: list[tuple[int, object]], t_ns: int):
    """GT pose in the aligned frame at ``t_ns``, or None."""
    msg = odom_at_time(odom_msgs, t_ns)
    if msg is None:
        return None
    gx, gy, gyaw = world_pose_to_aligned(
        gt_init_pose,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        yaw_from_odom(msg),
    )
    return gx, gy, gyaw


def slam_pose_aligned(node) -> tuple[float, float, float] | None:
    """SLAM pose is already in the GT-aligned frame (origin at calibration)."""
    if node._gt_init_pose is None or node._latest_result is None:
        return None
    slam = node._latest_result.pose
    return float(slam.x()), float(slam.y()), float(slam.rotation().yaw())


def aggregate_pose_steps(
    steps: list[PoseStepSample],
    *,
    event: str | None = None,
    err_field: str = "filter_err_m",
) -> dict[str, Any]:
    """Summary stats for a subset of pose steps (e.g. IMU-only filter errors)."""
    vals = []
    yaw_vals = []
    deltas = []
    for s in steps:
        if event is not None and s.event != event:
            continue
        err = getattr(s, err_field, None)
        if err is None or math.isnan(err):
            continue
        vals.append(err)
        if err_field.startswith("filter"):
            yaw_f = "filter_yaw_err_rad"
            delta_f = "filter_err_delta_m"
        elif err_field.startswith("imu_only"):
            yaw_f = "imu_only_yaw_err_rad"
            delta_f = "imu_only_err_delta_m"
        elif err_field.startswith("wheel"):
            yaw_f = "wheel_yaw_err_rad"
            delta_f = None
        elif err_field.startswith("supervisor"):
            yaw_f = "supervisor_yaw_err_rad"
            delta_f = None
        elif err_field.startswith("slam"):
            yaw_f = "slam_yaw_err_rad"
            delta_f = None
        else:
            yaw_f = None
            delta_f = None
        if yaw_f is not None:
            y = getattr(s, yaw_f, None)
            if y is not None and not math.isnan(y):
                yaw_vals.append(abs(math.degrees(y)))
        if delta_f is not None:
            d = getattr(s, delta_f, None)
            if d is not None and not math.isnan(d):
                deltas.append(abs(d))
    if not vals:
        return {"steps": 0}
    vals_sorted = sorted(vals)
    p95 = vals_sorted[int(0.95 * (len(vals_sorted) - 1))]
    out: dict[str, Any] = {
        "steps": len(vals),
        "mean_err_m": sum(vals) / len(vals),
        "median_err_m": median(vals),
        "p95_err_m": p95,
        "max_err_m": max(vals),
    }
    if yaw_vals:
        out["mean_yaw_err_deg"] = sum(yaw_vals) / len(yaw_vals)
        out["max_yaw_err_deg"] = max(yaw_vals)
    if deltas:
        out["mean_step_delta_m"] = sum(deltas) / len(deltas)
        out["max_step_delta_m"] = max(deltas)
    return out


def _ros_time(stamp) -> object:
    """Coerce bag/interpolated stamps into builtin_interfaces.msg.Time."""
    from builtin_interfaces.msg import Time

    if isinstance(stamp, Time):
        return stamp
    t = Time()
    t.sec = int(stamp.sec)
    t.nanosec = int(stamp.nanosec)
    return t


def cones_to_marker_array(cones: list[Cone2D], stamp) -> object:
    from visualization_msgs.msg import Marker, MarkerArray

    ros_stamp = _ros_time(stamp)
    arr = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    clear.header.frame_id = "base_link"
    arr.markers.append(clear)
    for i, c in enumerate(cones):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = ros_stamp
        m.action = Marker.ADD
        m.type = Marker.CUBE
        m.id = i
        m.pose.position.x = float(c.x)
        m.pose.position.y = float(c.y)
        m.pose.orientation.w = 1.0
        m.scale.x = 0.12
        m.scale.y = 0.1
        m.scale.z = 0.3
        m.color.a = 1.0
        arr.markers.append(m)
    return arr


def aggregate_map_match(
    gt_map: list[MapPoint],
    slam_map: list[MapPoint],
    *,
    gate_m: float = 0.5,
) -> MapMatchStats:
    """Greedy nearest-neighbour match SLAM landmarks to GT cones (aligned frame)."""
    if not gt_map and not slam_map:
        return MapMatchStats()
    used_gt: set[int] = set()
    dists: list[float] = []
    for s in slam_map:
        best_i = None
        best_d = gate_m
        for i, g in enumerate(gt_map):
            if i in used_gt:
                continue
            d = pose_err_m(s.x, s.y, g.x, g.y)
            if d < best_d:
                best_d = d
                best_i = i
        if best_i is not None:
            used_gt.add(best_i)
            dists.append(best_d)
    matched = len(dists)
    stats = MapMatchStats(
        gt_cones=len(gt_map),
        slam_landmarks=len(slam_map),
        matched=matched,
        false_positive=len(slam_map) - matched,
        false_negative=len(gt_map) - matched,
    )
    if dists:
        stats.mean_match_err_m = sum(dists) / len(dists)
        stats.max_match_err_m = max(dists)
    return stats


def _extract_maps(
    node,
    world_track: list[WorldCone],
    *,
    min_observations: int = 3,
) -> tuple[list[MapPoint], list[MapPoint]]:
    """Final GT and SLAM cone maps in the GT-aligned frame."""
    if node._gt_init_pose is None:
        return [], []
    try:
        node._db.update_from_estimate(node._graph.landmark_position)
    except Exception:
        pass

    gt_map: list[MapPoint] = []
    for cone in world_track:
        ax, ay, _ = world_pose_to_aligned(node._gt_init_pose, cone.x, cone.y, 0.0)
        gt_map.append(MapPoint(x=ax, y=ay, color=cone.color))

    slam_map: list[MapPoint] = []
    for lm in node._db:
        if lm.n_observations < min_observations:
            continue
        slam_map.append(
            MapPoint(
                x=float(lm.position[0]),
                y=float(lm.position[1]),
                n_obs=int(lm.n_observations),
                landmark_id=int(lm.id),
            ),
        )
    return gt_map, slam_map


def map_to_rows(points: list[MapPoint], source: str) -> list[dict[str, float | int | str]]:
    """Fixed columns so GT and SLAM rows can be written in one CSV."""
    rows: list[dict[str, float | int | str]] = []
    for p in points:
        rows.append(
            {
                "source": source,
                "x": p.x,
                "y": p.y,
                "landmark_id": p.landmark_id if p.landmark_id is not None else "",
                "n_obs": p.n_obs if p.n_obs else "",
                "color": p.color if p.color is not None else "",
            },
        )
    return rows


def map_stats_to_dict(stats: MapMatchStats) -> dict[str, Any]:
    return {
        "gt_cones": stats.gt_cones,
        "slam_landmarks": stats.slam_landmarks,
        "matched": stats.matched,
        "false_positive": stats.false_positive,
        "false_negative": stats.false_negative,
        "mean_match_err_m": stats.mean_match_err_m,
        "max_match_err_m": stats.max_match_err_m,
    }


def aggregate_slam(samples: list[SlamSample]) -> dict[str, Any]:
    if not samples:
        return {"scans": 0}
    errs = [s.err_m for s in samples]
    yaw_deg = [abs(math.degrees(s.yaw_err_rad)) for s in samples]
    errs_sorted = sorted(errs)
    p95 = errs_sorted[int(0.95 * (len(errs_sorted) - 1))]
    return {
        "scans": len(samples),
        "mean_err_m": sum(errs) / len(errs),
        "median_err_m": median(errs),
        "p95_err_m": p95,
        "max_err_m": max(errs),
        "mean_yaw_err_deg": sum(yaw_deg) / len(yaw_deg),
        "median_yaw_err_deg": median(yaw_deg),
        "max_yaw_err_deg": max(yaw_deg),
    }


def _record_scan(node, t_s: float) -> SlamSample | None:
    if node._latest_result is None:
        return None
    if node._gt_init_pose is None or node._latest_gt is None:
        return None
    gt_now = odom_to_pose3(node._latest_gt)
    gt_aligned = node._gt_init_pose.inverse().compose(gt_now)
    slam = node._latest_result.pose
    err_m = float(
        math.hypot(slam.x() - gt_aligned.x(), slam.y() - gt_aligned.y()),
    )
    return SlamSample(
        t_s=t_s,
        step=float(node._graph.step),
        slam_x=float(slam.x()),
        slam_y=float(slam.y()),
        slam_yaw=float(slam.rotation().yaw()),
        gt_x=float(gt_aligned.x()),
        gt_y=float(gt_aligned.y()),
        gt_yaw=float(gt_aligned.rotation().yaw()),
        err_m=err_m,
        yaw_err_rad=yaw_err(slam.rotation().yaw(), gt_aligned.rotation().yaw()),
    )


def _record_traj(node, t_s: float) -> TrajectoryPoint | None:
    if node._gt_init_pose is None or node._latest_gt is None:
        return None
    gt_now = odom_to_pose3(node._latest_gt)
    gt_aligned = node._gt_init_pose.inverse().compose(gt_now)
    slam_x = slam_y = None
    err_m = None
    if node._latest_result is not None:
        slam = node._latest_result.pose
        slam_x = float(slam.x())
        slam_y = float(slam.y())
        err_m = float(math.hypot(slam_x - gt_aligned.x(), slam_y - gt_aligned.y()))
    return TrajectoryPoint(
        t_s=t_s,
        gt_x=float(gt_aligned.x()),
        gt_y=float(gt_aligned.y()),
        slam_x=slam_x,
        slam_y=slam_y,
        err_m=err_m,
    )


def _append_pose_step(
    out: ReplayResult,
    *,
    t_s: float,
    event: str,
    gt_init_pose,
    odom_msgs: list[tuple[int, object]],
    event_t_ns: int,
    filt,
    filter_sync_pose,
    imu_only_filt=None,
    imu_only_sync_pose=None,
    wheel_filt=None,
    wheel_sync_pose=None,
    supervisor_msg=None,
    slam_committed: bool = False,
    node=None,
    prev_filter_err: float | None,
    prev_imu_only_err: float | None = None,
) -> tuple[float | None, float | None]:
    """Record one pose-step row; return updated prev errors after IMU steps."""
    if gt_init_pose is None:
        return prev_filter_err, prev_imu_only_err
    gt = aligned_gt_at_time(gt_init_pose, odom_msgs, event_t_ns)
    if gt is None:
        return prev_filter_err, prev_imu_only_err
    gx, gy, gyaw = gt
    step = PoseStepSample(t_s=t_s, event=event, gt_x=gx, gt_y=gy, gt_yaw=gyaw)

    if filt is not None and filt.is_calibrated() and filter_sync_pose is not None:
        st = filt.state
        fx, fy, fyaw = odom_frame_to_gt_aligned(
            filter_sync_pose, st.x, st.y, st.yaw,
        )
        step.filter_x = fx
        step.filter_y = fy
        step.filter_yaw = fyaw
        step.filter_err_m = pose_err_m(fx, fy, gx, gy)
        step.filter_yaw_err_rad = yaw_err(fyaw, gyaw)
        step.filter_vx = st.vx
        step.filter_vy = st.vy
        step.filter_yaw_rate = st.yaw_rate
        if event == "imu" and prev_filter_err is not None:
            step.filter_err_delta_m = step.filter_err_m - prev_filter_err
        if event == "imu":
            prev_filter_err = step.filter_err_m

    if (
        imu_only_filt is not None
        and imu_only_filt.is_calibrated()
        and imu_only_sync_pose is not None
    ):
        st_io = imu_only_filt.state
        ix, iy, iyaw = odom_frame_to_gt_aligned(
            imu_only_sync_pose, st_io.x, st_io.y, st_io.yaw,
        )
        step.imu_only_x = ix
        step.imu_only_y = iy
        step.imu_only_yaw = iyaw
        step.imu_only_err_m = pose_err_m(ix, iy, gx, gy)
        step.imu_only_yaw_err_rad = yaw_err(iyaw, gyaw)
        step.imu_only_vx = st_io.vx
        step.imu_only_vy = st_io.vy
        if event == "imu" and prev_imu_only_err is not None:
            step.imu_only_err_delta_m = step.imu_only_err_m - prev_imu_only_err
        if event == "imu":
            prev_imu_only_err = step.imu_only_err_m

    if (
        wheel_filt is not None
        and wheel_filt.is_calibrated()
        and wheel_sync_pose is not None
    ):
        st_w = wheel_filt.state
        wx, wy, wyaw = odom_frame_to_gt_aligned(
            wheel_sync_pose, st_w.x, st_w.y, st_w.yaw,
        )
        step.wheel_x = wx
        step.wheel_y = wy
        step.wheel_yaw = wyaw
        step.wheel_err_m = pose_err_m(wx, wy, gx, gy)
        step.wheel_yaw_err_rad = yaw_err(wyaw, gyaw)
        step.wheel_vx = st_w.vx
        step.wheel_yaw_rate = st_w.yaw_rate

    if supervisor_msg is not None and filter_sync_pose is not None:
        sx, sy, syaw = odom_frame_to_gt_aligned(
            filter_sync_pose,
            supervisor_msg.pose.pose.position.x,
            supervisor_msg.pose.pose.position.y,
            yaw_from_odom(supervisor_msg),
        )
        step.supervisor_x = sx
        step.supervisor_y = sy
        step.supervisor_yaw = syaw
        step.supervisor_err_m = pose_err_m(sx, sy, gx, gy)
        step.supervisor_yaw_err_rad = yaw_err(syaw, gyaw)

    if node is not None:
        slam = slam_pose_aligned(node)
        if slam is not None:
            sx, sy, syaw = slam
            step.slam_x = sx
            step.slam_y = sy
            step.slam_yaw = syaw
            step.slam_err_m = pose_err_m(sx, sy, gx, gy)
            step.slam_yaw_err_rad = yaw_err(syaw, gyaw)
            step.slam_committed = slam_committed

    out.pose_steps.append(step)
    return prev_filter_err, prev_imu_only_err


def replay_slam(
    events: list[tuple[int, str, object]],
    *,
    odom_msgs: list[tuple[int, object]],
    world_track: list[WorldCone],
    strategy: str,
    sync_ns: int,  # kept for API compat; GT uses header-time odom interpolation
    gt_range_m: float = 20.0,
    gt_min_range_m: float = 0.5,
    gt_hfov_deg: float = 60.0,
    gt_scan_period_ns: int = 100_000_000,
    gt_scan_center_frac: float = 0.0,
    imu_time_scale: float | None = None,
) -> ReplayResult:
    import numpy as np
    import rclpy
    from cone_slam.cone_graph_slam_node import ConeGraphSlamNode
    from lifecycle_msgs.msg import State as StateMsg
    from rclpy.lifecycle import State as LifecycleState
    from odom_from_bag import IMU_DECIMATION
    from odometry_filter_cpp import (
        EkfParams,
        OdometryFilterCpp,
        steering_to_road_wheel_rad,
    )

    rclpy.init()
    node = ConeGraphSlamNode()
    stub = LifecycleState(StateMsg.PRIMARY_STATE_UNCONFIGURED, "unconfigured")
    node._behavior = strategy
    node.on_configure(stub)
    node.on_activate(stub)

    # The production /odom is published by the C++ 9-state EKF
    # (odometry_filter_node); sim_supervisor's Python complementary
    # OdometryFilter is now only a legacy fallback. Replay the EKF here
    # via the OdometryFilterCpp port so "EKF" and "IMU-only" match what
    # the pipeline actually runs. The legacy complementary filter
    # integrated centripetal accel-y straight into vy (no Coriolis
    # cross-term), so vy ran to several m/s in corners and the pose
    # diverged tens of metres — worse than wheel-only DR.
    filt = OdometryFilterCpp(EkfParams())
    imu_only_filt = OdometryFilterCpp(EkfParams())
    rpm_series = _scalar_series_from_bucket(
        {"/motor_rpm": [(b, m) for b, t, m in events if t == "/motor_rpm"]},
        "/motor_rpm",
    )
    steer_series = _scalar_series_from_bucket(
        {
            "/steering_angle": [
                (b, m) for b, t, m in events if t == "/steering_angle"
            ],
        },
        "/steering_angle",
    )
    steer_units = infer_steering_angle_units(
        events,
        rpm_series,
        steer_series,
        imu_decimation=IMU_DECIMATION,
    )
    wheel_filt = OdometryFilterCpp(EkfParams())

    # Rescale the IMU integration clock onto true-motion time (sim ran slower
    # than wall, so the bridge's node_->now() stamps over-count dt -> inflated
    # distance). None = auto-calibrate from GT twist-vs-path; 1.0 = off.
    if imu_time_scale is None:
        imu_scale, _scale_meta = estimate_imu_time_scale(odom_msgs)
    else:
        imu_scale = float(imu_time_scale)

    out = ReplayResult(
        wheel_steer_units=steer_units,
        imu_time_scale=imu_scale,
    )
    use_lidar_trigger = any(t == GT_CONE_TRIGGER_TOPIC for _, t, _ in events)
    imu_idx = 0
    prev_filter_err: float | None = None
    prev_imu_only_err: float | None = None
    filter_sync_pose = None  # filter /odom pose at SLAM GT-alignment instant
    imu_only_sync_pose = None
    wheel_sync_pose = None

    def _maybe_latch_filter_sync() -> None:
        nonlocal filter_sync_pose, imu_only_sync_pose, wheel_sync_pose
        if (
            filter_sync_pose is None
            and node._gt_init_pose is not None
            and filt.is_calibrated()
        ):
            st = filt.state
            filter_sync_pose = _state_to_pose3(st.x, st.y, st.yaw)
        if (
            imu_only_sync_pose is None
            and node._gt_init_pose is not None
            and imu_only_filt.is_calibrated()
        ):
            st_io = imu_only_filt.state
            imu_only_sync_pose = _state_to_pose3(st_io.x, st_io.y, st_io.yaw)
        if (
            wheel_sync_pose is None
            and node._gt_init_pose is not None
            and wheel_filt.is_calibrated()
        ):
            st_w = wheel_filt.state
            wheel_sync_pose = _state_to_pose3(st_w.x, st_w.y, st_w.yaw)

    try:
        for bag_t_ns, topic, msg in events:
            event_t_ns = msg_time_ns(bag_t_ns, msg)
            t_s = event_t_ns * 1e-9
            # Filter integration runs on the motion-time-corrected clock; GT
            # lookup / recording stay on the original (wall) event time.
            t_filt = t_s * imu_scale
            if topic == "/imu":
                imu_idx += 1
                if imu_idx % IMU_DECIMATION == 0:
                    accel = np.array(
                        [
                            msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z,
                        ],
                    )
                    gyro = np.array(
                        [
                            msg.angular_velocity.x,
                            msg.angular_velocity.y,
                            msg.angular_velocity.z,
                        ],
                    )
                    filt.push_imu(t_filt, accel, gyro)
                    imu_only_filt.push_imu(t_filt, accel, gyro)
                    if not wheel_filt.is_calibrated():
                        wheel_filt.push_imu(t_filt, accel, gyro)
                    else:
                        rv = _sample_latest_before(rpm_series, event_t_ns)
                        sv = _sample_latest_before(steer_series, event_t_ns)
                        rpm = (
                            rv
                            if rv is not None
                            else wheel_filt.latest_rpm
                        )
                        if sv is None:
                            steer = wheel_filt.latest_steering_rad
                            steer_units_for_sample = "radians"
                        else:
                            steer = sv
                            steer_units_for_sample = steer_units
                        wheel_filt.push_wheel_sensors(
                            t_filt,
                            rpm,
                            steer,
                            steering_units=steer_units_for_sample,
                        )
                # SLAM's own gtsam preintegrator integrates gyro*dt and
                # accel*dt straight off the IMU header stamp (and the
                # cone-trigger stamp below), so it must see the same
                # motion-time-compressed clock as the EKF ports above
                # (t_filt = t_s * imu_scale). Otherwise it integrates
                # over wall dt and over-rotates ~(1/scale - 1) of every
                # turn, blowing data association out of the gate after
                # corners (the post-turn SLAM detonation). Restamp the
                # message in place — replay is the last consumer of this
                # stamp, and the EKF ports already read accel/gyro arrays
                # directly. No-op when scale is off (1.0).
                if imu_scale != 1.0:
                    sns = int(event_t_ns * imu_scale)
                    msg.header.stamp.sec = sns // 1_000_000_000
                    msg.header.stamp.nanosec = sns % 1_000_000_000
                node._on_imu(msg)
                out.n_imu += 1
                _maybe_latch_filter_sync()
                if filt.is_calibrated() and node._gt_init_pose is not None:
                    prev_filter_err, prev_imu_only_err = _append_pose_step(
                        out,
                        t_s=t_s,
                        event="imu",
                        gt_init_pose=node._gt_init_pose,
                        odom_msgs=odom_msgs,
                        event_t_ns=event_t_ns,
                        filt=filt,
                        filter_sync_pose=filter_sync_pose,
                        imu_only_filt=imu_only_filt,
                        imu_only_sync_pose=imu_only_sync_pose,
                        wheel_filt=wheel_filt,
                        wheel_sync_pose=wheel_sync_pose,
                        node=node,
                        prev_filter_err=prev_filter_err,
                        prev_imu_only_err=prev_imu_only_err,
                    )
            elif topic == "/motor_rpm":
                rpm_val = float(msg.data)
                filt.push_rpm(t_s, rpm_val)
                if wheel_filt.is_calibrated():
                    wheel_filt.latest_rpm = rpm_val
                node._on_rpm(msg)
                out.n_rpm += 1
                _maybe_latch_filter_sync()
                if filt.is_calibrated() and node._gt_init_pose is not None:
                    prev_filter_err, prev_imu_only_err = _append_pose_step(
                        out,
                        t_s=t_s,
                        event="rpm",
                        gt_init_pose=node._gt_init_pose,
                        odom_msgs=odom_msgs,
                        event_t_ns=event_t_ns,
                        filt=filt,
                        filter_sync_pose=filter_sync_pose,
                        imu_only_filt=imu_only_filt,
                        imu_only_sync_pose=imu_only_sync_pose,
                        wheel_filt=wheel_filt,
                        wheel_sync_pose=wheel_sync_pose,
                        node=node,
                        prev_filter_err=prev_filter_err,
                        prev_imu_only_err=prev_imu_only_err,
                    )
            elif topic == "/steering_angle":
                angle = float(msg.data)
                road_angle = steering_to_road_wheel_rad(angle, units=steer_units)
                filt.push_steering(t_s, road_angle)
                wheel_filt.push_steering(t_s, road_angle)
            elif topic == "/odom":
                node._on_supervisor_odom(msg)
                out.n_supervisor_odom += 1
                _maybe_latch_filter_sync()
                if node._gt_init_pose is not None:
                    prev_filter_err, prev_imu_only_err = _append_pose_step(
                        out,
                        t_s=t_s,
                        event="supervisor_odom",
                        gt_init_pose=node._gt_init_pose,
                        odom_msgs=odom_msgs,
                        event_t_ns=event_t_ns,
                        filt=filt,
                        filter_sync_pose=filter_sync_pose,
                        imu_only_filt=imu_only_filt,
                        imu_only_sync_pose=imu_only_sync_pose,
                        wheel_filt=wheel_filt,
                        wheel_sync_pose=wheel_sync_pose,
                        supervisor_msg=msg,
                        node=node,
                        prev_filter_err=prev_filter_err,
                        prev_imu_only_err=prev_imu_only_err,
                    )
            elif topic == "/testing_only/odom":
                node._on_gt_odom(msg)
                out.n_odom += 1
                _maybe_latch_filter_sync()
                if node._gt_init_pose is not None:
                    prev_filter_err, prev_imu_only_err = _append_pose_step(
                        out,
                        t_s=t_s,
                        event="gt_odom",
                        gt_init_pose=node._gt_init_pose,
                        odom_msgs=odom_msgs,
                        event_t_ns=event_t_ns,
                        filt=filt,
                        filter_sync_pose=filter_sync_pose,
                        imu_only_filt=imu_only_filt,
                        imu_only_sync_pose=imu_only_sync_pose,
                        wheel_filt=wheel_filt,
                        wheel_sync_pose=wheel_sync_pose,
                        node=node,
                        prev_filter_err=prev_filter_err,
                        prev_imu_only_err=prev_imu_only_err,
                    )
                pt = _record_traj(node, t_s)
                if pt is not None:
                    out.trajectory.append(pt)
            elif topic == GT_CONE_TRIGGER_TOPIC and use_lidar_trigger:
                pass  # fall through to GT cone injection below
            elif not use_lidar_trigger and topic == "/testing_only/track":
                pass
            else:
                continue
            if topic in (GT_CONE_TRIGGER_TOPIC, "/testing_only/track"):
                if topic == GT_CONE_TRIGGER_TOPIC:
                    odom = odom_for_lidar_scan(
                        odom_msgs,
                        event_t_ns,
                        scan_period_ns=gt_scan_period_ns,
                        center_fraction=gt_scan_center_frac,
                    )
                else:
                    odom = odom_at_time(odom_msgs, event_t_ns)
                if odom is not None:
                    body = world_cones_to_body(
                        world_track,
                        odom,
                        range_m=gt_range_m,
                        min_range_m=gt_min_range_m,
                        hfov_half_deg=gt_hfov_deg,
                    )
                    if imu_scale != 1.0:
                        # Put the cone scan trigger on the same motion-time
                        # clock as the restamped IMU samples, so the
                        # preintegrator's integrate_to(t_scan) window lines
                        # up with them. (t_scan only drives preintegration
                        # timing; SLAM pose recording uses spatial frames,
                        # so this has no effect on the GT comparison.)
                        stamp = header_at_ns(int(event_t_ns * imu_scale)).stamp
                    else:
                        stamp = getattr(
                            getattr(odom, "header", None), "stamp", None)
                        if stamp is None:
                            stamp = header_at_ns(event_t_ns).stamp
                    markers = cones_to_marker_array(body, stamp)
                    node._on_cones(markers)
                    out.n_cone_updates += 1
                    _maybe_latch_filter_sync()
                    if node._gt_init_pose is not None:
                        prev_filter_err, prev_imu_only_err = _append_pose_step(
                            out,
                            t_s=t_s,
                            event="cone",
                            gt_init_pose=node._gt_init_pose,
                            odom_msgs=odom_msgs,
                            event_t_ns=event_t_ns,
                            filt=filt,
                            filter_sync_pose=filter_sync_pose,
                            imu_only_filt=imu_only_filt,
                            imu_only_sync_pose=imu_only_sync_pose,
                            wheel_filt=wheel_filt,
                            wheel_sync_pose=wheel_sync_pose,
                            node=node,
                            slam_committed=True,
                            prev_filter_err=prev_filter_err,
                            prev_imu_only_err=prev_imu_only_err,
                        )
                    sample = _record_scan(node, t_s)
                    if sample is not None:
                        out.samples.append(sample)
        try:
            min_obs = int(node.get_parameter("min_observations_for_publish").value)
        except Exception:
            min_obs = 3
        out.gt_map, out.slam_map = _extract_maps(
            node, world_track, min_observations=min_obs,
        )
        out.map_stats = aggregate_map_match(out.gt_map, out.slam_map)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return out


def samples_to_rows(samples: list[SlamSample]) -> list[dict[str, float]]:
    rows = []
    for s in samples:
        rows.append(
            {
                "t_s": s.t_s,
                "cone_source": "gt",
                "step": s.step,
                "slam_x": s.slam_x,
                "slam_y": s.slam_y,
                "slam_yaw": s.slam_yaw,
                "gt_x": s.gt_x,
                "gt_y": s.gt_y,
                "gt_yaw": s.gt_yaw,
                "err_m": s.err_m,
                "yaw_err_rad": s.yaw_err_rad,
            }
        )
    return rows


def pose_steps_to_rows(steps: list[PoseStepSample]) -> list[dict[str, float | str | bool]]:
    rows: list[dict[str, float | str | bool]] = []
    for s in steps:
        rows.append(
            {
                "t_s": s.t_s,
                "event": s.event,
                "gt_x": s.gt_x,
                "gt_y": s.gt_y,
                "gt_yaw": s.gt_yaw,
                "filter_x": s.filter_x if s.filter_x is not None else float("nan"),
                "filter_y": s.filter_y if s.filter_y is not None else float("nan"),
                "filter_yaw": s.filter_yaw if s.filter_yaw is not None else float("nan"),
                "filter_err_m": s.filter_err_m
                if s.filter_err_m is not None
                else float("nan"),
                "filter_yaw_err_rad": s.filter_yaw_err_rad
                if s.filter_yaw_err_rad is not None
                else float("nan"),
                "filter_err_delta_m": s.filter_err_delta_m
                if s.filter_err_delta_m is not None
                else float("nan"),
                "filter_vx": s.filter_vx if s.filter_vx is not None else float("nan"),
                "filter_vy": s.filter_vy if s.filter_vy is not None else float("nan"),
                "filter_yaw_rate": s.filter_yaw_rate
                if s.filter_yaw_rate is not None
                else float("nan"),
                "imu_only_x": s.imu_only_x if s.imu_only_x is not None else float("nan"),
                "imu_only_y": s.imu_only_y if s.imu_only_y is not None else float("nan"),
                "imu_only_yaw": s.imu_only_yaw
                if s.imu_only_yaw is not None
                else float("nan"),
                "imu_only_err_m": s.imu_only_err_m
                if s.imu_only_err_m is not None
                else float("nan"),
                "imu_only_yaw_err_rad": s.imu_only_yaw_err_rad
                if s.imu_only_yaw_err_rad is not None
                else float("nan"),
                "imu_only_err_delta_m": s.imu_only_err_delta_m
                if s.imu_only_err_delta_m is not None
                else float("nan"),
                "imu_only_vx": s.imu_only_vx
                if s.imu_only_vx is not None
                else float("nan"),
                "imu_only_vy": s.imu_only_vy
                if s.imu_only_vy is not None
                else float("nan"),
                "wheel_x": s.wheel_x if s.wheel_x is not None else float("nan"),
                "wheel_y": s.wheel_y if s.wheel_y is not None else float("nan"),
                "wheel_yaw": s.wheel_yaw if s.wheel_yaw is not None else float("nan"),
                "wheel_err_m": s.wheel_err_m if s.wheel_err_m is not None else float("nan"),
                "wheel_yaw_err_rad": s.wheel_yaw_err_rad
                if s.wheel_yaw_err_rad is not None
                else float("nan"),
                "wheel_vx": s.wheel_vx if s.wheel_vx is not None else float("nan"),
                "wheel_yaw_rate": s.wheel_yaw_rate
                if s.wheel_yaw_rate is not None
                else float("nan"),
                "supervisor_x": s.supervisor_x
                if s.supervisor_x is not None
                else float("nan"),
                "supervisor_y": s.supervisor_y
                if s.supervisor_y is not None
                else float("nan"),
                "supervisor_err_m": s.supervisor_err_m
                if s.supervisor_err_m is not None
                else float("nan"),
                "supervisor_yaw_err_rad": s.supervisor_yaw_err_rad
                if s.supervisor_yaw_err_rad is not None
                else float("nan"),
                "slam_x": s.slam_x if s.slam_x is not None else float("nan"),
                "slam_y": s.slam_y if s.slam_y is not None else float("nan"),
                "slam_yaw": s.slam_yaw if s.slam_yaw is not None else float("nan"),
                "slam_err_m": s.slam_err_m if s.slam_err_m is not None else float("nan"),
                "slam_yaw_err_rad": s.slam_yaw_err_rad
                if s.slam_yaw_err_rad is not None
                else float("nan"),
                "slam_committed": s.slam_committed,
            },
        )
    return rows


def trajectory_to_rows(traj: list[TrajectoryPoint]) -> list[dict[str, float]]:
    rows = []
    for p in traj:
        rows.append(
            {
                "t_s": p.t_s,
                "cone_source": "gt",
                "gt_x": p.gt_x,
                "gt_y": p.gt_y,
                "slam_x": p.slam_x if p.slam_x is not None else float("nan"),
                "slam_y": p.slam_y if p.slam_y is not None else float("nan"),
                "err_m": p.err_m if p.err_m is not None else float("nan"),
            }
        )
    return rows
