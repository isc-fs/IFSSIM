from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import dataclass, field
from statistics import median
from types import SimpleNamespace
from typing import Any


@dataclass
class Cone2D:
    x: float
    y: float
    color: int = 4  # fs_msgs UNKNOWN


@dataclass(frozen=True)
class WorldCone:
    """Cone position in odom/world frame (from latched /testing_only/track)."""

    x: float
    y: float
    color: int = 4


@dataclass
class MatchResult:
    pred_idx: int
    gt_idx: int
    err_m: float


@dataclass
class FrameMetrics:
    t_s: float
    latency_ms: float
    n_points: int
    n_gt: int
    n_pred: int
    n_tp: int
    n_fp: int
    n_fn: int
    mean_match_err_m: float
    max_match_err_m: float
    gt_cones: list[Cone2D] = field(default_factory=list)
    pred_cones: list[Cone2D] = field(default_factory=list)
    matches: list[MatchResult] = field(default_factory=list)


def yaw_from_odom(odom) -> float:
    q = odom.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def stamp_ns(msg) -> int:
    """ROS message header stamp in nanoseconds."""
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def msg_time_ns(bag_t_ns: int, msg: Any) -> int:
    """Prefer the message header stamp; fall back to the bag record time."""
    try:
        t = stamp_ns(msg)
        if t > 0:
            return t
    except (AttributeError, TypeError, ValueError):
        pass
    return bag_t_ns


def latch_track_layout(track_msgs: list[tuple[int, Any]]) -> list[WorldCone] | None:
    """First non-empty /testing_only/track (layout is static for the session)."""
    for _bag_t_ns, msg in sorted(
        track_msgs, key=lambda m: msg_time_ns(m[0], m[1])
    ):
        track = getattr(msg, "track", None)
        if not track:
            continue
        return [
            WorldCone(
                x=float(cone.location.x),
                y=float(cone.location.y),
                color=int(cone.color),
            )
            for cone in track
        ]
    return None


def filter_cones_in_fov(
    cones: list[Cone2D],
    *,
    range_m: float | None = 20.0,
    min_range_m: float = 0.5,
    hfov_half_deg: float = 60.0,
) -> list[Cone2D]:
    """Keep only body-frame cones inside the LiDAR visibility zone."""
    out: list[Cone2D] = []
    for c in cones:
        if cone_in_lidar_fov(
            c.x,
            c.y,
            hfov_half_deg=hfov_half_deg,
            min_range_m=min_range_m,
            max_range_m=range_m,
        ):
            out.append(c)
    return out


def cone_in_lidar_fov(
    bx: float,
    by: float,
    *,
    hfov_half_deg: float = 60.0,
    min_range_m: float = 0.0,
    max_range_m: float | None = None,
) -> bool:
    """True if a body-frame point is inside the LiDAR visibility zone."""
    r = math.hypot(bx, by)
    if r < min_range_m:
        return False
    if max_range_m is not None and r > max_range_m:
        return False
    if bx <= 0.0:
        return False
    half = math.radians(hfov_half_deg)
    return abs(math.atan2(by, bx)) <= half + 1e-9


def track_at_or_before(msgs: list[tuple[int, Any]], t_ns: int) -> Any | None:
    """Most recent latched message at or before ``t_ns`` (for /testing_only/track)."""
    best = None
    best_ts = -1
    for ts, msg in msgs:
        if ts <= t_ns and ts > best_ts:
            best_ts = ts
            best = msg
    return best


def world_cones_to_body(
    world: list[WorldCone],
    odom_msg,
    *,
    range_m: float | None = None,
    min_range_m: float = 0.5,
    hfov_half_deg: float | None = 60.0,
) -> list[Cone2D]:
    """Transform latched world cones into base_link at the given GT odom pose."""
    ox = odom_msg.pose.pose.position.x
    oy = odom_msg.pose.pose.position.y
    yaw = yaw_from_odom(odom_msg)
    c, s = math.cos(yaw), math.sin(yaw)
    out: list[Cone2D] = []
    for cone in world:
        dx = cone.x - ox
        dy = cone.y - oy
        bx = c * dx + s * dy
        by = -s * dx + c * dy
        r = math.hypot(bx, by)
        if min_range_m > 0.0 and r < min_range_m:
            continue
        if range_m is not None and r > range_m:
            continue
        if hfov_half_deg is not None and not cone_in_lidar_fov(
            bx, by, hfov_half_deg=hfov_half_deg, min_range_m=0.0
        ):
            continue
        out.append(Cone2D(x=bx, y=by, color=cone.color))
    return out


def track_cones_to_body(
    track_msg,
    odom_msg,
    *,
    range_m: float | None = None,
    min_range_m: float = 0.5,
    hfov_half_deg: float | None = 60.0,
) -> list[Cone2D]:
    """Transform sim GT cones (odom/world) into base_link using GT odom pose."""
    world = [
        WorldCone(
            x=float(cone.location.x),
            y=float(cone.location.y),
            color=int(cone.color),
        )
        for cone in track_msg.track
    ]
    return world_cones_to_body(
        world,
        odom_msg,
        range_m=range_m,
        min_range_m=min_range_m,
        hfov_half_deg=hfov_half_deg,
    )


def _interp_angle(a0: float, a1: float, alpha: float) -> float:
    d = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
    return a0 + alpha * d


def _slerp_quat(q0, q1, alpha: float):
    """Spherical interpolation between two unit quaternions (w, x, y, z)."""
    dot = (
        q0.w * q1.w
        + q0.x * q1.x
        + q0.y * q1.y
        + q0.z * q1.z
    )
    if dot < 0.0:
        dot = -dot
        q1 = SimpleNamespace(w=-q1.w, x=-q1.x, y=-q1.y, z=-q1.z)
    if dot > 0.9995:
        return SimpleNamespace(
            w=q0.w + alpha * (q1.w - q0.w),
            x=q0.x + alpha * (q1.x - q0.x),
            y=q0.y + alpha * (q1.y - q0.y),
            z=q0.z + alpha * (q1.z - q0.z),
        )
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta0 = math.sin(theta0)
    theta = theta0 * alpha
    s0 = math.sin(theta0 - theta) / sin_theta0
    s1 = math.sin(theta) / sin_theta0
    return SimpleNamespace(
        w=s0 * q0.w + s1 * q1.w,
        x=s0 * q0.x + s1 * q1.x,
        y=s0 * q0.y + s1 * q1.y,
        z=s0 * q0.z + s1 * q1.z,
    )


def header_at_ns(t_ns: int) -> SimpleNamespace:
    """Synthetic ROS header stamp for interpolated messages."""
    sec, nanosec = divmod(int(t_ns), 1_000_000_000)
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))


def odom_at_time(odom_msgs: list[tuple[int, Any]], t_ns: int) -> Any | None:
    """Interpolate /testing_only/odom pose at ``t_ns`` (header or bag time)."""
    if not odom_msgs:
        return None
    if t_ns <= odom_msgs[0][0]:
        return odom_msgs[0][1]
    if t_ns >= odom_msgs[-1][0]:
        return odom_msgs[-1][1]

    stamps = [row[0] for row in odom_msgs]
    idx = bisect_right(stamps, t_ns) - 1
    if idx < 0:
        return odom_msgs[0][1]
    t0, m0 = odom_msgs[idx]
    t1, m1 = odom_msgs[idx + 1]
    if t1 == t0:
        return m0

    alpha = (t_ns - t0) / (t1 - t0)
    p0 = m0.pose.pose.position
    p1 = m1.pose.pose.position
    q0 = m0.pose.pose.orientation
    q1 = m1.pose.pose.orientation
    q_mid = _slerp_quat(q0, q1, alpha)
    norm = math.sqrt(q_mid.w ** 2 + q_mid.x ** 2 + q_mid.y ** 2 + q_mid.z ** 2)
    if norm > 0:
        q_mid.w /= norm
        q_mid.x /= norm
        q_mid.y /= norm
        q_mid.z /= norm
    odom = SimpleNamespace(
        header=header_at_ns(t_ns),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(
                    x=p0.x + alpha * (p1.x - p0.x),
                    y=p0.y + alpha * (p1.y - p0.y),
                    z=p0.z + alpha * (p1.z - p0.z),
                ),
                orientation=q_mid,
            )
        ),
    )
    if hasattr(m0, "twist") and hasattr(m1, "twist"):
        tw0 = m0.twist.twist
        tw1 = m1.twist.twist
        odom.twist = SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(
                    x=tw0.linear.x + alpha * (tw1.linear.x - tw0.linear.x),
                    y=tw0.linear.y + alpha * (tw1.linear.y - tw0.linear.y),
                    z=tw0.linear.z + alpha * (tw1.linear.z - tw0.linear.z),
                ),
                angular=SimpleNamespace(
                    x=tw0.angular.x + alpha * (tw1.angular.x - tw0.angular.x),
                    y=tw0.angular.y + alpha * (tw1.angular.y - tw0.angular.y),
                    z=tw0.angular.z + alpha * (tw1.angular.z - tw0.angular.z),
                ),
            )
        )
    return odom


# Sim LiDAR is 10 Hz (settings.json RotationsPerSecond); GT pose at scan start
# misaligns the body frame vs points integrated over the sweep (~100 ms).
DEFAULT_LIDAR_SCAN_PERIOD_NS = 100_000_000


def advance_odom_pose(odom, dt_s: float):
    """Dead-reckon GT pose forward by ``dt_s`` using body-frame twist."""
    if dt_s <= 0.0:
        return odom
    yaw = yaw_from_odom(odom)
    twist = getattr(odom, "twist", None)
    if twist is None:
        return odom
    vx = float(twist.twist.linear.x)
    vy = float(twist.twist.linear.y)
    wz = float(twist.twist.angular.z)
    c, s = math.cos(yaw), math.sin(yaw)
    dx_w = (c * vx - s * vy) * dt_s
    dy_w = (s * vx + c * vy) * dt_s
    new_yaw = yaw + wz * dt_s
    cn, sn = math.cos(new_yaw), math.sin(new_yaw)
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    return SimpleNamespace(
        header=header_at_ns(0),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(
                    x=float(p.x) + dx_w,
                    y=float(p.y) + dy_w,
                    z=float(p.z),
                ),
                orientation=SimpleNamespace(
                    w=math.cos(new_yaw / 2),
                    x=0.0,
                    y=0.0,
                    z=math.sin(new_yaw / 2),
                ),
            )
        ),
        twist=odom.twist,
    )


def odom_for_lidar_scan(
    odom_msgs: list[tuple[int, Any]],
    scan_t_ns: int,
    *,
    scan_period_ns: int = DEFAULT_LIDAR_SCAN_PERIOD_NS,
    center_fraction: float = 0.0,
) -> Any | None:
    """GT odom pose advanced through the LiDAR sweep (0 = LiDAR header time)."""
    odom0 = odom_at_time(odom_msgs, scan_t_ns)
    if odom0 is None:
        return None
    dt_s = scan_period_ns * center_fraction * 1e-9
    if hasattr(odom0, "twist") and dt_s > 0.0:
        return advance_odom_pose(odom0, dt_s)
    offset_ns = int(scan_period_ns * center_fraction)
    return odom_at_time(odom_msgs, scan_t_ns + offset_ns)


def gt_cone_range_m(cone: Cone2D) -> float:
    """Euclidean distance from ego (base_link origin) to a body-frame GT cone."""
    return math.hypot(cone.x, cone.y)


def match_range_err_pairs(frames: list[FrameMetrics]) -> list[tuple[float, float]]:
    """(GT range m, match error m) for every matched pair across frames."""
    out: list[tuple[float, float]] = []
    for fm in frames:
        for m in fm.matches:
            g = fm.gt_cones[m.gt_idx]
            out.append((gt_cone_range_m(g), m.err_m))
    return out


def summarize_error_by_range(
    pairs: list[tuple[float, float]],
    *,
    bin_width_m: float = 2.0,
    max_range_m: float | None = None,
) -> list[dict[str, float | int]]:
    """Bin matched pairs by GT cone distance; report mean/median error per bin."""
    if not pairs or bin_width_m <= 0.0:
        return []
    if max_range_m is None:
        max_range_m = max(r for r, _ in pairs)
    n_bins = max(1, int(math.ceil(max_range_m / bin_width_m)))
    by_bin: list[list[float]] = [[] for _ in range(n_bins)]
    for r, err in pairs:
        idx = min(n_bins - 1, int(r // bin_width_m))
        by_bin[idx].append(err)
    out: list[dict[str, float | int]] = []
    for i, errs in enumerate(by_bin):
        if not errs:
            continue
        lo = i * bin_width_m
        out.append(
            {
                "bin_lo_m": lo,
                "bin_hi_m": lo + bin_width_m,
                "count": len(errs),
                "mean_err_m": sum(errs) / len(errs),
                "median_err_m": median(errs),
            }
        )
    return out


def summarize_match_bias(frames: list[FrameMetrics]) -> dict[str, float]:
    """Mean pred−GT offset over matched pairs (detects systematic BEV shift)."""
    dx: list[float] = []
    dy: list[float] = []
    for fm in frames:
        for m in fm.matches:
            p = fm.pred_cones[m.pred_idx]
            g = fm.gt_cones[m.gt_idx]
            dx.append(p.x - g.x)
            dy.append(p.y - g.y)
    if not dx:
        return {}
    return {
        "mean_bias_x_m": sum(dx) / len(dx),
        "mean_bias_y_m": sum(dy) / len(dy),
        "median_bias_x_m": median(dx),
        "median_bias_y_m": median(dy),
        "n_bias_pairs": float(len(dx)),
    }


def _dist(a: Cone2D, b: Cone2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def match_cones(
    pred: list[Cone2D],
    gt: list[Cone2D],
    *,
    gate_m: float = 1.5,
) -> tuple[list[MatchResult], list[int], list[int]]:
    """Greedy nearest-neighbor matching within gate_m (pred ↔ gt)."""
    if not pred and not gt:
        return [], [], []
    if not pred:
        return [], [], list(range(len(gt)))
    if not gt:
        return [], list(range(len(pred))), []

    pairs: list[tuple[float, int, int]] = []
    for pi, p in enumerate(pred):
        for gi, g in enumerate(gt):
            d = _dist(p, g)
            if d <= gate_m:
                pairs.append((d, pi, gi))
    pairs.sort(key=lambda t: t[0])

    used_p: set[int] = set()
    used_g: set[int] = set()
    matches: list[MatchResult] = []
    for d, pi, gi in pairs:
        if pi in used_p or gi in used_g:
            continue
        used_p.add(pi)
        used_g.add(gi)
        matches.append(MatchResult(pred_idx=pi, gt_idx=gi, err_m=d))

    fp = [i for i in range(len(pred)) if i not in used_p]
    fn = [i for i in range(len(gt)) if i not in used_g]
    return matches, fp, fn


def evaluate_frame(
    *,
    t_s: float,
    latency_ms: float,
    n_points: int,
    pred: list[Cone2D],
    gt: list[Cone2D],
    gate_m: float,
) -> FrameMetrics:
    matches, fp_idx, fn_idx = match_cones(pred, gt, gate_m=gate_m)
    errs = [m.err_m for m in matches]
    return FrameMetrics(
        t_s=t_s,
        latency_ms=latency_ms,
        n_points=n_points,
        n_gt=len(gt),
        n_pred=len(pred),
        n_tp=len(matches),
        n_fp=len(fp_idx),
        n_fn=len(fn_idx),
        mean_match_err_m=sum(errs) / len(errs) if errs else 0.0,
        max_match_err_m=max(errs) if errs else 0.0,
        gt_cones=gt,
        pred_cones=pred,
        matches=matches,
    )


def aggregate_metrics(frames: list[FrameMetrics]) -> dict[str, Any]:
    if not frames:
        return {"frames": 0}
    tp = sum(f.n_tp for f in frames)
    fp = sum(f.n_fp for f in frames)
    fn = sum(f.n_fn for f in frames)
    prec = tp / (tp + fp) if (tp + fp) else 0.0
    rec = tp / (tp + fn) if (tp + fn) else 0.0
    all_errs = [m.err_m for f in frames for m in f.matches]
    all_errs.sort()
    p95 = all_errs[int(0.95 * (len(all_errs) - 1))] if all_errs else 0.0
    latencies = [f.latency_ms for f in frames]
    gt_counts = [f.n_gt for f in frames]
    pred_counts = [f.n_pred for f in frames]
    return {
        "frames": len(frames),
        "total_tp": tp,
        "total_fp": fp,
        "total_fn": fn,
        "precision": prec,
        "recall": rec,
        "mean_match_err_m": sum(all_errs) / len(all_errs) if all_errs else 0.0,
        "median_match_err_m": median(all_errs) if all_errs else 0.0,
        "p95_match_err_m": p95,
        "max_match_err_m": max(all_errs) if all_errs else 0.0,
        "mean_latency_ms": sum(latencies) / len(latencies) if latencies else 0.0,
        "median_latency_ms": median(latencies) if latencies else 0.0,
        "max_latency_ms": max(latencies) if latencies else 0.0,
        "mean_gt_per_frame": sum(gt_counts) / len(gt_counts) if gt_counts else 0.0,
        "median_gt_per_frame": median(gt_counts) if gt_counts else 0.0,
        "mean_pred_per_frame": sum(pred_counts) / len(pred_counts) if pred_counts else 0.0,
        "median_pred_per_frame": median(pred_counts) if pred_counts else 0.0,
    }


def nearest_by_time(msgs: list[tuple[int, Any]], t_ns: int, max_delta_ns: int) -> Any | None:
    if not msgs:
        return None
    best = None
    best_dt = max_delta_ns + 1
    for ts, msg in msgs:
        dt = abs(ts - t_ns)
        if dt < best_dt:
            best_dt = dt
            best = msg
    return best if best_dt <= max_delta_ns else None
