from __future__ import annotations

import math
from bisect import bisect_right
from collections.abc import Iterable
from dataclasses import asdict, dataclass, field
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
    # Big-orange (finish-gate) classification. A detection is big orange when
    # the detector's fitted height exceeds the strategy threshold — the same
    # rule cone_detection_node uses to route cones onto /Conos_Orange.
    n_pred_big: int = 0
    n_big_tp: int = 0
    # Detected as big orange but not a big-orange GT cone: either unmatched or
    # matched to a small cone. These are what can trip the stop latch.
    n_false_big: int = 0
    # Big-orange GT cone in view that was not detected as big orange.
    n_missed_big: int = 0


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


def header_stamp_ns(msg: Any) -> int | None:
    """Header stamp in ns, or ``None`` when the message has no usable header."""
    try:
        t = stamp_ns(msg)
    except (AttributeError, TypeError, ValueError):
        return None
    return t if t > 0 else None


def compute_bag_sim_offset_ns(items: Iterable[tuple[int, Any]]) -> int:
    """Median ``bag_record_ns - header_stamp_ns`` over messages with a header.

    Bridge "Option 2" stamps headered topics (``/imu``, ``/lidar/*``,
    ``/testing_only/odom``) with absolute UE sim time, while headerless
    ``std_msgs/Float32`` topics (``/motor_rpm``, ``/steering_angle``,
    ``/brake_pressure``) only carry the bag record (wall-clock) time. Those two
    clocks differ by ~1.78e18 ns, so mixing them makes the headerless samples
    sort to the end of the run and never associate with IMU events. This offset
    maps bag time back onto the sim clock. Returns 0 when no headered messages
    are present (degrades to the previous behaviour).
    """
    diffs: list[int] = []
    for bag_t_ns, msg in items:
        h = header_stamp_ns(msg)
        if h is not None:
            diffs.append(int(bag_t_ns) - h)
    if not diffs:
        return 0
    diffs.sort()
    return diffs[len(diffs) // 2]


def aligned_time_ns(bag_t_ns: int, msg: Any, offset_ns: int) -> int:
    """Sim-clock time for a message.

    Headered messages use their header stamp directly; headerless ones are
    mapped via ``bag_t_ns - offset_ns`` (see :func:`compute_bag_sim_offset_ns`)
    so they share the sim clock used by ``/imu`` and ``/testing_only/odom``.
    """
    h = header_stamp_ns(msg)
    if h is not None:
        return h
    return int(bag_t_ns) - offset_ns


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


# Physical cone heights (m) for the GT vertical-FOV visibility gate. FSAE small
# cones ≈0.325 m, big orange ≈0.505 m; defaults match the detector's apex
# templates (cone_fit ``_CONE_SMALL_D`` / ``_CONE_BIG_D``).
CONE_HEIGHT_SMALL_M = 0.35
CONE_HEIGHT_BIG_M = 0.55
FS_CONE_ORANGE_BIG = 2  # fs_msgs Cone.ORANGE_BIG
FS_CONE_UNKNOWN = 4  # fs_msgs Cone.UNKNOWN; detection does not assign blue/yellow


def cone_in_vertical_fov(
    r_m: float,
    cone_height_m: float,
    *,
    lidar_height_m: float,
    vfov_lower_deg: float,
    vfov_upper_deg: float = 90.0,
) -> bool:
    """True if a ground cone's vertical extent intersects the LiDAR's vertical FOV.

    A cone sitting on the ground at horizontal range ``r_m`` spans sensor-relative
    elevation ``atan2(-lidar_height_m, r)`` (its base) up to
    ``atan2(cone_height_m - lidar_height_m, r)`` (its tip). With a high mount and a
    close cone the whole span drops below ``vfov_lower_deg``, so no beam can hit it
    — the near-field blind cone (e.g. h=1.1 m, vlower=-12.4° → small cones blind
    within ~3.4 m). The GT must not score cones the sensor physically cannot see.
    """
    if r_m <= 0.0:
        return False
    base_deg = math.degrees(math.atan2(-lidar_height_m, r_m))
    top_deg = math.degrees(math.atan2(cone_height_m - lidar_height_m, r_m))
    return top_deg >= vfov_lower_deg and base_deg <= vfov_upper_deg


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
    lidar_height_m: float | None = None,
    vfov_lower_deg: float | None = None,
    vfov_upper_deg: float = 90.0,
    cone_height_small_m: float = CONE_HEIGHT_SMALL_M,
    cone_height_big_m: float = CONE_HEIGHT_BIG_M,
) -> list[Cone2D]:
    """Transform latched world cones into base_link at the given GT odom pose.

    When ``lidar_height_m`` and ``vfov_lower_deg`` are given, cones whose vertical
    extent falls entirely outside the LiDAR vertical FOV are dropped (see
    :func:`cone_in_vertical_fov`) so the GT only counts physically visible cones.
    """
    ox = odom_msg.pose.pose.position.x
    oy = odom_msg.pose.pose.position.y
    yaw = yaw_from_odom(odom_msg)
    c, s = math.cos(yaw), math.sin(yaw)
    vfov_on = lidar_height_m is not None and vfov_lower_deg is not None
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
        if vfov_on:
            ch = (
                cone_height_big_m
                if cone.color == FS_CONE_ORANGE_BIG
                else cone_height_small_m
            )
            if not cone_in_vertical_fov(
                r,
                ch,
                lidar_height_m=lidar_height_m,
                vfov_lower_deg=vfov_lower_deg,
                vfov_upper_deg=vfov_upper_deg,
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


def classify_detections(cones: Iterable[Any], big_orange_threshold_m: float) -> list[Cone2D]:
    """Detector observations (``x``, ``y``, ``height_m``) to ``Cone2D``.

    Big orange iff ``height_m > big_orange_threshold_m`` — strictly greater,
    exactly as ``cone_detection_node._cones_to_markers`` routes /Conos_Orange.
    Everything else is ``UNKNOWN``: blue/yellow is assigned later, by SLAM.
    """
    return [
        Cone2D(
            x=float(c.x),
            y=float(c.y),
            color=FS_CONE_ORANGE_BIG if c.height_m > big_orange_threshold_m else FS_CONE_UNKNOWN,
        )
        for c in cones
    ]


def _big_orange_counts(
    pred: list[Cone2D],
    gt: list[Cone2D],
    matches: list[MatchResult],
    fp_idx: list[int],
    fn_idx: list[int],
) -> tuple[int, int, int, int]:
    """(n_pred_big, n_big_tp, n_false_big, n_missed_big) for one frame.

    Matching is by position only, so a big-orange detection sitting on a small
    cone matches it and counts as a false big orange (a size error), not a TP.
    """
    def is_big(c: Cone2D) -> bool:
        return c.color == FS_CONE_ORANGE_BIG

    tp = false_big = missed = 0
    for m in matches:
        p_big, g_big = is_big(pred[m.pred_idx]), is_big(gt[m.gt_idx])
        if p_big and g_big:
            tp += 1
        elif p_big:
            false_big += 1
        elif g_big:
            missed += 1
    false_big += sum(1 for i in fp_idx if is_big(pred[i]))
    missed += sum(1 for i in fn_idx if is_big(gt[i]))
    return sum(1 for c in pred if is_big(c)), tp, false_big, missed


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
    n_pred_big, n_big_tp, n_false_big, n_missed_big = _big_orange_counts(
        pred, gt, matches, fp_idx, fn_idx
    )
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
        n_pred_big=n_pred_big,
        n_big_tp=n_big_tp,
        n_false_big=n_false_big,
        n_missed_big=n_missed_big,
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
        **_aggregate_big_orange(frames),
    }


def _aggregate_big_orange(frames: list[FrameMetrics]) -> dict[str, Any]:
    big_tp = sum(f.n_big_tp for f in frames)
    false_big = [f.n_false_big for f in frames]
    missed_big = sum(f.n_missed_big for f in frames)
    total_false = sum(false_big)
    return {
        "total_pred_big": sum(f.n_pred_big for f in frames),
        "total_big_tp": big_tp,
        "total_false_big": total_false,
        "total_missed_big": missed_big,
        # None, not 0.0, when there is nothing to score: a run with no big
        # oranges in view has no big-orange precision, rather than a bad one.
        "big_orange_precision": big_tp / (big_tp + total_false) if (big_tp + total_false) else None,
        "big_orange_recall": big_tp / (big_tp + missed_big) if (big_tp + missed_big) else None,
        "false_big_per_frame": total_false / len(frames),
        "max_false_big_per_frame": max(false_big),
        "frames_with_false_big": sum(1 for n in false_big if n >= 1),
        # The stop latch fires on >= 2 big-orange cones in ONE message, so a
        # frame with two false big oranges can stop the car by itself.
        "frames_with_2plus_false_big": sum(1 for n in false_big if n >= 2),
    }


@dataclass
class LatchEvent:
    t_s: float
    anchor_x: float
    anchor_y: float
    n_cones: int
    travelled_m: float
    nearest_gt_big_m: float | None
    premature: bool | None


class StopLatchReplay:
    """Offline replay of ``control_node._on_orange``'s stop latch.

    Feed every LiDAR frame, in order, with the car's world pose and the frame's
    big-orange detections (unfiltered, as /Conos_Orange carries them). The rule,
    in the node's order: ignore fewer than 2 cones; ignore until the car has
    travelled ``min_travel_m``; ignore while ``final_lap`` is false (trackdrive
    lap gate, from /slam/final_lap, true when absent). Then latch once, for
    good, at the cones' centroid projected into the world frame.

    A latch is *premature* when that anchor is farther than ``finish_tol_m``
    from every ground-truth big-orange cone: the car would stop somewhere that
    is not a gate. ``premature`` is None when there is no big-orange ground
    truth to judge against.

    Approximation: the node accumulates travel from SLAM pose at control rate;
    here it accumulates from the ground-truth pose at LiDAR rate.
    """

    def __init__(
        self,
        *,
        min_travel_m: float,
        gt_big_world: list[tuple[float, float]],
        finish_tol_m: float = 3.0,
    ) -> None:
        self.min_travel_m = min_travel_m
        self.finish_tol_m = finish_tol_m
        self.gt_big_world = list(gt_big_world)
        self.travelled_m = 0.0
        self.frames = 0
        self.event: LatchEvent | None = None
        self._last_xy: tuple[float, float] | None = None

    def update(
        self,
        *,
        t_s: float,
        x: float,
        y: float,
        yaw: float,
        big_cones_body: list[Cone2D],
        final_lap: bool = True,
    ) -> LatchEvent | None:
        self.frames += 1
        if self._last_xy is not None:
            self.travelled_m += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        if self.event is not None:
            return None
        if len(big_cones_body) < 2:
            return None
        if self.travelled_m < self.min_travel_m:
            return None
        if not final_lap:
            return None
        n = len(big_cones_body)
        sx = sum(c.x for c in big_cones_body) / n
        sy = sum(c.y for c in big_cones_body) / n
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        ax = x + cos_y * sx - sin_y * sy
        ay = y + sin_y * sx + cos_y * sy
        nearest = (
            min(math.hypot(ax - gx, ay - gy) for gx, gy in self.gt_big_world)
            if self.gt_big_world
            else None
        )
        self.event = LatchEvent(
            t_s=t_s,
            anchor_x=ax,
            anchor_y=ay,
            n_cones=n,
            travelled_m=self.travelled_m,
            nearest_gt_big_m=nearest,
            premature=None if nearest is None else nearest > self.finish_tol_m,
        )
        return self.event

    def summary(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "evaluated": True,
            "min_travel_m": self.min_travel_m,
            "finish_tol_m": self.finish_tol_m,
            "frames": self.frames,
            "travelled_m": self.travelled_m,
            "latched": self.event is not None,
            "premature": self.event.premature if self.event else False,
        }
        if self.event is not None:
            out["event"] = asdict(self.event)
        return out


def scan_stats(result: Any) -> dict[str, float]:
    """Per-scan structure from a cone_detection ``DetectionResult``.

    ``rotated_xyz`` is the cropped scan after ground rotation and
    ``outlier_xyz`` its non-ground (RANSAC outlier) points, so the ground
    fraction is within the detector's input crop. Missing fields give NaN.
    """
    n_scan = len(getattr(result, "rotated_xyz", ()))
    n_above = len(getattr(result, "outlier_xyz", ()))
    counters = getattr(result, "debug_counters", None) or {}
    return {
        "n_clusters": float(counters.get("n_clusters", math.nan)),
        "ground_fraction": (1.0 - n_above / n_scan) if n_scan else math.nan,
        "above_ground_points": float(n_above),
    }


def aggregate_scan_stats(rows: list[dict[str, float]]) -> dict[str, float]:
    out: dict[str, float] = {}
    for key in ("n_clusters", "ground_fraction", "above_ground_points"):
        vals = sorted(r[key] for r in rows if key in r and not math.isnan(r[key]))
        if not vals:
            continue
        out[f"mean_{key}"] = sum(vals) / len(vals)
        out[f"p95_{key}"] = vals[int(0.95 * (len(vals) - 1))]
    return out


# Metrics compared by ``reference_deltas``: (section of the summary, key).
REFERENCE_KEYS: tuple[tuple[str, str], ...] = (
    ("gt_metrics", "recall"),
    ("gt_metrics", "precision"),
    ("gt_metrics", "big_orange_recall"),
    ("gt_metrics", "big_orange_precision"),
    ("gt_metrics", "false_big_per_frame"),
    ("gt_metrics", "frames_with_2plus_false_big"),
    ("gt_metrics", "mean_match_err_m"),
    ("scan_stats", "mean_n_clusters"),
    ("scan_stats", "mean_ground_fraction"),
    ("scan_stats", "mean_above_ground_points"),
    ("stop_latch", "premature"),
)


def reference_deltas(
    current: dict[str, Any],
    reference: dict[str, Any],
    keys: tuple[tuple[str, str], ...] = REFERENCE_KEYS,
) -> dict[str, dict[str, Any]]:
    """Each key present in both runs: value, reference value and numeric delta."""
    out: dict[str, dict[str, Any]] = {}
    for section, key in keys:
        cur = (current.get(section) or {}).get(key)
        ref = (reference.get(section) or {}).get(key)
        if cur is None or ref is None:
            continue
        row: dict[str, Any] = {"value": cur, "reference": ref}
        if isinstance(cur, (int, float)) and not isinstance(cur, bool) and isinstance(
            ref, (int, float)
        ) and not isinstance(ref, bool):
            row["delta"] = cur - ref
        out[key] = row
    return out


def pick_scan_center_fraction(
    odom_msgs: list[tuple[int, Any]],
    scan_ts: list[int],
    preds: list[list[Cone2D]],
    world_track: list[WorldCone],
    *,
    scan_period_ns: int = DEFAULT_LIDAR_SCAN_PERIOD_NS,
    gate_m: float = 1.5,
    gt_gate: dict[str, Any] | None = None,
    candidates: tuple[float, ...] = (0.0,),
) -> tuple[float, dict[str, float]]:
    """Diagnostic: sweep the GT scan-center offset and return the best-aligned one.

    For each ``center_fraction`` in ``candidates`` this advances the GT odom
    pose through the LiDAR sweep (``odom_for_lidar_scan``), projects the latched
    world track into body frame, and greedy-matches it against the per-scan
    detector predictions. The candidate is scored by F1 (primary, so it favours
    offsets where many cones actually align rather than one lucky match), with
    mean match error as the tie-break. Returns ``(best_fraction, meta)`` where
    ``meta`` carries the winner's ``calib_*`` metrics consumed by
    ``diagnose_perception_timing.py``.

    With the bridge stamping absolute sim capture time (Option 2), the best
    fraction should sit near 0 across the whole bag — a non-zero, time-varying
    winner here would mean the LiDAR stamps are still misaligned.
    """
    gate = dict(gt_gate or {})
    period_ms = scan_period_ns * 1e-6

    def _empty_meta(frac: float) -> dict[str, float]:
        return {
            "calib_scan_offset_ms": frac * period_ms,
            "calib_mean_match_err_m": 0.0,
            "calib_f1": 0.0,
            "calib_bias_x_m": 0.0,
            "calib_bias_y_m": 0.0,
            "calib_bias_pairs": 0.0,
        }

    if not scan_ts or not candidates:
        return 0.0, _empty_meta(0.0)

    best_frac = 0.0
    best_meta = _empty_meta(0.0)
    best_key: tuple[float, float] | None = None  # (f1 desc, -err) — higher is better

    for frac in candidates:
        frames: list[FrameMetrics] = []
        for scan_t_ns, pred in zip(scan_ts, preds):
            odom = odom_for_lidar_scan(
                odom_msgs,
                scan_t_ns,
                scan_period_ns=scan_period_ns,
                center_fraction=frac,
            )
            if odom is None:
                continue
            gt = world_cones_to_body(world_track, odom, **gate)
            frames.append(
                evaluate_frame(
                    t_s=scan_t_ns * 1e-9,
                    latency_ms=0.0,
                    n_points=0,
                    pred=pred,
                    gt=gt,
                    gate_m=gate_m,
                )
            )
        if not frames:
            continue
        agg = aggregate_metrics(frames)
        bias = summarize_match_bias(frames)
        prec, rec = agg["precision"], agg["recall"]
        f1 = 2.0 * prec * rec / (prec + rec) if (prec + rec) > 0.0 else 0.0
        mean_err = agg["mean_match_err_m"]
        key = (f1, -mean_err)
        if best_key is None or key > best_key:
            best_key = key
            best_frac = frac
            best_meta = {
                "calib_scan_offset_ms": frac * period_ms,
                "calib_mean_match_err_m": mean_err,
                "calib_f1": f1,
                "calib_bias_x_m": bias.get("mean_bias_x_m", 0.0),
                "calib_bias_y_m": bias.get("mean_bias_y_m", 0.0),
                "calib_bias_pairs": bias.get("n_bias_pairs", 0.0),
            }

    return best_frac, best_meta


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
