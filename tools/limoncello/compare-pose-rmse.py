#!/usr/bin/env python3
"""Pose RMSE: LIMOncello's /limoncello/state vs GT /testing_only/odom.

Smoke-test analyzer for the LIMOncello SLAM evaluation. Reads a bag
that contains both topics, time-aligns them via nearest-neighbour
matching on header.stamp, anchors LIMOncello's pose to GT at the
first timestamp (since LIMOncello starts at its own world origin
after IMU calibration completes), computes per-axis residuals + RMSE
+ max error.

Acceptance criterion: ||pose error|| < 0.5 m RMSE across the lap.

## Usage

    python3 compare-pose-rmse.py <results.bag>

prints a summary table to stdout. Add --csv <path> to dump the
aligned (gt_x, gt_y, lim_x, lim_y, err_xy) series for offline plotting.

## Time alignment

LIMOncello publishes state at IMU rate (~325 Hz post-throttling),
GT odom at the sim's bridge rate (~83 Hz). For each GT sample we
pick the LIMOncello sample with the closest header.stamp; samples
more than 50 ms apart are dropped to avoid pulling a stale frame.

## Anchoring (SE(2) Umeyama)

LIMOncello starts at (0, 0, 0) in its own `map` frame and bootstraps
its yaw from the IMU (gravity-only, which constrains roll + pitch but
NOT yaw — so LIMOncello's yaw at t=0 is arbitrary relative to the
sim's world). GT publishes in the sim's world frame. So we cannot
just subtract a translation: the trajectories may differ by a
constant yaw offset.

We solve the rigid 2D alignment problem (Umeyama, 1991) — find the
SE(2) transform R·p + t that minimises sum-squared residuals when
applied to LIMOncello's xy series against GT's. Result: a single
rotation + translation, applied to every LIMOncello sample before
the RMSE computation. The reported "alignment yaw" is the rotation
angle in degrees — if it's near 0° the two frames already agreed,
near ±90° / 180° the IMU+LiDAR axis convention differs from the GT
publisher's convention.

If RMSE remains high AFTER alignment, that's the real signal that
LIMOncello isn't tracking — geometric divergence, not frame drift.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


LIM_TOPIC = "/limoncello/state"
GT_TOPIC = "/testing_only/odom"


def read_odom_series(bag_dir: Path) -> Tuple[List[Tuple[float, Tuple[float, float, float]]],
                                             List[Tuple[float, Tuple[float, float, float]]]]:
    """Return (lim_series, gt_series) — each a list of (stamp_s, (x, y, z))."""
    storage = rosbag2_py.StorageOptions(uri=str(bag_dir))
    converter = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    lim: List[Tuple[float, Tuple[float, float, float]]] = []
    gt: List[Tuple[float, Tuple[float, float, float]]] = []
    while reader.has_next():
        topic, data, _t_ns = reader.read_next()
        if topic == LIM_TOPIC:
            msg = deserialize_message(data, Odometry)
            s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            lim.append((s, (p.x, p.y, p.z)))
        elif topic == GT_TOPIC:
            msg = deserialize_message(data, Odometry)
            s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            gt.append((s, (p.x, p.y, p.z)))

    lim.sort(key=lambda r: r[0])
    gt.sort(key=lambda r: r[0])
    return lim, gt


def align_nearest(lim: List[Tuple[float, Tuple[float, float, float]]],
                  gt: List[Tuple[float, Tuple[float, float, float]]],
                  max_dt_s: float = 0.05) -> List[Tuple[float, Tuple[float, float, float], Tuple[float, float, float]]]:
    """Pair each GT sample with the nearest LIMOncello sample within max_dt_s."""
    if not lim or not gt:
        return []

    out = []
    i = 0
    for t_gt, p_gt in gt:
        # advance i to the LIMOncello sample nearest in time
        while i + 1 < len(lim) and abs(lim[i + 1][0] - t_gt) <= abs(lim[i][0] - t_gt):
            i += 1
        # back up one step if previous is closer (binary-search lite)
        if i > 0 and abs(lim[i - 1][0] - t_gt) < abs(lim[i][0] - t_gt):
            i -= 1
        if abs(lim[i][0] - t_gt) <= max_dt_s:
            out.append((t_gt, p_gt, lim[i][1]))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", type=Path, help="Results bag (with both topics).")
    ap.add_argument("--csv", type=Path, default=None,
                    help="Write per-sample residuals to this CSV.")
    ap.add_argument("--max-dt-ms", type=float, default=50.0,
                    help="Drop pairs whose stamps differ by more than this (ms).")
    args = ap.parse_args()

    lim, gt = read_odom_series(args.bag.expanduser().resolve())
    if not lim:
        print(f"error: no {LIM_TOPIC} messages in bag", file=sys.stderr)
        return 1
    if not gt:
        print(f"error: no {GT_TOPIC} messages in bag", file=sys.stderr)
        return 1

    pairs = align_nearest(lim, gt, max_dt_s=args.max_dt_ms * 1e-3)
    if not pairs:
        print(f"error: no time-aligned pairs (max-dt {args.max_dt_ms} ms too tight?)",
              file=sys.stderr)
        return 1

    # Umeyama SE(2) alignment of LIMOncello's xy series onto GT's xy
    # series. The classic closed form (Umeyama 1991, eqs. 34-43) for
    # n point-pairs:
    #   μ_g, μ_l : centroids
    #   M = Σ (g_i - μ_g)(l_i - μ_l)^T
    #   R = U V^T from SVD(M)
    #   t = μ_g - R μ_l
    # We do 2D so the math is hand-coded — no numpy dependency outside
    # the bag reader (already available in the container).
    gx_arr = [g[0] for _t, g, _l in pairs]
    gy_arr = [g[1] for _t, g, _l in pairs]
    lx_arr = [l[0] for _t, _g, l in pairs]
    ly_arr = [l[1] for _t, _g, l in pairs]
    n = len(pairs)
    mu_gx, mu_gy = sum(gx_arr) / n, sum(gy_arr) / n
    mu_lx, mu_ly = sum(lx_arr) / n, sum(ly_arr) / n
    # 2×2 cross-cov matrix M (g - μ_g)·(l - μ_l)^T summed over samples.
    m00 = sum((gx_arr[i] - mu_gx) * (lx_arr[i] - mu_lx) for i in range(n))
    m01 = sum((gx_arr[i] - mu_gx) * (ly_arr[i] - mu_ly) for i in range(n))
    m10 = sum((gy_arr[i] - mu_gy) * (lx_arr[i] - mu_lx) for i in range(n))
    m11 = sum((gy_arr[i] - mu_gy) * (ly_arr[i] - mu_ly) for i in range(n))
    # For 2D Umeyama with R ∈ SO(2), the optimal rotation angle is
    # atan2(m10 - m01, m00 + m11). (Derives from the SVD closed-form
    # for 2×2 by inspection; the off-diagonal anti-symmetric part
    # picks out the rotation, the symmetric part the scale we don't
    # need since we force det(R) = 1.)
    yaw = math.atan2(m10 - m01, m00 + m11)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    # t = μ_g - R μ_l
    tx = mu_gx - (cos_y * mu_lx - sin_y * mu_ly)
    ty = mu_gy - (sin_y * mu_lx + cos_y * mu_ly)
    yaw_deg = math.degrees(yaw)

    # Z just gets a translation — gravity is correctly observed in
    # the IMU calibration window, so vertical drift is bounded and
    # one-shot.
    z_offset = pairs[0][1][2] - pairs[0][2][2]

    t0 = pairs[0][0]
    errs_xy: List[float] = []
    errs_xyz: List[float] = []
    rows: List[Tuple[float, float, float, float, float, float, float]] = []

    for t, (gx, gy, gz), (lx, ly, lz) in pairs:
        # Apply the SE(2) alignment: R · (lx, ly) + (tx, ty).
        lx_a = cos_y * lx - sin_y * ly + tx
        ly_a = sin_y * lx + cos_y * ly + ty
        lz_a = lz + z_offset
        ex, ey, ez = lx_a - gx, ly_a - gy, lz_a - gz
        err_xy = math.sqrt(ex * ex + ey * ey)
        err_xyz = math.sqrt(ex * ex + ey * ey + ez * ez)
        errs_xy.append(err_xy)
        errs_xyz.append(err_xyz)
        rows.append((t - t0, gx, gy, lx_a, ly_a, err_xy, err_xyz))

    rmse_xy = math.sqrt(sum(e * e for e in errs_xy) / len(errs_xy))
    rmse_xyz = math.sqrt(sum(e * e for e in errs_xyz) / len(errs_xyz))
    max_xy = max(errs_xy)
    max_xyz = max(errs_xyz)

    final_err_xy = errs_xy[-1]
    duration = rows[-1][0]

    print(f"LIMOncello pose evaluation against {GT_TOPIC}")
    print(f"  bag:                  {args.bag.name}")
    print(f"  duration:             {duration:.1f} s")
    print(f"  paired samples:       {len(pairs)} (gt:{len(gt)} lim:{len(lim)})")
    print(f"  SE(2) alignment:      yaw={yaw_deg:+.2f}°  t=({tx:+.3f}, {ty:+.3f})")
    print(f"  z-offset:             {z_offset:+.3f} m")
    print()
    print(f"  RMSE (xy):            {rmse_xy:6.3f} m")
    print(f"  RMSE (xyz):           {rmse_xyz:6.3f} m")
    print(f"  Max  (xy):            {max_xy:6.3f} m")
    print(f"  Max  (xyz):           {max_xyz:6.3f} m")
    print(f"  Final-frame (xy):     {final_err_xy:6.3f} m")
    print()
    if rmse_xy < 0.5:
        print(f"  PASS — RMSE < 0.5 m. LIMOncello tracks FS-DV scenes cleanly.")
    elif rmse_xy < 1.5:
        print(f"  MARGINAL — RMSE 0.5–1.5 m. Tracks but tuning may help.")
    else:
        print(f"  FAIL — RMSE > 1.5 m. LIMOncello not tracking cleanly on this bag.")

    if args.csv:
        with open(args.csv, "w") as f:
            f.write("t,gt_x,gt_y,lim_x,lim_y,err_xy,err_xyz\n")
            for r in rows:
                f.write(",".join(f"{v:.6f}" for v in r) + "\n")
        print(f"  csv written → {args.csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
