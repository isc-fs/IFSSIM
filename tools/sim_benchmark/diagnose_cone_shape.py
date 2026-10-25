"""Estimate the simulator's real cone shape (slope c, apex height d) per cone type.

The template fit assumes z = d - c * r with fixed (c, d) per type. This script
measures what the simulated cones actually look like: for every scan, clusters
are produced by the production ground removal + DBSCAN (ground plane at z=0),
matched to the GT track (/testing_only/track at the scan's GT odom pose) to get
their type, and every cluster with enough points is fit with the free
4-parameter ``cone_fit_varpro``. Repeating over all scans and cones gives a
distribution per type rather than a single noisy fit.

Cross-check: one-sided clusters leave VARPRO's (a, b) poorly constrained along
the line of sight, which trades off against c. Holding (a, b) at the GT cone
position turns (c, d) into a plain linear regression of z on r, so both
estimates are reported; agreement means the shape estimate is trustworthy.

Example::

    python tools/sim_benchmark/diagnose_cone_shape.py results/capture/<sim_bag>
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from common import default_results_root, maybe_reexec_in_docker, resolve_benchmark_path, write_json
from perception_metrics import (
    DEFAULT_LIDAR_SCAN_PERIOD_NS,
    FS_CONE_ORANGE_BIG,
    latch_track_layout,
    msg_time_ns,
    odom_for_lidar_scan,
    world_cones_to_body,
)
from run_perception_benchmark import _pointcloud_to_xyz, _read_bag_by_topic

# fs_msgs Cone colours (see perception_metrics.FS_CONE_ORANGE_BIG).
COLOR_NAMES = {0: "blue", 1: "yellow", 2: "orange_big", 3: "orange_small", 4: "unknown"}


def _type_of(color: int) -> str:
    return "big" if color == FS_CONE_ORANGE_BIG else "small"


def _stats(values) -> dict:
    import numpy as np

    v = np.asarray(values, dtype=np.float64)
    if v.size == 0:
        return {"n": 0}
    q25, q50, q75 = np.percentile(v, [25, 50, 75])
    return {
        "n": int(v.size), "median": float(q50), "iqr": [float(q25), float(q75)],
        "mean": float(v.mean()), "std": float(v.std()),
    }


def main() -> None:
    maybe_reexec_in_docker("diagnose_cone_shape.py")

    ap = argparse.ArgumentParser(description="Estimate sim cone shape (c, d) per type with VARPRO.")
    ap.add_argument("bag")
    ap.add_argument("--lidar-topic", default="/lidar/Lidar1")
    ap.add_argument("--odom-topic", default="/testing_only/odom")
    ap.add_argument("--track-topic", default="/testing_only/track")
    ap.add_argument("--min-points", type=int, default=60, help="skip clusters with fewer fit points")
    ap.add_argument("--max-range-m", type=float, default=12.0, help="skip clusters farther than this")
    ap.add_argument("--match-gate-m", type=float, default=0.5, help="cluster centroid ↔ GT cone gate")
    ap.add_argument("--max-extent-m", type=float, default=0.45, help="skip merged/odd clusters wider than this")
    ap.add_argument("--every", type=int, default=1, help="use every Nth scan")
    ap.add_argument("--gt-scan-period-ms", type=float, default=DEFAULT_LIDAR_SCAN_PERIOD_NS / 1e6)
    ap.add_argument("--results-root", default=default_results_root())
    args = ap.parse_args()

    import numpy as np

    from cone_detection.cone_detection import ConeDetectionConfig, clustering_separation_rt
    from cone_detection.cone_fit import cone_fit_varpro

    bag_path = resolve_benchmark_path(args.bag)
    buckets = _read_bag_by_topic(str(bag_path), {args.lidar_topic, args.odom_topic, args.track_topic})
    world = latch_track_layout(buckets[args.track_topic])
    if not world:
        raise SystemExit(f"{bag_path}: no {args.track_topic} layout, cannot label cone types")
    from collections import Counter

    print("GT track:", dict(Counter(COLOR_NAMES.get(c.color, str(c.color)) for c in world)))
    odom_msgs = sorted(
        ((msg_time_ns(t, m), m) for t, m in buckets[args.odom_topic]), key=lambda x: x[0]
    )
    cfg = ConeDetectionConfig()
    scan_period_ns = int(args.gt_scan_period_ms * 1e6)

    fits: list[dict] = []
    n_scans = 0
    for i, (bag_t, cloud) in enumerate(buckets[args.lidar_topic]):
        if i % args.every:
            continue
        odom = odom_for_lidar_scan(odom_msgs, msg_time_ns(bag_t, cloud), scan_period_ns=scan_period_ns)
        if odom is None:
            continue
        gt = world_cones_to_body(world, odom, range_m=args.max_range_m + 1.0, hfov_half_deg=None)
        if not gt:
            continue
        xyz = _pointcloud_to_xyz(cloud)
        r2 = xyz[:, 0] ** 2 + xyz[:, 1] ** 2
        xyz = xyz[r2 <= cfg.input_range_crop_m ** 2]
        labels, pts, _ = clustering_separation_rt(xyz, cfg)
        n_scans += 1
        if len(labels) == 0:
            continue
        gt_xy = np.array([[g.x, g.y] for g in gt])
        for lab in np.unique(labels):
            if lab == -1:
                continue
            p = pts[labels == lab]
            p = p[p[:, 2] > cfg.floor_margin_m]
            if len(p) < args.min_points:
                continue
            cen = p[:, :2].mean(axis=0)
            rng = float(np.hypot(*cen))
            if rng > args.max_range_m or np.ptp(p[:, :2], axis=0).max() > args.max_extent_m:
                continue
            dist = np.hypot(*(gt_xy - cen).T)
            j = int(np.argmin(dist))
            # Require a unique match: nearest GT inside the gate, runner-up well outside.
            if dist[j] > args.match_gate_m or (len(dist) > 1 and np.partition(dist, 1)[1] < 2 * args.match_gate_m):
                continue
            g = gt[j]
            a, b, c, d = cone_fit_varpro(p)
            x, y, z = (p[:, k].astype(np.float64) for k in range(3))
            rmse = math.sqrt(float(np.mean((z - (d - c * np.hypot(x - a, y - b))) ** 2)))
            # GT-axis cross-check: (a, b) fixed at the GT cone → z = d - c r is linear.
            r_gt = np.hypot(x - g.x, y - g.y)
            c_gt, d_gt = np.polyfit(r_gt, z, 1)
            fits.append({
                "scan": i, "type": _type_of(g.color), "color": COLOR_NAMES.get(g.color, str(g.color)),
                "range_m": rng, "n": int(len(p)), "z_max": float(z.max()),
                "c": float(c), "d": float(d), "rmse_mm": 1000.0 * rmse,
                "ab_minus_gt_m": [float(a - g.x), float(b - g.y)],
                "c_gt_axis": float(-c_gt), "d_gt_axis": float(d_gt),
            })

    if not fits:
        raise SystemExit("no clusters passed the filters; lower --min-points or raise --max-range-m")

    summary: dict = {"bag": str(bag_path), "scans_used": n_scans, "filters": vars(args) | {"bag": None}}
    print(f"{n_scans} scans, {len(fits)} cluster fits\n")
    for kind in ("small", "big"):
        sel = [f for f in fits if f["type"] == kind]
        # VARPRO snaps c to 0 when it lands in the degenerate basin; not a shape estimate.
        ok = [f for f in sel if f["c"] > 0.5]
        off = np.array([f["ab_minus_gt_m"] for f in sel]) if sel else np.zeros((0, 2))
        block = {
            "fits": len(sel), "fits_degenerate_c0": len(sel) - len(ok),
            "varpro_c": _stats([f["c"] for f in ok]), "varpro_d": _stats([f["d"] for f in ok]),
            "varpro_base_radius_m": _stats([f["d"] / f["c"] for f in ok]),
            "gt_axis_c": _stats([f["c_gt_axis"] for f in sel]), "gt_axis_d": _stats([f["d_gt_axis"] for f in sel]),
            "rmse_mm": _stats([f["rmse_mm"] for f in ok]),
            "max_point_height_m": _stats([f["z_max"] for f in sel]),
            "axis_offset_vs_gt_m": {
                "mean_xy": off.mean(axis=0).tolist() if len(off) else None,
                "median_abs": float(np.median(np.hypot(*off.T))) if len(off) else None,
            },
        }
        summary[kind] = block
        if not sel:
            print(f"{kind}: no fits")
            continue
        vc, vd, gc, gd = block["varpro_c"], block["varpro_d"], block["gt_axis_c"], block["gt_axis_d"]
        print(
            f"{kind:5s}: {len(sel)} fits ({len(sel) - len(ok)} degenerate)\n"
            f"   VARPRO   c = {vc['median']:.2f} (IQR {vc['iqr'][0]:.2f}–{vc['iqr'][1]:.2f})   "
            f"d = {vd['median']:.3f} m (IQR {vd['iqr'][0]:.3f}–{vd['iqr'][1]:.3f})   "
            f"base r = d/c = {block['varpro_base_radius_m']['median']:.3f} m   RMSE {block['rmse_mm']['median']:.0f} mm\n"
            f"   GT axis  c = {gc['median']:.2f} (IQR {gc['iqr'][0]:.2f}–{gc['iqr'][1]:.2f})   "
            f"d = {gd['median']:.3f} m (IQR {gd['iqr'][0]:.3f}–{gd['iqr'][1]:.3f})\n"
            f"   highest return {block['max_point_height_m']['median']:.3f} m (median)   "
            f"fitted axis − GT: mean {np.round(block['axis_offset_vs_gt_m']['mean_xy'], 3).tolist()} m, "
            f"median |Δ| {block['axis_offset_vs_gt_m']['median_abs']:.3f} m"
        )

    out_dir = Path(args.results_root) / "cone_shape"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"{bag_path.name}_cone_shape.json"
    write_json(out, summary | {"fits": fits})
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
