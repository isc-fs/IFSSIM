from __future__ import annotations

import argparse
import json
from pathlib import Path

from common import default_results_root, maybe_reexec_in_docker, resolve_benchmark_path, write_json
from perception_metrics import (
    Cone2D,
    DEFAULT_LIDAR_SCAN_PERIOD_NS,
    filter_cones_in_fov,
    latch_track_layout,
    msg_time_ns,
    pick_scan_center_fraction,
)
from run_perception_benchmark import _pointcloud_to_xyz, _read_bag_by_topic, _NullLogger


def _offset_candidates(min_ms: int, max_ms: int, step_ms: int, period_ms: float) -> tuple[float, ...]:
    return tuple((ms / period_ms) for ms in range(min_ms, max_ms + 1, step_ms))


def main() -> None:
    maybe_reexec_in_docker("diagnose_perception_timing.py")

    ap = argparse.ArgumentParser(
        description="Estimate perception GT timing offset in windows across a bag."
    )
    ap.add_argument("bag")
    ap.add_argument("--lidar-topic", default="/lidar/Lidar1")
    ap.add_argument("--odom-topic", default="/testing_only/odom")
    ap.add_argument("--track-topic", default="/testing_only/track")
    ap.add_argument("--windows", type=int, default=8)
    ap.add_argument("--frames-per-window", type=int, default=24)
    ap.add_argument("--min-offset-ms", type=int, default=-600)
    ap.add_argument("--max-offset-ms", type=int, default=100)
    ap.add_argument("--step-ms", type=int, default=10)
    ap.add_argument("--gt-scan-period-ms", type=float, default=DEFAULT_LIDAR_SCAN_PERIOD_NS / 1e6)
    ap.add_argument("--gt-range-m", type=float, default=20.0)
    ap.add_argument("--gt-min-range-m", type=float, default=0.5)
    ap.add_argument("--gt-hfov-deg", type=float, default=60.0)
    ap.add_argument("--match-gate-m", type=float, default=1.5)
    ap.add_argument("--results-root", default=default_results_root())
    args = ap.parse_args()

    bag_path = resolve_benchmark_path(args.bag)
    need = {args.lidar_topic, args.odom_topic, args.track_topic}
    buckets = _read_bag_by_topic(str(bag_path), need)
    lidar_msgs = buckets[args.lidar_topic]
    if not lidar_msgs:
        raise RuntimeError(f"missing lidar topic in bag: {args.lidar_topic}")

    odom_msgs = sorted(
        (msg_time_ns(bag_t, msg), msg)
        for bag_t, msg in buckets[args.odom_topic]
    )
    world_track = latch_track_layout(buckets[args.track_topic])
    if not odom_msgs or world_track is None:
        raise RuntimeError("bag needs /testing_only/odom and /testing_only/track")

    from cone_detection.strategies.base_cone_detection import BaseConeDetection

    strategy = BaseConeDetection(logger=_NullLogger())
    strategy.configure()

    scan_period_ns = int(args.gt_scan_period_ms * 1e6)
    period_ms = scan_period_ns * 1e-6
    candidates = _offset_candidates(
        args.min_offset_ms, args.max_offset_ms, args.step_ms, period_ms
    )
    gt_gate = dict(
        range_m=args.gt_range_m,
        min_range_m=args.gt_min_range_m,
        hfov_half_deg=args.gt_hfov_deg,
    )

    n = len(lidar_msgs)
    windows = max(1, args.windows)
    rows: list[dict] = []
    for w in range(windows):
        start = int(round(w * n / windows))
        end = int(round((w + 1) * n / windows))
        if end <= start:
            continue
        count = min(args.frames_per_window, end - start)
        step = max(1, (end - start) // count)
        indices = list(range(start, end, step))[:count]

        scan_ts: list[int] = []
        preds: list[list[Cone2D]] = []
        for idx in indices:
            bag_t_ns, cloud = lidar_msgs[idx]
            scan_t_ns = msg_time_ns(bag_t_ns, cloud)
            xyz = _pointcloud_to_xyz(cloud)
            res = strategy.detect_cones(xyz)
            pred = filter_cones_in_fov(
                [Cone2D(x=c.x, y=c.y, color=4) for c in res.cones],
                **gt_gate,
            )
            scan_ts.append(scan_t_ns)
            preds.append(pred)

        frac, meta = pick_scan_center_fraction(
            odom_msgs,
            scan_ts,
            preds,
            world_track,
            scan_period_ns=scan_period_ns,
            gate_m=args.match_gate_m,
            gt_gate=gt_gate,
            candidates=candidates,
        )
        row = {
            "window": w,
            "frame_start": start,
            "frame_end": end - 1,
            "samples": len(indices),
            "t_start_s": scan_ts[0] * 1e-9 if scan_ts else None,
            "t_end_s": scan_ts[-1] * 1e-9 if scan_ts else None,
            "best_offset_ms": meta.get("calib_scan_offset_ms", frac * period_ms),
            "best_center_frac": frac,
            "mean_match_err_m": meta.get("calib_mean_match_err_m"),
            "f1": meta.get("calib_f1"),
            "bias_x_m": meta.get("calib_bias_x_m"),
            "bias_y_m": meta.get("calib_bias_y_m"),
            "pairs": meta.get("calib_bias_pairs"),
        }
        rows.append(row)
        print(
            f"window {w}: frames {start}-{end - 1}, "
            f"offset={row['best_offset_ms']:+.0f} ms, "
            f"err={row['mean_match_err_m']:.3f} m, "
            f"f1={row['f1']:.3f}, pairs={int(row['pairs'] or 0)}"
        )

    out_dir = Path(args.results_root) / "perception_timing"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"{bag_path.name}_timing.json"
    write_json(out, {"bag": str(bag_path), "windows": rows})
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
