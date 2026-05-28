from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from statistics import mean, median

from common import (
    bag_storage_id,
    default_results_root,
    make_run_dir,
    resolve_benchmark_path,
    write_csv,
    write_json,
)
from perception_profiling import (
    summarize_point_counts,
    summarize_stage_rows,
    write_profile_stages_csv,
)
from perception_metrics import (
    Cone2D,
    aggregate_metrics,
    evaluate_frame,
    filter_cones_in_fov,
    latch_track_layout,
    msg_time_ns,
    DEFAULT_LIDAR_SCAN_PERIOD_NS,
    odom_for_lidar_scan,
    summarize_match_bias,
    world_cones_to_body,
)
from report_html import write_run_report


class _NullLogger:
    def info(self, _msg: str) -> None:
        pass

    def error(self, _msg: str) -> None:
        pass


def _open_bag(path: str):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    r = SequentialReader()
    r.open(
        StorageOptions(uri=path, storage_id=bag_storage_id(path)),
        ConverterOptions("", ""),
    )
    return r


def _topic_classes(reader) -> dict[str, object]:
    from rosidl_runtime_py.utilities import get_message

    out: dict[str, object] = {}
    for tt in reader.get_all_topics_and_types():
        out[tt.name] = get_message(tt.type)
    return out


def _pointcloud_to_xyz(msg):
    import numpy as np

    floats_per_point = msg.point_step // 4
    num_points = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.float32).reshape(num_points, floats_per_point)
    return np.ascontiguousarray(raw[:, :3])


def _read_bag_by_topic(bag: str, topics: set[str]) -> dict[str, list[tuple[int, object]]]:
    from rclpy.serialization import deserialize_message

    reader = _open_bag(bag)
    classes = _topic_classes(reader)
    buckets: dict[str, list[tuple[int, object]]] = {t: [] for t in topics}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in buckets:
            buckets[topic].append((t_ns, deserialize_message(raw, classes[topic])))
    return buckets


def _cone_dict(c: Cone2D) -> dict:
    return {"x": c.x, "y": c.y, "color": c.color}


def _frame_sample_dict(fm) -> dict:
    return {
        "t_s": fm.t_s,
        "latency_ms": fm.latency_ms,
        "n_points": fm.n_points,
        "n_gt": fm.n_gt,
        "n_pred": fm.n_pred,
        "n_tp": fm.n_tp,
        "n_fp": fm.n_fp,
        "n_fn": fm.n_fn,
        "mean_match_err_m": fm.mean_match_err_m,
        "max_match_err_m": fm.max_match_err_m,
        "gt": [_cone_dict(c) for c in fm.gt_cones],
        "pred": [_cone_dict(c) for c in fm.pred_cones],
        "matches": [
            {"pred_idx": m.pred_idx, "gt_idx": m.gt_idx, "err_m": m.err_m} for m in fm.matches
        ],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Offline perception benchmark runner.")
    ap.add_argument("bag")
    ap.add_argument("--strategy", default="base", choices=["base"])
    ap.add_argument("--lidar-topic", default="/lidar/Lidar1")
    ap.add_argument("--odom-topic", default="/testing_only/odom")
    ap.add_argument("--track-topic", default="/testing_only/track")
    ap.add_argument("--match-gate-m", type=float, default=1.5)
    ap.add_argument(
        "--gt-range-m",
        type=float,
        default=20.0,
        help="Only GT cones within this body-frame radius (matches cone_detection range_gate_max_m).",
    )
    ap.add_argument(
        "--gt-hfov-deg",
        type=float,
        default=60.0,
        help="GT horizontal half-FOV in body frame (matches LiDAR ±60° in settings.json).",
    )
    ap.add_argument(
        "--gt-min-range-m",
        type=float,
        default=0.5,
        help="Exclude GT cones closer than this (sim LiDAR MinRange = 0.5 m).",
    )
    ap.add_argument(
        "--gt-scan-period-ms",
        type=float,
        default=DEFAULT_LIDAR_SCAN_PERIOD_NS / 1e6,
        help="LiDAR sweep period for GT pose centre (10 Hz sim → 100 ms).",
    )
    ap.add_argument(
        "--gt-scan-center-frac",
        type=float,
        default=0.0,
        help="Advance GT odom into the LiDAR sweep (0 = header stamp; try 0.3–0.5 if late BEV looks shifted).",
    )
    ap.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Process only the first N LiDAR frames (0 = all). Useful for quick GT-alignment checks.",
    )
    ap.add_argument("--sync-ms", type=float, default=50.0, help="(unused) kept for CLI compat.")
    ap.add_argument("--bev-samples", type=int, default=8, help="Frames shown in BEV report plots.")
    ap.add_argument(
        "--profile",
        action="store_true",
        help="Record per-stage timings (RANSAC, DBSCAN, fits) into profile.json and the HTML report.",
    )
    ap.add_argument(
        "--profile-frames",
        type=int,
        default=0,
        help="Cap profiled frames (0 = all lidar frames). Use e.g. 200 for a quick breakdown.",
    )
    ap.add_argument(
        "--ransac-ablation",
        action="store_true",
        help="With --profile: compare RANSAC with 5k iter subsample vs full-cloud iteration scoring.",
    )
    ap.add_argument(
        "--ransac-ablation-frames",
        type=int,
        default=30,
        help="Frames used for --ransac-ablation (evenly spaced through the bag).",
    )
    ap.add_argument("--results-root", default=default_results_root())
    args = ap.parse_args()
    bag_path = resolve_benchmark_path(args.bag)

    from cone_detection.strategies.base_cone_detection import BaseConeDetection

    need = {args.lidar_topic, args.odom_topic, args.track_topic}
    buckets = _read_bag_by_topic(str(bag_path), need)
    lidar_msgs = buckets[args.lidar_topic]
    if not lidar_msgs:
        raise RuntimeError(f"missing lidar topic in bag: {args.lidar_topic}")

    has_gt = bool(buckets[args.odom_topic] and buckets[args.track_topic])
    odom_msgs = sorted(
        (msg_time_ns(bag_t, msg), msg)
        for bag_t, msg in buckets[args.odom_topic]
    )
    world_track = latch_track_layout(buckets[args.track_topic]) if has_gt else None
    has_gt = has_gt and world_track is not None
    gt_gate = dict(
        range_m=args.gt_range_m,
        min_range_m=args.gt_min_range_m,
        hfov_half_deg=args.gt_hfov_deg,
    )
    scan_period_ns = int(args.gt_scan_period_ms * 1e6)

    strategy = BaseConeDetection(logger=_NullLogger())
    strategy.configure()

    csv_rows: list[dict[str, float]] = []
    frame_metrics = []
    lat_ms: list[float] = []
    n_cones_total = 0
    profile_rows: list[dict[str, float]] = []
    profile_limit = args.profile_frames if args.profile_frames > 0 else len(lidar_msgs)

    for frame_i, (bag_t_ns, cloud) in enumerate(lidar_msgs):
        if args.max_frames and frame_i >= args.max_frames:
            break
        scan_t_ns = msg_time_ns(bag_t_ns, cloud)
        t_dec0 = time.perf_counter()
        xyz = _pointcloud_to_xyz(cloud)
        decode_ms = (time.perf_counter() - t_dec0) * 1000.0
        if args.profile and len(profile_rows) < profile_limit:
            stage: dict[str, float] = {}
            t0 = time.perf_counter()
            res = strategy.detect_cones(
                xyz, stage_timings=stage, ransac_iter_subsample_max=5000
            )
            detect_ms = (time.perf_counter() - t0) * 1000.0
            row = {
                "n_points": float(xyz.shape[0]),
                "decode_ms": decode_ms,
                "detect_ms": detect_ms,
                "total_ms": decode_ms + detect_ms,
                **stage,
            }
            profile_rows.append(row)
            dt_ms = row["total_ms"]
        else:
            t0 = time.perf_counter()
            res = strategy.detect_cones(xyz)
            dt_ms = decode_ms + (time.perf_counter() - t0) * 1000.0
        lat_ms.append(dt_ms)
        pred = [Cone2D(x=c.x, y=c.y, color=4) for c in res.cones]
        pred = filter_cones_in_fov(pred, **gt_gate)
        n_cones_total += len(pred)

        gt: list[Cone2D] = []
        if has_gt and world_track is not None:
            odom = odom_for_lidar_scan(
                odom_msgs,
                scan_t_ns,
                scan_period_ns=scan_period_ns,
                center_fraction=args.gt_scan_center_frac,
            )
            if odom is not None:
                gt = world_cones_to_body(world_track, odom, **gt_gate)

        fm = evaluate_frame(
            t_s=scan_t_ns * 1e-9,
            latency_ms=dt_ms,
            n_points=int(xyz.shape[0]),
            pred=pred,
            gt=gt,
            gate_m=args.match_gate_m,
        )
        frame_metrics.append(fm)
        row = {
            "t_s": fm.t_s,
            "latency_ms": fm.latency_ms,
            "n_points": float(fm.n_points),
            "n_cones": float(fm.n_pred),
            "n_gt": float(fm.n_gt),
            "n_tp": float(fm.n_tp),
            "n_fp": float(fm.n_fp),
            "n_fn": float(fm.n_fn),
            "mean_match_err_m": fm.mean_match_err_m,
        }
        csv_rows.append(row)

    run_dir = make_run_dir(args.results_root, "perception", args.strategy)
    summary: dict = {
        "module": "perception",
        "strategy": args.strategy,
        "bag": str(bag_path),
        "frames": len(frame_metrics),
        "mean_latency_ms": mean(lat_ms) if lat_ms else 0.0,
        "median_latency_ms": median(lat_ms) if lat_ms else 0.0,
        "max_latency_ms": max(lat_ms) if lat_ms else 0.0,
        "mean_cones_per_frame": (n_cones_total / len(frame_metrics)) if frame_metrics else 0.0,
        "median_cones_per_frame": median([fm.n_pred for fm in frame_metrics]) if frame_metrics else 0.0,
        "gt_eval": has_gt,
        "match_gate_m": args.match_gate_m,
        "gt_range_m": args.gt_range_m,
        "gt_min_range_m": args.gt_min_range_m,
        "gt_hfov_deg": args.gt_hfov_deg,
        "gt_scan_period_ms": args.gt_scan_period_ms,
        "gt_scan_center_frac": args.gt_scan_center_frac,
        "gt_track_cones": len(world_track) if world_track else 0,
        "csv": str(run_dir / "results.csv"),
    }
    if has_gt:
        scored = [f for f in frame_metrics if f.n_gt > 0]
        summary["gt_metrics"] = aggregate_metrics(scored if scored else frame_metrics)
        summary["gt_metrics"]["frames_with_gt"] = len(scored)
        summary["gt_metrics"]["world_track_cones"] = len(world_track or [])
        bias = summarize_match_bias(scored if scored else frame_metrics)
        if bias:
            summary["gt_metrics"].update(bias)
    else:
        summary["gt_metrics"] = None

    if args.profile and profile_rows:
        # Drop first profile frame (Numba/scipy cold-start on the hot path).
        profile_for_stats = profile_rows[1:] if len(profile_rows) > 1 else profile_rows
        from cone_detection.cone_detection import ConeDetectionConfig

        cfg_prof = strategy.CONE_DETECTION_CONFIG or ConeDetectionConfig()
        profile_summary: dict = {
            "profile_frames": len(profile_rows),
            "profile_frames_excluding_warmup": len(profile_for_stats),
            "stages": summarize_stage_rows(profile_for_stats),
            "point_counts": summarize_point_counts(profile_for_stats),
            "early_exit_small_m": cfg_prof.template_early_exit_height_small_m,
            "early_exit_big_m": cfg_prof.template_early_exit_height_big_m,
            "use_collinear_fit": cfg_prof.use_collinear_fit,
            "template_fit_early_exit": cfg_prof.template_fit_early_exit,
        }
        if args.ransac_ablation:
            import numpy as np
            from cone_detection.cone_detection import ConeDetectionConfig
            from cone_detection.ransac import ransac2

            cfg = strategy.CONE_DETECTION_CONFIG or ConeDetectionConfig()
            n_ab = min(args.ransac_ablation_frames, len(lidar_msgs))
            if n_ab > 0:
                step = max(1, len(lidar_msgs) // n_ab)
                indices = list(range(0, len(lidar_msgs), step))[:n_ab]
                # JIT-compile both ransac2 code paths before timing.
                _, warm_cloud = lidar_msgs[indices[0]]
                warm_a = np.c_[np.ones(1), _pointcloud_to_xyz(warm_cloud)[:1]]
                ransac2(
                    warm_a,
                    prob=cfg.ransac_prob,
                    threshold=cfg.ransac_threshold,
                    iter_subsample_max=5000,
                )
                ransac2(
                    warm_a,
                    prob=cfg.ransac_prob,
                    threshold=cfg.ransac_threshold,
                    iter_subsample_max=0,
                )
                sub_ms: list[float] = []
                full_ms: list[float] = []
                for idx in indices:
                    _, cloud = lidar_msgs[idx]
                    xyz = _pointcloud_to_xyz(cloud)
                    A = np.c_[np.ones(xyz.shape[0]), xyz]
                    t0 = time.perf_counter()
                    ransac2(
                        A,
                        prob=cfg.ransac_prob,
                        threshold=cfg.ransac_threshold,
                        iter_subsample_max=5000,
                    )
                    sub_ms.append((time.perf_counter() - t0) * 1000.0)
                    t0 = time.perf_counter()
                    ransac2(
                        A,
                        prob=cfg.ransac_prob,
                        threshold=cfg.ransac_threshold,
                        iter_subsample_max=0,
                    )
                    full_ms.append((time.perf_counter() - t0) * 1000.0)
                med_sub = median(sub_ms) if sub_ms else 0.0
                med_full = median(full_ms) if full_ms else 0.0
                profile_summary["ransac_ablation"] = {
                    "frames": len(sub_ms),
                    "with_subsample_5000": {
                        "mean_ms": mean(sub_ms),
                        "median_ms": med_sub,
                        "p95_ms": max(sub_ms) if len(sub_ms) < 20 else sorted(sub_ms)[
                            int(0.95 * len(sub_ms))
                        ],
                    },
                    "without_subsample": {
                        "mean_ms": mean(full_ms),
                        "median_ms": med_full,
                        "p95_ms": max(full_ms) if len(full_ms) < 20 else sorted(full_ms)[
                            int(0.95 * len(full_ms))
                        ],
                    },
                    "median_speedup_x": (med_full / med_sub) if med_sub > 0 else 0.0,
                }
        summary["profile"] = profile_summary
        write_json(run_dir / "profile.json", profile_summary)
        write_profile_stages_csv(run_dir / "profile_stages.csv", profile_summary)
        print(f"Wrote {run_dir / 'profile.json'}")
        print(f"Wrote {run_dir / 'profile_stages.csv'}")

    write_csv(run_dir / "results.csv", csv_rows)

    with (run_dir / "frame_details.jsonl").open("w") as fh:
        for fm in frame_metrics:
            fh.write(
                json.dumps(
                    {
                        "t_s": fm.t_s,
                        "n_gt": fm.n_gt,
                        "n_pred": fm.n_pred,
                        "n_tp": fm.n_tp,
                        "n_fp": fm.n_fp,
                        "n_fn": fm.n_fn,
                        "mean_match_err_m": fm.mean_match_err_m,
                        "match_errs": [m.err_m for m in fm.matches],
                    }
                )
                + "\n"
            )

    if frame_metrics and args.bev_samples > 0:
        with_gt_idx = [i for i, f in enumerate(frame_metrics) if f.n_gt > 0]
        pool = with_gt_idx if with_gt_idx else list(range(len(frame_metrics)))
        step = max(1, len(pool) // args.bev_samples)
        indices = pool[::step][: args.bev_samples]
        samples = [_frame_sample_dict(frame_metrics[i]) for i in indices]
        (run_dir / "frame_samples.json").write_text(json.dumps(samples, indent=2))

    report = write_run_report(summary, run_dir, Path(args.results_root))
    write_json(run_dir / "results.json", summary)
    print(f"Wrote {run_dir / 'results.json'}")
    print(f"Wrote {report}")
    if not has_gt:
        missing = [
            t
            for t in (args.odom_topic, args.track_topic)
            if not buckets[t]
        ]
        print(
            f"Note: bag missing {', '.join(missing)} — "
            "GT vs prediction plots need both odom and track."
        )
        if not buckets[args.track_topic] and buckets[args.odom_topic]:
            print(
                "  /testing_only/odom is present; /testing_only/track was likely "
                "skipped at capture (fs_msgs not on AMENT_PREFIX_PATH). Re-record."
            )


if __name__ == "__main__":
    from common import maybe_reexec_in_docker

    maybe_reexec_in_docker("run_perception_benchmark.py")
    main()
