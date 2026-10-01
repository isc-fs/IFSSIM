from __future__ import annotations

import argparse
import dataclasses
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
    FS_CONE_ORANGE_BIG,
    Cone2D,
    StopLatchReplay,
    aggregate_metrics,
    aggregate_scan_stats,
    aligned_time_ns,
    classify_detections,
    compute_bag_sim_offset_ns,
    evaluate_frame,
    filter_cones_in_fov,
    gt_cone_range_m,
    latch_track_layout,
    msg_time_ns,
    DEFAULT_LIDAR_SCAN_PERIOD_NS,
    odom_for_lidar_scan,
    reference_deltas,
    scan_stats,
    summarize_match_bias,
    track_at_or_before,
    world_cones_to_body,
    yaw_from_odom,
)
import pipeline_overrides
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
    """Decode exactly as the car does, with cone_detection_node's own reader.

    It reads x/y/z at their declared byte offsets, so the Hesai ATX's packed
    26-byte point decodes as well as the sim's layout. The previous local copy
    reshaped by ``point_step // 4`` and could not read real-car bags.
    """
    from cone_detection.cone_detection_node import ConeDetectionNode

    return ConeDetectionNode.pointcloud2_to_xyz(msg)


DEFAULT_STOP_LATCH_MIN_TRAVEL_M = 30.0


def _stop_latch_min_travel(script: Path | None = None) -> tuple[float, str]:
    """``control_node.stop_latch_min_travel`` and where it came from.

    This checkout's params.yaml when it is reachable (host / --local-ros);
    inside the benchmark container only the bench folder and the detector
    sources are mounted, so fall back to the bringup package installed in the
    image, then to the pipeline default. The source is recorded in the summary
    so an image value can never pass for the checkout's.
    """
    import yaml

    here = (script or Path(__file__)).resolve()
    candidates: list[tuple[Path, str]] = []
    # In the container the script is /bench/<name>: there is no checkout above it.
    if len(here.parents) > 2:
        candidates.append(
            (here.parents[2] / "pipeline/bringup/config/params.yaml", "checkout params.yaml")
        )
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            (Path(get_package_share_directory("bringup")) / "config/params.yaml",
             "bringup share (image)")
        )
    except Exception:
        pass
    for path, source in candidates:
        try:
            params = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            value = params["control_node"]["ros__parameters"]["stop_latch_min_travel"]
            return float(value), f"{source}: {path}"
        except (OSError, KeyError, TypeError, ValueError):
            continue
    return DEFAULT_STOP_LATCH_MIN_TRAVEL_M, "default"


def _load_reference(path: str) -> dict:
    p = Path(path)
    if p.is_dir():
        p = p / "results.json"
    return json.loads(p.read_text(encoding="utf-8"))


def _read_bag_by_topic(
    bag: str, topics: set[str]
) -> dict[str, list[tuple[int, object]]]:
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


def _cluster_centroids(xyz, cfg) -> list[Cone2D]:
    """DBSCAN cluster centroids (body frame) for the BEV diagnostic overlay.

    Re-runs ground removal + DBSCAN with the detection config and averages each
    cluster's xy. Lets the report show whether a missed GT cone had *any* cluster
    (points present, but the fit/residual gate dropped it) versus no cluster at
    all (points missing — sensor/clustering gap). RANSAC subsampling makes this
    mildly non-deterministic vs the scored pass, which is fine for "is there a
    cluster here?".
    """
    import numpy as np
    from cone_detection.cone_detection import clustering_separation_rt

    labels, clean_data, _ = clustering_separation_rt(xyz, cfg)
    if len(labels) == 0:
        return []
    out: list[Cone2D] = []
    for lab in np.unique(labels):
        if lab == -1:  # DBSCAN noise
            continue
        pts = clean_data[labels == lab]
        if pts.shape[0] == 0:
            continue
        out.append(
            Cone2D(x=float(pts[:, 0].mean()), y=float(pts[:, 1].mean()), color=4)
        )
    return out


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
            {"pred_idx": m.pred_idx, "gt_idx": m.gt_idx, "err_m": m.err_m}
            for m in fm.matches
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
        "--lidar-height-m",
        type=float,
        default=1.1,
        help="LiDAR mount height (settings.json Lidar Z) for the GT vertical-FOV gate.",
    )
    ap.add_argument(
        "--vfov-lower-deg",
        type=float,
        default=-12.4,
        help="LiDAR VerticalFOVLower; GT cones whose tip falls below this are unseeable.",
    )
    ap.add_argument(
        "--vfov-upper-deg",
        type=float,
        default=5.9,
        help="LiDAR VerticalFOVUpper (matches settings.json).",
    )
    ap.add_argument("--cone-height-small-m", type=float, default=0.35)
    ap.add_argument("--cone-height-big-m", type=float, default=0.55)
    ap.add_argument(
        "--no-vfov-gate",
        action="store_true",
        help="Disable the GT vertical-FOV gate (count cones the sensor physically can't see).",
    )
    ap.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Process only the first N LiDAR frames (0 = all). Useful for quick GT-alignment checks.",
    )
    ap.add_argument(
        "--sync-ms", type=float, default=50.0, help="(unused) kept for CLI compat."
    )
    ap.add_argument(
        "--bev-samples", type=int, default=8, help="Frames shown in BEV report plots."
    )
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
    ap.add_argument(
        "--final-lap-topic",
        default="/slam/final_lap",
        help="std_msgs/Bool the stop latch is gated on in trackdrive; true when absent from the bag.",
    )
    ap.add_argument(
        "--stop-latch-min-travel-m",
        type=float,
        default=None,
        help="control_node stop_latch_min_travel (default: read from params.yaml).",
    )
    ap.add_argument(
        "--latch-finish-tol-m",
        type=float,
        default=3.0,
        help="A replayed stop latch is premature when its anchor is farther than this "
        "from every ground-truth big-orange cone.",
    )
    ap.add_argument(
        "--reference",
        help="results.json (or its run directory) of a reference run, e.g. the flat-world "
        "baseline; the summary then reports deltas against it.",
    )
    ap.add_argument("--results-root", default=default_results_root())
    ap.add_argument(
        "--pipeline-overrides",
        help="JSON file of parameter overrides (pipeline_overrides.py); uses cone_detection",
    )
    args = ap.parse_args()
    bag_path = resolve_benchmark_path(args.bag)
    overrides = pipeline_overrides.load(args.pipeline_overrides)
    pipeline_overrides.only(overrides, "cone_detection")

    from cone_detection.cone_detection import ConeDetectionConfig
    from cone_detection.strategies.base_cone_detection import BaseConeDetection

    detection_overrides = overrides.get("cone_detection", {})
    detection_config = pipeline_overrides.apply_dataclass(
        BaseConeDetection.CONE_DETECTION_CONFIG or ConeDetectionConfig(),
        detection_overrides,
        "cone_detection",
    )

    need = {args.lidar_topic, args.odom_topic, args.track_topic, args.final_lap_topic}
    buckets = _read_bag_by_topic(str(bag_path), need)
    lidar_msgs = buckets[args.lidar_topic]
    if not lidar_msgs:
        raise RuntimeError(f"missing lidar topic in bag: {args.lidar_topic}")

    has_gt = bool(buckets[args.odom_topic] and buckets[args.track_topic])
    odom_msgs = sorted(
        ((msg_time_ns(bag_t, msg), msg) for bag_t, msg in buckets[args.odom_topic]),
        key=lambda item: item[0],
    )
    world_track = latch_track_layout(buckets[args.track_topic]) if has_gt else None
    has_gt = has_gt and world_track is not None
    gt_gate = dict(
        range_m=args.gt_range_m,
        min_range_m=args.gt_min_range_m,
        hfov_half_deg=args.gt_hfov_deg,
    )
    vfov_gate: dict[str, float] = {}
    if not args.no_vfov_gate:
        vfov_gate = dict(
            lidar_height_m=args.lidar_height_m,
            vfov_lower_deg=args.vfov_lower_deg,
            vfov_upper_deg=args.vfov_upper_deg,
            cone_height_small_m=args.cone_height_small_m,
            cone_height_big_m=args.cone_height_big_m,
        )
    scan_period_ns = int(args.gt_scan_period_ms * 1e6)

    # /slam/final_lap is a header-less Bool, so its bag time is wall clock while
    # the LiDAR is stamped in sim time; map it onto the sim clock first.
    sim_offset_ns = compute_bag_sim_offset_ns(lidar_msgs)
    final_lap_msgs = sorted(
        (aligned_time_ns(bag_t, msg, sim_offset_ns), msg)
        for bag_t, msg in buckets[args.final_lap_topic]
    )

    if args.stop_latch_min_travel_m is not None:
        min_travel_m, min_travel_source = args.stop_latch_min_travel_m, "--stop-latch-min-travel-m"
    else:
        min_travel_m, min_travel_source = _stop_latch_min_travel()
    latch = (
        StopLatchReplay(
            min_travel_m=min_travel_m,
            gt_big_world=[(c.x, c.y) for c in world_track if c.color == FS_CONE_ORANGE_BIG],
            finish_tol_m=args.latch_finish_tol_m,
        )
        if has_gt and world_track is not None
        else None
    )

    if detection_overrides:
        # the strategy reads its config from the class; a subclass keeps the pipeline untouched
        BaseConeDetection = type(
            "BaseConeDetection",
            (BaseConeDetection,),
            {"CONE_DETECTION_CONFIG": detection_config},
        )
    strategy = BaseConeDetection(logger=_NullLogger())
    strategy.configure()
    big_orange_threshold_m = strategy.big_orange_height_threshold_m()

    csv_rows: list[dict[str, float]] = []
    scan_rows: list[dict[str, float]] = []
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
        all_pred = classify_detections(res.cones, big_orange_threshold_m)
        pred = filter_cones_in_fov(all_pred, **gt_gate)
        n_cones_total += len(pred)
        stats = scan_stats(res)
        scan_rows.append(stats)

        gt: list[Cone2D] = []
        if has_gt and world_track is not None:
            odom = odom_for_lidar_scan(
                odom_msgs,
                scan_t_ns,
                scan_period_ns=scan_period_ns,
            )
            if odom is not None:
                gt = world_cones_to_body(world_track, odom, **gt_gate, **vfov_gate)
                if latch is not None:
                    fl = track_at_or_before(final_lap_msgs, scan_t_ns)
                    pos = odom.pose.pose.position
                    # Unfiltered: the node latches on everything /Conos_Orange carries.
                    latch.update(
                        t_s=scan_t_ns * 1e-9,
                        x=float(pos.x),
                        y=float(pos.y),
                        yaw=yaw_from_odom(odom),
                        big_cones_body=[c for c in all_pred if c.color == FS_CONE_ORANGE_BIG],
                        final_lap=bool(fl.data) if fl is not None else True,
                    )

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
            "n_pred_big": float(fm.n_pred_big),
            "n_false_big": float(fm.n_false_big),
            "n_missed_big": float(fm.n_missed_big),
            **stats,
        }
        csv_rows.append(row)

    run_dir = make_run_dir(args.results_root, "perception", args.strategy)
    pipeline_overrides.write_effective(
        run_dir,
        "cone_detection",
        dataclasses.asdict(detection_config),
        detection_overrides,
    )
    summary: dict = {
        "module": "perception",
        "strategy": args.strategy,
        "bag": str(bag_path),
        "frames": len(frame_metrics),
        "mean_latency_ms": mean(lat_ms) if lat_ms else 0.0,
        "median_latency_ms": median(lat_ms) if lat_ms else 0.0,
        "max_latency_ms": max(lat_ms) if lat_ms else 0.0,
        "mean_cones_per_frame": (n_cones_total / len(frame_metrics))
        if frame_metrics
        else 0.0,
        "median_cones_per_frame": median([fm.n_pred for fm in frame_metrics])
        if frame_metrics
        else 0.0,
        "gt_eval": has_gt,
        "match_gate_m": args.match_gate_m,
        "gt_range_m": args.gt_range_m,
        "gt_min_range_m": args.gt_min_range_m,
        "gt_hfov_deg": args.gt_hfov_deg,
        "gt_scan_period_ms": args.gt_scan_period_ms,
        "gt_vfov_gate": not args.no_vfov_gate,
        "lidar_height_m": args.lidar_height_m,
        "vfov_lower_deg": args.vfov_lower_deg,
        "vfov_upper_deg": args.vfov_upper_deg,
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

    summary["big_orange_height_threshold_m"] = big_orange_threshold_m
    summary["scan_stats"] = aggregate_scan_stats(scan_rows)
    if latch is not None:
        summary["stop_latch"] = latch.summary()
        summary["stop_latch"]["min_travel_source"] = min_travel_source
        summary["stop_latch"]["final_lap_topic_present"] = bool(final_lap_msgs)
    else:
        summary["stop_latch"] = {
            "evaluated": False,
            "reason": "needs ground-truth odom and track in the bag",
        }
    if args.reference:
        summary["reference"] = {
            "path": args.reference,
            "deltas": reference_deltas(summary, _load_reference(args.reference)),
        }

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
                        "p95_ms": max(sub_ms)
                        if len(sub_ms) < 20
                        else sorted(sub_ms)[int(0.95 * len(sub_ms))],
                    },
                    "without_subsample": {
                        "mean_ms": mean(full_ms),
                        "median_ms": med_full,
                        "p95_ms": max(full_ms)
                        if len(full_ms) < 20
                        else sorted(full_ms)[int(0.95 * len(full_ms))],
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
                        "n_false_big": fm.n_false_big,
                        "n_missed_big": fm.n_missed_big,
                        "mean_match_err_m": fm.mean_match_err_m,
                        "match_errs": [m.err_m for m in fm.matches],
                        "match_ranges_m": [
                            gt_cone_range_m(fm.gt_cones[m.gt_idx]) for m in fm.matches
                        ],
                    }
                )
                + "\n"
            )

    if frame_metrics and args.bev_samples > 0:
        from cone_detection.cone_detection import ConeDetectionConfig

        cfg_bev = strategy.CONE_DETECTION_CONFIG or ConeDetectionConfig()
        with_gt_idx = [i for i, f in enumerate(frame_metrics) if f.n_gt > 0]
        pool = with_gt_idx if with_gt_idx else list(range(len(frame_metrics)))
        step = max(1, len(pool) // args.bev_samples)
        indices = pool[::step][: args.bev_samples]
        samples = []
        for i in indices:
            s = _frame_sample_dict(frame_metrics[i])
            _, cloud_i = lidar_msgs[i]
            cents = _cluster_centroids(_pointcloud_to_xyz(cloud_i), cfg_bev)
            cents = filter_cones_in_fov(cents, **gt_gate)
            s["clusters"] = [_cone_dict(c) for c in cents]
            samples.append(s)
        (run_dir / "frame_samples.json").write_text(json.dumps(samples, indent=2))

    report = write_run_report(summary, run_dir, Path(args.results_root))
    write_json(run_dir / "results.json", summary)
    print(f"Wrote {run_dir / 'results.json'}")
    print(f"Wrote {report}")
    if not has_gt:
        missing = [t for t in (args.odom_topic, args.track_topic) if not buckets[t]]
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
