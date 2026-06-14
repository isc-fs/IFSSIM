from __future__ import annotations

import argparse
from pathlib import Path

from common import bag_storage_id, default_results_root, make_run_dir, write_csv, write_json
from perception_metrics import DEFAULT_LIDAR_SCAN_PERIOD_NS, latch_track_layout, msg_time_ns
from report_html import write_run_report
from odom_from_bag import ODOM_SENSOR_TOPICS, synthesize_supervisor_odom
from slam_metrics import (
    BASE_TOPICS,
    GT_CONE_TRIGGER_TOPIC,
    aggregate_pose_steps,
    aggregate_slam,
    estimate_imu_time_scale,
    infer_steering_angle_units,
    map_stats_to_dict,
    map_to_rows,
    pose_steps_to_rows,
    replay_slam,
    samples_to_rows,
    trajectory_to_rows,
)


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


def _load_bag_events(bag: str) -> tuple[list[tuple[int, str, object]], dict[str, list]]:
    from rclpy.serialization import deserialize_message

    reader = _open_bag(bag)
    classes = _topic_classes(reader)
    topics = (
        BASE_TOPICS
        | {"/testing_only/track", GT_CONE_TRIGGER_TOPIC}
        | set(ODOM_SENSOR_TOPICS)
    )
    buckets: dict[str, list[tuple[int, object]]] = {t: [] for t in topics}
    events: list[tuple[int, str, object]] = []
    while reader.has_next():
        topic, raw, bag_t_ns = reader.read_next()
        if topic not in buckets:
            continue
        msg = deserialize_message(raw, classes[topic])
        buckets[topic].append((bag_t_ns, msg))
        events.append((bag_t_ns, topic, msg))
    # Replay sensor fusion in header-time order (bag log time can lag headers).
    events.sort(key=lambda e: msg_time_ns(e[0], e[2]))
    return events, buckets


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Offline SLAM benchmark vs sim GT (gated track cones + /testing_only/odom).",
    )
    ap.add_argument("bag")
    ap.add_argument("--strategy", default="trackdrive")
    ap.add_argument("--sync-ms", type=float, default=50.0, help="(unused) kept for CLI compat.")
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
        help="LiDAR period used when offsetting GT cone injection timestamps.",
    )
    ap.add_argument(
        "--gt-scan-center-frac",
        type=float,
        default=0.0,
        help=(
            "GT odom offset for LiDAR-triggered cone injection, in scan periods "
            "(same convention as run_perception_benchmark.py)."
        ),
    )
    ap.add_argument(
        "--imu-time-scale",
        default="auto",
        help=(
            "Rescale the IMU integration clock onto true-motion time "
            "(the sim runs slower than wall, so bridge node->now() stamps "
            "over-count dt and inflate distance). 'auto' = calibrate from GT "
            "twist-vs-path, 'off' = use raw stamps, or a float (e.g. 0.77)."
        ),
    )
    ap.add_argument("--results-root", default=default_results_root())
    args = ap.parse_args()

    if args.imu_time_scale == "auto":
        imu_time_scale = None
    elif args.imu_time_scale == "off":
        imu_time_scale = 1.0
    else:
        imu_time_scale = float(args.imu_time_scale)

    events, buckets = _load_bag_events(args.bag)
    if not buckets["/imu"]:
        raise RuntimeError("bag missing /imu")
    if not buckets["/testing_only/odom"]:
        raise RuntimeError("bag missing /testing_only/odom")

    # Resolve the IMU-clock rescale once (GT twist-vs-path), then apply it to
    # BOTH the synthesized /odom and the in-replay EKF so they agree.
    if imu_time_scale is None:
        imu_scale_value, _ = estimate_imu_time_scale(buckets["/testing_only/odom"])
    else:
        imu_scale_value = imu_time_scale

    odom_source = "bag" if buckets["/odom"] else "missing"
    if not buckets["/odom"]:
        rpm_series = [
            (msg_time_ns(bag_t, msg), float(msg.data))
            for bag_t, msg in buckets.get("/motor_rpm", [])
        ]
        steer_series = [
            (msg_time_ns(bag_t, msg), float(msg.data))
            for bag_t, msg in buckets.get("/steering_angle", [])
        ]
        rpm_series.sort(key=lambda row: row[0])
        steer_series.sort(key=lambda row: row[0])
        steering_units = infer_steering_angle_units(
            events,
            rpm_series,
            steer_series,
        )
        for t_ns, odom_msg in synthesize_supervisor_odom(
            buckets,
            steering_units=steering_units,
            imu_time_scale=imu_scale_value,
        ):
            buckets["/odom"].append((t_ns, odom_msg))
            events.append((t_ns, "/odom", odom_msg))
        if buckets["/odom"]:
            odom_source = "synthesized"
            events.sort(key=lambda e: msg_time_ns(e[0], e[2]))

    has_supervisor_odom = bool(buckets["/odom"])
    sync_ns = int(args.sync_ms * 1e6)
    odom_msgs = sorted(
        (msg_time_ns(bag_t, msg), msg)
        for bag_t, msg in buckets["/testing_only/odom"]
    )
    world_track = latch_track_layout(buckets["/testing_only/track"])
    if world_track is None:
        raise RuntimeError(
            "bag missing /testing_only/track — re-capture with sim running "
            "(see capture_benchmark_bag.py)"
        )

    run_dir = make_run_dir(args.results_root, "slam", args.strategy)
    gt_kwargs = {
        "gt_range_m": args.gt_range_m,
        "gt_min_range_m": args.gt_min_range_m,
        "gt_hfov_deg": args.gt_hfov_deg,
        "gt_scan_period_ns": int(args.gt_scan_period_ms * 1e6),
        "gt_scan_center_frac": args.gt_scan_center_frac,
    }

    res = replay_slam(
        events,
        odom_msgs=odom_msgs,
        world_track=world_track,
        strategy=args.strategy,
        sync_ns=sync_ns,
        imu_time_scale=imu_scale_value,
        **gt_kwargs,
    )
    write_csv(run_dir / "samples.csv", samples_to_rows(res.samples))
    write_csv(run_dir / "trajectory.csv", trajectory_to_rows(res.trajectory))
    write_csv(run_dir / "pose_steps.csv", pose_steps_to_rows(res.pose_steps))
    write_csv(
        run_dir / "map_cones.csv",
        map_to_rows(res.gt_map, "gt") + map_to_rows(res.slam_map, "slam"),
    )
    gt_stats = aggregate_slam(res.samples)
    filter_stats = aggregate_pose_steps(res.pose_steps, event="imu")
    imu_only_stats = aggregate_pose_steps(
        res.pose_steps,
        event="imu",
        err_field="imu_only_err_m",
    )
    wheel_stats = aggregate_pose_steps(
        res.pose_steps,
        event="imu",
        err_field="wheel_err_m",
    )
    slam_cone_stats = aggregate_pose_steps(
        res.pose_steps,
        event="cone",
        err_field="slam_err_m",
    )
    slam_gt_stats = aggregate_pose_steps(
        res.pose_steps,
        event="gt_odom",
        err_field="slam_err_m",
    )
    supervisor_stats = aggregate_pose_steps(
        res.pose_steps,
        event="supervisor_odom",
        err_field="supervisor_err_m",
    )

    summary: dict = {
        "module": "slam",
        "strategy": args.strategy,
        "bag": args.bag,
        "has_supervisor_odom": has_supervisor_odom,
        "odom_source": odom_source,
        "cone_trigger": GT_CONE_TRIGGER_TOPIC
        if buckets.get(GT_CONE_TRIGGER_TOPIC)
        else "/testing_only/track",
        "gt_track_cones": len(world_track),
        "gt_range_m": args.gt_range_m,
        "gt_min_range_m": args.gt_min_range_m,
        "gt_hfov_deg": args.gt_hfov_deg,
        "gt_scan_period_ms": args.gt_scan_period_ms,
        "gt_scan_center_frac": args.gt_scan_center_frac,
        "gt_cones": gt_stats,
        "filter_odom": filter_stats,
        "imu_only_odom": imu_only_stats,
        "wheel_odom": wheel_stats,
        "supervisor_odom": supervisor_stats,
        "slam_at_cones": slam_cone_stats,
        "slam_at_gt_rate": slam_gt_stats,
        "pose_steps": len(res.pose_steps),
        "map": map_stats_to_dict(res.map_stats) if res.map_stats else {},
        "wheel_steer_units": res.wheel_steer_units,
        "imu_time_scale": res.imu_time_scale,
        "imu_time_scale_mode": args.imu_time_scale,
        "csv": str(run_dir / "samples.csv"),
        "pose_steps_csv": str(run_dir / "pose_steps.csv"),
        "map_csv": str(run_dir / "map_cones.csv"),
    }

    report = write_run_report(summary, run_dir, Path(args.results_root))
    write_json(run_dir / "results.json", summary)
    print(f"Wrote {run_dir / 'results.json'}")
    print(f"Wrote {report}")
    if abs(res.imu_time_scale - 1.0) > 1e-6:
        print(
            f"IMU clock rescaled by {res.imu_time_scale:.4f} "
            f"(mode={args.imu_time_scale}; corrects ~{(1.0 / res.imu_time_scale - 1.0) * 100:.1f}% "
            "wall-vs-sim time stretch)."
        )
    if odom_source == "synthesized":
        print(
            f"Synthesized {len(buckets['/odom'])} /odom samples from bag sensors "
            f"(IMU + motor_rpm + steering via the 9-state EKF / OdometryFilterCpp)."
        )
    elif not has_supervisor_odom:
        print(
            "WARNING: no /odom in bag and could not synthesize it "
            "(need /imu and /motor_rpm at minimum). SLAM runs without pose prior (#545)."
        )
    if not buckets.get(GT_CONE_TRIGGER_TOPIC):
        print(
            f"Note: no {GT_CONE_TRIGGER_TOPIC} in bag — GT cones injected on "
            "/testing_only/track only (sparse)."
        )


if __name__ == "__main__":
    from common import maybe_reexec_in_docker

    maybe_reexec_in_docker("run_slam_benchmark.py")
    main()
