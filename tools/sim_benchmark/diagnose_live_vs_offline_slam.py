"""Throwaway: did the LIVE pipeline SLAM break on this bag, while the offline
benchmark SLAM did not?

The bag records the live run's own SLAM output (/slam/pose) and its
self-reported GT-aligned error (/cone_slam/gt_error_m), alongside the GT pose
(/testing_only/odom). This prints the live SLAM error over time so we can see
whether it diverged DURING the live run — the offline benchmark on the same bag
tracks at <0.3 m, so a live blow-up here proves the bug is live-runtime only.

Also prints /clock vs /imu vs /lidar stamp cadence to expose sim-clock
quantization and any sensor-stream desync.

    python diagnose_live_vs_offline_slam.py results/capture/<bag>
"""
from __future__ import annotations

import argparse
import math

from common import maybe_reexec_in_docker, resolve_benchmark_path
from perception_metrics import msg_time_ns, header_stamp_ns


def _open(path: str):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id="mcap"), ConverterOptions("", ""))
    return r


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--results-root", default=None)
    args = ap.parse_args()
    maybe_reexec_in_docker("diagnose_live_vs_offline_slam.py")

    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    bag = str(resolve_benchmark_path(args.bag))
    reader = _open(bag)
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = ("/slam/pose", "/cone_slam/gt_error_m", "/cone_slam/gt_aligned",
            "/testing_only/odom", "/clock", "/imu", "/lidar/Lidar1", "/odom")
    cls = {n: get_message(t) for n, t in topics.items() if n in want}

    slam_err = []          # (t, err) self-reported live SLAM GT error
    gt_aligned = []        # (t, x, y) live SLAM pose aligned into GT frame
    gt = []                # (t, x, y, yaw)
    clock_ts, imu_ts, lidar_ts, odom_ts = [], [], [], []

    while reader.has_next():
        name, data, bag_t = reader.read_next()
        if name not in cls:
            continue
        msg = deserialize_message(data, cls[name])
        if name == "/cone_slam/gt_error_m":
            # std_msgs/Float32 — no header; use bag time, offset later.
            slam_err.append((bag_t, float(msg.data)))
        elif name == "/cone_slam/gt_aligned":
            t = msg_time_ns(bag_t, msg)
            p = msg.pose.pose
            gt_aligned.append((t, p.position.x, p.position.y))
        elif name == "/testing_only/odom":
            t = msg_time_ns(bag_t, msg)
            p = msg.pose.pose
            gt.append((t, p.position.x, p.position.y, _yaw(p.orientation)))
        elif name == "/clock":
            clock_ts.append(msg.clock.sec * 1_000_000_000 + msg.clock.nanosec)
        elif name == "/imu":
            h = header_stamp_ns(msg)
            if h:
                imu_ts.append(h)
        elif name == "/lidar/Lidar1":
            h = header_stamp_ns(msg)
            if h:
                lidar_ts.append(h)
        elif name == "/odom":
            h = header_stamp_ns(msg)
            if h:
                odom_ts.append(h)

    def span(ts):
        ts = sorted(ts)
        if len(ts) < 2:
            return (0.0, 0.0, 0.0)
        dur = (ts[-1] - ts[0]) / 1e9
        return (ts[0] / 1e9, ts[-1] / 1e9, dur)

    print(f"bag={bag}\n")
    print("=== stream cadence (header/clock stamps, seconds) ===")
    for name, ts in (("/clock", clock_ts), ("/imu", imu_ts),
                     ("/lidar/Lidar1", lidar_ts), ("/odom", odom_ts)):
        s, e, d = span(ts)
        hz = (len(ts) / d) if d > 0 else 0.0
        print(f"  {name:16s} n={len(ts):6d}  start={s:.3f} end={e:.3f} "
              f"dur={d:.2f}s  rate={hz:.1f} Hz")
    # distinct sim-time ticks on /clock vs imu count
    if clock_ts and imu_ts:
        print(f"\n  distinct /clock ticks per IMU sample = "
              f"{len(set(clock_ts))/max(1,len(imu_ts)):.3f} "
              f"(1.0 = every IMU advances sim time; <1 = coarse clock)")

    print("\n=== LIVE SLAM self-reported GT error over the run ===")
    if not slam_err:
        print("  (no /cone_slam/gt_error_m in bag)")
    else:
        slam_err.sort()
        t0 = slam_err[0][0]
        errs = [e for _, e in slam_err]
        print(f"  samples={len(errs)}  min={min(errs):.2f}  "
              f"max={max(errs):.2f}  final={errs[-1]:.2f} m")
        # print a coarse time series (every ~2 s)
        print("  t(s)   live_slam_err_m")
        next_t = t0
        for t, e in slam_err:
            if t >= next_t:
                print(f"  {(t-t0)/1e9:5.1f}   {e:7.2f}")
                next_t = t + 2_000_000_000
    # end-of-run GT vs live-aligned SLAM
    if gt and gt_aligned:
        gt.sort(); gt_aligned.sort()
        print(f"\n  GT final pose       = ({gt[-1][1]:+.1f}, {gt[-1][2]:+.1f})")
        print(f"  live SLAM final pose= ({gt_aligned[-1][1]:+.1f}, "
              f"{gt_aligned[-1][2]:+.1f})  (gt_aligned frame)")


if __name__ == "__main__":
    main()
