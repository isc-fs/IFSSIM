"""Throwaway: separate true gyro bias (regression intercept) from the bias the
OdometryFilter would adopt (mean of first 3 s). If they differ a lot, the 3 s
calibration window is contaminated by real rotation -> constant yaw drift.

    python diagnose_gyro_bias.py results/capture/<bag_name>
"""
from __future__ import annotations

import argparse
import math

from common import maybe_reexec_in_docker, resolve_benchmark_path
from perception_metrics import msg_time_ns
from odometry_filter_cpp import EkfParams


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
    maybe_reexec_in_docker("diagnose_gyro_bias.py")

    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    bag = str(resolve_benchmark_path(args.bag))
    reader = _open(bag)
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cls = {n: get_message(t) for n, t in topics.items()
           if n in ("/imu", "/testing_only/odom")}

    imu, odom = [], []
    while reader.has_next():
        name, data, bag_t = reader.read_next()
        if name not in cls:
            continue
        msg = deserialize_message(data, cls[name])
        t = msg_time_ns(bag_t, msg) * 1e-9
        if name == "/imu":
            imu.append((t, msg.angular_velocity.z))
        else:
            p = msg.pose.pose
            odom.append((t, _yaw(p.orientation)))
    imu.sort(); odom.sort()
    t0 = imu[0][0]
    calib_s = EkfParams().calibration_seconds

    # GT yaw-rate by central diff
    ot = [r[0] for r in odom]
    yy = [r[1] for r in odom]
    gt_t, gt_yr = [], []
    for i in range(1, len(ot) - 1):
        dt = ot[i + 1] - ot[i - 1]
        if dt <= 0:
            continue
        dyaw = math.atan2(math.sin(yy[i + 1] - yy[i - 1]),
                          math.cos(yy[i + 1] - yy[i - 1]))
        gt_t.append(ot[i]); gt_yr.append(dyaw / dt)

    def interp(ts, xs, t):
        if t <= ts[0]:
            return xs[0]
        if t >= ts[-1]:
            return xs[-1]
        lo, hi = 0, len(ts) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if ts[m] <= t:
                lo = m
            else:
                hi = m
        f = (t - ts[lo]) / (ts[hi] - ts[lo]) if ts[hi] > ts[lo] else 0.0
        return xs[lo] + f * (xs[hi] - xs[lo])

    # Pair gyro to GT yaw-rate; regression gyro = slope*gt + intercept
    gz, yr = [], []
    for t, g in imu:
        if t < gt_t[0] or t > gt_t[-1]:
            continue
        gz.append(g); yr.append(interp(gt_t, gt_yr, t))
    n = len(gz)
    mg, my = sum(gz) / n, sum(yr) / n
    num = sum((a - mg) * (b - my) for a, b in zip(gz, yr))
    den = sum((b - my) ** 2 for b in yr) or 1.0
    slope = num / den
    intercept = mg - slope * my  # gyro reading when GT yaw-rate = 0  == TRUE bias

    # Bias the filter would adopt: mean gyro over first calib_s seconds
    win = [g for t, g in imu if (t - t0) < calib_s]
    filt_bias = sum(win) / len(win)
    gt_yr_win = [interp(gt_t, gt_yr, t) for t, _ in imu if (t - t0) < calib_s]
    mean_gt_yr_win = sum(gt_yr_win) / len(gt_yr_win)

    print(f"\nbag={bag}")
    print(f"paired samples={n}   calib window={calib_s}s ({len(win)} imu samples)")
    print("\n=== gyro-Z bias breakdown (rad/s) ===")
    print(f"  TRUE gyro bias (regression intercept, GT yawrate->0) = {intercept:+.4f}  = {math.degrees(intercept):+.2f} deg/s")
    print(f"  gyro/GT slope                                        = {slope:+.4f}")
    print(f"  mean GT yaw-rate during first {calib_s}s             = {mean_gt_yr_win:+.4f}  = {math.degrees(mean_gt_yr_win):+.2f} deg/s")
    print(f"  FILTER-adopted bias (mean gyro first {calib_s}s)     = {filt_bias:+.4f}  = {math.degrees(filt_bias):+.2f} deg/s")
    err = filt_bias - intercept
    print(f"\n  >> bias error the filter injects = adopted - true = {err:+.4f} rad/s = {math.degrees(err):+.2f} deg/s")
    print("     (this is the constant yaw-rate offset that drifts the heading)")


if __name__ == "__main__":
    main()
