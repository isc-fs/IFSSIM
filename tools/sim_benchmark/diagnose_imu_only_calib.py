"""Decisive check for the IMU-only 90-degree-rotation / short-path artifact.

Two concrete, NON-averaged questions:

1. Is the car actually stationary during the 3 s accel-bias calibration window?
   If not, ``accel_bias = accel_mean - [0,0,G]`` soaks the real forward
   acceleration into BA_X, so the unaided IMU-only filter integrates a wrong
   (often negative) forward velocity. -> prints GT speed at the window edges and
   the resulting BA_X / BA_Y the filter would adopt.

2. Does the surviving IMU-only motion run ~90 deg off the true heading?
   Replays the IMU-only filter exactly like slam_metrics and, at sampled
   timestamps, prints the world-frame direction of the IMU-only velocity vector
   vs the GT velocity vector. A forward-collapse leaves only the lateral
   centripetal leak -> velocity points ~+/-90 deg off GT.

Run like the benchmark (auto re-exec in Docker):
    python diagnose_imu_only_calib.py results/capture/<bag>/<bag>_0.mcap
"""
from __future__ import annotations

import argparse
import math

import numpy as np

from common import maybe_reexec_in_docker, resolve_benchmark_path
from perception_metrics import msg_time_ns
from odometry_filter_cpp import OdometryFilterCpp, EkfParams


def _open(path: str):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id="mcap"),
           ConverterOptions("", ""))
    return r


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--results-root", default=None)
    args = ap.parse_args()
    maybe_reexec_in_docker("diagnose_imu_only_calib.py")

    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    bag = str(resolve_benchmark_path(args.bag))
    reader = _open(bag)
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cls = {n: get_message(t) for n, t in topics.items()
           if n in ("/imu", "/testing_only/odom")}

    imu = []   # (t, ax, ay, az, gx, gy, gz)
    odom = []  # (t, x, y, yaw, vx, vy)
    while reader.has_next():
        name, data, bag_t = reader.read_next()
        if name not in cls:
            continue
        msg = deserialize_message(data, cls[name])
        t = msg_time_ns(bag_t, msg) * 1e-9
        if name == "/imu":
            a = msg.linear_acceleration
            g = msg.angular_velocity
            imu.append((t, a.x, a.y, a.z, g.x, g.y, g.z))
        else:
            p = msg.pose.pose
            tw = msg.twist.twist.linear
            imu_yaw = _yaw_from_quat(p.orientation)
            odom.append((t, p.position.x, p.position.y, imu_yaw, tw.x, tw.y))

    imu.sort(); odom.sort()
    print(f"bag={bag}")
    print(f"/imu={len(imu)}  /testing_only/odom={len(odom)}")
    if len(imu) < 10 or len(odom) < 10:
        print("not enough data"); return

    ot = [r[0] for r in odom]
    ospeed = [math.hypot(r[4], r[5]) for r in odom]

    def gt_speed(t):
        if t <= ot[0]:
            return ospeed[0]
        if t >= ot[-1]:
            return ospeed[-1]
        lo, hi = 0, len(ot) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if ot[m] <= t:
                lo = m
            else:
                hi = m
        f = (t - ot[lo]) / (ot[hi] - ot[lo]) if ot[hi] > ot[lo] else 0.0
        return ospeed[lo] + f * (ospeed[hi] - ospeed[lo])

    def gt_velvec(t):  # world-frame GT velocity vector via finite diff of pose
        if t <= ot[0] or t >= ot[-1]:
            return (0.0, 0.0)
        lo, hi = 0, len(ot) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if ot[m] <= t:
                lo = m
            else:
                hi = m
        dt = ot[hi] - ot[lo]
        if dt <= 0:
            return (0.0, 0.0)
        return ((odom[hi][1] - odom[lo][1]) / dt,
                (odom[hi][2] - odom[lo][2]) / dt)

    # --- Q1: stationarity during the 3 s calibration window ---
    t0 = imu[0][0]
    calib_s = EkfParams().calibration_seconds
    win = [r for r in imu if (r[0] - t0) < calib_s]
    ax_mean = sum(r[1] for r in win) / len(win)
    ay_mean = sum(r[2] for r in win) / len(win)
    az_mean = sum(r[3] for r in win) / len(win)
    print("\n=== Q1: is the car stationary during the 3 s calib window? ===")
    print(f"  window: {len(win)} IMU samples, t in [{t0:.3f}, {t0+calib_s:.3f}]")
    print(f"  GT speed at window start  = {gt_speed(t0):.3f} m/s")
    print(f"  GT speed at window mid    = {gt_speed(t0+calib_s*0.5):.3f} m/s")
    print(f"  GT speed at window end    = {gt_speed(t0+calib_s):.3f} m/s")
    print(f"  mean accel in window      = ({ax_mean:+.3f}, {ay_mean:+.3f}, {az_mean:+.3f})")
    print("  -> filter would set BA_X,BA_Y = mean accel x,y (gravity removed on z):")
    print(f"     BA_X = {ax_mean:+.3f} m/s^2   BA_Y = {ay_mean:+.3f} m/s^2")
    print("     (BA_X far from 0 == launch/forward accel soaked into the bias)")

    # --- Q2: replay IMU-only filter; print velocity-vector direction vs GT ---
    filt = OdometryFilterCpp(EkfParams())
    print("\n=== Q2: IMU-only velocity direction vs GT (90 deg check) ===")
    print("   t-t0    gt_spd  io_vx   io_vy  | gt_dir  io_dir  d(io-gt)")
    next_print = calib_s + 0.5
    for (t, ax, ay, az, gx, gy, gz) in imu:
        filt.push_imu(t, np.array([ax, ay, az]), np.array([gx, gy, gz]))
        if t - t0 >= next_print and filt._calib.completed:
            st = filt.state
            yaw = st.yaw
            # IMU-only velocity rotated body->world
            iox = st.vx * math.cos(yaw) - st.vy * math.sin(yaw)
            ioy = st.vx * math.sin(yaw) + st.vy * math.cos(yaw)
            gvx, gvy = gt_velvec(t)
            gt_dir = math.degrees(math.atan2(gvy, gvx))
            io_dir = math.degrees(math.atan2(ioy, iox))
            d = math.degrees(math.atan2(math.sin(math.radians(io_dir - gt_dir)),
                                        math.cos(math.radians(io_dir - gt_dir))))
            print(f"  {t-t0:6.2f}  {gt_speed(t):6.2f}  {st.vx:+6.2f} {st.vy:+6.2f} "
                  f"| {gt_dir:+7.1f} {io_dir:+7.1f}  {d:+7.1f}")
            next_print += 2.0


if __name__ == "__main__":
    main()
