"""Decisive check: is the raw /imu body frame consistent with GT motion,
or rotated/sign-flipped?

Reads /imu (accel, gyro) and /testing_only/odom (GT pose+twist) from a bag and
correlates, in the GT body frame:

* gyro.z   vs GT yaw-rate              -> gyro Z sign/axis
* accel.x  vs d|v|/dt (longitudinal)   -> is +x really "forward"?
* accel.x  vs v*yaw_rate (centripetal) -> is +x actually lateral? (90 deg swap)
* accel.y  vs the same two references  -> mirror check for +y

A clean frame: accel.x tracks longitudinal accel, accel.y tracks lateral
(centripetal). A 90 deg body-axis swap shows accel.x tracking centripetal and
accel.y tracking longitudinal. A sign flip shows a strong *negative*
correlation where a positive one is expected.

Run like the benchmark (auto re-exec in Docker):
    python diagnose_imu_frame.py results/capture/<bag_name>
"""
from __future__ import annotations

import argparse
import math

from common import maybe_reexec_in_docker, resolve_benchmark_path
from perception_metrics import msg_time_ns, stamp_ns


def _open(path: str):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    r = SequentialReader()
    sid = "mcap" if path.endswith(".mcap") or True else "sqlite3"
    r.open(StorageOptions(uri=path, storage_id="mcap"),
           ConverterOptions("", ""))
    return r


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _corr(a, b):
    n = len(a)
    if n < 3:
        return float("nan")
    ma = sum(a) / n
    mb = sum(b) / n
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    da = math.sqrt(sum((x - ma) ** 2 for x in a))
    db = math.sqrt(sum((y - mb) ** 2 for y in b))
    if da == 0 or db == 0:
        return float("nan")
    return num / (da * db)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--results-root", default=None)
    args = ap.parse_args()
    maybe_reexec_in_docker("diagnose_imu_frame.py")

    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    bag = str(resolve_benchmark_path(args.bag))
    reader = _open(bag)
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cls = {n: get_message(t) for n, t in topics.items()
           if n in ("/imu", "/testing_only/odom")}

    imu = []   # (t_s, ax, ay, gz)
    odom = []  # (t_s, x, y, yaw, vx_body)
    while reader.has_next():
        name, data, bag_t = reader.read_next()
        if name not in cls:
            continue
        msg = deserialize_message(data, cls[name])
        t = msg_time_ns(bag_t, msg) * 1e-9
        if name == "/imu":
            imu.append((t, msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.angular_velocity.z))
        else:
            p = msg.pose.pose
            yaw = _yaw_from_quat(p.orientation)
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            odom.append((t, p.position.x, p.position.y, yaw, vx, vy))

    print(f"bag={bag}")
    print(f"/imu samples={len(imu)}  /testing_only/odom samples={len(odom)}")
    if len(odom) < 5 or len(imu) < 5:
        print("not enough data")
        return

    # GT references at odom rate: speed, longitudinal accel (d|v|/dt), yaw-rate.
    odom.sort()
    ot = [r[0] for r in odom]
    speed = [math.hypot(r[4], r[5]) for r in odom]
    yaw = [r[3] for r in odom]

    def _interp(ts, xs, t):
        if t <= ts[0]:
            return xs[0]
        if t >= ts[-1]:
            return xs[-1]
        lo, hi = 0, len(ts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        f = (t - ts[lo]) / (ts[hi] - ts[lo]) if ts[hi] > ts[lo] else 0.0
        return xs[lo] + f * (xs[hi] - xs[lo])

    # Build GT yaw-rate and longitudinal accel series (central difference).
    gyawrate = []
    glonaccel = []
    gt2 = ot[1:-1]
    for i in range(1, len(ot) - 1):
        dt = ot[i + 1] - ot[i - 1]
        if dt <= 0:
            gyawrate.append(0.0)
            glonaccel.append(0.0)
            continue
        dyaw = math.atan2(math.sin(yaw[i + 1] - yaw[i - 1]),
                          math.cos(yaw[i + 1] - yaw[i - 1]))
        gyawrate.append(dyaw / dt)
        glonaccel.append((speed[i + 1] - speed[i - 1]) / dt)

    # Pair each IMU sample to GT references at its timestamp.
    ax_s, ay_s, gz_s = [], [], []
    lon_ref, cen_ref, yr_ref = [], [], []
    for (t, ax, ay, gz) in imu:
        if t < gt2[0] or t > gt2[-1]:
            continue
        v = _interp(ot, speed, t)
        yr = _interp(gt2, gyawrate, t)
        lon = _interp(gt2, glonaccel, t)
        ax_s.append(ax); ay_s.append(ay); gz_s.append(gz)
        lon_ref.append(lon)
        cen_ref.append(v * yr)   # centripetal magnitude (lateral accel)
        yr_ref.append(yr)

    print(f"\npaired IMU-in-GT-span samples: {len(ax_s)}")
    print("\n--- gyro.z vs GT yaw-rate ---")
    print(f"  corr(gyro_z, gt_yawrate)      = {_corr(gz_s, yr_ref):+.3f}   "
          "(expect ~ +1; strong negative => Z sign flip)")
    # robust slope
    if len(gz_s) > 3:
        mg = sum(gz_s) / len(gz_s); my = sum(yr_ref) / len(yr_ref)
        num = sum((g - mg) * (y - my) for g, y in zip(gz_s, yr_ref))
        den = sum((y - my) ** 2 for y in yr_ref) or 1.0
        print(f"  slope gyro_z / gt_yawrate     = {num/den:+.3f}   (expect ~ +1)")

    print("\n--- accel.x (should be LONGITUDINAL/forward) ---")
    print(f"  corr(accel_x, longitudinal)   = {_corr(ax_s, lon_ref):+.3f}   (expect positive)")
    print(f"  corr(accel_x, centripetal)    = {_corr(ax_s, cen_ref):+.3f}   (expect ~0)")
    print("\n--- accel.y (should be LATERAL/centripetal) ---")
    print(f"  corr(accel_y, longitudinal)   = {_corr(ay_s, lon_ref):+.3f}   (expect ~0)")
    print(f"  corr(accel_y, centripetal)    = {_corr(ay_s, cen_ref):+.3f}   (expect strong, sign = convention)")

    print("\nINTERPRETATION:")
    print("  If accel_x tracks CENTRIPETAL and accel_y tracks LONGITUDINAL,")
    print("  the IMU body axes are swapped (90 deg rotation) in the raw data.")


if __name__ == "__main__":
    main()
