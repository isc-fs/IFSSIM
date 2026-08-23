#!/usr/bin/env python3
"""Measure EKF odometry drift against ground truth from a recorded bag.

Compares `/odom` (the EKF's dead-reckoned estimate) with `/testing_only/odom`
(sim ground truth) and reports how far the estimate wanders over a run.

THE FRAME TRAP — READ THIS BEFORE TRUSTING ANY NUMBER
-----------------------------------------------------
`/odom` and `/testing_only/odom` are NOT in the same frame. `/odom` is
dead-reckoned from wherever the filter started; ground truth is absolute sim
world, and the two differ by a documented 90 degree UE<->ENU rotation plus the
spawn offset.

Comparing them raw produces ~90 m position error and ~88 degrees of yaw error on
a run whose real drift is under 5 m. That looks like catastrophic divergence and
is entirely an artifact. The tell is that the "error" SHRINKS as the car returns
toward its start point — accumulating drift does not do that.

So this script rigidly aligns the first `/odom` sample onto ground truth
(rotate + translate) before measuring anything. Everything after that is
residual, which is the thing worth reporting.

WHAT THE NUMBERS MEAN
---------------------
  vx drift std   spread of the velocity error. This is the metric the EKF's
                 tuning is most directly judged on.
  position error grows roughly linearly for honest dead reckoning. Reported as
                 a percentage of distance travelled, which is comparable across
                 runs of different length — a raw metre count is not.
  yaw error      gyro-driven, so it is the first place a heading problem shows.

USAGE
    python3 tools/sim_benchmark/check_odom_drift.py <bag_dir>
    python3 tools/sim_benchmark/check_odom_drift.py <bag_dir> \\
        --max-drift-pct 2.0 --max-yaw-deg 10

With thresholds it exits non-zero on breach, so it can gate a regression run
rather than just printing something for a human to skim.

Run inside the pipeline container (needs rosbag2_py + the message types).
"""
from __future__ import annotations

import argparse
import math
import statistics as st
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("error: needs a ROS 2 environment (run inside dv_pipeline_stack)", file=sys.stderr)
    raise SystemExit(2)

ODOM_TOPIC = "/odom"
GT_TOPIC = "/testing_only/odom"


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap_deg(rad: float) -> float:
    return math.degrees((rad + math.pi) % (2.0 * math.pi) - math.pi)


def read_bag(path: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {c.name: c.type for c in reader.get_all_topics_and_types()}
    for t in (ODOM_TOPIC, GT_TOPIC):
        if t not in types:
            raise SystemExit(f"error: bag has no {t} — cannot measure drift")
    cls = {t: get_message(types[t]) for t in (ODOM_TOPIC, GT_TOPIC)}

    odom, gt, t0 = [], [], None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in cls:
            continue
        if t0 is None:
            t0 = stamp
        m = deserialize_message(data, cls[topic])
        rec = ((stamp - t0) / 1e9,
               m.pose.pose.position.x, m.pose.pose.position.y,
               _yaw(m.pose.pose.orientation),
               m.twist.twist.linear.x)
        (odom if topic == ODOM_TOPIC else gt).append(rec)
    return odom, gt


def _nearest(seq, t):
    lo, hi = 0, len(seq) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if seq[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return seq[lo]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--max-drift-pct", type=float,
                    help="fail if final position error exceeds this %% of distance travelled")
    ap.add_argument("--max-yaw-deg", type=float,
                    help="fail if final yaw error exceeds this many degrees")
    args = ap.parse_args()

    odom, gt = read_bag(args.bag)
    if len(odom) < 2 or len(gt) < 2:
        raise SystemExit("error: not enough odometry samples")

    # Rigid alignment on the first /odom sample. See the frame trap above.
    ot, ox, oy, oyaw, _ = odom[0]
    g0 = _nearest(gt, ot)
    dth = g0[3] - oyaw
    cos_t, sin_t = math.cos(dth), math.sin(dth)
    print(f"frame alignment: rotated /odom by {math.degrees(dth):+.1f} deg onto ground truth")

    pos_err, yaw_err, vx_err, times = [], [], [], []
    for t, x, y, yw, vx in odom:
        rx, ry = x - ox, y - oy
        ax = cos_t * rx - sin_t * ry + g0[1]
        ay = sin_t * rx + cos_t * ry + g0[2]
        g = _nearest(gt, t)
        pos_err.append(math.hypot(ax - g[1], ay - g[2]))
        yaw_err.append(abs(_wrap_deg((yw + dth) - g[3])))
        vx_err.append(vx - g[4])
        times.append(t)

    dist = sum(math.hypot(gt[i][1] - gt[i - 1][1], gt[i][2] - gt[i - 1][2])
               for i in range(1, len(gt)))
    drift_pct = 100.0 * pos_err[-1] / dist if dist > 0 else float("nan")

    print(f"\nrun: {times[-1]:.0f} s, {dist:.0f} m travelled")
    print(f"  vx drift      mean {st.mean(vx_err):+.4f}  std {st.pstdev(vx_err):.4f} m/s")
    print(f"  position err  mean {st.mean(pos_err):.2f}  final {pos_err[-1]:.2f}  max {max(pos_err):.2f} m")
    print(f"  yaw err       mean {st.mean(yaw_err):.2f}  final {yaw_err[-1]:.2f}  max {max(yaw_err):.2f} deg")
    print(f"  final drift   {drift_pct:.2f}% of distance travelled")

    # Growth profile. Honest dead reckoning grows roughly linearly; a step or a
    # plateau is a sign of something other than integration error.
    n, q = len(pos_err), max(1, len(pos_err) // 4)
    print("  growth:")
    for i in range(4):
        seg = pos_err[i * q:(i + 1) * q] or [0.0]
        a, b = times[min(i * q, n - 1)], times[min((i + 1) * q - 1, n - 1)]
        print(f"    {a:6.1f}-{b:6.1f}s  mean {st.mean(seg):6.2f} m  max {max(seg):6.2f} m")

    failed = []
    if args.max_drift_pct is not None and drift_pct > args.max_drift_pct:
        failed.append(f"drift {drift_pct:.2f}% > {args.max_drift_pct}%")
    if args.max_yaw_deg is not None and yaw_err[-1] > args.max_yaw_deg:
        failed.append(f"yaw {yaw_err[-1]:.2f} deg > {args.max_yaw_deg}")
    if failed:
        print("\nFAIL: " + "; ".join(failed))
        return 1
    if args.max_drift_pct is not None or args.max_yaw_deg is not None:
        print("\nPASS: within thresholds")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
