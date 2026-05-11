#!/usr/bin/env python3
"""Side-by-side comparison of two odometry filters on a recorded bag.

Reads ``lap_postveto_*`` (or any bag with /testing_only/odom + /odom +
/odom_ekf), aligns the three pose streams to a common start frame,
and reports the position-vs-GT residual envelope for each filter
over the autonomy window.

Goal: show whether robot_localization's EKF (publishing /odom_ekf)
holds the ‖map → odom‖ correction sub-metre, where the existing
complementary filter (publishing /odom) accumulated 15-35 m.

Usage:

    # Live capture (one-shot)
    docker exec -d ifssim-dv_pipeline_stack-1 bash -lc \\
      "source /opt/ros/humble/setup.bash && \\
       ros2 launch /dv_pipeline_stack_ws/dvpc_ekf.launch.py &"
    docker exec -d ifssim-dv_pipeline_stack-1 bash -lc \\
      "ros2 bag record -s mcap -o /tmp/compare_bag -a"
    # ... drive a lap ...
    docker exec ... pkill -INT -f 'ros2 bag record'

    # Offline analysis on a recorded bag
    docker exec -i ifssim-dv_pipeline_stack-1 bash -lc \\
      "source /opt/ros/humble/setup.bash && \\
       source /dv_pipeline_stack_ws/install/setup.bash && \\
       export AMENT_PREFIX_PATH=/dv_pipeline_stack_ws/install/fs_msgs:\\\$AMENT_PREFIX_PATH && \\
       python3 /dv_pipeline_stack_ws/bags/compare_filters.py /dv_pipeline_stack_ws/bags/<bag-dir>"

Output: per-second residual table for each filter against
GT-aligned, plus a summary of mean / max / p95 deviations and the
peak ‖integrator pose − GT pose‖ for each.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def yaw_of(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def load_bag(bag_dir: Path):
    storage = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id=("mcap" if any(p.suffix == ".mcap" for p in bag_dir.iterdir())
                    else "sqlite3"),
    )
    conv = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, conv)
    classes = {t.name: get_message(t.type)
               for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        cls = classes.get(topic)
        if cls is None:
            continue
        yield topic, t_ns, deserialize_message(raw, cls)


def main() -> None:
    bag_dir = Path(sys.argv[1])
    print(f"=== filter comparison: {bag_dir.name} ===\n")

    # Sources we care about.
    series = {
        "/testing_only/odom": [],   # GT
        "/odom":              [],   # current complementary filter
        "/odom_ekf":          [],   # robot_localization EKF
    }
    t0_ns: int | None = None
    for topic, t_ns, msg in load_bag(bag_dir):
        if topic not in series:
            continue
        if t0_ns is None:
            t0_ns = t_ns
        t = (t_ns - t0_ns) * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        series[topic].append((t, p.x, p.y, yaw_of(q)))

    n_gt = len(series["/testing_only/odom"])
    n_old = len(series["/odom"])
    n_ekf = len(series["/odom_ekf"])
    print(f"  GT (testing_only/odom): {n_gt} samples")
    print(f"  Complementary (/odom):  {n_old} samples")
    print(f"  EKF (/odom_ekf):        {n_ekf} samples")
    print()

    if n_gt == 0:
        print("No /testing_only/odom in bag — cannot compute residuals.",
              file=sys.stderr)
        sys.exit(2)

    # Anchor each stream to a STABLE position during the first 0.5 s.
    # Using the very first sample is fragile: GT often has an outlier
    # transient (sim_supervisor pose pre-stabilisation) ~10 m off,
    # which spoils the anchor and makes every later residual look huge.
    # Median of the first 0.5 s of samples is robust against single
    # outliers.
    def anchor(s, t_window: float = 0.5):
        if not s:
            return s
        head = [r for r in s if r[0] <= s[0][0] + t_window]
        if not head:
            head = [s[0]]
        xs = sorted(r[1] for r in head)
        ys = sorted(r[2] for r in head)
        yaws = [r[3] for r in head]
        x0 = xs[len(xs) // 2]
        y0 = ys[len(ys) // 2]
        # Circular mean for yaw — wrap-safe.
        sin_sum = sum(math.sin(y) for y in yaws)
        cos_sum = sum(math.cos(y) for y in yaws)
        yaw0 = math.atan2(sin_sum, cos_sum)
        c, ss = math.cos(-yaw0), math.sin(-yaw0)
        out = []
        for t, x, y, yaw in s:
            dx, dy = x - x0, y - y0
            xa = c * dx - ss * dy
            ya = ss * dx + c * dy
            yawa = ((yaw - yaw0) + math.pi) % (2 * math.pi) - math.pi
            out.append((t, xa, ya, yawa))
        return out

    gt = anchor(series["/testing_only/odom"])
    old = anchor(series["/odom"])
    ekf = anchor(series["/odom_ekf"])

    def latest_before(s, t_target):
        best = None
        for sample in s:
            if sample[0] > t_target:
                break
            best = sample
        return best

    def residuals(filter_stream, name):
        if not filter_stream:
            print(f"  {name}: no samples — skipped.")
            return
        rows = []
        for t, x, y, yaw in filter_stream:
            g = latest_before(gt, t)
            if g is None:
                continue
            err = math.hypot(x - g[1], y - g[2])
            rows.append((t, err))
        if not rows:
            print(f"  {name}: no GT-aligned rows.")
            return
        errs = sorted(e for _, e in rows)
        n = len(errs)
        print(f"  {name:>16s}  n={n:5d}  "
              f"mean={sum(errs)/n:6.3f} m  "
              f"p50={errs[n//2]:6.3f} m  "
              f"p95={errs[int(0.95*n)]:6.3f} m  "
              f"max={errs[-1]:6.3f} m  "
              f"@t_max={rows[max(range(len(rows)), key=lambda i: rows[i][1])][0]:.1f} s")

    print("=== Position residual ‖filter − GT‖ (anchored, m) ===")
    residuals(old, "complementary")
    residuals(ekf, "EKF (robot_loc)")
    print()

    # 5-second checkpoint table — easier to eyeball where each
    # filter starts diverging from GT.
    print("=== Per-5-s checkpoints (errors in m) ===")
    print(f"  {'t [s]':>6}  {'old':>8}  {'EKF':>8}  {'old−EKF':>8}")
    t = 0.0
    end = gt[-1][0] if gt else 0.0
    while t <= end:
        og = latest_before(old, t)
        eg = latest_before(ekf, t)
        g  = latest_before(gt,  t)
        if g:
            e_old = (math.hypot(og[1] - g[1], og[2] - g[2])
                     if og else float("nan"))
            e_ekf = (math.hypot(eg[1] - g[1], eg[2] - g[2])
                     if eg else float("nan"))
            d = e_old - e_ekf
            print(f"  {t:6.1f}  {e_old:8.3f}  {e_ekf:8.3f}  {d:+8.3f}")
        t += 5.0


if __name__ == "__main__":
    main()
