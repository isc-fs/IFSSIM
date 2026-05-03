#!/usr/bin/env python3
"""tools/diff_lidar_clouds.py — Phase 4 dual-path validation for #223.

Captures /lidar/Lidar1 scans under each LiDAR backend and computes the
per-point KD-tree NN-distance distribution between them. The acceptance
criteria from the design doc are:

  Static scene (parked):  >= 99 % of GPU points within  3 cm of nearest CPU point
  Driving scene (lap):    >= 95 % of GPU points within  5 cm of nearest CPU point

Workflow (the ROS subscriptions happen inside the bridge container; this
script is meant to be copied in and invoked there):

  # 1. Park the car at a stable scene.
  # 2. Confirm settings.json LidarPath: "cpu" and Play.
  docker cp tools/diff_lidar_clouds.py ifssim-dv_pipeline_stack-1:/tmp/
  docker exec ifssim-dv_pipeline_stack-1 \\
      python3 /tmp/diff_lidar_clouds.py record /tmp/cpu.npz --count 5

  # 3. Stop PIE, switch settings.json to "gpu", restart UE5, re-park, Play.
  docker exec ifssim-dv_pipeline_stack-1 \\
      python3 /tmp/diff_lidar_clouds.py record /tmp/gpu.npz --count 5

  # 4. Compute the NN distribution.
  docker exec ifssim-dv_pipeline_stack-1 \\
      python3 /tmp/diff_lidar_clouds.py compare /tmp/cpu.npz /tmp/gpu.npz

The captures are stored as numpy .npz with keys scan_0..scan_{N-1}; each
scan is an (N_pts, 3) float32 array of (x, y, z) in metres, ROS REP-103
vehicle frame.

Limitations: the two captures aren't simultaneous, so any pose drift
between them inflates the NN distances slightly. For the static-scene
criterion this is not a concern (parked car); for the driving-scene
criterion it is, which is why a real dual-path realtime mode (both
backends running side-by-side, /lidar/Lidar1 + /lidar/Lidar1_gpu) is
the rigorous form. Score this as a sanity diff; the strict acceptance
gate awaits dual-path realtime if needed.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# Local imports — only available inside the ROS-2 container.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2  # type: ignore
except ImportError as exc:  # pragma: no cover
    print(
        f"error: ROS-2 Python bindings not available ({exc}). "
        "Run inside the bridge container.",
        file=sys.stderr,
    )
    sys.exit(2)


def _scan_to_array(msg: PointCloud2) -> np.ndarray:
    """Decode a PointCloud2 into an (N, 3) float32 array."""
    pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not pts:
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray([(float(p[0]), float(p[1]), float(p[2])) for p in pts],
                      dtype=np.float32)


def cmd_record(args: argparse.Namespace) -> int:
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    rclpy.init()
    node = Node("diff_lidar_record")
    captures: list[np.ndarray] = []

    def cb(msg: PointCloud2) -> None:
        if len(captures) >= args.count:
            return
        arr = _scan_to_array(msg)
        captures.append(arr)
        print(f"  scan {len(captures):2}/{args.count}: {len(arr):6} pts")

    sub = node.create_subscription(PointCloud2, args.topic, cb, qos)
    print(f"Subscribing to {args.topic}, capturing {args.count} scans...")

    deadline = args.timeout
    while len(captures) < args.count and rclpy.ok() and deadline > 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        deadline -= 0.1

    rclpy.shutdown()

    if not captures:
        print("error: no scans captured (is /lidar/Lidar1 publishing?)", file=sys.stderr)
        return 1

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, **{f"scan_{i}": s for i, s in enumerate(captures)})
    print(f"Saved {len(captures)} scans to {out} "
          f"(total {sum(len(s) for s in captures)} pts)")
    return 0


def cmd_compare(args: argparse.Namespace) -> int:
    try:
        from scipy.spatial import cKDTree  # noqa: WPS433
    except ImportError:
        print("error: scipy not available in this container; "
              "install it or run on the host with PointCloud .npz files.",
              file=sys.stderr)
        return 2

    cpu = np.load(args.cpu)
    gpu = np.load(args.gpu)

    cpu_keys = sorted(k for k in cpu.files if k.startswith("scan_"))
    gpu_keys = sorted(k for k in gpu.files if k.startswith("scan_"))
    if not cpu_keys or not gpu_keys:
        print("error: no scans in one of the .npz files", file=sys.stderr)
        return 1

    print(f"CPU file: {len(cpu_keys)} scans  |  GPU file: {len(gpu_keys)} scans")

    # Compare using the median-population scan from each file (robust against
    # outliers like a partial first scan during play-up).
    def _representative(d, ks):
        sizes = [(k, d[k].shape[0]) for k in ks]
        sizes.sort(key=lambda kv: kv[1])
        return d[sizes[len(sizes) // 2][0]]

    cpu_pts = _representative(cpu, cpu_keys)
    gpu_pts = _representative(gpu, gpu_keys)
    print(f"  comparing {cpu_pts.shape[0]} CPU pts vs {gpu_pts.shape[0]} GPU pts")

    if len(cpu_pts) == 0 or len(gpu_pts) == 0:
        print("error: empty point cloud in capture", file=sys.stderr)
        return 1

    # NN distance: each GPU point → nearest CPU point.
    tree = cKDTree(cpu_pts)
    dists, _ = tree.query(gpu_pts, k=1)
    dists_cm = dists * 100.0

    pct_3 = (dists < 0.03).mean() * 100.0
    pct_5 = (dists < 0.05).mean() * 100.0

    print()
    print(f"NN distance (GPU → nearest CPU):")
    print(f"  mean        {dists_cm.mean():7.2f} cm")
    print(f"  50th pct    {np.percentile(dists_cm, 50):7.2f} cm")
    print(f"  95th pct    {np.percentile(dists_cm, 95):7.2f} cm")
    print(f"  99th pct    {np.percentile(dists_cm, 99):7.2f} cm")
    print(f"  max         {dists_cm.max():7.2f} cm")
    print()
    print(f"  >= {pct_3:5.1f} % within 3 cm  (static-scene gate: 99 %)")
    print(f"  >= {pct_5:5.1f} % within 5 cm  (driving-scene gate: 95 %)")
    print()
    static_pass  = pct_3 >= 99.0
    driving_pass = pct_5 >= 95.0
    print(f"  static-scene  : {'PASS' if static_pass  else 'FAIL'}")
    print(f"  driving-scene : {'PASS' if driving_pass else 'FAIL'}")
    return 0 if (static_pass or driving_pass) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_rec = sub.add_parser("record", help="Capture /lidar/Lidar1 scans to a .npz file")
    p_rec.add_argument("out", help="Output .npz path")
    p_rec.add_argument("--count", type=int, default=5, help="Number of scans (default 5)")
    p_rec.add_argument("--topic", default="/lidar/Lidar1", help="Topic to subscribe (default /lidar/Lidar1)")
    p_rec.add_argument("--timeout", type=float, default=15.0,
                       help="Max seconds to wait for the requested count (default 15)")
    p_rec.set_defaults(func=cmd_record)

    p_cmp = sub.add_parser("compare", help="KD-tree NN distance between two .npz captures")
    p_cmp.add_argument("cpu", help="CPU-mode .npz capture")
    p_cmp.add_argument("gpu", help="GPU-mode .npz capture")
    p_cmp.set_defaults(func=cmd_compare)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
