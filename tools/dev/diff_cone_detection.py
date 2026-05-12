#!/usr/bin/env python3
"""tools/dev/diff_cone_detection.py — Phase 5 acceptance test for #223.

Captures /Conos_raw cone-detection MarkerArrays under each LiDAR backend
and computes 2D centroid NN distance between them. The actually-load-
bearing test for shipping the GPU LiDAR — point-cloud per-point gate
(Phase 4) hits a fundamental rasterization-vs-trace floor at cone
silhouette edges, but cluster centroids should still match because the
cone clustering tolerates several cm of point spread inside the cone
diameter.

Workflow (runs inside the bridge container; copy in via docker cp):

  # Park the car at a stable scene; settings.json LidarPath: "cpu"; Play.
  # Then: curl -X POST http://localhost:8000/api/pipeline/start
  python3 /tmp/diff_cone_detection.py record /tmp/cones_cpu.npy --window 8

  # Stop PIE, switch to "gpu", restart UE5, re-park, Play, restart pipeline.
  python3 /tmp/diff_cone_detection.py record /tmp/cones_gpu.npy --window 8

  # Diff:
  python3 /tmp/diff_cone_detection.py compare /tmp/cones_cpu.npy /tmp/cones_gpu.npy
"""
from __future__ import annotations

import argparse
import sys
import time
from collections import Counter

import numpy as np

try:
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # type: ignore
    from visualization_msgs.msg import MarkerArray  # type: ignore
except ImportError as exc:  # pragma: no cover
    print(f"error: ROS-2 bindings not available ({exc}). Run inside bridge container.",
          file=sys.stderr)
    sys.exit(2)


def cmd_record(args: argparse.Namespace) -> int:
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
    rclpy.init()
    node = Node("cone_capture")
    captures: list[list[tuple[float, float, float]]] = []

    def cb(msg: MarkerArray) -> None:
        cones = [(m.pose.position.x, m.pose.position.y, m.pose.position.z)
                 for m in msg.markers if m.action != 2]
        captures.append(cones)

    sub = node.create_subscription(MarkerArray, args.topic, cb, qos)
    print(f"Subscribing to {args.topic}, sampling for {args.window} s...")
    deadline = time.time() + args.window
    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    sizes = Counter(len(c) for c in captures)
    print(f"  capture-count distribution: {dict(sizes)}")

    # /Conos_raw publishes per-scan, often with empty/partial messages.
    # Use the *median* of the non-trivial captures (filtering size > 5)
    # as the stable "fully-detected" cone state.
    nonzero = [c for c in captures if len(c) > 5]
    if not nonzero:
        print("error: no fully-populated cone msgs in capture window", file=sys.stderr)
        return 1
    nonzero.sort(key=len)
    final = nonzero[len(nonzero) // 2]
    arr = np.asarray(final, dtype=np.float32)
    np.save(args.out, arr)
    print(f"  saved {arr.shape[0]} cones to {args.out}  "
          f"(x: {arr[:,0].min():.2f} .. {arr[:,0].max():.2f}, "
          f"y: {arr[:,1].min():.2f} .. {arr[:,1].max():.2f})")
    return 0


def cmd_compare(args: argparse.Namespace) -> int:
    try:
        from scipy.spatial import cKDTree  # noqa: WPS433
    except ImportError:
        print("error: scipy not available", file=sys.stderr)
        return 2

    cpu = np.load(args.cpu)
    gpu = np.load(args.gpu)
    print(f"CPU cones: {len(cpu)}  |  GPU cones: {len(gpu)}")

    if len(cpu) == 0 or len(gpu) == 0:
        print("error: empty capture", file=sys.stderr)
        return 1

    # 2D centroid match (cone z is always 0 for ground-detected cones).
    tree = cKDTree(cpu[:, :2])
    d_gc, idx_gc = tree.query(gpu[:, :2], k=1)
    tree2 = cKDTree(gpu[:, :2])
    d_cg, _ = tree2.query(cpu[:, :2], k=1)

    print(f"\nGPU → nearest CPU cone:")
    print(f"  median: {np.median(d_gc)*100:6.2f} cm")
    print(f"  mean  : {d_gc.mean()*100:6.2f} cm")
    print(f"  max   : {d_gc.max()*100:6.2f} cm")
    print(f"  within  30 cm: {(d_gc < 0.30).sum()}/{len(d_gc)}")
    print(f"  within   1 m : {(d_gc < 1.00).sum()}/{len(d_gc)}")

    print(f"\nCPU → nearest GPU cone:")
    print(f"  median: {np.median(d_cg)*100:6.2f} cm")
    print(f"  mean  : {d_cg.mean()*100:6.2f} cm")
    print(f"  max   : {d_cg.max()*100:6.2f} cm")
    print(f"  within  30 cm: {(d_cg < 0.30).sum()}/{len(d_cg)}")

    # Acceptance: median < 5 cm AND ≥ 90 % of cones in either direction
    # within 30 cm (one cone diameter).
    median_ok = max(np.median(d_gc), np.median(d_cg)) * 100 < 5.0
    coverage_g = (d_gc < 0.30).mean() >= 0.90
    coverage_c = (d_cg < 0.30).mean() >= 0.90
    overall_pass = median_ok and coverage_g and coverage_c

    print()
    print(f"  median < 5 cm  : {'PASS' if median_ok  else 'FAIL'}")
    print(f"  GPU coverage   : {'PASS' if coverage_g else 'FAIL'}  (≥ 90 % within 30 cm)")
    print(f"  CPU coverage   : {'PASS' if coverage_c else 'FAIL'}")
    print(f"  acceptance     : {'PASS' if overall_pass else 'FAIL'}")

    if args.verbose:
        print("\nPer-GPU-cone diffs (sorted by GPU.x):")
        order = np.argsort(gpu[:, 0])
        for i in order:
            g = gpu[i]
            c = cpu[idx_gc[i]]
            d = d_gc[i] * 100
            print(f"  GPU({g[0]:6.2f},{g[1]:+6.2f})  →  CPU({c[0]:6.2f},{c[1]:+6.2f})  Δ={d:6.1f} cm")

    return 0 if overall_pass else 1


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    p_rec = sub.add_parser("record", help="Capture /Conos_raw centroids to a .npy")
    p_rec.add_argument("out", help="Output .npy path")
    p_rec.add_argument("--topic", default="/Conos_raw")
    p_rec.add_argument("--window", type=float, default=8.0,
                       help="Sampling window in seconds (default 8)")
    p_rec.set_defaults(func=cmd_record)

    p_cmp = sub.add_parser("compare", help="2D centroid NN diff between two captures")
    p_cmp.add_argument("cpu", help="CPU .npy")
    p_cmp.add_argument("gpu", help="GPU .npy")
    p_cmp.add_argument("--verbose", action="store_true",
                       help="Print per-cone deltas")
    p_cmp.set_defaults(func=cmd_compare)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
