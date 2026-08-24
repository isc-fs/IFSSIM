#!/usr/bin/env python3
"""Discriminating test: is seeding broken, or is resetScenario insufficient?

verify_determinism.py showed that `resetScenario <seed>` does NOT reproduce the
published IMU sequence. That has two very different possible causes:

  (a) the RNG seeding does not actually work, or
  (b) the seeding is fine, but resetScenario leaves physics-solver state
      (suspension compression, contacts, residual motion) from the previous
      run, so the acceleration SIGNAL differs even when the noise draw matches.

`/imu` carries signal + noise together, so that test cannot separate them.

This one can. A fresh PIE session rebuilds physics from scratch and re-runs
BeginPlay, which re-seeds from settings.json. So comparing the first samples of
two FRESH sessions removes the carried-over-state variable entirely:

    match   -> seeding works; the fault is resetScenario (cause b)
    differ  -> seeding does not reach the sensors (cause a)

Either answer is actionable, which is the point of running it.

HOW IT WORKS
------------
Subscribes to /imu and watches for a gap in the stream. Stopping PIE halts the
publishers; starting it again resumes them. So:

    phase 1  capture the first N samples of the CURRENT session
    phase 2  wait for the stream to stop  (you press Stop)
    phase 3  capture the first N samples after it resumes  (you press Play)

Both sessions seed from the same settings.json value, so they are same-seed by
construction — no reset involved anywhere.

USAGE (inside the pipeline container, sim in Play):
    python3 /tmp/vdf.py --samples 300
Then, when prompted, press Stop and then Play in the editor.
"""
from __future__ import annotations

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
except ImportError:
    print("error: needs rclpy — run inside dv_pipeline_stack", file=sys.stderr)
    raise SystemExit(2)


class Watcher(Node):
    def __init__(self):
        super().__init__("determinism_fresh")
        self.samples: list[tuple] = []
        self.last_msg_time = 0.0
        self.count = 0
        self.collecting = False
        self.create_subscription(Imu, "/imu", self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Imu):
        self.last_msg_time = time.time()
        self.count += 1
        if self.collecting:
            # Keep the SIM timestamp. Comparing by arrival order is not safe:
            # the bridge reconnects asynchronously relative to sim start, so
            # sample #1 of two sessions can sit at different sim times, and the
            # physics signal at those times legitimately differs. Aligning on
            # sim time is what makes the comparison mean anything.
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.samples.append((round(st, 6),
                                 msg.linear_acceleration.x,
                                 msg.linear_acceleration.y,
                                 msg.angular_velocity.z))


def pump(node, seconds: float):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def collect(node, n: int, timeout: float = 60.0) -> list:
    node.samples.clear()
    node.collecting = True
    end = time.time() + timeout
    while len(node.samples) < n and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.collecting = False
    return list(node.samples)


def wait_for_stop(node, quiet_for: float = 2.0, timeout: float = 600.0) -> bool:
    """Return once the stream has been silent for `quiet_for` seconds."""
    node.last_msg_time = time.time()
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - node.last_msg_time > quiet_for:
            return True
    return False


def wait_for_resume(node, timeout: float = 600.0) -> bool:
    before = node.count
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.count > before + 5:
            return True
    return False


def longest_common_run(a: list, b: list) -> int:
    if not a or not b:
        return 0
    best, index = 0, {}
    for i, v in enumerate(a):
        index.setdefault(v, []).append(i)
    for j, v in enumerate(b):
        for i in index.get(v, ())[:64]:
            k = 0
            while i + k < len(a) and j + k < len(b) and a[i + k] == b[j + k]:
                k += 1
            best = max(best, k)
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", type=int, default=300)
    args = ap.parse_args()

    rclpy.init()
    node = Watcher()
    try:
        pump(node, 1.0)
        if node.count == 0:
            print("error: no /imu traffic — is the sim in Play?", file=sys.stderr)
            return 2

        print(f"session 1: capturing first {args.samples} samples...")
        A = collect(node, args.samples)
        print(f"  got {len(A)}")

        print("\n>>> PRESS STOP IN THE EDITOR NOW (waiting for the stream to halt) <<<",
              flush=True)
        if not wait_for_stop(node):
            print("timed out waiting for Stop", file=sys.stderr)
            return 2
        print("  stream halted — PIE stopped")

        print("\n>>> NOW PRESS PLAY AGAIN (waiting for the stream to resume) <<<",
              flush=True)
        if not wait_for_resume(node):
            print("timed out waiting for Play", file=sys.stderr)
            return 2
        print("  stream resumed — fresh session")

        print(f"session 2: capturing first {args.samples} samples...")
        B = collect(node, args.samples)
        print(f"  got {len(B)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    n = min(len(A), len(B))
    if n < 20:
        print("\nINCONCLUSIVE: too few samples")
        return 2

    # Align on SIM TIME: compare only samples whose sim timestamps coincide.
    da = {t: v for t, *v in A}
    db = {t: v for t, *v in B}
    shared = sorted(set(da) & set(db))
    print(f"\nsim-time range  A: {A[0][0]:.3f}..{A[-1][0]:.3f}   B: {B[0][0]:.3f}..{B[-1][0]:.3f}")
    print(f"samples at identical sim times: {len(shared)}")
    if shared:
        same = sum(1 for t in shared if da[t] == db[t])
        print(f"  of those, byte-identical values: {same}/{len(shared)}")
        for t in shared[:3]:
            print(f"    t={t:.3f}  A={da[t][0]:+.6f}  B={db[t][0]:+.6f}"
                  f"  {'SAME' if da[t]==db[t] else 'DIFF'}")
        run = same if same >= len(shared) * 0.9 else 0
        n = len(shared)
    else:
        print("  no overlapping sim times — sessions did not cover the same window")
        run = longest_common_run([tuple(v) for _, *v in A], [tuple(v) for _, *v in B])
    print(f"  A[:1]={A[:1]}")
    print(f"  B[:1]={B[:1]}")

    print()
    if run >= max(10, n // 4):
        print("MATCH -> RNG seeding WORKS across fresh sessions.")
        print("         The fault is resetScenario: it does not restore physics state,")
        print("         so repeats via reset are not true repeats. Fix the reset, or")
        print("         require a PIE restart between compared runs.")
        return 0
    print("DIFFER -> seeding does NOT reach the sensors even on a fresh session.")
    print("          The problem is in the seeding path itself, not resetScenario.")
    print("          Note this could also be physics settling differing between")
    print("          sessions; if so the signal, not the noise, is what differs.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
