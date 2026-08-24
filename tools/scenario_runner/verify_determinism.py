#!/usr/bin/env python3
"""Prove (or disprove) that the simulator reproduces a run from its seed.

WHAT THIS SETTLES
-----------------
Stage 0 seeded every stochastic source in the sim and claimed runs are now
reproducible. That claim was never demonstrated. An unproven determinism claim
is worse than none, because comparisons get built on top of it — so this test
either confirms it or finds the hole.

METHOD
------
Subscribe to a noisy topic FIRST, then issue `resetScenario <seed>`, then
capture the next K samples. Because the subscriber is already running when the
reset lands, capture starts at a known point in the sequence.

That ordering is the whole trick. An earlier attempt sampled with
`ros2 topic echo --once` AFTER resetting and compared whatever arrived, i.e.
sample N of one run against sample M of another. It showed no match and proved
nothing — the windows were never aligned.

Three runs:
    A: seed S      B: seed S (same)      C: seed S' (different)

    A vs B  must MATCH    — same seed reproduces
    A vs C  must DIFFER   — different seed actually changes the draw

Both directions matter. A test that only checks A==B passes trivially if the
sensor is silent or the topic is constant, so C is the control that proves the
observable is actually sensitive to the seed.

WHY /imu AND NOT GROUND-TRUTH POSE
----------------------------------
A stationary car's pose trace is identical no matter what, so it would report
success while measuring nothing. IMU noise is live whether or not the car
moves, and it is exactly what the seeding changed. Full closed-loop determinism
(identical control input producing identical trajectory) is a further step and
needs scripted control, which does not exist yet.

RUN IT INSIDE THE PIPELINE CONTAINER (needs rclpy), with the sim in Play:
    docker exec ifssim-dv_pipeline_stack-1 bash -lc \\
      'source /opt/ros/humble/setup.bash && python3 /tmp/verify_determinism.py'
"""
from __future__ import annotations

import argparse
import socket
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


def reset_scenario(seed: int, host: str, port: int, timeout: float = 20.0) -> str:
    s = socket.socket()
    s.settimeout(timeout)
    s.connect((host, port))
    s.sendall(f"resetScenario {seed}\n".encode())
    buf = b""
    try:
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
    except socket.timeout:
        pass
    s.close()
    return buf.decode(errors="replace").strip()


class Collector(Node):
    def __init__(self):
        super().__init__("determinism_collector")
        self.samples: list[tuple[float, float, float]] = []
        self.collecting = False
        self.create_subscription(Imu, "/imu", self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Imu):
        if self.collecting:
            self.samples.append((msg.linear_acceleration.x,
                                 msg.linear_acceleration.y,
                                 msg.angular_velocity.z))


def capture(node: Collector, seed: int, n: int, host: str, port: int) -> list:
    """Reset, then collect the next n samples. Subscriber is already live."""
    node.samples.clear()
    node.collecting = False
    # Drain anything in flight so the first captured sample is post-reset.
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.01)
    node.samples.clear()

    reply = reset_scenario(seed, host, port)
    node.collecting = True

    deadline = time.time() + 30.0
    while len(node.samples) < n and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.collecting = False
    print(f"  seed {seed}: reset -> {reply}   captured {len(node.samples)} samples")
    return list(node.samples)


def longest_common_run(a: list, b: list) -> int:
    """Longest identical contiguous block. Tolerates a constant offset between
    the two capture windows, which a naive element-wise compare would not."""
    if not a or not b:
        return 0
    best = 0
    index = {}
    for i, v in enumerate(a):
        index.setdefault(v, []).append(i)
    for j, v in enumerate(b):
        for i in index.get(v, ())[:64]:      # cap the search; ties are cheap
            k = 0
            while i + k < len(a) and j + k < len(b) and a[i + k] == b[j + k]:
                k += 1
            best = max(best, k)
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=99)
    ap.add_argument("--other-seed", type=int, default=12345)
    ap.add_argument("--samples", type=int, default=400)
    ap.add_argument("--host", default="host.docker.internal")
    ap.add_argument("--port", type=int, default=41451)
    args = ap.parse_args()

    rclpy.init()
    node = Collector()
    try:
        print("capturing three runs (subscriber starts before each reset)")
        A = capture(node, args.seed, args.samples, args.host, args.port)
        B = capture(node, args.seed, args.samples, args.host, args.port)
        C = capture(node, args.other_seed, args.samples, args.host, args.port)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if min(len(A), len(B), len(C)) < 20:
        print("\nINCONCLUSIVE: too few samples — is the sim in Play and /imu publishing?")
        return 2

    ab = longest_common_run(A, B)
    ac = longest_common_run(A, C)
    n = min(len(A), len(B), len(C))

    print(f"\nlongest identical run, same seed      A vs B : {ab} / {n}")
    print(f"longest identical run, different seed A vs C : {ac} / {n}")
    print(f"A[:2]={A[:2]}\nB[:2]={B[:2]}")

    same_ok = ab >= max(10, n // 4)
    diff_ok = ac < max(10, n // 4)

    print()
    if same_ok and diff_ok:
        print("PASS: the same seed reproduces the noise sequence, a different seed changes it.")
        return 0
    if not same_ok:
        print("FAIL: the same seed did NOT reproduce. Seeding does not reach this observable,")
        print("      or something outside the seeded streams is perturbing it.")
    if not diff_ok:
        print("FAIL: a different seed produced the SAME sequence. The observable is not")
        print("      actually seed-sensitive, so a same-seed match would have meant nothing.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
