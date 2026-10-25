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
Subscribe FIRST, then issue `resetScenario <seed>`, then capture the next K game
ticks. Because the subscriber is already running when the reset lands, capture
starts at a known point in the sequence.

That ordering is the whole trick. An earlier attempt sampled with
`ros2 topic echo --once` AFTER resetting and compared whatever arrived, i.e.
sample N of one run against sample M of another. It showed no match and proved
nothing — the windows were never aligned.

WHAT IS COMPARED (#643)
-----------------------
The injected noise, one value per game tick:

    residual = /imu angular_velocity.z - /testing_only/odom twist.angular.z

Both come from the same sensor frame, and odom carries the noise-free yaw rate.
The car rests on flat ground after the reset, so its yaw rate is not excited and
the residual is exactly the gyro-z bias plus white noise. All of the IMU's draws
(accelerometer and gyro, bias and white noise) come from one stream in a fixed
order every tick, so the gyro-z draw only repeats tick after tick if the whole
stream does.

Why yaw only: the reset drops the car onto its suspension, which excites pitch
(up to 0.17 rad/s), and the IMU and the ground truth do not sample that motion
at the same instant (measured: they differ by 0.02 rad/s RMS while the car
settles, against 0.003 rad/s of noise). A pitch or roll residual therefore
carries motion as well as noise.

Comparing raw /imu samples, as this script first did, cannot work:
  - The sim sends each tick's frame several times (a 400 Hz stream over a 60 Hz
    tick), and how many copies each tick gets varies from run to run. Ticks are
    recovered from the header stamp: the bridge bumps a repeated /imu stamp by
    1 ns, so flooring to the microsecond maps every copy back to its tick. The
    sim clock advances exactly 1/60 s per tick, and resetScenario replies with
    the sim time of the first tick that runs with the new seed (it resets at
    the start of that tick, before anything ticks), so each tick is numbered
    from the reset: tick 0 holds the new seed's first draw in every run. A tick
    with no message on either topic (both are best-effort, and odom goes out on
    every 4th stream frame) is a gap, not a shift of every later tick, and
    anything still in flight from before the reset is numbered below 0 and is
    dropped.
  - /imu is signal plus noise, and the signal is physics. After a teleport the
    car's physics is not bit-identical between runs (measured: the
    ground-truth yaw rate differs at the 1e-7 rad/s level within a quarter of a
    second, and the accelerometer by more), and the seed makes no claim about
    it. docs/ENVIRONMENT_ROADMAP.md (rule 9) treats physics as statistical for
    this reason.

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
import json
import socket
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
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


# The gyro noise is ~3.5e-3 rad/s (settings.json GyroNoiseStd). Float32 on the
# wire and the two body-frame rotations agree to ~1e-9, so 1e-6 separates "the
# same draw" from "a different draw" by three orders of magnitude either way.
MATCH_TOL = 1e-6


TICK_US = 1e6 / 60.0   # bUseFixedFrameRate, FixedFrameRate=60 (Config/DefaultEngine.ini)


def tick_key(stamp) -> int:
    """The game tick a message belongs to: its stamp floored to the microsecond.
    Ticks are 16.7 ms apart; the bridge's per-copy /imu bumps are 1 ns."""
    return (stamp.sec * 1_000_000_000 + stamp.nanosec) // 1000


class Collector(Node):
    def __init__(self):
        super().__init__("determinism_collector")
        self.collecting = False
        self.imu: dict[int, float] = {}
        self.truth: dict[int, float] = {}
        self.create_subscription(Imu, "/imu", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/testing_only/odom", self._on_odom,
                                 qos_profile_sensor_data)

    def _on_imu(self, msg: Imu):
        if self.collecting:
            self.imu.setdefault(tick_key(msg.header.stamp), msg.angular_velocity.z)

    def _on_odom(self, msg: Odometry):
        if self.collecting:
            self.truth.setdefault(tick_key(msg.header.stamp), msg.twist.twist.angular.z)

    def ticks(self) -> list[int]:
        return sorted(set(self.imu) & set(self.truth))

    def residuals(self, reset_us: float) -> dict[int, float]:
        """Residual by tick number from the reset (tick 0 = the new seed's first draw)."""
        out = {}
        for t in self.ticks():
            k = round((t - reset_us) / TICK_US)
            if k >= 0:
                out[k] = self.imu[t] - self.truth[t]
        return out


def capture(node: Collector, seed: int, n: int, host: str, port: int) -> dict:
    """Reset, then collect the yaw-rate residual for the next n ticks. The
    subscriber is already live."""
    node.collecting = False
    # Drain anything in flight so the first captured tick is (nearly) post-reset.
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.01)
    node.imu.clear()
    node.truth.clear()

    reply = reset_scenario(seed, host, port)
    node.collecting = True
    try:
        reset_us = float(json.loads(reply)["sim_time"]) * 1e6
    except (ValueError, KeyError, TypeError):
        print(f"  seed {seed}: reset -> {reply}")
        print("  the reply has no sim_time; this needs a plugin with #643's resetScenario")
        raise SystemExit(2)

    deadline = time.time() + 30.0
    while len(node.ticks()) < n and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.collecting = False
    res = node.residuals(reset_us)
    span = max(res) + 1 if res else 0
    print(f"  seed {seed}: reset -> {reply}   {len(res)} of the first {span} ticks "
          f"have both topics")
    return res


def same(u: float, v: float) -> bool:
    return abs(u - v) <= MATCH_TOL


def compare(a: dict, b: dict) -> tuple[int, int]:
    """(matching, compared) over the ticks both captures have. Ticks are
    numbered from the reset, so there is no offset to search: the same seed
    must match tick for tick."""
    common = [t for t in a if t in b]
    return sum(same(a[t], b[t]) for t in common), len(common)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=99)
    ap.add_argument("--other-seed", type=int, default=12345)
    ap.add_argument("--ticks", type=int, default=180,
                    help="ticks with both topics to collect per run (60 ticks per sim second)")
    ap.add_argument("--host", default="host.docker.internal")
    ap.add_argument("--port", type=int, default=41451)
    args = ap.parse_args()

    rclpy.init()
    node = Collector()
    try:
        print("capturing three runs (subscriber starts before each reset)")
        A = capture(node, args.seed, args.ticks, args.host, args.port)
        B = capture(node, args.seed, args.ticks, args.host, args.port)
        C = capture(node, args.other_seed, args.ticks, args.host, args.port)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if min(len(A), len(B), len(C)) < 30:
        print("\nINCONCLUSIVE: too few ticks — is the sim in Play, and are /imu and")
        print("              /testing_only/odom publishing with matching stamps?")
        return 2

    ab, ab_n = compare(A, B)
    ac, ac_n = compare(A, C)

    print(f"\nmatching ticks, same seed      A vs B : {ab} / {ab_n}")
    print(f"matching ticks, different seed A vs C : {ac} / {ac_n}")
    first = sorted(set(A) & set(B))[:3]
    print(f"gyro-z noise at ticks {first}:  A {[round(A[t], 6) for t in first]}")
    print(f"{'':>{len(f'gyro-z noise at ticks {first}:')}}  B {[round(B[t], 6) for t in first]}")

    # Every tick both runs captured must match; different seeds should agree on
    # nothing. The floor on compared ticks keeps a near-empty overlap from
    # passing by default.
    same_ok = ab_n >= 30 and ab == ab_n
    diff_ok = ac_n >= 30 and ac <= 1

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
