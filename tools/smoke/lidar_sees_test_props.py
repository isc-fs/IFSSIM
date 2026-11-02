#!/usr/bin/env python3
"""
The GPU LiDAR row of docs/collision_matrix.md: does /lidar_points return points
from both test props? Spawns them with validateCollisionMatrix (10 m ahead by
default, within the LiDAR's 25-30 m range), counts returns inside each prop's
box over a few scans, then clears them and counts again. Each prop must show
points with the props in place and none once they are gone.

Boxes are in the sensor frame (x forward, y left), offset from the car as the
RPC reports, and measured up from the ground right in front of each prop (the
road 0.3-0.8 m before it): a ground estimate taken further away is off by
several cm when the car sits slightly nose-down, enough to put road points
inside a 15 cm box. Only scans stamped (in sim time, from /clock)
after the spawn or the clear are counted: the LiDAR's capture, GPU readback and
transport add latency, and with the UE window in the background a scan can
arrive well after it was captured.

The car must stand still while the scans are compared (with the regen-only
service brake a stopped car can creep), so the check engages the EBS first,
waits for the car to stop, and leaves the EBS engaged.

Run inside dv_pipeline_stack (needs rclpy), with the sim running:
    docker cp tools/smoke/lidar_sees_test_props.py ifssim-dv_pipeline_stack-1:/tmp/
    docker exec ifssim-dv_pipeline_stack-1 bash -lc \\
      'source /opt/ros/humble/setup.bash && python3 /tmp/lidar_sees_test_props.py'
"""
import argparse
import json
import socket
import statistics as st
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
except ImportError:
    print("error: needs rclpy, run inside dv_pipeline_stack", file=sys.stderr)
    raise SystemExit(2)

MIN_POINTS = 50   # per scan, with the props in place


def rpc(host, port, cmd):
    s = socket.create_connection((host, port), timeout=30)
    s.sendall((cmd + "\n").encode())
    buf = b""
    while b"\n" not in buf:
        chunk = s.recv(65536)
        if not chunk:
            break
        buf += chunk
    s.close()
    return buf.decode().strip()


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def sim_now(node, clock):
    """The latest /clock, after letting a few more arrive."""
    t0 = time.time()
    seen = clock[0]
    while (clock[0] is None or clock[0] == seen) and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.05)
    return clock[0] or 0


def grab(node, scans, n, after_ns):
    """n scans captured after after_ns (sim time)."""
    scans.clear()
    t0 = time.time()
    while sum(stamp_ns(m.header.stamp) > after_ns for m in scans) < n and time.time() - t0 < 120:
        rclpy.spin_once(node, timeout_sec=0.05)
    return [[(float(p[0]), float(p[1]), float(p[2]))
             for p in point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)]
            for m in scans if stamp_ns(m.header.stamp) > after_ns][:n]


def count(points, props):
    """Points inside each prop's box, at least 5 cm above the local road."""
    out = {}
    for p in props:
        sx, sy, sz = p["size_m"]
        front = p["ahead_m"] - sx / 2
        in_y = [(x, z) for x, y, z in points if abs(y - p["left_m"]) <= sy / 2]
        road = [z for x, z in in_y if front - 0.8 <= x <= front - 0.3]
        ground = st.median(road) if road else float("nan")
        out[p["name"]] = sum(
            1 for x, y, z in points
            if abs(x - p["ahead_m"]) <= sx / 2 + 0.05 and abs(y - p["left_m"]) <= sy / 2 + 0.05
            and ground + 0.05 < z <= ground + sz + 0.05)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", default="host.docker.internal:41451")
    ap.add_argument("--ahead", type=float, default=10.0)
    ap.add_argument("--scans", type=int, default=6)
    args = ap.parse_args()
    host, port = args.sim.rsplit(":", 1)
    port = int(port)

    rclpy.init()
    node = Node("lidar_sees_test_props")
    scans, clock = [], [None]
    node.create_subscription(PointCloud2, "/lidar_points", scans.append, qos_profile_sensor_data)
    node.create_subscription(Clock, "/clock", lambda m: clock.__setitem__(0, stamp_ns(m.clock)),
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    time.sleep(2.0)

    rpc(host, port, "activateEbs")
    t0 = time.time()
    while time.time() - t0 < 20:
        k = json.loads(rpc(host, port, "simGetGroundTruthKinematics"))
        if (k["vx"] ** 2 + k["vy"] ** 2) ** 0.5 < 0.002:
            break
        time.sleep(0.2)
    else:
        print("FAIL: the car did not stop within 20 s with the EBS engaged")
        return 1

    # A scan stamped shortly after the RPC returned can still show the scene
    # from before the change: measured up to +0.12 s with the UE window in the
    # background, gone from about +0.2 s. 0.5 s of sim time is five scans of
    # margin.
    margin_ns = 500_000_000

    reply = json.loads(rpc(host, port, f"validateCollisionMatrix {args.ahead}"))
    if "props" not in reply:
        print(f"FAIL: {reply}")
        return 1
    props = reply["props"]
    after = sim_now(node, clock) + margin_ns
    with_props = [count(s, props) for s in grab(node, scans, args.scans, after)]
    rpc(host, port, "validateCollisionMatrix clear")
    after = sim_now(node, clock) + margin_ns
    without = [count(s, props) for s in grab(node, scans, args.scans, after)]
    node.destroy_node()
    rclpy.shutdown()

    ok = bool(with_props) and bool(without)
    for p in props:
        a = [c[p["name"]] for c in with_props]
        b = [c[p["name"]] for c in without]
        good = bool(a) and min(a) >= MIN_POINTS and bool(b) and max(b) == 0
        ok &= good
        print(f"  {'ok  ' if good else 'FAIL'} {p['name']:10s} points in its box per scan: "
              f"{st.median(a) if a else 0:.0f} with the props (min {min(a) if a else 0}), "
              f"{max(b) if b else 0} once cleared")
    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
