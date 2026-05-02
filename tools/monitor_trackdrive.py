#!/usr/bin/env python3
"""
Monitor a trackdrive run end-to-end:
  - Reset the car back to the start gate (so the run starts clean)
  - event_start trackdrive
  - Sample vehicle telemetry at 5 Hz for `duration_s`
  - Snapshot the referee state at the end (laps, doo, oc)
  - Tail the controller's DIAG lines from the bridge container so we can see
    what xte / yaw_err / Stanley / target velocity were doing each tick

Usage: python3 tools/monitor_trackdrive.py [duration_s]
"""
import json
import math
import subprocess
import sys
import time
import urllib.request

API = "http://localhost:8000"
CONTAINER = "ifssim-dv_pipeline_stack-1"
ROS_SETUP = "source /opt/ros/humble/setup.bash && source /dv_pipeline_stack_ws/install/setup.bash"


def get(path):
    with urllib.request.urlopen(f"{API}{path}", timeout=5) as r:
        return json.loads(r.read())


def post(path, body=None):
    data = json.dumps(body).encode() if body else b""
    req = urllib.request.Request(
        f"{API}{path}", data=data,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


def quat_to_yaw_deg(qw, qx, qy, qz):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def main():
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 45

    # --- Clean slate: reload track + reset to gate ---
    print("=== Reload track ===")
    r = post("/api/track/ci_test.csv/load")
    print(f"  cones={r['result']['cones']} car_aligned={r['result']['car_aligned']}")
    time.sleep(0.5)

    s = get("/api/vehicle/state")
    yaw = quat_to_yaw_deg(s["qw"], s["qx"], s["qy"], s["qz"])
    print(f"  start: pos=({s['x']:+.2f},{s['y']:+.2f}) yaw={yaw:+.1f}° v={s['speed']:.2f}")

    # --- Start the trackdrive event ---
    print("\n=== event_start trackdrive ===")
    r = post("/api/event/start", {"event_type": "trackdrive", "num_laps": 1})
    print(f"  {r}")

    # --- Monitor telemetry for `duration` seconds ---
    print(f"\n=== Monitoring {duration} s ({200} ms cadence) ===")
    print(f"{'t':>5} {'x':>8} {'y':>8} {'v':>5} {'yaw':>6} {'thr':>5} {'brk':>5} {'str':>6} hb")
    samples = []
    t0 = time.time()
    last_print = -1.0
    while time.time() - t0 < duration:
        try:
            s = get("/api/vehicle/state")
        except Exception:
            time.sleep(0.2)
            continue
        t = time.time() - t0
        c = s["controls"]
        yaw = quat_to_yaw_deg(s["qw"], s["qx"], s["qy"], s["qz"])
        samples.append((t, s["x"], s["y"], s["speed"], yaw,
                        c["throttle"], c["brake"], c["steering"], c["handbrake"]))
        # Print every 1 s to keep output readable
        if t - last_print >= 1.0:
            last_print = t
            print(f"{t:>5.1f} {s['x']:+8.2f} {s['y']:+8.2f} {s['speed']:>5.2f} "
                  f"{yaw:+6.1f} {c['throttle']:>5.2f} {c['brake']:>5.2f} "
                  f"{c['steering']:+6.2f} {c['handbrake']}")
        time.sleep(0.2)

    # --- Final state + referee ---
    print("\n=== Final ===")
    s = get("/api/vehicle/state")
    yaw = quat_to_yaw_deg(s["qw"], s["qx"], s["qy"], s["qz"])
    print(f"  pos=({s['x']:+.2f},{s['y']:+.2f}) yaw={yaw:+.1f}° v={s['speed']:.2f}")

    ref = get("/api/event/state")
    print(f"  laps={ref.get('laps')}/{ref.get('required_laps')}  finished={ref.get('finished')}")
    print(f"  doo={ref.get('doo_counter')} oc={ref.get('oc_counter')} cones={ref.get('cones')}")

    # --- Trajectory summary ---
    if samples:
        max_v = max(s[3] for s in samples)
        moving = [s for s in samples if s[3] > 0.1]
        if moving:
            x0, y0 = samples[0][1], samples[0][2]
            x_end, y_end = samples[-1][1], samples[-1][2]
            total_displacement = math.hypot(x_end - x0, y_end - y0)
            # Path length (sum of segments)
            path_len = sum(
                math.hypot(samples[i][1] - samples[i - 1][1],
                           samples[i][2] - samples[i - 1][2])
                for i in range(1, len(samples)))
            print(f"  max_speed={max_v:.2f} m/s  path_length={path_len:.1f} m  "
                  f"end_displacement={total_displacement:.1f} m")


if __name__ == "__main__":
    main()
