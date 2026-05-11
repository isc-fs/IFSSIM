"""
Bench EBS via the ROS/pipeline path and compare to direct-RPC brake.

Goal: confirm (or refute) the 2 m/s² decel we saw during fix/21 autonomous
runs. The bench_accel.py script showed that direct-RPC `setCarControls
0 0 1` gives 11 m/s² avg decel. The pipeline's EBS path (publish
`std_msgs/Empty` on `/signal/ebs`, bridge calls the same RPC and then
`disableApiControl`) should be equivalent. If it isn't, the gap is in the
bridge's subscriber latency or a timing race between the final
velocity-controller command and the bridge's full-brake override.

Procedure:
  1. LAUNCH: throttle=1.0 via direct RPC until v >= TARGET_V.
  2. EBS:    publish one Empty message to /signal/ebs through
             `docker exec ... ros2 topic pub --once`, then STOP sending
             any direct RPC controls. Poll getCarState for speed/pos.
  3. Report decel shape (avg, peak, stop distance).

Requires: UE5 in Play mode, dv_pipeline_stack container up with the bridge
subscribed to /signal/ebs (the default state of the container — pipeline
does not need to be running for this test).
"""
import csv
import math
import socket
import subprocess
import time
from pathlib import Path

HOST = "127.0.0.1"
PORT = 41451
SAMPLE_HZ = 50
TARGET_V = 17.0
T_MAX_LAUNCH = 8.0
T_MAX_EBS = 10.0
OUT_CSV = Path("tests/output/bench_ebs.csv")
ROS_CONTAINER = "ifssim-dv_pipeline_stack-1"


def _send(sock: socket.socket, cmd: str) -> str:
    sock.sendall((cmd + "\n").encode("utf-8"))
    buf = b""
    while b"\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf += chunk
    return buf.decode("utf-8", errors="replace").strip()


def _parse_state(resp: str) -> dict:
    import json
    try:
        return json.loads(resp)
    except json.JSONDecodeError:
        return {}


def _publish_ebs_via_ros() -> None:
    """Fire one Empty on /signal/ebs through the running dv_pipeline_stack."""
    cmd = [
        "docker", "exec", ROS_CONTAINER, "bash", "-lc",
        "source /opt/ros/humble/setup.bash && "
        "source /dv_pipeline_stack_ws/install/setup.bash && "
        "ros2 topic pub --once -t 1 /signal/ebs std_msgs/msg/Empty '{}'",
    ]
    # Fire-and-forget: we don't want to block the sampling loop.
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main() -> None:
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    sock = socket.create_connection((HOST, PORT), timeout=2.0)
    print(f"Connected to {HOST}:{PORT}")
    print(_send(sock, "enableApiControl"))

    dt = 1.0 / SAMPLE_HZ
    samples: list[tuple[float, str, float, float, float]] = []
    t0 = time.monotonic()
    x0 = None

    def sample(phase: str) -> dict:
        nonlocal x0
        st = _parse_state(_send(sock, "getCarState"))
        v = float(st.get("speed", 0.0))
        x = float(st.get("x", 0.0))
        y = float(st.get("y", 0.0))
        if x0 is None:
            x0 = (x, y)
        dx = math.hypot(x - x0[0], y - x0[1])
        t = time.monotonic() - t0
        samples.append((t, phase, v, x, dx))
        return st

    # LAUNCH
    print(f"\n=== LAUNCH to v >= {TARGET_V} m/s ===")
    _send(sock, f"setCarControls 1.0 0.0 0.0")
    t_start = time.monotonic()
    while time.monotonic() - t_start < T_MAX_LAUNCH:
        st = sample("LAUNCH")
        if float(st.get("speed", 0.0)) >= TARGET_V:
            break
        time.sleep(dt)

    # Fire EBS via ROS path
    t_fire_wall = time.monotonic()
    print(f"\n=== EBS via /signal/ebs @ t={t_fire_wall - t0:.3f}s ===")
    _publish_ebs_via_ros()

    # From this point on, do NOT send any more setCarControls.
    # Bridge will apply setCarControls 0 0 1 + disableApiControl.
    t_start = time.monotonic()
    while time.monotonic() - t_start < T_MAX_EBS:
        st = sample("EBS")
        if float(st.get("speed", 0.0)) <= 0.1 and time.monotonic() - t_start > 0.3:
            break
        time.sleep(dt)

    sock.close()

    with OUT_CSV.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "phase", "v_mps", "x_world", "x_traveled"])
        for row in samples:
            w.writerow([f"{row[0]:.3f}", row[1], f"{row[2]:.4f}",
                        f"{row[3]:.4f}", f"{row[4]:.4f}"])
    print(f"\nWrote {len(samples)} samples to {OUT_CSV}")

    def windowed(phase_name: str):
        rows = [r for r in samples if r[1] == phase_name]
        if len(rows) < 2:
            return None
        peak = 0.0
        for a, b in zip(rows, rows[1:]):
            ddt = b[0] - a[0]
            if ddt <= 0:
                continue
            ai = (b[2] - a[2]) / ddt
            if abs(ai) > abs(peak):
                peak = ai
        return {
            "v_start": rows[0][2], "v_end": rows[-1][2],
            "dt": rows[-1][0] - rows[0][0],
            "dx": rows[-1][4] - rows[0][4],
            "avg_a": ((rows[-1][2] - rows[0][2]) / (rows[-1][0] - rows[0][0])
                     if rows[-1][0] > rows[0][0] else 0.0),
            "peak_a": peak,
        }

    print("\n=== SUMMARY ===")
    for name in ("LAUNCH", "EBS"):
        s = windowed(name)
        if s is None:
            continue
        print(f"{name:6s}  v {s['v_start']:6.2f} -> {s['v_end']:6.2f} m/s"
              f"   dt={s['dt']:.2f}s   dx={s['dx']:.1f}m"
              f"   avg a={s['avg_a']:+6.2f} m/s²   peak a={s['peak_a']:+6.2f} m/s²")

    # Latency detail: how long after the EBS fire until speed actually
    # starts dropping significantly?
    ebs_rows = [r for r in samples if r[1] == "EBS"]
    if ebs_rows:
        v0 = ebs_rows[0][2]
        for r in ebs_rows:
            if r[2] < v0 - 0.5:
                print(f"\nv dropped by 0.5 m/s at t+{r[0] - ebs_rows[0][0]:.3f}s after EBS publish")
                break


if __name__ == "__main__":
    main()
