"""
Bench the new `activateEbs` RPC directly, bypassing the ROS topic path.

Isolates the plugin-side handbrake latch from the DDS discovery overhead
that made bench_ebs.py noisy. Same LAUNCH phase, but triggers EBS by
firing the `activateEbs` command over the same TCP RPC socket — so we're
measuring only the sim's response to the handbrake latch.

After fix/22 this should match bench_accel's BRAKE phase shape
(~11 m/s² avg, ~12 m stopping distance from 17 m/s).
"""
import csv
import math
import socket
import time
from pathlib import Path

HOST = "127.0.0.1"
PORT = 41451
SAMPLE_HZ = 50
TARGET_V = 17.0
T_MAX_LAUNCH = 8.0
T_MAX_EBS = 10.0
OUT_CSV = Path("tests/output/bench_ebs_rpc.csv")


def _send(sock, cmd):
    sock.sendall((cmd + "\n").encode("utf-8"))
    buf = b""
    while b"\n" not in buf:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf += chunk
    return buf.decode("utf-8", errors="replace").strip()


def _parse(resp):
    import json
    try:
        return json.loads(resp)
    except json.JSONDecodeError:
        return {}


def main():
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    sock = socket.create_connection((HOST, PORT), timeout=2.0)
    print(f"Connected to {HOST}:{PORT}")
    print(_send(sock, "enableApiControl"))

    dt = 1.0 / SAMPLE_HZ
    samples = []
    t0 = time.monotonic()
    x0 = None

    def sample(phase):
        nonlocal x0
        st = _parse(_send(sock, "getCarState"))
        v = float(st.get("speed", 0.0))
        x = float(st.get("x", 0.0))
        y = float(st.get("y", 0.0))
        if x0 is None:
            x0 = (x, y)
        dx = math.hypot(x - x0[0], y - x0[1])
        samples.append((time.monotonic() - t0, phase, v, x, dx))
        return st

    print(f"\n=== LAUNCH to v >= {TARGET_V} ===")
    _send(sock, "setCarControls 1.0 0.0 0.0")
    t_start = time.monotonic()
    while time.monotonic() - t_start < T_MAX_LAUNCH:
        st = sample("LAUNCH")
        if float(st.get("speed", 0.0)) >= TARGET_V:
            break
        time.sleep(dt)

    t_fire = time.monotonic() - t0
    print(f"\n=== EBS via activateEbs RPC @ t={t_fire:.3f}s ===")
    print(_send(sock, "activateEbs"))

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

    def stats(phase_name):
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
        s = stats(name)
        if s is None:
            continue
        print(f"{name:6s}  v {s['v_start']:6.2f} -> {s['v_end']:6.2f} m/s"
              f"   dt={s['dt']:.2f}s   dx={s['dx']:.1f}m"
              f"   avg a={s['avg_a']:+6.2f} m/s²   peak a={s['peak_a']:+6.2f} m/s²")

    ebs_rows = [r for r in samples if r[1] == "EBS"]
    if ebs_rows:
        v0 = ebs_rows[0][2]
        for r in ebs_rows:
            if r[2] < v0 - 0.5:
                print(f"\nv dropped 0.5 m/s at t+{r[0] - ebs_rows[0][0]:.3f}s after EBS RPC")
                break


if __name__ == "__main__":
    main()
