"""
Bench the sim's true longitudinal accel/decel capability at fixed inputs.

Bypasses the entire pipeline — talks directly to the UE5 RPC (TCP 41451)
so we measure what the Chaos vehicle actually delivers under
`setCarControls throttle steering brake` without the velocity
controller's `throttle_max=0.4` clip or any EBS/latch logic getting in
the way.

Three phases:
  1. LAUNCH       — throttle=1.0, brake=0 until v >= TARGET_V or T_MAX_LAUNCH
  2. COAST        — throttle=0, brake=0 for T_COAST  (measures drag alone)
  3. FULL BRAKE   — throttle=0, brake=1 until stopped
                    This is the "what does the sim call brake" measurement —
                    once regen is modeled, re-bench this and compare.

Run on the host with UE5 in Play mode, pipeline stopped, car at spawn:

    python tools/diagnostics/bench_accel.py

Emits a CSV (timestamp, phase, v_mps, x_m) and a terminal summary with
peak accel, terminal speed, coast decel (drag), and brake decel.
"""
import csv
import math
import socket
import time
from pathlib import Path

HOST = "127.0.0.1"
PORT = 41451
SAMPLE_HZ = 50
TARGET_V = 20.0            # m/s — stop launch phase once we exceed this
T_MAX_LAUNCH = 8.0         # s   — safety cap on launch phase
T_COAST = 1.0              # s   — coast duration
T_MAX_BRAKE = 10.0         # s   — safety cap on brake phase
OUT_CSV = Path("tests/output/bench_accel.csv")


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
    """getCarState returns a flat JSON-ish string; we just need speed + pos."""
    import json
    try:
        return json.loads(resp)
    except json.JSONDecodeError:
        return {}


def _set_controls(sock, throttle, steering, brake):
    _send(sock, f"setCarControls {throttle:.4f} {steering:.4f} {brake:.4f}")


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
        state = _parse_state(_send(sock, "getCarState"))
        v = float(state.get("speed", 0.0))
        x = float(state.get("x", 0.0))
        y = float(state.get("y", 0.0))
        if x0 is None:
            x0 = (x, y)
        dx = math.hypot(x - x0[0], y - x0[1])
        t = time.monotonic() - t0
        samples.append((t, phase, v, x, dx))
        return state

    def phase(name: str, throttle: float, brake: float, t_max: float,
              stop_v_geq: float | None = None, stop_v_leq: float | None = None):
        print(f"\n=== {name}  throttle={throttle} brake={brake}  max {t_max}s ===")
        _set_controls(sock, throttle, 0.0, brake)
        t_start = time.monotonic()
        while time.monotonic() - t_start < t_max:
            st = sample(name)
            v = float(st.get("speed", 0.0))
            if stop_v_geq is not None and v >= stop_v_geq:
                print(f"  -> reached v={v:.2f} m/s, advancing")
                break
            if stop_v_leq is not None and v <= stop_v_leq and time.monotonic() - t_start > 0.5:
                print(f"  -> v<={stop_v_leq}, stopping")
                break
            time.sleep(dt)

    try:
        phase("LAUNCH", 1.0, 0.0, T_MAX_LAUNCH, stop_v_geq=TARGET_V)
        phase("COAST",  0.0, 0.0, T_COAST)
        phase("BRAKE",  0.0, 1.0, T_MAX_BRAKE, stop_v_leq=0.1)
    finally:
        _set_controls(sock, 0.0, 0.0, 1.0)
        sock.close()

    with OUT_CSV.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "phase", "v_mps", "x_world", "x_traveled"])
        for row in samples:
            w.writerow([f"{row[0]:.3f}", row[1], f"{row[2]:.4f}", f"{row[3]:.4f}", f"{row[4]:.4f}"])
    print(f"\nWrote {len(samples)} samples to {OUT_CSV}")

    def stats_for(phase_name: str):
        rows = [r for r in samples if r[1] == phase_name]
        if len(rows) < 2:
            return None
        t0r, _, v0, _, x0r = rows[0]
        t1r, _, v1, _, x1r = rows[-1]
        dv = v1 - v0
        dt_r = t1r - t0r
        dx = x1r - x0r
        avg_a = dv / dt_r if dt_r > 0 else 0.0
        peak_a = 0.0
        for a, b in zip(rows, rows[1:]):
            ddt = b[0] - a[0]
            if ddt <= 0:
                continue
            ai = (b[2] - a[2]) / ddt
            if abs(ai) > abs(peak_a):
                peak_a = ai
        return {"v_start": v0, "v_end": v1, "dt": dt_r, "dx": dx, "avg_a": avg_a, "peak_a": peak_a}

    print("\n=== SUMMARY ===")
    for name in ("LAUNCH", "COAST", "BRAKE"):
        s = stats_for(name)
        if s is None:
            continue
        print(f"{name:6s}  v {s['v_start']:6.2f} -> {s['v_end']:6.2f} m/s"
              f"   dt={s['dt']:.2f}s   dx={s['dx']:.1f}m"
              f"   avg a={s['avg_a']:+6.2f} m/s²   peak a={s['peak_a']:+6.2f} m/s²")


if __name__ == "__main__":
    main()
