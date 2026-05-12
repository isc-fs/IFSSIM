#!/usr/bin/env python3
"""
Drive the FSCar in steady-state circles at a sweep of (steer, speed) combos,
fit each resulting trajectory to a circle, and compute the lateral
acceleration the chassis is actually producing. The output tells us the
real Chaos vehicle's grip envelope, which the velocity controller's
`max_normal_acceleration` parameter is supposed to reflect.

Bypasses the autonomy pipeline entirely — talks directly to the FSDS RPC
server (port 41451) over TCP, so the sim must be in Play mode but the
ROS pipeline does NOT need to be enabled.

Output:
- Console table of (steer_cmd, target_v, observed_v, R_obs, R_kinematic, a_lat)
- /tmp/circle_test.png (or tools/dev/circle_test.png on host) with two plots:
    * R observed vs commanded steer for each target speed
    * a_lat vs target speed (the headline plot — where the chassis caps out)
"""
import argparse
import json
import math
import socket
import time
from dataclasses import dataclass

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


WHEELBASE_M = 1.60          # IFS-08 (settings.json)
MAX_STEER_DEG = 28.0        # IFS-08 MaxSteerAngle (settings.json)


class FSDSClient:
    """Newline-delimited TCP RPC to the FSDS plugin's command server."""

    def __init__(self, host: str = "localhost", port: int = 41451, timeout: float = 5.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((host, port))
        self.buf = b""

    def cmd(self, command: str) -> str:
        self.sock.sendall((command + "\n").encode("utf-8"))
        while b"\n" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise RuntimeError("Connection closed by sim")
            self.buf += chunk
        line, _, self.buf = self.buf.partition(b"\n")
        return line.decode("utf-8").strip()

    def car_state(self) -> dict:
        return json.loads(self.cmd("getCarState"))

    def set_controls(self, throttle: float, steering: float, brake: float = 0.0):
        # Sim convention: throttle/brake [0,1], steering [-1,+1] (positive = right).
        self.cmd(f"setCarControls {throttle:.4f} {steering:.4f} {brake:.4f}")


def fit_circle(xs: np.ndarray, ys: np.ndarray) -> tuple[float, float, float]:
    """Algebraic (Kasa) circle fit. Returns (cx, cy, R). Stable for samples
    that span well under a full circle as long as you have ~half a turn."""
    A = np.column_stack([2 * xs, 2 * ys, np.ones_like(xs)])
    b = xs * xs + ys * ys
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    R = math.sqrt(c + cx * cx + cy * cy)
    return float(cx), float(cy), float(R)


@dataclass
class Run:
    steer_cmd: float       # [0,1] in setCarControls units
    steer_deg: float       # mapped to wheel-angle degrees
    target_v: float        # m/s (target during ramp)
    observed_v: float      # m/s (mean during measurement window)
    R_obs: float           # m (fitted)
    R_kinematic: float     # m (bicycle model: L / tan(δ))
    a_lat: float           # m/s² = observed_v² / R_obs


def teleport_clear(client: FSDSClient, x: float = 50.0, y: float = 0.0):
    # Inside the track loop — empty space away from cones, but well within
    # the customMap's ground plane. Identity orientation = facing +X (east)
    # per UEQuatToENU convention. Earlier the script teleported to
    # (200, 0) which is past the map edge; the physics engine choked and
    # UE5 crashed.
    client.cmd(f"simSetVehiclePose {x:.2f} {y:.2f} 0.5")
    time.sleep(0.5)
    client.set_controls(0.0, 0.0, 1.0)  # full brake to kill residual velocity
    time.sleep(0.7)
    client.set_controls(0.0, 0.0, 0.0)


def ramp_to_speed(client: FSDSClient, target_v: float, max_seconds: float = 10.0):
    """Hold steering = 0 and apply throttle until reaching target_v.
    Gentle throttle so we don't overshoot at low target speeds."""
    if target_v <= 4.0:
        throttle = 0.20
    elif target_v <= 6.0:
        throttle = 0.30
    elif target_v <= 8.0:
        throttle = 0.40
    else:
        throttle = 0.55
    client.set_controls(throttle, 0.0, 0.0)
    t0 = time.time()
    while time.time() - t0 < max_seconds:
        s = client.car_state()
        if s["speed"] >= target_v:
            return s["speed"]
        time.sleep(0.05)
    return client.car_state()["speed"]


def steady_state_lap(
    client: FSDSClient,
    steer_cmd: float,
    target_v: float,
    log_seconds: float = 6.0,
) -> Run | None:
    """Apply steer + throttle, wait for transient, then log positions."""
    # Throttle policy: cruise control PID is too much for this script,
    # so just apply constant throttle that gets us close to target_v.
    # 0.30 ≈ 5 m/s, 0.40 ≈ 7 m/s, 0.50 ≈ 9 m/s on the IFS-08 motor curve.
    if target_v <= 4.0:
        throttle = 0.20
    elif target_v <= 6.0:
        throttle = 0.30
    elif target_v <= 8.0:
        throttle = 0.40
    else:
        throttle = 0.55

    client.set_controls(throttle, steer_cmd, 0.0)
    # Let the chassis settle into steady-state (transient ~2 s)
    time.sleep(2.5)

    # Log
    t_end = time.time() + log_seconds
    xs, ys, vs = [], [], []
    while time.time() < t_end:
        s = client.car_state()
        xs.append(s["x"])
        ys.append(s["y"])
        vs.append(s["speed"])
        time.sleep(0.04)  # ~25 Hz

    # Coast
    client.set_controls(0.0, 0.0, 0.5)

    if len(xs) < 30:
        return None

    xs = np.asarray(xs)
    ys = np.asarray(ys)
    vs = np.asarray(vs)

    cx, cy, R = fit_circle(xs, ys)
    observed_v = float(np.mean(vs))
    steer_deg = steer_cmd * MAX_STEER_DEG
    R_kin = WHEELBASE_M / math.tan(math.radians(max(0.5, abs(steer_deg))))
    a_lat = observed_v * observed_v / R

    return Run(
        steer_cmd=steer_cmd,
        steer_deg=steer_deg,
        target_v=target_v,
        observed_v=observed_v,
        R_obs=R,
        R_kinematic=R_kin,
        a_lat=a_lat,
    )


def run_grid(
    client_factory,
    steers: list[float],
    speeds: list[float],
    log_seconds: float,
) -> list[Run]:
    runs: list[Run] = []
    client = client_factory()
    client.cmd("enableApiControl")
    for steer in steers:
        for v in speeds:
            print(f"\n=== steer={steer:+.2f} ({steer*MAX_STEER_DEG:+.1f}°), target v={v:.1f} ===")
            try:
                teleport_clear(client)
                client.cmd("enableApiControl")  # idempotent
                achieved = ramp_to_speed(client, v)
                print(f"  ramp reached {achieved:.2f} m/s")
                run = steady_state_lap(client, steer, v, log_seconds)
            except (ConnectionResetError, ConnectionAbortedError, OSError, RuntimeError) as e:
                print(f"  connection error ({e!r}); reconnecting and skipping this combo")
                try:
                    client.sock.close()
                except Exception:
                    pass
                time.sleep(2.0)
                client = client_factory()
                client.cmd("enableApiControl")
                continue
            if run is None:
                print("  not enough samples, skipping")
                continue
            print(
                f"  v_obs={run.observed_v:5.2f}  R_obs={run.R_obs:6.2f} m  "
                f"R_kin={run.R_kinematic:6.2f} m  a_lat={run.a_lat:5.2f} m/s²  "
                f"slip={run.R_obs / run.R_kinematic:.2f}×"
            )
            runs.append(run)
    try:
        client.sock.close()
    except Exception:
        pass
    return runs


def render(runs: list[Run], out_path: str):
    if not runs:
        print("no runs to render")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    fig.patch.set_facecolor("#1a1a1a")
    for ax in (ax1, ax2):
        ax.set_facecolor("#0e0e0e")
        ax.tick_params(colors="#ccc")
        for s in ax.spines.values():
            s.set_color("#444")
        ax.grid(alpha=0.15, color="#666")

    steers = sorted({r.steer_cmd for r in runs})
    cmap = plt.get_cmap("viridis")
    colors = {s: cmap(i / max(1, len(steers) - 1)) for i, s in enumerate(steers)}

    # Left: a_lat vs observed v, one curve per steer
    for s in steers:
        sub = sorted([r for r in runs if r.steer_cmd == s], key=lambda r: r.observed_v)
        if not sub:
            continue
        vs = [r.observed_v for r in sub]
        a = [r.a_lat for r in sub]
        ax1.plot(vs, a, marker="o", color=colors[s],
                 label=f"steer {s*MAX_STEER_DEG:+.0f}°")
    # Reference μg lines
    for a_ref, lbl, ls in [(7.0, "max_normal_accel=7", "--"),
                          (10.0, "10 m/s² (~1g)", ":"),
                          (13.7, "μg = 13.7 m/s² (TireMu=1.4)", "-.")]:
        ax1.axhline(a_ref, color="#ff6a00", lw=1, ls=ls, alpha=0.6)
        ax1.text(ax1.get_xlim()[1] * 0.95 if ax1.get_xlim()[1] else 0,
                 a_ref + 0.2, lbl, color="#ff6a00", fontsize=8, ha="right")
    ax1.set_xlabel("observed speed (m/s)", color="#ccc")
    ax1.set_ylabel("lateral acceleration a_lat (m/s²)", color="#ccc")
    ax1.set_title("Chaos grip envelope: a_lat = v² / R_obs",
                  color="#ffb81c", pad=12)
    ax1.legend(facecolor="#222", edgecolor="#444", labelcolor="#ccc", fontsize=9)

    # Right: R_obs vs R_kinematic (bicycle model). Diagonal = neutral, above = understeer.
    for s in steers:
        sub = [r for r in runs if r.steer_cmd == s]
        if not sub:
            continue
        rk = [r.R_kinematic for r in sub]
        ro = [r.R_obs for r in sub]
        ax2.scatter(rk, ro, color=colors[s], s=60,
                    label=f"steer {s*MAX_STEER_DEG:+.0f}°")
    rk_max = max(r.R_kinematic for r in runs) * 1.1
    ax2.plot([0, rk_max], [0, rk_max], color="#888", ls="--", lw=1,
             label="neutral (R_obs = R_kin)")
    ax2.set_xlabel("R_kinematic = L / tan(δ)  (m)", color="#ccc")
    ax2.set_ylabel("R_observed (m)", color="#ccc")
    ax2.set_title("Observed vs bicycle-model radius (above line = understeer)",
                  color="#ffb81c", pad=12)
    ax2.legend(facecolor="#222", edgecolor="#444", labelcolor="#ccc", fontsize=9)

    fig.savefig(out_path, facecolor=fig.get_facecolor(), dpi=120,
                bbox_inches="tight")
    print(f"\nSaved {out_path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=41451)
    p.add_argument("--out", default="/tmp/circle_test.png")
    p.add_argument(
        "--steers",
        default="0.3,0.5,0.8",
        help="setCarControls steering values (positive=right). Comma-separated.",
    )
    p.add_argument(
        "--speeds",
        default="3,4,5,6,7,8",
        help="target speeds m/s. Comma-separated.",
    )
    p.add_argument("--log-seconds", type=float, default=6.0)
    args = p.parse_args()

    steers = [float(x) for x in args.steers.split(",")]
    speeds = [float(x) for x in args.speeds.split(",")]

    print(f"Connecting to FSDS RPC at {args.host}:{args.port}")

    def make_client():
        return FSDSClient(args.host, args.port)

    runs = run_grid(make_client, steers, speeds, args.log_seconds)

    print("\n\n=== Summary ===")
    print(f"{'steer°':>7}  {'v_tgt':>5}  {'v_obs':>5}  {'R_obs':>6}  {'R_kin':>6}  {'a_lat':>5}  {'slip':>5}")
    for r in runs:
        print(
            f"{r.steer_deg:+7.1f}  {r.target_v:5.1f}  {r.observed_v:5.2f}  "
            f"{r.R_obs:6.2f}  {r.R_kinematic:6.2f}  {r.a_lat:5.2f}  "
            f"{r.R_obs / r.R_kinematic:5.2f}"
        )

    render(runs, args.out)

    # Brake to a stop on a fresh client (run_grid closed its own).
    try:
        cleanup = make_client()
        cleanup.set_controls(0.0, 0.0, 1.0)
        time.sleep(1.5)
        cleanup.set_controls(0.0, 0.0, 0.0)
        cleanup.sock.close()
    except Exception as e:
        print(f"  (cleanup brake skipped: {e!r})")


if __name__ == "__main__":
    main()
