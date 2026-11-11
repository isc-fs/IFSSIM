"""Drive the car with scripted inputs and check it against settings.json.

    cd tools
    python3 -m characterise --sim 127.0.0.1:41451 --settings ../settings.json \\
        --out /tmp/characterisation [--sim-log path/to/sim.log]

Needs a running sim (Play mode or packaged) and nothing else: no pipeline,
no Mission Control. It drives over the plugin's RPC (FSDSRpcServer.cpp):

  loadTrack characterisation.csv   the acceleration cones on flat ground (no world),
                                   car at the start gate, heading reset
  simSetVehiclePose x y z      position-only teleport into open ground beside it
  activateEbs / releaseEbs / enableApiControl / setCarControls t s r
  simPause / simContinueForTime dt / simIsPaused / simResume   sim-time steps
  simGetGroundTruthKinematics + getSimStatus (sim_time)          the trace

Manoeuvres (about 90 s of sim time): steady turns at ~2 m/s at full lock
both ways and at steer 0.25/0.5/0.75, a launch from rest at full throttle
over 30 m, a coast-down from 10 m/s and an EBS stop from 7 m/s. The turns
run in a block 25 m left of the start, the straights in a lane 20 m right of
it, both clear of the cones (|y| <= 1.5 m).

Sim time: the sim is paused and stepped (simContinueForTime) so every
sample has an exact sim timestamp and the result doesn't depend on the
machine's speed. If stepping doesn't work it falls back to the wall clock
(samples still stamped with sim_time) and records the real-time factor.

Writes <out>/result.json (checks, status, summaries), <out>/summary.md (the
PR comment section, from analysis.markdown) and <out>/traces.json (every
sample). Exit code: 0 pass, 1 a hard check failed, 2 couldn't run or a hard
check couldn't be measured. Leaves the car stopped with EBS latched and the
sim running, the state it boots in.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    __package__ = "characterise"

from . import analysis  # noqa: E402

DEFAULT_PORT = 41451


class RpcError(RuntimeError):
    pass


class ManoeuvreError(RuntimeError):
    pass


# ------------------------------------------------------------------ RPC
class Rpc:
    """The plugin's line protocol on one kept-open connection."""

    def __init__(self, host: str, port: int = DEFAULT_PORT, timeout: float = 10.0):
        self.host, self.port, self.timeout = host, port, timeout
        self.sock: Optional[socket.socket] = None
        self.buf = b""

    def _connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.buf = b""

    def cmd(self, line: str) -> str:
        for attempt in (0, 1):
            try:
                if self.sock is None:
                    self._connect()
                self.sock.sendall((line + "\n").encode())
                while b"\n" not in self.buf:
                    chunk = self.sock.recv(65536)
                    if not chunk:
                        raise ConnectionError("closed by the sim")
                    self.buf += chunk
                out, _, self.buf = self.buf.partition(b"\n")
                return out.decode("utf-8", "replace").strip()
            except OSError as e:
                self.close()
                if attempt:
                    raise RpcError(f"{line.split()[0]}: {e}") from e
        raise AssertionError("unreachable")

    def json(self, line: str) -> dict:
        r = self.cmd(line)
        try:
            d = json.loads(r)
        except ValueError as e:
            raise RpcError(f"{line.split()[0]}: not JSON: {r[:120]!r}") from e
        if isinstance(d, dict) and d.get("error"):
            raise RpcError(f"{line.split()[0]}: {d['error']}")
        return d

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None


# ------------------------------------------------------------------ clocks
class StepClock:
    """Sim-time steps: pause, then simContinueForTime dt and wait until paused again."""

    mode = "step"

    def __init__(self, rpc: Rpc, timeout_s: float = 5.0):
        self.rpc, self.timeout_s = rpc, timeout_s
        rpc.cmd("simPause")

    def advance(self, dt: float):
        self.rpc.cmd(f"simContinueForTime {dt:.4f}")
        t0 = time.monotonic()
        while self.rpc.cmd("simIsPaused") != "true":
            if time.monotonic() - t0 > self.timeout_s:
                raise RpcError(f"a {dt:.3f} s step didn't end within {self.timeout_s:.0f} s")
            time.sleep(0.002)

    def close(self):
        self.rpc.cmd("simResume")


class WallClock:
    mode = "wall"

    def __init__(self, rpc: Rpc):
        rpc.cmd("simResume")

    def advance(self, dt: float):
        time.sleep(dt)

    def close(self):
        pass


def sim_time(rpc: Rpc) -> float:
    return float(rpc.json("getSimStatus")["sim_time"])


def pick_clock(rpc: Rpc, mode: str, log) -> object:
    """``step`` if a probe step advances sim time and re-pauses; else the wall clock."""
    if mode == "wall":
        return WallClock(rpc)
    try:
        clock = StepClock(rpc)
        time.sleep(0.2)  # the pause lands on the game thread's next frame
        t0 = sim_time(rpc)
        clock.advance(0.1)
        t1 = sim_time(rpc)
        time.sleep(0.1)
        if sim_time(rpc) != t1:
            raise RpcError("sim time moves while paused")
        if not 0.05 <= t1 - t0 <= 1.0:
            raise RpcError(f"a 0.1 s step advanced sim time by {t1 - t0:.3f} s")
        return clock
    except (RpcError, ValueError, KeyError) as e:
        if mode == "step":
            raise
        log(f"sim-time stepping unavailable ({e}); using the wall clock")
        rpc.cmd("simResume")
        return WallClock(rpc)


# ------------------------------------------------------------------ the car
class Car:
    def __init__(self, rpc: Rpc, clock, dt: float = 0.05):
        self.rpc, self.clock, self.dt = rpc, clock, dt
        self.z_floor: Optional[float] = None
        self.sim_s = 0.0
        self.wall_s = 0.0
        self.samples_taken = 0
        self.last_t: Optional[float] = None

    def sample(self) -> dict:
        k = self.rpc.json("simGetGroundTruthKinematics")
        t = sim_time(self.rpc)
        s = {"t": t, "x": float(k["px"]), "y": float(k["py"]), "z": float(k["pz"]),
             "vx": float(k["vx"]), "vy": float(k["vy"]), "wz": float(k["wz"]),
             "yaw": analysis.yaw_from_quat(float(k["qw"]), float(k["qx"]), float(k["qy"]), float(k["qz"]))}
        if self.z_floor is not None and s["z"] < self.z_floor:
            raise ManoeuvreError(f"the car fell (z {s['z']:.2f} m): off the edge of the ground?")
        self.samples_taken += 1
        self.last_t = s["t"]
        return s

    def controls(self, throttle: float, steer: float, regen: float = 0.0):
        self.rpc.cmd(f"setCarControls {throttle:.4f} {steer:.4f} {regen:.4f}")

    def step(self, dt: Optional[float] = None) -> dict:
        w0 = time.monotonic()
        t0 = self.last_t if self.last_t is not None else sim_time(self.rpc)
        self.clock.advance(dt or self.dt)
        s = self.sample()
        self.sim_s += s["t"] - t0
        self.wall_s += time.monotonic() - w0
        return s

    def wait(self, seconds: float) -> dict:
        s = self.sample()
        t_end = s["t"] + seconds
        while s["t"] < t_end:
            s = self.step()
        return s


class SpeedPI:
    """Throttle (and a little regen) to hold a speed. Fixed gains, anti-windup."""

    def __init__(self, target: float, throttle_max: float = 0.35, kp: float = 0.08, ki: float = 0.05):
        self.target, self.tmax, self.kp, self.ki = target, throttle_max, kp, ki
        self.i = 0.0

    def __call__(self, v: float, dt: float) -> Tuple[float, float]:
        e = self.target - v
        u = self.kp * e + self.ki * self.i
        if not ((u >= self.tmax and e > 0) or (u <= 0 and e < 0)):  # no wind-up into a limit
            self.i += e * dt
        throttle = min(max(u, 0.0), self.tmax)
        regen = min(max(-self.kp * e, 0.0), 0.3) if e < -0.3 else 0.0
        return throttle, regen


# ------------------------------------------------------------------ manoeuvres
@dataclass
class Layout:
    # Flat ground: acceleration.csv loads the Montmeló world, where the turns
    # block would sit in the grandstand.
    track: str = "characterisation.csv"
    turn_origin: Tuple[float, float] = (-10.0, 25.0)   # start frame: x forward, y left (m)
    lane_origin: Tuple[float, float] = (-40.0, -20.0)
    lane_length_m: float = 85.0
    turn_speed: float = 2.0
    lock_measure_s: float = 10.0
    sweep_measure_s: float = 5.0
    settle_s: float = 3.0
    sweep: Tuple[float, ...] = (-0.25, -0.5, -0.75)
    launch_throttle: float = 1.0
    launch_distance_m: float = 30.0
    coast_from: float = 10.0
    coast_s: float = 5.0
    ebs_from: float = 7.0


class Session:
    def __init__(self, car: Car, layout: Layout, log=print):
        self.car, self.lay, self.log = car, layout, log
        self.p0: Optional[Tuple[float, float]] = None
        self.yaw0 = 0.0
        self.z0 = 0.0

    def _load(self):
        r = self.car.rpc.json(f"loadTrack {self.lay.track}")
        if not r.get("car_aligned", False):
            raise ManoeuvreError(f"loadTrack {self.lay.track} didn't place the car: {r}")

    def frame(self):
        """Load the track and take the start pose as the frame everything else is placed in."""
        self.car.rpc.cmd("activateEbs")
        self._load()
        s = self.car.wait(1.0)
        self.p0, self.yaw0, self.z0 = (s["x"], s["y"]), s["yaw"], s["z"]
        self.car.z_floor = self.z0 - 1.0
        self.log(f"start pose ({s['x']:.2f}, {s['y']:.2f}, {s['z']:.2f}) heading {math.degrees(s['yaw']):.1f} deg")

    def world(self, local: Tuple[float, float]) -> Tuple[float, float]:
        c, sn = math.cos(self.yaw0), math.sin(self.yaw0)
        return (self.p0[0] + c * local[0] - sn * local[1], self.p0[1] + sn * local[0] + c * local[1])

    def place(self, local: Tuple[float, float]) -> dict:
        """At rest at ``local`` (start frame), heading as at the start gate, ready to drive."""
        rpc = self.car.rpc
        rpc.cmd("activateEbs")
        self._load()                       # resets the heading to the start gate's
        self.car.wait(0.3)
        x, y = self.world(local)
        rpc.cmd(f"simSetVehiclePose {x:.3f} {y:.3f} {self.z0 + 0.05:.3f}")  # position only: keeps heading
        s = self.car.wait(1.0)
        if math.hypot(s["x"] - x, s["y"] - y) > 1.0:
            raise ManoeuvreError(f"teleport to ({x:.1f}, {y:.1f}) landed at ({s['x']:.1f}, {s['y']:.1f})")
        rpc.cmd("releaseEbs")
        rpc.cmd("enableApiControl")
        self.car.controls(0, 0, 0)
        return self.car.sample()

    def stop(self, t_max: float = 6.0) -> List[dict]:
        """EBS until standstill; the trace starts at the trigger."""
        self.car.controls(0, 0, 0)
        self.car.rpc.cmd("activateEbs")
        out = [self.car.sample()]
        still = 0.0
        while out[-1]["t"] - out[0]["t"] < t_max:
            s = self.car.step()
            out.append(s)
            still = still + self.car.dt if analysis.speed(s) < 0.05 else 0.0
            if still >= 0.5:
                break
        return out

    def turn(self, steer: float, measure_s: float) -> List[dict]:
        s = self.place(self.lay.turn_origin)
        pi = SpeedPI(self.lay.turn_speed)
        t_settle = s["t"] + self.lay.settle_s
        t_end = t_settle + measure_s
        trace: List[dict] = []
        while s["t"] < t_end:
            th, rg = pi(analysis.speed(s), self.car.dt)
            self.car.controls(th, steer, rg)
            s = self.car.step()
            if s["t"] >= t_settle:
                trace.append(s)
        self.stop()
        return trace

    def _straight(self, s0: dict, s: dict) -> bool:
        return math.hypot(s["x"] - s0["x"], s["y"] - s0["y"]) < self.lay.lane_length_m

    def launch(self) -> dict:
        s0 = s = self.place(self.lay.lane_origin)
        trace = [s]
        self.car.controls(self.lay.launch_throttle, 0, 0)
        t_max = s["t"] + 15.0
        while s["t"] < t_max and self._straight(s0, s):
            s = self.car.step()
            trace.append(s)
            if analysis.path_length([trace[0], s])[-1] >= self.lay.launch_distance_m + 1.0:
                break
        self.stop()
        return {"throttle": self.lay.launch_throttle, "distance_m": self.lay.launch_distance_m,
                "samples": trace}

    def _ramp(self, s0: dict, v: float, t_max: float = 10.0) -> dict:
        s = s0
        t_end = s["t"] + t_max
        while analysis.speed(s) < v:
            if s["t"] > t_end or not self._straight(s0, s):
                raise ManoeuvreError(f"didn't reach {v:g} m/s (got {analysis.speed(s):.1f})")
            self.car.controls(1.0, 0, 0)
            s = self.car.step()
        return s

    def coast(self) -> dict:
        s0 = self.place(self.lay.lane_origin)
        s = self._ramp(s0, self.lay.coast_from)
        self.car.controls(0, 0, 0)
        trace = [s]
        t_end = s["t"] + self.lay.coast_s
        while s["t"] < t_end and self._straight(s0, s):
            s = self.car.step()
            trace.append(s)
        self.stop()
        return {"samples": trace, "truncated": trace[-1]["t"] < t_end}

    def ebs(self) -> dict:
        s0 = self.place(self.lay.lane_origin)
        s = self._ramp(s0, self.lay.ebs_from)
        pi = SpeedPI(self.lay.ebs_from, throttle_max=1.0)
        t_end = s["t"] + 1.0
        while s["t"] < t_end:
            th, rg = pi(analysis.speed(s), self.car.dt)
            self.car.controls(th, 0, rg)
            s = self.car.step()
        return {"samples": self.stop(t_max=8.0)}


def drive(session: Session) -> dict:
    """Every manoeuvre; returns analysis.evaluate's ``meas`` (minus the log)."""
    lay, log = session.lay, session.log
    session.frame()
    meas: Dict[str, object] = {"turns": []}
    for steer, secs in [(-1.0, lay.lock_measure_s), (1.0, lay.lock_measure_s)] + \
                       [(s, lay.sweep_measure_s) for s in lay.sweep]:
        log(f"turn, steer {steer:+.2f}")
        meas["turns"].append({"steer": steer, "samples": session.turn(steer, secs)})
    log("launch")
    meas["launch"] = session.launch()
    log("coast-down")
    meas["coast"] = session.coast()
    log("EBS stop")
    meas["ebs"] = session.ebs()
    return meas


# ------------------------------------------------------------------ main
def characterise(rpc: Rpc, settings: dict, layout: Layout, clock_mode: str = "auto",
                 sim_log: Optional[Path] = None, dt: float = 0.05, log=print) -> Tuple[dict, dict]:
    """Run everything. Returns (result, measurements); never raises for a failed run."""
    result: Dict[str, object] = {"version": 1, "status": "error", "checks": []}
    meas: Dict[str, object] = {}
    try:
        exp = analysis.expectations(settings)
    except (KeyError, ValueError, TypeError) as e:
        result["error"] = f"settings.json: {e}"
        return result, meas
    result["expectations"] = exp
    clock = None
    car = None
    w0 = time.monotonic()
    try:
        rpc.cmd("ping")
        clock = pick_clock(rpc, clock_mode, log)
        car = Car(rpc, clock, dt)
        meas = drive(Session(car, layout, log))
    except Exception as e:  # noqa: BLE001 - a bug here must still leave a result and a tidy sim
        result["error"] = f"{type(e).__name__}: {e}"
    finally:
        try:
            rpc.cmd("setCarControls 0 0 0")
            rpc.cmd("activateEbs")
            if clock is not None:
                clock.close()
            else:
                rpc.cmd("simResume")
        except RpcError as e:
            log(f"couldn't leave the sim tidy: {e}")
    if car is not None:
        result["clock"] = {"mode": clock.mode, "sim_s": round(car.sim_s, 2),
                           "wall_s": round(time.monotonic() - w0, 1),
                           "rtf": round(car.sim_s / car.wall_s, 3) if clock.mode == "wall" and car.wall_s else None,
                           "dt": dt, "samples": car.samples_taken}
    if sim_log and sim_log.exists():
        meas["wheelbase_log"] = analysis.parse_wheelbase_log(sim_log.read_text(errors="replace"))
    if result.get("error"):
        return result, meas
    ev = analysis.evaluate(exp, meas)
    result.update(ev)
    return result, meas


def parse_host(s: str) -> Tuple[str, int]:
    host, _, port = s.rpartition(":")
    return (host, int(port)) if host else (s, DEFAULT_PORT)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m characterise", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", default=f"127.0.0.1:{DEFAULT_PORT}", help="host:port of the sim's RPC")
    ap.add_argument("--settings", type=Path, required=True, help="the settings.json the sim runs with")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--sim-log", type=Path, help="the sim's log, for its wheel-placement line")
    ap.add_argument("--clock", choices=("auto", "step", "wall"), default="auto")
    ap.add_argument("--dt", type=float, default=0.05, help="control and sample period, sim s")
    ap.add_argument("--track", default=Layout.track)
    a = ap.parse_args(argv)

    log = lambda m: print(f"[characterise] {m}", flush=True)  # noqa: E731

    def _terminated(*_):
        raise KeyboardInterrupt  # so characterise()'s finally un-pauses the sim and latches EBS
    signal.signal(signal.SIGTERM, _terminated)
    try:
        settings = json.loads(a.settings.read_text())
    except (OSError, ValueError) as e:
        settings = {}
        log(f"can't read {a.settings}: {e}")
    host, port = parse_host(a.sim)
    rpc = Rpc(host, port)
    try:
        result, meas = characterise(rpc, settings, Layout(track=a.track), a.clock, a.sim_log, a.dt, log)
    finally:
        rpc.close()
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (a.out / "summary.md").write_text(analysis.markdown(result))
    (a.out / "traces.json").write_text(json.dumps(meas) + "\n")
    print(analysis.markdown(result))
    return {"pass": 0, "fail": 1}.get(result["status"], 2)


if __name__ == "__main__":
    sys.exit(main())
