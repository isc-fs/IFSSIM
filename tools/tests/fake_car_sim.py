"""A stand-in for the sim's RPC with a kinematic-bicycle car, for the car
characterisation's tests (test_characterise.py).

It answers the methods tools/characterise drives, with the plugin's reply
shapes (FSDSRpcServer.cpp). Time only moves on simContinueForTime, which
integrates synchronously and leaves the sim paused: a stepped sim. The car can
be built wrong on purpose (wheelbase, lock, a squared steering input), which is
what the tests use to prove the checks catch those bugs.
"""
from __future__ import annotations

import json
import math
import random
import socket
import socketserver
import threading
from dataclasses import dataclass


@dataclass
class Plant:
    wheelbase: float = 1.627
    front_weight: float = 0.438
    max_steer_deg: float = 18.2
    squared_steer: bool = False
    accel_per_throttle: float = 5.0    # m/s^2 at throttle 1
    a0: float = 0.196                  # coast: a0 + b*v^2
    b: float = 0.0021
    ebs_decel: float = 6.0
    pos_noise: float = 0.01            # m
    start_heading_deg: float = 30.0    # not along an axis, so frame handling is tested
    start_xy: tuple = (12.0, -7.0)
    z: float = 0.30
    step_works: bool = True
    seed: int = 1


class FakeSim:
    def __init__(self, plant: Plant | None = None):
        self.p = plant or Plant()
        self.rng = random.Random(self.p.seed)
        self.lock = threading.Lock()
        self.t = 1.0
        self.paused = False
        self.ebs = True
        self.api = False
        self.ctl = (0.0, 0.0, 0.0)
        self.calls: list[str] = []
        self._place_cog(*self.p.start_xy, math.radians(self.p.start_heading_deg))

    # -- state
    @property
    def _b(self) -> float:
        return self.p.wheelbase * self.p.front_weight

    def _place_cog(self, x: float, y: float, psi: float):
        self.psi = psi
        self.xr = x - self._b * math.cos(psi)
        self.yr = y - self._b * math.sin(psi)
        self.u = 0.0

    def _delta(self) -> float:
        s = max(-1.0, min(1.0, self.ctl[1]))
        s = math.copysign(s * s, s) if self.p.squared_steer else s
        return s * math.radians(self.p.max_steer_deg)

    def _yaw_rate(self) -> float:
        return -self.u * math.tan(self._delta()) / self.p.wheelbase   # + steer = right

    def _integrate(self, dt: float):
        n = max(1, int(round(dt / 0.005)))
        h = dt / n
        for _ in range(n):
            th, _, rg = self.ctl
            a = self.p.accel_per_throttle * th - 4.0 * rg
            if self.u > 0:
                a -= self.p.a0 + self.p.b * self.u * self.u + (self.p.ebs_decel if self.ebs else 0.0)
            self.u = max(0.0, self.u + a * h)
            r = self._yaw_rate()
            self.xr += self.u * math.cos(self.psi) * h
            self.yr += self.u * math.sin(self.psi) * h
            self.psi += r * h
        self.t += dt

    def _kinematics(self) -> dict:
        b, c, s = self._b, math.cos(self.psi), math.sin(self.psi)
        r = self._yaw_rate()
        n = self.p.pos_noise
        return {
            "px": self.xr + b * c + self.rng.gauss(0, n), "py": self.yr + b * s + self.rng.gauss(0, n),
            "pz": self.p.z, "vx": self.u * c - r * b * s, "vy": self.u * s + r * b * c, "vz": 0.0,
            "ax": 0.0, "ay": 0.0, "az": 0.0, "wx": 0.0, "wy": 0.0, "wz": r,
            "qw": math.cos(self.psi / 2), "qx": 0.0, "qy": 0.0, "qz": math.sin(self.psi / 2),
        }

    # -- protocol
    def handle(self, line: str) -> str:
        parts = line.split()
        m = parts[0] if parts else ""
        with self.lock:
            self.calls.append(m)
            if m == "ping":
                return "true"
            if m == "getSimStatus":
                return json.dumps({"map": "fake", "fps": 60.0, "paused": self.paused,
                                   "api_control": self.api, "sim_time": round(self.t, 6)})
            if m == "simPause":
                self.paused = True
                return "true"
            if m == "simResume":
                self.paused = False
                return "true"
            if m == "simIsPaused":
                return "true" if self.paused else "false"
            if m == "simContinueForTime":
                if self.p.step_works:
                    self._integrate(float(parts[1]))
                    self.paused = True
                else:
                    self.paused = False   # never comes back: a broken step
                return "true"
            if m == "loadTrack":
                self._place_cog(*self.p.start_xy, math.radians(self.p.start_heading_deg))
                return json.dumps({"loaded": parts[1], "cones": 40, "car_aligned": True,
                                   "environment": "off"})
            if m == "simSetVehiclePose":
                self._place_cog(float(parts[1]), float(parts[2]), self.psi)
                return "true"
            if m == "activateEbs":
                self.ebs, self.api, self.ctl = True, False, (0.0, 0.0, 0.0)
                return "true"
            if m == "releaseEbs":
                self.ebs, self.api = False, True
                return "true"
            if m == "enableApiControl":
                self.api = True
                return "true"
            if m in ("setCarControls", "setVehicleCommand"):
                if self.api and not self.ebs and len(parts) >= 4:
                    self.ctl = tuple(float(x) for x in parts[1:4])
                return "true"
            if m == "simGetGroundTruthKinematics":
                return json.dumps(self._kinematics())
            return json.dumps({"error": "unknown method"})


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        for raw in self.rfile:
            line = raw.decode().strip()
            if not line:
                continue
            self.wfile.write((self.server.sim.handle(line) + "\n").encode())


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def serve(sim: FakeSim):
    """Start it on a free local port: (server, port). server.shutdown() stops it."""
    srv = _Server(("127.0.0.1", 0), _Handler)
    srv.sim = sim
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, srv.server_address[1]


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]
