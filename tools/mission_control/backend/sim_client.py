"""
Sim client wrapper for the Mission Control backend.
Uses a persistent TCP connection (reconnects on failure) to avoid flooding
UE5 with a new connection for every API call.
All methods are safe to call when the sim is disconnected.
"""

import sys
import os
import json
import socket
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "python"))

from ifssim import IFSSIMClient


class SimConnection:
    """Persistent TCP connection to the IFSSIM RPC server."""

    def __init__(self, host: str = "127.0.0.1", port: int = 41451):
        self.host = host
        self.port = port
        self._sock: socket.socket | None = None
        self._rbuf = b""
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def _connect(self) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((self.host, self.port))
            s.settimeout(5.0)
            self._sock = s
            self._rbuf = b""
            return True
        except Exception:
            self._sock = None
            return False

    def _disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._rbuf = b""

    # ------------------------------------------------------------------
    # Core send/receive — the server keeps the connection alive and sends
    # exactly one newline-terminated line per command.
    # ------------------------------------------------------------------

    def _cmd(self, cmd: str) -> str:
        with self._lock:
            for attempt in range(2):
                try:
                    if self._sock is None and not self._connect():
                        return ""

                    self._sock.sendall((cmd + "\n").encode())

                    # Read until newline
                    while b"\n" not in self._rbuf:
                        chunk = self._sock.recv(65536)
                        if not chunk:
                            raise ConnectionError("socket closed by server")
                        self._rbuf += chunk

                    line, _, self._rbuf = self._rbuf.partition(b"\n")
                    return line.decode().strip()

                except Exception:
                    self._disconnect()
                    if attempt == 0:
                        continue
            return ""

    def _json_cmd(self, cmd: str) -> dict:
        resp = self._cmd(cmd)
        if not resp:
            return {}
        try:
            return json.loads(resp)
        except Exception:
            return {"raw": resp}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def is_connected(self) -> bool:
        return self._cmd("ping") == "true"

    def get_status(self) -> dict:
        result = self._json_cmd("getSimStatus")
        return {
            "map": result.get("map", "unknown"),
            "fps": result.get("fps", 0),
            "paused": result.get("paused", False),
            "api_control": result.get("api_control", False),
        }

    def _require_connected(self):
        """Raise RuntimeError if the sim is not reachable."""
        if not self.is_connected():
            raise RuntimeError("Simulator not connected")

    def pause(self):
        self._require_connected()
        self._cmd("simPause")

    def resume(self):
        self._require_connected()
        self._cmd("simResume")

    def is_paused(self) -> bool:
        return self._cmd("simIsPaused") == "true"

    def set_event(self, event_type: str, num_laps: int = 10) -> dict:
        self._require_connected()
        return self._json_cmd(f"setEventType {event_type} {num_laps}")

    def get_referee_state(self) -> dict:
        return self._json_cmd("getRefereeState")

    def res_activate(self):
        self._require_connected()
        # Order matters: apply the brake while api_control is still enabled,
        # THEN disable api_control. A control node that hasn't died yet will
        # keep sending throttle, but UE5 ignores it once api_control is off,
        # so the brake we just latched holds.
        self._cmd("setCarControls 0 0 1")
        self._cmd("disableApiControl")

    def res_release(self):
        self._require_connected()
        self._cmd("enableApiControl")
        self._cmd("setCarControls 0 0 0")

    def get_vehicle_state(self) -> dict:
        state = self._json_cmd("getCarState")
        controls = self._json_cmd("getCarControls")
        state["controls"] = controls
        return state

    def get_vehicle_pose(self) -> dict:
        return self._json_cmd("simGetVehiclePose")

    def teleport(self, x: float, y: float, z: float,
                 qw: float = 1.0, qx: float = 0.0,
                 qy: float = 0.0, qz: float = 0.0):
        self._cmd(f"simSetVehiclePose {x} {y} {z} {qw} {qx} {qy} {qz}")

    def teleport_pos(self, x: float, y: float, z: float):
        """Teleport position only — orientation unchanged."""
        self._cmd(f"simSetVehiclePose {x} {y} {z}")

    def load_track(self, filepath: str) -> dict:
        return self._json_cmd(f"loadTrack {filepath}")

    def get_settings(self) -> str:
        return self._cmd("getSettingsString")
