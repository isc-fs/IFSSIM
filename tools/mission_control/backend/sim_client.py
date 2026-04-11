"""
Sim client wrapper — thin layer over IFSSIMClient for the Mission Control backend.
All methods are safe to call even when the sim is disconnected.
"""

import sys
import os
import json
import socket

# Add the Python client to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "python"))

from ifssim import IFSSIMClient


def _safe_cmd(host: str, port: int, cmd: str) -> str:
    """Send a single TCP command and get response. Independent socket per call."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3)
        s.connect((host, port))
        s.sendall((cmd + "\n").encode())
        import time; time.sleep(0.1)
        data = s.recv(65536).decode().strip()
        s.close()
        return data
    except Exception:
        return ""


class SimConnection:
    """Managed connection to the IFSSIM simulator."""

    def __init__(self, host: str = "127.0.0.1", port: int = 41451):
        self.host = host
        self.port = port

    def _cmd(self, cmd: str) -> str:
        """Send a command, return raw response string."""
        return _safe_cmd(self.host, self.port, cmd)

    def _json_cmd(self, cmd: str) -> dict:
        """Send a command, parse JSON response."""
        resp = self._cmd(cmd)
        if not resp:
            return {}
        try:
            return json.loads(resp)
        except Exception:
            return {"raw": resp}

    def is_connected(self) -> bool:
        return self._cmd("ping") == "true"

    # --- Sim Control ---

    def get_status(self) -> dict:
        result = self._json_cmd("getSimStatus")
        return {
            "map": result.get("map", "unknown"),
            "fps": result.get("fps", 0),
            "paused": result.get("paused", False),
            "api_control": result.get("api_control", False),
        }

    def pause(self):
        return self._cmd("simPause")

    def resume(self):
        return self._cmd("simResume")

    def reset(self):
        return self._cmd("reset")

    def is_paused(self) -> bool:
        return self._cmd("simIsPaused") == "true"

    # --- Event Control ---

    def set_event(self, event_type: str, num_laps: int = 10) -> dict:
        return self._json_cmd(f"setEventType {event_type} {num_laps}")

    def get_referee_state(self) -> dict:
        return self._json_cmd("getRefereeState")

    # --- RES (Remote Emergency Stop) ---

    def res_activate(self):
        """Emergency stop: full brake, zero throttle."""
        self._cmd("setCarControls 0 0 1")

    def res_release(self):
        """Release brakes."""
        self._cmd("enableApiControl")
        self._cmd("setCarControls 0 0 0")

    # --- Vehicle ---

    def get_vehicle_state(self) -> dict:
        state = self._json_cmd("getCarState")
        controls = self._json_cmd("getCarControls")
        state["controls"] = controls
        return state

    def get_vehicle_pose(self) -> dict:
        return self._json_cmd("simGetVehiclePose")

    def teleport(self, x: float, y: float, z: float):
        self._cmd(f"simSetVehiclePose {x} {y} {z}")

    # --- Track ---

    def load_track(self, filepath: str) -> dict:
        return self._json_cmd(f"loadTrack {filepath}")

    def get_settings(self) -> str:
        return self._cmd("getSettingsString")
