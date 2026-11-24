"""Client for the emulated boards (tools/vhil_boards/server.py), used by sim_supervisor_node.

With ``IFSSIM_VHIL_BOARDS=host:port`` set, the uDV the pipeline talks to is the real
uDV firmware running on IFS-vHIL's emulated MainLite, together with an emulated ECU
and AMS, instead of the Python emulation in this package. The micro-ROS link
(USB CDC) between the uDV and the pipeline is not modelled in vHIL, so this node is
its other end: it publishes what the firmware says (AS state, mission) and hands the
firmware what the pipeline sent (``/dv/status``, ``/ctrl/cmd``) and what the operator
panel asked for (``/sim/*``).

Pure Python (sockets and JSON), no ROS import, so it is tested without a ROS install.
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time
from typing import Any

ENV = "IFSSIM_VHIL_BOARDS"
PANEL = ("mission", "intent", "estop")
STALE_S = 1.0  # no state this long: the boards are not answering


def address_from_env(env: dict[str, str] | None = None) -> tuple[str, int] | None:
    text = (env if env is not None else os.environ).get(ENV, "").strip()
    if not text:
        return None
    host, _, port = text.rpartition(":")
    return (host or "127.0.0.1", int(port))


class VhilLink:
    """Reconnecting TCP/JSON-lines client. ``state`` is the boards' latest report."""

    def __init__(self, address: tuple[str, int]):
        self.address = address
        self.state: dict[str, Any] = {}
        self.state_at = 0.0
        self._sock: socket.socket | None = None
        self._panel: dict[str, dict[str, Any]] = {}  # the operator's last words, replayed on reconnect
        self._lock = threading.Lock()
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # -- what the boards say ---------------------------------------------
    def fresh(self) -> bool:
        return (time.monotonic() - self.state_at) < STALE_S

    def assi(self) -> int | None:
        """The ASSI byte for /assi/state, or None while the boards are silent."""
        return int(self.state["assi"]) if self.fresh() and "assi" in self.state else None

    def mission(self) -> int:
        return int(self.state.get("mission", -1)) if self.fresh() else -1

    def torque_nm(self) -> float | None:
        """The torque the emulated ECU asks the inverter for (negative = forward)."""
        return float(self.state["torque_nm"]) if self.fresh() and "torque_nm" in self.state else None

    # -- what the pipeline and the operator say ----------------------------
    def send(self, **msg: Any) -> None:
        if msg.get("cmd") in PANEL:
            self._panel[msg["cmd"]] = msg
        with self._lock:
            sock = self._sock
        if sock is None:
            return
        try:
            sock.sendall((json.dumps(msg) + "\n").encode())
        except OSError:
            pass

    def close(self) -> None:
        self._stop = True
        self.send(cmd="quit")
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass

    # -- connection --------------------------------------------------------
    def _run(self) -> None:
        while not self._stop:
            try:
                sock = socket.create_connection(self.address, timeout=2.0)
                sock.settimeout(None)
            except OSError:
                time.sleep(0.5)
                continue
            with self._lock:
                self._sock = sock
            for msg in list(self._panel.values()):
                self.send(**msg)
            try:
                for line in sock.makefile("r"):
                    try:
                        self.state = json.loads(line)
                        self.state_at = time.monotonic()
                    except ValueError:
                        pass
            except OSError:
                pass
            with self._lock:
                self._sock = None
            time.sleep(0.2)
