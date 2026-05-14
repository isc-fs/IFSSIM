"""Bag-recording client for the Mission Control session UX (#465).

Calls /bag_recorder/start + /bag_recorder/stop on the dv_pipeline_stack
DDS graph. The actual `ros2 bag record` subprocess lives inside the
dv_pipeline_stack container (see pipeline/bag_recorder_node/) where
it shares the SHM-tuned Fast DDS context with the publishers — that's
the only way to get full-fidelity 10 Hz LiDAR + camera capture, since
mc_backend is forced to UDPv4-only for its StartMission action client.

## Wire-format contract

  compose_bag_name(event_type, track) → str   (kept here — name
      synthesis is a backend concern, the ROS node accepts whatever
      string the backend hands it)

  request_start(ros_bridge, bag_name) → dict:
      ok          — bool
      state       — "recording" | "failed"
      name        — bag_name (echoed)
      path        — absolute host path of final bag dir
      error       — diagnostic on ok=false

  request_stop(ros_bridge) → dict:
      ok          — bool
      state       — "stopped" | "failed" | "none"
      path        — absolute host path of finalised bag dir
      error       — diagnostic on ok=false

Returns plain dicts (not ROS response objects) so main.py's lifecycle
code can store + serialise them without re-importing rclpy. The
ros_bridge handle is the existing one from `ros_bridge.py` — this
module just borrows its rclpy.Node to host two service clients.
"""
from __future__ import annotations

import logging
import re
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

_LOG = logging.getLogger(__name__)


# Name sanitisation lives here (not in the ROS node) because it's a
# pure function and the mc_backend is the only producer of bag names.
# Keeping it on this side means the test suite can drive it without
# spinning up rclpy.
_NAME_BAD = re.compile(r"[^A-Za-z0-9_-]+")


def _sanitize(s: str) -> str:
    s = Path(s).stem
    return _NAME_BAD.sub("_", s).strip("_")[:48]


def compose_bag_name(
    event_type: str,
    track: Optional[str],
    now: Optional[datetime] = None,
) -> str:
    """Return a filesystem-safe bag dir name.

    Shape: `<event>_<track>_<YYYYMMDD_HHMMSS>`. Either segment may be
    absent (we fall back to "unknown" / "no-track"). The timestamp is
    always present so consecutive recordings can't collide.
    """
    when = (now or datetime.now(timezone.utc)).strftime("%Y%m%d_%H%M%S")
    event = _sanitize(event_type) or "unknown"
    trk = _sanitize(track) if track else "no-track"
    return f"{event}_{trk}_{when}"


# Service client cache — created lazily on the FIRST request_start /
# request_stop call so the import path stays cheap when ROS isn't
# present (CI, unit tests). Re-used across calls because creating a
# rclpy service client involves DDS discovery.
_clients_lock = threading.Lock()
_start_client = None
_stop_client = None
_start_srv_type = None
_stop_srv_type = None


def _ensure_clients(ros_bridge) -> None:
    """Lazy-init StartBag + StopBag service clients on the bridge's node."""
    global _start_client, _stop_client, _start_srv_type, _stop_srv_type
    with _clients_lock:
        if _start_client is not None and _stop_client is not None:
            return
        from dv_msgs.srv import StartBag, StopBag
        _start_srv_type = StartBag
        _stop_srv_type = StopBag
        _start_client = ros_bridge._node.create_client(
            StartBag, "/bag_recorder/start",
        )
        _stop_client = ros_bridge._node.create_client(
            StopBag, "/bag_recorder/stop",
        )


def _wait_service(client, name: str, timeout_s: float) -> bool:
    """Block until the service server is reachable, or timeout."""
    if not client.wait_for_service(timeout_sec=timeout_s):
        _LOG.warning(
            "bag_recorder: %s server not reachable within %.1fs", name, timeout_s,
        )
        return False
    return True


def _call_sync(client, request, timeout_s: float):
    """Send a service request synchronously via rclpy's spin executor.

    ros_bridge spins its node on a dedicated thread, so we can just
    fire-and-wait on the future. Returns the response or None on
    timeout / failure.
    """
    future = client.call_async(request)
    # ros_bridge's executor will tick this future on its own thread.
    # We block here for up to timeout_s.
    deadline = threading.Event()

    def _on_done(_fut):
        deadline.set()

    future.add_done_callback(_on_done)
    if not deadline.wait(timeout_s):
        return None
    if future.exception() is not None:
        _LOG.warning("bag_recorder: service raised: %s", future.exception())
        return None
    return future.result()


def request_start(ros_bridge, bag_name: str, *, timeout_s: float = 5.0) -> dict:
    """Ask bag_recorder_node to start a recording.

    Returns a state dict (ok / state / name / path / error). On any
    failure to reach the service, ok=false with a diagnostic.
    """
    try:
        _ensure_clients(ros_bridge)
    except Exception as ex:
        return {
            "ok": False,
            "state": "failed",
            "name": bag_name,
            "path": "",
            "error": f"bag_recorder service client init failed: {ex}",
        }

    if not _wait_service(_start_client, "/bag_recorder/start", timeout_s):
        return {
            "ok": False,
            "state": "failed",
            "name": bag_name,
            "path": "",
            "error": (
                "/bag_recorder/start not reachable — is the "
                "dv_pipeline_stack container running and healthy?"
            ),
        }

    req = _start_srv_type.Request()
    req.bag_name = bag_name

    resp = _call_sync(_start_client, req, timeout_s)
    if resp is None:
        return {
            "ok": False,
            "state": "failed",
            "name": bag_name,
            "path": "",
            "error": f"/bag_recorder/start timed out after {timeout_s:.1f}s",
        }

    return {
        "ok": bool(resp.ok),
        "state": resp.state or ("recording" if resp.ok else "failed"),
        "name": bag_name,
        "path": resp.bag_path,
        "error": resp.error,
    }


def request_stop(ros_bridge, *, timeout_s: float = 15.0) -> dict:
    """Ask bag_recorder_node to stop the active recording.

    Idempotent: returns state="none" if nothing was recording. Timeout
    is intentionally generous (15 s default) because the server side
    has to SIGINT the recorder, wait for mcap to flush its chunk
    index, then move the staged dir into the bind-mounted output
    directory — the move can be a couple of seconds for a multi-GB
    bag on macOS virtiofs.
    """
    try:
        _ensure_clients(ros_bridge)
    except Exception as ex:
        return {
            "ok": False,
            "state": "failed",
            "path": "",
            "error": f"bag_recorder service client init failed: {ex}",
        }

    if not _wait_service(_stop_client, "/bag_recorder/stop", timeout_s=2.0):
        return {
            "ok": False,
            "state": "failed",
            "path": "",
            "error": "/bag_recorder/stop not reachable",
        }

    req = _stop_srv_type.Request()
    resp = _call_sync(_stop_client, req, timeout_s)
    if resp is None:
        return {
            "ok": False,
            "state": "failed",
            "path": "",
            "error": f"/bag_recorder/stop timed out after {timeout_s:.1f}s",
        }

    return {
        "ok": bool(resp.ok),
        "state": resp.state or ("stopped" if resp.ok else "failed"),
        "path": resp.bag_path,
        "error": resp.error,
    }


def _reset_clients_for_test() -> None:
    """Test-only: drop the cached clients so a new ros_bridge mock is picked up."""
    global _start_client, _stop_client, _start_srv_type, _stop_srv_type
    with _clients_lock:
        _start_client = None
        _stop_client = None
        _start_srv_type = None
        _stop_srv_type = None
