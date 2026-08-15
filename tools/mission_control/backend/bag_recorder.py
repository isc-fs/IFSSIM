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
import os
import re
import tarfile
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


# ---------------------------------------------------------------------
# #498 — auto-pull a finalised bag from the dv_pipeline_stack volume
# onto the host filesystem.
# ---------------------------------------------------------------------
# Gated by the IFSSIM_BAG_AUTO_PULL env var. When set, the StopBag
# handler in main.py calls `auto_pull_and_clean(bag_name)` after a
# successful stop; we use the Python `docker` SDK to:
#   1. `container.get_archive(/bags/<name>)` → stream the bag as a tar
#      from dv_pipeline_stack, write it to a host-bind-mounted
#      destination (/host_bags/ → host ./bags/).
#   2. `container.exec_run("rm -rf /bags/<name>")` → clean the
#      volume-side copy.
#
# We picked the SDK over CLI subprocess for one practical reason: the
# Linux `docker cp` CLI inside mc_backend can't pass a Windows host
# path (`C:/Users/...`) because it parses at the first colon and
# treats `C` as a container name. The SDK uses the HTTP API directly,
# no argv parsing, no path translation involved on our end — we just
# write the tarball through the bind-mount.
#
# The bind-mount (/host_bags/) brings back a small bit of the
# virtiofs/9p slowness #490 retired, but only for the *write* of the
# finalised tarball at session-stop. The recording itself lands in
# the named volume on container ext4 (fast); this only kicks in once
# the bag is closed.
_DEFAULT_PULL_TIMEOUT_S = 120.0
# Host-side mount where mc_backend writes the pulled bag. Bind-mounted
# in docker-compose.yml to `./bags/`.
_HOST_BAGS_DIR = "/host_bags"


def _get_docker_client():
    """Lazy-construct and cache a docker SDK client.

    Cached because socket-discovery + initial handshake is a few ms;
    we keep one client per process. Probing connectivity here lets
    `is_auto_pull_ready` return a clean diagnostic without a partial
    pull attempt.
    """
    global _docker_client_cache
    try:
        return _docker_client_cache
    except NameError:
        pass
    try:
        import docker  # type: ignore[import]
        client = docker.from_env(timeout=3)
        # Probe — raises on broken socket / daemon down / permission.
        client.ping()
    except Exception as ex:  # noqa: BLE001 — surfaces any SDK failure
        _LOG.warning(
            "bag_recorder: docker SDK unavailable (%s) — auto-pull disabled",
            ex,
        )
        client = None
    _docker_client_cache = client
    return client


def is_auto_pull_enabled() -> bool:
    """Read the env-gate at call-time so a `docker compose restart`
    with the flag flipped is enough to change behaviour."""
    raw = os.environ.get("IFSSIM_BAG_AUTO_PULL", "1").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _ensure_host_dir() -> Optional[str]:
    """Ensure the host-bags landing directory exists from mc_backend's
    perspective (bind-mounted to the host's ./bags/). Returns the path
    on success, None if the bind-mount isn't where we expect it.
    """
    p = Path(_HOST_BAGS_DIR)
    try:
        p.mkdir(parents=True, exist_ok=True)
    except OSError as ex:
        _LOG.warning("bag_recorder: cannot create %s: %s", _HOST_BAGS_DIR, ex)
        return None
    return str(p)


# The sim-side sensors that must be REPLAYED onto the real car when it's up on
# stands: LiDAR (the Hesai sees an empty garage) and IMU (the jacked car reads
# zero motion). Everything else — motor_rpm, steering_angle, tf_static — the
# live car supplies. See tools/lift_to_car.sh for the full rationale.
#
# LiDAR is /lidar_points — the bridge was unified onto the car's Hesai topic
# (2026-07-12), so the extracted cloud lands on the car's native perception
# topic with no remap on replay. /lidar/Lidar1 is kept as a legacy fallback so
# re-deriving a PRE-rename bag still captures its LiDAR (ros2 bag convert
# `topics:` is a filter — names absent from the input are simply skipped, and a
# bag only ever has one of the two names).
_CAR_PARITY_TOPICS = ("/imu", "/lidar_points", "/lidar/Lidar1")

# `tarfile` grew the `filter=` extraction policy (+ tarfile.data_filter) in
# Python 3.12; older runtimes reject the kwarg with a TypeError. We gate on it
# so the same code runs under 3.10 (container) and 3.12+ (CI).
_TAR_HAS_FILTER = hasattr(tarfile, "data_filter")


def is_car_parity_enabled() -> bool:
    """When set (default on), auto_pull also derives a car-liftable
    `<name>_carparity` mcap (LiDAR+IMU only) beside each finalised full dump."""
    raw = os.environ.get("IFSSIM_BAG_CAR_PARITY", "1").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _stream_extract(container, src_path: str, host_dir: str) -> Optional[str]:
    """Stream `container:src_path` out as a tar (docker get_archive) and extract
    it into `host_dir`. The tar's root entry is src_path's basename, so a
    `/bags/<x>` lands as `host_dir/<x>/...`. Returns None on success or a
    diagnostic string on failure.

    Streams to a temp file on disk before extracting — buffering a multi-GB
    bag in a BytesIO trips mc_backend's mem_limit and gets it SIGKILL'd
    mid-stop (live RAM stays at the ~256 KB chunk size this way).
    """
    import tempfile
    try:
        bits, _stat = container.get_archive(src_path)
    except Exception as ex:  # noqa: BLE001
        return f"get_archive({src_path!r}) failed: {ex}"

    tmp = tempfile.NamedTemporaryFile(
        prefix="bag_pull_", suffix=".tar", dir=host_dir, delete=False,
    )
    try:
        for chunk in bits:
            tmp.write(chunk)
        tmp.flush()
        tmp.close()
    except Exception as ex:  # noqa: BLE001
        try:
            os.unlink(tmp.name)
        except OSError:
            pass
        return f"streaming tarball to temp file failed: {ex}"

    try:
        with tarfile.open(name=tmp.name, mode="r") as tar:
            for member in tar:
                # Refuse any entry whose normalised path escapes host_dir.
                # `filter="data"` is the 3.12+ safe extraction policy (blocks
                # abs paths, `..`, special files). Our explicit traversal check
                # below is belt-and-braces + covers <3.12 where filter is absent.
                target = os.path.normpath(os.path.join(host_dir, member.name))
                if not (target == host_dir or target.startswith(host_dir + os.sep)):
                    return (
                        f"tarball contains suspicious member path: {member.name!r} "
                        f"(would land outside {host_dir!r})"
                    )
                if _TAR_HAS_FILTER:
                    tar.extract(member, path=host_dir, filter="data")
                else:
                    tar.extract(member, path=host_dir)
    except Exception as ex:  # noqa: BLE001
        return f"tar extract to {host_dir} failed: {ex}"
    finally:
        try:
            os.unlink(tmp.name)
        except OSError as ex:
            _LOG.warning("bag_recorder: temp tarball cleanup failed: %s", ex)
    return None


def _make_car_parity_copy(container, bag_name: str, cp_name: str) -> tuple:
    """Run `ros2 bag convert` inside dv_pipeline_stack to write `/bags/<cp_name>`
    containing ONLY the car-parity replay topics (mcap). The conversion runs
    where rosbag2 + the sensor_msgs defs live, so it works on both the mcap
    full dump and legacy db3 bags. Returns (ok, error); best-effort — a failure
    here never fails the full-bag pull.
    """
    import base64
    topic_list = ", ".join(_CAR_PARITY_TOPICS)
    cfg_yaml = (
        "output_bags:\n"
        f"  - uri: /bags/{cp_name}\n"
        "    storage_id: mcap\n"
        f"    topics: [{topic_list}]\n"
    )
    # Ship the convert config as base64 so no quoting/escaping can corrupt the
    # YAML inside the bash -lc string. `ros2 bag convert` refuses to overwrite
    # an existing uri, so clear any stale target first.
    cfg_b64 = base64.b64encode(cfg_yaml.encode()).decode()
    script = (
        "set -e; "
        "source /opt/ros/humble/setup.bash; "
        "source /dv_pipeline_stack_ws/install/setup.bash 2>/dev/null || true; "
        f"rm -rf /bags/{cp_name}; "
        "cfg=$(mktemp --suffix=.yaml); "
        f"echo {cfg_b64} | base64 -d > \"$cfg\"; "
        f"ros2 bag convert -i /bags/{bag_name} -o \"$cfg\"; "
        "rm -f \"$cfg\""
    )
    try:
        rc, output = container.exec_run(
            ["bash", "-lc", script], demux=False, stdout=True, stderr=True,
        )
    except Exception as ex:  # noqa: BLE001
        return False, f"ros2 bag convert exec raised: {ex}"
    if rc != 0:
        text = (output or b"").decode(errors="replace").strip()[-300:]
        return False, f"ros2 bag convert rc={rc}: {text}"
    return True, ""


def auto_pull_and_clean(
    bag_name: str,
    *,
    timeout_s: float = _DEFAULT_PULL_TIMEOUT_S,
) -> dict:
    """Move a finalised bag from the dv_pipeline_stack volume to the
    host filesystem, then delete the volume-side copy.

    Returns a dict with:
        ok          — bool. False on any failure (transfer, rm, env).
        bag_name    — echoed.
        host_path   — final container-side path to the bag (which is
                      `/host_bags/<bag_name>`, bind-mounted to the
                      host's `./bags/<bag_name>`). Empty if not pulled.
        error       — diagnostic on ok=false. Empty on success.

    Failure semantics: on transfer failure we DO NOT delete the
    volume-side copy (the user can recover with `tools/pull-bag.sh`).
    On rm failure we keep ok=true and surface a warning in `error` —
    the bag is safe on the host, the volume orphan is an annoyance,
    not a data-loss risk.
    """
    out: dict = {
        "ok": False,
        "bag_name": bag_name,
        "host_path": "",
        "error": "",
    }

    if not is_auto_pull_enabled():
        out["error"] = "IFSSIM_BAG_AUTO_PULL disabled"
        return out
    if not bag_name:
        out["error"] = "empty bag_name"
        return out

    # Defence in depth: reject suspicious bag names that could escape
    # /host_bags via traversal. Recorder's compose_bag_name sanitises
    # upstream, but anything we untar runs in mc_backend's filesystem.
    if "/" in bag_name or ".." in bag_name or bag_name.startswith("-"):
        out["error"] = f"refusing to pull bag with suspicious name: {bag_name!r}"
        return out

    client = _get_docker_client()
    if client is None:
        out["error"] = "docker SDK / docker.sock unavailable from mc_backend"
        return out

    container_name = os.environ.get(
        "DV_PIPELINE_STACK_CONTAINER", "ifssim-dv_pipeline_stack-1",
    ).strip()
    host_dir = _ensure_host_dir()
    if host_dir is None:
        out["error"] = (
            f"host-bags bind-mount {_HOST_BAGS_DIR!r} not present — "
            "is mc_backend missing the `./bags:/host_bags` mount?"
        )
        return out

    try:
        container = client.containers.get(container_name)
    except Exception as ex:  # noqa: BLE001
        out["error"] = f"container {container_name!r} not found: {ex}"
        return out

    # Pull the full bag: docker get_archive → extract into the host bind-mount.
    # The tarball's root entry is `<bag_name>/`, so it lands at
    # /host_bags/<bag_name>/... (host ./bags/<bag_name>/).
    err = _stream_extract(container, f"/bags/{bag_name}", host_dir)
    if err:
        out["error"] = err
        return out

    out["ok"] = True
    out["host_path"] = f"{host_dir}/{bag_name}"

    # Derive a car-liftable bag (LiDAR+IMU only) from the full dump via
    # `ros2 bag convert` inside dv_pipeline_stack, and pull it alongside. This
    # is the self-service replacement for tools/record_bag.sh --car-parity: the
    # frontend records the full mcap, and this strips it to exactly the sensor
    # set the real car needs replayed on the stand (see tools/lift_to_car.sh).
    # Best-effort — a failure here never fails the full-bag pull.
    volume_bags = [f"/bags/{bag_name}"]
    if is_car_parity_enabled():
        cp_name = f"{bag_name}_carparity"
        cp_ok, cp_err = _make_car_parity_copy(container, bag_name, cp_name)
        if cp_ok:
            volume_bags.append(f"/bags/{cp_name}")  # clean up either way
            cp_extract_err = _stream_extract(
                container, f"/bags/{cp_name}", host_dir)
            if cp_extract_err is None:
                out["car_parity_path"] = f"{host_dir}/{cp_name}"
            else:
                out["car_parity_error"] = cp_extract_err
                _LOG.warning(
                    "bag_recorder: car-parity pull failed: %s", cp_extract_err)
        else:
            out["car_parity_error"] = cp_err
            _LOG.warning("bag_recorder: car-parity convert failed: %s", cp_err)

    # Best-effort clean of the volume-side copies (full + car-parity). Even on
    # failure here the bags are safely on the host, so we keep ok=true and just
    # surface a warning.
    try:
        rc, output = container.exec_run(
            ["rm", "-rf", *volume_bags],
            demux=False, stdout=True, stderr=True,
        )
        if rc != 0:
            text = (output or b"").decode(errors="replace").strip()[:200]
            out["error"] = (
                f"transfer ok, but cleanup failed (rc={rc}): {text}; "
                "bag is on the host, volume has an orphan."
            )
    except Exception as ex:  # noqa: BLE001
        out["error"] = (
            f"transfer ok, but cleanup raised: {ex}; "
            "bag is on the host, volume has an orphan."
        )

    return out


def _reset_docker_probe_cache_for_test() -> None:
    """Test-only: drop the cached docker SDK client so a new
    monkeypatched docker module is picked up."""
    global _docker_client_cache
    try:
        del _docker_client_cache
    except NameError:
        pass
