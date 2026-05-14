"""Bag-recording helpers for the Mission Control session UX (#465).

The Mission Control session-start endpoint accepts an optional
`record_bag` flag. When set, this module:

  1. Composes a safe bag name from event_type + track + timestamp.
  2. Pre-checks the dv_pipeline_stack container's `/bags` mount for
     ≥10 GiB free (refuses to start otherwise — silent disk fills
     during long test sessions used to be a thing).
  3. Spawns `ros2 bag record -s mcap -a -o /bags/<name>` inside
     dv_pipeline_stack via `docker exec -d`.
  4. On session-stop, sends SIGINT to the recorder PID and copies the
     bag out to the host `bags/` directory (next to the
     analyze_*.py scripts the team uses for offline post-mortems).

Pure helpers — no FastAPI / no ros_bridge import — so the test suite
can exercise the docker plumbing with `subprocess` mocked. The session
handler in `main.py` orchestrates the lifecycle and stores the live
state dict in a module-level slot.

Wire-format contract:

  start_recording(...)  →  state dict with keys:
      name        — bag dir name (no host path)
      container   — docker container the recorder runs in
      pid         — ros2-bag-record PID inside the container
      started_at  — UTC unix timestamp at exec time
      state       — "recording"

  stop_recording(state, host_bags_dir)  →  same dict, mutated:
      state       — "stopped" or "failed"
      host_path   — absolute path of the copied-out bag (only on success)
      error       — string (only on failure)

The session handler is expected to call `stop_recording` exactly once
per `start_recording` and to never mutate the state dict directly.
"""
from __future__ import annotations

import logging
import re
import shutil
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

_LOG = logging.getLogger(__name__)


# Minimum free space (GiB) we require on the container's /bags mount
# before agreeing to start recording. An ATX-S01 scan @ 10 Hz with the
# full /imu /testing_only/odom /Conos /Conos_raw /odom /Path /slam/pose
# /control_command set lands around 800 MB / minute in mcap. A 30-min
# test session is ~24 GB; 10 GiB free is the floor below which we
# assume the operator will get cut off mid-lap. Tunable per-call but
# the default has been empirically right for ~6 months.
DEFAULT_MIN_FREE_GIB = 10


class BagRecorderError(Exception):
    """Base for any failure that should bubble back to the API."""


class DiskFullError(BagRecorderError):
    """Free space below the configured floor."""


class DockerExecError(BagRecorderError):
    """`docker exec` itself failed (container down, command bad, etc.)."""


# Sanitiser for the track segment of the bag name. Compose into a
# filesystem-safe form; the source can be "track_20260512_151240.csv"
# (a generated track) or "trackdrive" (the event name) — we strip
# extensions, replace non-[A-Za-z0-9_-] with "_", and clamp length so
# the final path stays well under PATH_MAX on every host.
_NAME_BAD = re.compile(r"[^A-Za-z0-9_-]+")


def _sanitize(s: str) -> str:
    s = Path(s).stem
    return _NAME_BAD.sub("_", s).strip("_")[:48]


def compose_bag_name(
    event_type: str,
    track: Optional[str],
    now: Optional[datetime] = None,
) -> str:
    """Return a directory name suitable for `ros2 bag record -o`.

    Shape: `<event>_<track>_<YYYYMMDD_HHMMSS>`. Either segment may be
    absent (we fall back to "unknown") but the timestamp is always
    present so two consecutive recordings can never collide.
    """
    when = (now or datetime.now(timezone.utc)).strftime("%Y%m%d_%H%M%S")
    event = _sanitize(event_type) or "unknown"
    trk = _sanitize(track) if track else "no-track"
    return f"{event}_{trk}_{when}"


def check_free_disk(
    container: str,
    path: str = "/bags",
    min_gib: int = DEFAULT_MIN_FREE_GIB,
    *,
    _run: callable = subprocess.run,
) -> tuple[bool, int]:
    """Return (ok, free_gib). Uses `df -P` inside the container.

    `_run` is injectable so the test suite can drive synthetic df output
    without touching docker.
    """
    proc = _run(
        ["docker", "exec", container, "df", "-PB1G", path],
        capture_output=True, text=True, check=False, timeout=10,
    )
    if proc.returncode != 0:
        raise DockerExecError(f"df failed: {proc.stderr.strip() or 'no output'}")
    # df -P prints a header then exactly one row for the queried path.
    # Block-1G output: "Filesystem 1G-blocks Used Available Capacity Mounted on"
    lines = [ln for ln in proc.stdout.splitlines() if ln.strip()]
    if len(lines) < 2:
        raise DockerExecError(f"df output unparseable: {proc.stdout!r}")
    cols = lines[-1].split()
    try:
        avail_gib = int(cols[3])
    except (IndexError, ValueError):
        raise DockerExecError(f"df row unparseable: {lines[-1]!r}") from None
    return avail_gib >= min_gib, avail_gib


def start_recording(
    container: str,
    bag_name: str,
    *,
    container_bags_dir: str = "/bags",
    min_free_gib: int = DEFAULT_MIN_FREE_GIB,
    _run: callable = subprocess.run,
) -> dict:
    """Spawn `ros2 bag record` inside `container`, return state dict.

    Pre-checks free disk; raises `DiskFullError` if below floor.
    On success the recorder runs detached (`docker exec -d`) and we
    capture its PID for the matching stop. The caller is responsible
    for persisting the returned dict.
    """
    ok, free_gib = check_free_disk(
        container, path=container_bags_dir, min_gib=min_free_gib, _run=_run,
    )
    if not ok:
        raise DiskFullError(
            f"only {free_gib} GiB free on {container}:{container_bags_dir} "
            f"(need ≥{min_free_gib} GiB) — refusing to start recording"
        )

    # ros2 bag record needs the workspace sourced for the storage
    # plugin (mcap) to be discoverable, and the ROS distro sourced for
    # rclpy. The container's normal entrypoint sources these; we
    # replicate that here so a `docker exec -d` recording doesn't
    # depend on the calling shell's environment.
    bag_path = f"{container_bags_dir}/{bag_name}"
    cmd_in_container = (
        "source /opt/ros/humble/setup.bash && "
        "source /dv_pipeline_stack_ws/install/setup.bash && "
        f"mkdir -p {container_bags_dir} && "
        f"cd {container_bags_dir} && "
        # `setsid` puts the recorder in its own process group so our
        # eventual SIGINT reaches `ros2 bag record` itself, not just
        # the bash wrapper. The `echo $!` is captured separately
        # below via the PID poll — we use `pgrep` rather than parsing
        # docker-exec's own stdout because `docker exec -d` returns
        # immediately and discards the child's output.
        f"exec ros2 bag record -s mcap -a -o {bag_name} "
        f"> /tmp/{bag_name}.log 2>&1"
    )

    # `-d` detaches; subprocess returns as soon as docker exec
    # confirms the child started.
    proc = _run(
        ["docker", "exec", "-d", container, "bash", "-lc", cmd_in_container],
        capture_output=True, text=True, check=False, timeout=10,
    )
    if proc.returncode != 0:
        raise DockerExecError(
            f"docker exec failed (rc={proc.returncode}): "
            f"{proc.stderr.strip() or 'no output'}"
        )

    # Poll for the PID via pgrep. The recorder is a python3 process
    # whose args contain the bag name (we use it as a unique tag) so
    # pgrep -f matches reliably even with multiple bags ever recorded.
    pid: Optional[int] = None
    deadline = time.time() + 3.0
    while time.time() < deadline:
        pid_proc = _run(
            ["docker", "exec", container, "pgrep", "-f", f"ros2 bag record .* {bag_name}"],
            capture_output=True, text=True, check=False, timeout=5,
        )
        if pid_proc.returncode == 0:
            for line in pid_proc.stdout.splitlines():
                line = line.strip()
                if line.isdigit():
                    pid = int(line)
                    break
        if pid is not None:
            break
        time.sleep(0.2)

    state = {
        "name": bag_name,
        "path_in_container": bag_path,
        "container": container,
        "pid": pid,
        "started_at": time.time(),
        "state": "recording" if pid is not None else "starting",
    }
    if pid is None:
        # Treat as starting — the next state poll can promote to
        # "recording" once pgrep finds it, or to "failed" if the
        # recorder died on startup.
        _LOG.warning(
            "bag_recorder: %s started but pgrep didn't find PID within 3s "
            "(may have failed; check /tmp/%s.log inside %s)",
            bag_name, bag_name, container,
        )
    return state


def stop_recording(
    state: dict,
    host_bags_dir: Path,
    *,
    _run: callable = subprocess.run,
    _sleep: callable = time.sleep,
) -> dict:
    """SIGINT the recorder, wait for clean close, copy bag to host.

    `state` is the dict returned by `start_recording`. Mutated in-place
    with the result and returned for caller convenience.
    """
    if state.get("state") in ("stopped", "failed", "none"):
        return state

    container = state["container"]
    name = state["name"]
    pid = state.get("pid")

    # SIGINT triggers ros2 bag record's clean-shutdown path (close
    # current mcap chunk, write the index, exit). SIGKILL would leave
    # a truncated final chunk. If pid is None we never found one, so
    # fall back to pkill -f for the same arg pattern start_recording
    # used to discover it.
    if pid is not None:
        _run(
            ["docker", "exec", container, "kill", "-INT", str(pid)],
            capture_output=True, text=True, check=False, timeout=5,
        )
    else:
        _run(
            ["docker", "exec", container, "pkill", "-INT", "-f",
             f"ros2 bag record .* {name}"],
            capture_output=True, text=True, check=False, timeout=5,
        )

    # Wait up to 5 s for the recorder to disappear. ros2 bag's close
    # path is typically <500 ms; the long tail is just the final
    # chunk flush on a big bag.
    deadline = time.time() + 5.0
    while time.time() < deadline:
        check = _run(
            ["docker", "exec", container, "pgrep", "-f",
             f"ros2 bag record .* {name}"],
            capture_output=True, text=True, check=False, timeout=5,
        )
        if check.returncode != 0:
            break
        _sleep(0.2)

    # Copy out. The host dir is the operator's offline-analysis
    # landing zone (`bags/` next to `tools/bags/`). `docker cp -L`
    # follows symlinks; mcap files aren't symlinked but the bag
    # directory could be on some setups.
    host_path = host_bags_dir / name
    host_bags_dir.mkdir(parents=True, exist_ok=True)
    if host_path.exists():
        # Should not happen (the timestamp-named bag is unique) but
        # be defensive: never overwrite a finished bag.
        shutil.rmtree(host_path)
    cp = _run(
        ["docker", "cp", f"{container}:{state['path_in_container']}",
         str(host_path)],
        capture_output=True, text=True, check=False, timeout=120,
    )
    if cp.returncode != 0:
        state["state"] = "failed"
        state["error"] = f"docker cp failed: {cp.stderr.strip() or 'no output'}"
        return state

    state["state"] = "stopped"
    state["host_path"] = str(host_path)
    state["stopped_at"] = time.time()
    return state
