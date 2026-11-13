"""The local benchmark queue: a folder that ``run_spec.py --work`` (on the host) and
Mission Control's backend (in Docker, which sees the results folder) share.

Mission Control queues a spec here (target *Local*), ``run_spec.py --queue`` does the
same from a shell, and one worker on the host runs the jobs one at a time, because
they share the sim. Benchmarks don't run inside the backend's container: the pipeline
stack they start needs host paths and the host's Docker.

Layout, under ``<results>/.bench/``::

    queue/<id>/job.json     what to run: the spec, its name, who asked, when
    queue/<id>/state.json   queued | running | done | failed | cancelled, progress, exit
    queue/<id>/cancel       present when someone asked to cancel it
    queue/<id>/worker.log   the run's output
    worker.json             the worker's heartbeat (pid, host, time, current job)
    stop                    present when someone asked the worker to stop
    rig.json                this machine's checks (launch/rig.py), refreshed by the worker

Only the standard library here: the backend imports this file as it is.
"""

from __future__ import annotations

import json
import os
import stat
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

WORK = ".bench"
STATES = ("queued", "running", "done", "failed", "cancelled")
FINISHED = ("done", "failed", "cancelled")
HEARTBEAT_STALE_S = 30.0  # the worker writes its heartbeat every few seconds


def bench_dir(results: Path | str) -> Path:
    return Path(results) / WORK


def queue_dir(results: Path | str) -> Path:
    return bench_dir(results) / "queue"


def now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _adopt(path: Path) -> None:
    """Mission Control's backend runs as root in its container: what it creates here
    goes to whoever owns the folder above, so the host's worker can write it too.

    Docker Desktop (Windows, Mac) shows every folder of a bind mount as root's, so
    there is no owner to give it to; it gets the permissions of the folder above
    instead. Without that the queue's folders were root's and 755 to WSL, and a
    worker started there couldn't write a job's state."""
    if os.geteuid() != 0:
        return
    above = None
    for up in path.parents:
        try:
            st = up.stat()
        except OSError:
            continue
        if above is None:
            above = st
        if st.st_uid != 0:
            os.chown(path, st.st_uid, st.st_gid)
            return
    if above is not None:
        mode = stat.S_IMODE(above.st_mode)
        os.chmod(path, mode if path.is_dir() else mode & 0o666)


def make_dirs(d: Path) -> None:
    """``mkdir -p``, each new folder adopted (``_adopt``)."""
    missing = []
    for p in [d, *d.parents]:
        if p.exists():
            break
        missing.append(p)
    for p in reversed(missing):
        p.mkdir(exist_ok=True)
        _adopt(p)


def write_json(path: Path, doc: dict[str, Any]) -> None:
    """Written whole, then renamed: a reader never sees half a file."""
    make_dirs(path.parent)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(doc, indent=2, default=str))
    _adopt(tmp)
    tmp.replace(path)


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


# ------------------------------------------------------------------ queueing
def enqueue(
    results: Path | str,
    spec: dict[str, Any],
    *,
    requested_by: str,
    trigger: str = "local",
    name: str | None = None,
) -> str:
    """Add a spec to the queue; returns its id (sortable by time)."""
    if not isinstance(spec, dict):
        raise ValueError("a spec is a mapping")
    # microseconds, so two specs queued in the same second still run in order
    jid = f"{datetime.now(timezone.utc):%Y%m%d_%H%M%S_%f}_{uuid.uuid4().hex[:4]}"
    d = queue_dir(results) / jid
    make_dirs(d.parent)
    d.mkdir()
    _adopt(d)
    write_json(
        d / "job.json",
        {
            "id": jid,
            "name": name or spec.get("name") or "",
            "spec": spec,
            "requested_by": requested_by,
            "trigger": trigger,
            "created": now(),
        },
    )
    write_json(d / "state.json", {"state": "queued"})
    return jid


def job(results: Path | str, jid: str) -> dict[str, Any] | None:
    d = queue_dir(results) / jid
    j = read_json(d / "job.json")
    if j is None:
        return None
    st = read_json(d / "state.json") or {"state": "queued"}
    return j | {
        "state": st.get("state", "queued"),
        "status": st,
        "cancel_requested": (d / "cancel").exists(),
        "log": str(d / "worker.log") if (d / "worker.log").exists() else None,
    }


def jobs(results: Path | str, limit: int | None = None) -> list[dict[str, Any]]:
    """Every job, newest first."""
    q = queue_dir(results)
    if not q.is_dir():
        return []
    ids = sorted((p.name for p in q.iterdir() if p.is_dir()), reverse=True)
    out = [j for j in (job(results, i) for i in ids[:limit] if i) if j is not None]
    return out


def next_queued(results: Path | str) -> dict[str, Any] | None:
    """The oldest job still waiting."""
    waiting = [j for j in jobs(results) if j["state"] == "queued"]
    return waiting[-1] if waiting else None


def set_state(results: Path | str, jid: str, **fields: Any) -> dict[str, Any]:
    path = queue_dir(results) / jid / "state.json"
    st = (read_json(path) or {"state": "queued"}) | fields
    write_json(path, st)
    return st


def cancel(results: Path | str, jid: str) -> str | None:
    """Cancel a job: a waiting one at once, a running one at the worker's next check.
    Returns its state after the request, or None if there is no such job."""
    j = job(results, jid)
    if j is None:
        return None
    if j["state"] == "queued":
        set_state(results, jid, state="cancelled", ended=now())
        return "cancelled"
    if j["state"] == "running":
        flag = queue_dir(results) / jid / "cancel"
        flag.touch()
        _adopt(flag)
    return j["state"]


def cancel_requested(results: Path | str, jid: str) -> bool:
    return (queue_dir(results) / jid / "cancel").exists()


# ------------------------------------------------------------------ the worker's heartbeat
def heartbeat(results: Path | str, **fields: Any) -> None:
    import socket

    write_json(
        bench_dir(results) / "worker.json",
        {"pid": os.getpid(), "host": socket.gethostname(), "at": time.time()} | fields,
    )


def worker(results: Path | str, stale_s: float = HEARTBEAT_STALE_S) -> dict[str, Any]:
    """The worker's last heartbeat, with ``alive``: whether it is recent, and
    ``stopping``: whether it was asked to stop."""
    hb = read_json(bench_dir(results) / "worker.json") or {}
    at = hb.get("at")
    alive = isinstance(at, (int, float)) and time.time() - at < stale_s
    return hb | {"alive": alive, "stopping": stop_requested(results)}


def request_stop(results: Path | str) -> None:
    """Ask the worker to stop. It exits once no spec is running: a running one finishes
    (or is cancelled) first."""
    flag = bench_dir(results) / "stop"
    make_dirs(flag.parent)
    flag.touch()
    _adopt(flag)


def stop_requested(results: Path | str) -> bool:
    return (bench_dir(results) / "stop").exists()


def clear_stop(results: Path | str) -> None:
    (bench_dir(results) / "stop").unlink(missing_ok=True)


def summary(results: Path | str, limit: int = 50) -> dict[str, Any]:
    """What a queue view shows: the worker, and the jobs, newest first."""
    return {"worker": worker(results), "jobs": jobs(results, limit)}
