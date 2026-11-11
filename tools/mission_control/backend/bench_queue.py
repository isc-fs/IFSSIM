"""The Benchmarks tab's backend: queue a run spec, show the queue and the rig.

Target *Local* writes the spec into the local queue, a folder under the benchmark
results that this container and the host share (``launch/localq.py`` in
tools/sim_benchmark). ``run_spec.py --work`` runs it on the host: a benchmark needs
the host's Docker and paths, which this container doesn't have. The worker also
writes the rig's checks (``rig.json``) there, which ``rig`` returns.

Target *Runner* (the bench PC, through GitHub) is not built yet.
"""

from __future__ import annotations

import copy
import importlib.util
import os
import time
from pathlib import Path
from types import ModuleType
from typing import Any

import benchmark_control_runner as _bcr

TARGETS = ("local", "runner")
RIG_STALE_S = 180.0  # the worker refreshes rig.json every minute


def results_root() -> Path:
    return Path(_bcr._results_root())


_localq: ModuleType | None = None


def localq() -> ModuleType:
    """tools/sim_benchmark/launch/localq.py, loaded as a file: it needs only the standard
    library, and this container has tools/ (read-only) but not its packages."""
    global _localq
    if _localq is None:
        path = _bcr._tools_dir() / "sim_benchmark" / "launch" / "localq.py"
        spec = importlib.util.spec_from_file_location("ifssim_localq", path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"no local queue code at {path}")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _localq = mod
    return _localq


class LaunchError(ValueError):
    """A launch request that can't be queued (HTTP 400, or 501 for a target to come)."""

    def __init__(self, message: str, status_code: int = 400):
        super().__init__(message)
        self.status_code = status_code


def launch(
    spec: Any, target: str = "local", requested_by: str | None = None, name: str | None = None
) -> dict[str, Any]:
    if target not in TARGETS:
        raise LaunchError(f"target: one of {', '.join(TARGETS)}")
    if target == "runner":
        raise LaunchError("the Runner target (the bench PC through GitHub) isn't built yet", 501)
    if not isinstance(spec, dict) or not isinstance(spec.get("benchmarks"), dict) or not spec["benchmarks"]:
        raise LaunchError("spec: a run spec with at least one benchmark")
    spec = copy.deepcopy(spec)
    missions = spec["benchmarks"].get("missions")
    if isinstance(missions, dict):
        # the mission benchmark drives this backend; its default is the web UI (:3000),
        # which the in-sim panel doesn't need running
        settings = missions.setdefault("settings", {}) or {}
        missions["settings"] = settings
        settings.setdefault("mc_url", os.environ.get("MC_SELF_URL", "http://127.0.0.1:8000"))
    lq = localq()
    jid = lq.enqueue(
        results_root(),
        spec,
        requested_by=requested_by or "mission_control",
        trigger="mission_control",
        name=name,
    )
    worker = lq.worker(results_root())
    return {
        "id": jid,
        "target": target,
        "worker_running": worker["alive"],
        # the spec runs only once a worker takes it: say how to start one
        "hint": None
        if worker["alive"]
        else "no worker is running: start one on this machine with "
        "`python3 tools/sim_benchmark/run_spec.py --work`",
    }


def queue(limit: int = 50) -> dict[str, Any]:
    return localq().summary(results_root(), limit)


def cancel(job_id: str) -> str | None:
    if "/" in job_id or job_id.startswith("."):
        return None
    return localq().cancel(results_root(), job_id)


def rig() -> dict[str, Any]:
    """The rig's last checks, as the worker wrote them, with how old they are. Stale
    (no worker, or old checks) means the boxes can't be trusted: show them grey."""
    lq = localq()
    status = lq.read_json(lq.bench_dir(results_root()) / "rig.json")
    if status is None:
        return {"known": False, "stale": True, "reason": "no worker has checked this rig yet"}
    age = time.time() - float(status.get("checked_at") or 0)
    alive = lq.worker(results_root())["alive"]
    stale = age > RIG_STALE_S or not alive
    return status | {
        "known": True,
        "age_s": round(age, 1),
        "stale": stale,
        "reason": None if not stale else ("the worker isn't running" if not alive else "old checks"),
    }
