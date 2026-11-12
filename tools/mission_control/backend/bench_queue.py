"""The Benchmarks tab's backend: queue a run spec, show the queue and the rig.

Target *Local* writes the spec into the local queue, a folder under the benchmark
results that this container and the host share (``launch/localq.py`` in
tools/sim_benchmark). ``run_spec.py --work`` runs it on the host: a benchmark needs
the host's Docker and paths, which this container doesn't have. The worker also
writes the rig's checks (``rig.json``) there, which ``rig`` returns.

Target *Runner* (the bench PC, through GitHub) is not built yet. ``presets`` hands the
sim's menu the spec presets as JSON.
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


_loaded: dict[str, ModuleType] = {}


def _launch_module(name: str) -> ModuleType:
    """tools/sim_benchmark/launch/<name>.py, loaded as a file: this container has tools/
    (read-only) but not its packages. Only for the modules that need nothing else of
    launch/ (localq: the standard library; rig: that and PyYAML)."""
    if name not in _loaded:
        path = _bcr._tools_dir() / "sim_benchmark" / "launch" / f"{name}.py"
        spec = importlib.util.spec_from_file_location(f"ifssim_{name}", path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"no {name} code at {path}")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _loaded[name] = mod
    return _loaded[name]


def localq() -> ModuleType:
    return _launch_module("localq")


def presets_dir() -> Path:
    # bench.yaml's `presets:`; bench.yaml itself isn't in this container
    return _bcr._tools_dir() / "sim_benchmark" / "specs"


def _about(text: str) -> str:
    """A preset's leading comment, up to its first example or command."""
    lines = []
    for line in text.splitlines():
        if not line.startswith("#"):
            break
        line = line[1:].strip()
        if not line or line.endswith(":") or line.startswith(("python3 ", "--")):
            break
        lines.append(line)
    return " ".join(lines)


def presets() -> dict[str, Any]:
    """The starting points for a spec (tools/sim_benchmark/specs/*.yaml), as JSON for the
    sim's menu, which has no YAML reader; each with the rig parts it needs (launch/rig.py)."""
    import yaml

    rig = _launch_module("rig")
    out = []
    for path in sorted(presets_dir().glob("*.yaml")):
        text = path.read_text()
        try:
            spec = yaml.safe_load(text)
        except yaml.YAMLError as e:
            out.append({"name": path.stem, "error": f"{path.name}: {e}"})
            continue
        if not isinstance(spec, dict):
            out.append({"name": path.stem, "error": f"{path.name}: a spec is a mapping"})
            continue
        out.append(
            {
                "name": path.stem,
                "title": spec.get("name") or path.stem,
                "about": _about(text),
                "spec": spec,
                "needs": rig.needs(spec),
            }
        )
    return {"presets": out}


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
