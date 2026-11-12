"""The Benchmarks tab's backend: describe what can run, check a spec, queue it, and show
the queue and the rig.

Target *Local* writes the spec into the local queue, a folder under the benchmark
results that this container and the host share (``launch/localq.py`` in
tools/sim_benchmark). ``run_spec.py --work`` runs it on the host: a benchmark needs
the host's Docker and paths, which this container doesn't have. The worker also
writes the rig's checks (``rig.json``) there, which ``rig`` returns.

The form in the sim's menu is drawn from ``manifest``: each benchmark's settings as
``bench.yaml`` declares them, the bags and tracks there are, the missions. ``check``
runs the worker's own validation on a spec before it is queued, and ``launch`` refuses
a spec that fails it. The sim has no YAML reader, so ``presets`` hands it the preset
specs and the override files as JSON.

Target *Runner* (the bench PC, through GitHub) is not built yet.
"""

from __future__ import annotations

import copy
import importlib
import importlib.util
import os
import sys
import time
from pathlib import Path
from types import ModuleType
from typing import Any

import benchmark_control_runner as _bcr

TARGETS = ("local", "runner")
RIG_STALE_S = 180.0  # the worker refreshes rig.json every minute
# Settings the form doesn't show: launch sets mc_url to this backend.
FORM_HIDDEN = frozenset({"mc_url"})
PACKAGE = "ifssim_launch"


def results_root() -> Path:
    return Path(_bcr._results_root())


def _launch_module(name: str) -> ModuleType:
    """tools/sim_benchmark/launch/<name>.py. This container has tools/ (read-only) but
    not its packages, so launch/ is loaded as a package from its folder: spec.py's
    relative imports then work. The modules need only the standard library and PyYAML."""
    if PACKAGE not in sys.modules:
        pkg_dir = _bcr._tools_dir() / "sim_benchmark" / "launch"
        spec = importlib.util.spec_from_file_location(
            PACKAGE, pkg_dir / "__init__.py", submodule_search_locations=[str(pkg_dir)]
        )
        if spec is None or spec.loader is None:
            raise RuntimeError(f"no benchmark launch code at {pkg_dir}")
        pkg = importlib.util.module_from_spec(spec)
        sys.modules[PACKAGE] = pkg
        spec.loader.exec_module(pkg)
    return importlib.import_module(f"{PACKAGE}.{name}")


def localq() -> ModuleType:
    return _launch_module("localq")


# ------------------------------------------------------------------ the manifest
class SetupError(RuntimeError):
    """The manifest (bench.yaml) can't be read here."""


def manifest_file() -> Path:
    """bench.yaml: next to tools/ (compose mounts it there, read-only)."""
    env = os.environ.get("IFSSIM_BENCH_MANIFEST", "").strip()
    return Path(env) if env else _bcr._tools_dir().parent / "bench.yaml"


def tracks_dir() -> Path:
    env = os.environ.get("TRACKS_DIR", "").strip()
    return Path(env) if env else _bcr._tools_dir().parent / "Content" / "tracks"


def load_manifest() -> Any:
    """The manifest, with results and bags where this process sees them: the paths in
    bench.yaml are the host's, relative to the repository."""
    mf = _launch_module("manifest")
    path = manifest_file()
    if not path.is_file():
        raise SetupError(
            f"no bench.yaml at {path}: Mission Control's container needs it mounted "
            "(docker-compose.yml); START again to pick up the change"
        )
    try:
        m = mf.load(path.parent)
    except mf.ManifestError as e:
        raise SetupError(str(e)) from e
    res = results_root()

    def here(p: Path) -> Path:
        try:
            return res / p.relative_to(m.results)
        except ValueError:
            return p

    return m.with_paths(results=res, bag_dirs={k: here(v) for k, v in m.bag_dirs.items()})


def _missions() -> list[str]:
    import yaml

    path = _bcr._tools_dir() / "sim_benchmark" / "missions.yaml"
    try:
        doc = yaml.safe_load(path.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return []
    return [k for k, v in doc.items() if isinstance(v, dict)]


def manifest() -> dict[str, Any]:
    """What the form offers: each benchmark with its settings (bench.yaml), the bags of
    each kind, the missions and the tracks."""
    try:
        m = load_manifest()
    except SetupError as e:
        return {"error": str(e), "benchmarks": [], "bags": {}, "missions": [], "tracks": []}
    mf = _launch_module("manifest")
    benches = []
    for b in m.benchmarks.values():
        benches.append(
            {
                "name": b.name,
                "title": b.title,
                "description": b.description,
                "bags": b.bags,
                "parts": list(b.parts),
                "components": sorted(b.components()),
                "settings": [
                    {
                        "name": s.name,
                        "type": s.type,
                        "help": s.help,
                        "choices": list(s.choices),
                        "default": s.default,
                    }
                    for s in b.settings.values()
                    if s.name not in FORM_HIDDEN
                ],
            }
        )
    bags: dict[str, list[dict[str, Any]]] = {}
    for kind in m.bag_dirs:
        bags[kind] = []
        for name in m.bags(kind):
            info = mf.bag_info(m.bag_path(kind, name))
            bags[kind].append(
                {"name": name, "duration_s": info.duration_s, "size_bytes": info.size_bytes}
            )
    td = tracks_dir()
    tracks = sorted(p.name for p in td.glob("*.csv")) if td.is_dir() else []
    return {
        "error": None,
        "benchmarks": benches,
        "bags": bags,
        "missions": _missions(),
        "tracks": tracks,
    }


# ------------------------------------------------------------------ presets and overrides
def presets_dir() -> Path:
    # bench.yaml's `presets:`
    return _bcr._tools_dir() / "sim_benchmark" / "specs"


def overrides_dir() -> Path:
    return presets_dir() / "overrides"


def _about(text: str) -> str:
    """A spec file's leading comment, up to its first example or command."""
    lines = []
    for line in text.splitlines():
        if not line.startswith("#"):
            break
        line = line[1:].strip()
        if not line or line.endswith(":") or line.startswith(("python3 ", "--")):
            break
        lines.append(line)
    return " ".join(lines)


def _spec_files(folder: Path) -> list[tuple[Path, str, Any, str | None]]:
    """(path, text, parsed, error) for each folder/*.yaml."""
    import yaml

    out = []
    for path in sorted(folder.glob("*.yaml")) if folder.is_dir() else []:
        text = path.read_text()
        try:
            doc = yaml.safe_load(text)
        except yaml.YAMLError as e:
            out.append((path, text, None, f"{path.name}: {e}"))
            continue
        if not isinstance(doc, dict):
            out.append((path, text, None, f"{path.name}: a spec is a mapping"))
            continue
        out.append((path, text, doc, None))
    return out


def presets() -> dict[str, Any]:
    """The starting points for a spec (tools/sim_benchmark/specs/*.yaml) and the parameter
    override files (specs/overrides/*.yaml), as JSON for the sim's menu, which has no
    YAML reader. Each preset with the rig parts it needs (launch/rig.py)."""
    rig = _launch_module("rig")
    out = []
    for path, text, spec, err in _spec_files(presets_dir()):
        if err:
            out.append({"name": path.stem, "error": err})
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
    overrides = []
    for path, text, doc, err in _spec_files(overrides_dir()):
        if err:
            overrides.append({"name": path.stem, "error": err})
            continue
        pipeline = doc.get("pipeline")
        if not isinstance(pipeline, dict) or not pipeline:
            overrides.append({"name": path.stem, "error": f"{path.name}: no pipeline: section"})
            continue
        overrides.append({"name": path.stem, "about": _about(text), "pipeline": pipeline})
    return {"presets": out, "overrides": overrides}


# ------------------------------------------------------------------ checking and queueing
class LaunchError(ValueError):
    """A launch request that can't be queued (HTTP 400, or 501 for a target to come)."""

    def __init__(self, message: str, status_code: int = 400):
        super().__init__(message)
        self.status_code = status_code


def _for_this_backend(spec: dict[str, Any]) -> dict[str, Any]:
    """The spec with the mission benchmark pointed at this backend: its default is the
    web UI (:3000), which the in-sim panel doesn't need running."""
    spec = copy.deepcopy(spec)
    missions = (spec.get("benchmarks") or {}).get("missions")
    if isinstance(missions, dict):
        settings = missions.setdefault("settings", {}) or {}
        missions["settings"] = settings
        settings.setdefault("mc_url", os.environ.get("MC_SELF_URL", "http://127.0.0.1:8000"))
    return spec


def check(spec: Any) -> dict[str, Any]:
    """The worker's validation of a spec (launch/spec.py), before it is queued: every
    problem at once, or how many jobs it makes. ``needs``: the rig parts it needs."""
    rig = _launch_module("rig")
    if not isinstance(spec, dict):
        return {"ok": False, "errors": ["spec: a mapping"], "jobs": 0, "labels": [], "needs": []}
    needs = rig.needs(spec)
    try:
        m = load_manifest()
    except SetupError as e:
        return {"ok": False, "errors": [f"can't check it: {e}"], "jobs": 0, "labels": [], "needs": needs}
    sp = _launch_module("spec")
    try:
        s = sp.validate(_for_this_backend(spec), m)
        jobs = sp.expand(s, m)
    except sp.SpecError as e:
        return {"ok": False, "errors": str(e).splitlines(), "jobs": 0, "labels": [], "needs": needs}
    return {
        "ok": True,
        "errors": [],
        "jobs": len(jobs),
        "labels": [j.label() for j in jobs[:8]],
        "needs": needs,
    }


def launch(
    spec: Any, target: str = "local", requested_by: str | None = None, name: str | None = None
) -> dict[str, Any]:
    if target not in TARGETS:
        raise LaunchError(f"target: one of {', '.join(TARGETS)}")
    if target == "runner":
        raise LaunchError("the Runner target (the bench PC through GitHub) isn't built yet", 501)
    if not isinstance(spec, dict) or not isinstance(spec.get("benchmarks"), dict) or not spec["benchmarks"]:
        raise LaunchError("spec: a run spec with at least one benchmark")
    spec = _for_this_backend(spec)
    try:
        m = load_manifest()
    except SetupError:
        m = None  # can't check it here: the worker does when it takes it
    if m is not None:
        sp = _launch_module("spec")
        try:
            sp.validate(spec, m)
        except sp.SpecError as e:
            raise LaunchError(f"the spec has problems: {'; '.join(str(e).splitlines())}") from e
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


def stop_worker() -> dict[str, Any]:
    """Ask the worker to stop: it exits once no spec is running (cancel one to stop sooner)."""
    lq = localq()
    lq.request_stop(results_root())
    return {"stopping": True, "worker": lq.worker(results_root())}


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
