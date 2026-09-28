"""Run specs: what to run, on what, with which settings and parameter overrides.

One format for every way of launching (the web form, PR and commit checks,
``bench-run`` on a laptop). Design: docs/history/2026-09-28_benchmark-launcher-design.md
§3 in IFSSIM. A spec, as YAML::

    schema: 1
    name: loop-closure gate retune
    notes: after #88
    code: {ifssim: dev, pipeline: feat/88-loop-closure, image: auto}
    benchmarks:
      sim_bag:
        bags: [trackdrive_track_20260404_013721_20260617_202851]   # or: all
        only: [perception, slam]
        repeats: 1
        settings: {gt_range_m: 20.0}
    compare_to: pinned
    pipeline:
      cone_detection: {residual_gate_mse: 0.05}
    sweep:
      pipeline.cone_detection.residual_gate_mse: [0.02, 0.05, 0.1]

Several sources merge in order (later wins, mappings merge, lists are
replaced): a preset, uploaded files, then single values (the form, or
``--set``). ``expand`` turns the result into jobs: one per sweep point,
benchmark, bag and repeat. Each job has a ``spec_id``: the hash of what
changes the measurement besides the code (settings, parts, overrides), or
``default`` when it sets none of them.
"""

from __future__ import annotations

import copy
import hashlib
import itertools
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .manifest import Manifest, ManifestError

SCHEMA = 1
DEFAULT_ID = "default"
TOP = {
    "schema",
    "name",
    "notes",
    "code",
    "benchmarks",
    "compare_to",
    "pipeline",
    "sweep",
}
PER_BENCHMARK = {"bags", "only", "repeats", "settings", "timeout_s"}
ALL_BAGS = "all"


class SpecError(ValueError):
    pass


# ------------------------------------------------------------------ sources
def load_file(path: Path | str) -> dict[str, Any]:
    try:
        doc = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as e:
        raise SpecError(f"{path}: {e}") from e
    if doc is None:
        return {}
    if not isinstance(doc, dict):
        raise SpecError(f"{path}: a spec is a mapping")
    return doc


def merge(*docs: dict[str, Any]) -> dict[str, Any]:
    """Later docs win; mappings merge key by key, anything else is replaced."""
    out: dict[str, Any] = {}
    for d in docs:
        out = _merge(out, d)
    return out


def _merge(a: Any, b: Any) -> Any:
    if isinstance(a, dict) and isinstance(b, dict):
        out = dict(a)
        for k, v in b.items():
            out[k] = _merge(a[k], v) if k in a else copy.deepcopy(v)
        return out
    return copy.deepcopy(b)


def assignment(text: str) -> dict[str, Any]:
    """``a.b.c=value`` (value read as YAML) -> ``{"a": {"b": {"c": value}}}``."""
    if "=" not in text:
        raise SpecError(f"--set {text!r}: expected key.path=value")
    key, raw = text.split("=", 1)
    return set_path({}, key.strip(), yaml.safe_load(raw) if raw.strip() else None)


def set_path(doc: dict[str, Any], dotted: str, value: Any) -> dict[str, Any]:
    parts = [p for p in dotted.split(".") if p]
    if not parts:
        raise SpecError(f"empty key in {dotted!r}")
    node = doc
    for p in parts[:-1]:
        node = node.setdefault(p, {})
        if not isinstance(node, dict):
            raise SpecError(f"{dotted}: {p} is not a mapping")
    node[parts[-1]] = value
    return doc


# --------------------------------------------------------------- validation
def validate(spec: dict[str, Any], m: Manifest) -> dict[str, Any]:
    """The spec, checked against the manifest and normalised (``bags: all`` resolved,
    numbers as floats). Every problem is listed at once."""
    errors: list[str] = []
    s = copy.deepcopy(spec)
    s.setdefault("schema", SCHEMA)
    if s["schema"] != SCHEMA:
        errors.append(f"schema: must be {SCHEMA}")
    for k in sorted(set(s) - TOP):
        errors.append(f"{k}: unknown key (known: {', '.join(sorted(TOP))})")

    code = s.get("code") or {}
    if not isinstance(code, dict):
        errors.append("code: a mapping of repository to branch, PR or commit")
        code = {}
    for k in sorted(set(code) - set(m.repos) - {"image"}):
        errors.append(
            f"code.{k}: not a repository in the manifest ({', '.join(m.repos)})"
        )

    benches = s.get("benchmarks")
    if not isinstance(benches, dict) or not benches:
        errors.append(
            "benchmarks: choose at least one "
            f"({', '.join(m.benchmarks) or 'the manifest lists none'})"
        )
        benches = {}
    used_components: set[str] = set()
    for name, b in benches.items():
        where = f"benchmarks.{name}"
        bench = m.benchmarks.get(name)
        if bench is None:
            errors.append(
                f"{where}: unknown benchmark (known: {', '.join(m.benchmarks)})"
            )
            continue
        b = benches[name] = dict(b or {})
        for k in sorted(set(b) - PER_BENCHMARK):
            errors.append(
                f"{where}.{k}: unknown key (known: {', '.join(sorted(PER_BENCHMARK))})"
            )

        only = b.get("only")
        if only is not None:
            if isinstance(only, str):
                only = b["only"] = [only]
            bad = [p for p in only if p not in bench.parts]
            if bad or not only:
                errors.append(
                    f"{where}.only: {', '.join(map(str, bad)) or 'empty'} "
                    f"(parts: {', '.join(bench.parts) or 'none'})"
                )
        used_components |= bench.components(only)

        if bench.bags:
            have = m.bags(bench.bags)
            bags = b.get("bags")
            if bags == ALL_BAGS:
                bags = b["bags"] = have
            if isinstance(bags, str):
                bags = b["bags"] = [bags]
            if not bags:
                errors.append(
                    f"{where}.bags: choose bags from {m.bag_dirs[bench.bags]} "
                    f"({len(have)} there), or 'all'"
                )
            else:
                missing = [x for x in bags if x not in have]
                if missing:
                    errors.append(
                        f"{where}.bags: not in {m.bag_dirs[bench.bags]}: {', '.join(missing)}"
                    )
        elif b.get("bags"):
            errors.append(f"{where}.bags: this benchmark takes no bags")

        reps = b.setdefault("repeats", 1)
        if isinstance(reps, bool) or not isinstance(reps, int) or reps < 1:
            errors.append(f"{where}.repeats: a whole number, at least 1")
        t = b.get("timeout_s")
        if t is not None and (
            isinstance(t, bool) or not isinstance(t, (int, float)) or t <= 0
        ):
            errors.append(f"{where}.timeout_s: seconds, more than 0")

        settings = b.get("settings") or {}
        if not isinstance(settings, dict):
            errors.append(f"{where}.settings: a mapping")
            settings = {}
        for k, v in list(settings.items()):
            st = bench.settings.get(k)
            if st is None:
                errors.append(
                    f"{where}.settings.{k}: unknown setting "
                    f"(known: {', '.join(bench.settings) or 'none'})"
                )
            elif v is not None:
                try:
                    settings[k] = st.check(v, f"{where}.settings.{k}")
                except ManifestError as e:
                    errors.append(str(e))
        b["settings"] = {k: v for k, v in settings.items() if v is not None}

    pipeline = s.get("pipeline") or {}
    if not isinstance(pipeline, dict):
        errors.append("pipeline: a mapping of component to parameters")
        pipeline = {}
    for comp, values in pipeline.items():
        if not isinstance(values, dict):
            errors.append(f"pipeline.{comp}: a mapping of parameter to value")
        elif comp not in used_components:
            errors.append(
                f"pipeline.{comp}: no chosen benchmark uses it "
                f"(they use: {', '.join(sorted(used_components)) or 'nothing'})"
            )
    s["pipeline"] = {k: v for k, v in pipeline.items() if v}

    sweep = s.get("sweep") or {}
    if not isinstance(sweep, dict):
        errors.append("sweep: a mapping of key.path to a list of values")
        sweep = {}
    for key, values in sweep.items():
        if not isinstance(values, list) or not values:
            errors.append(f"sweep.{key}: a non-empty list of values")
        if not key.startswith(("pipeline.", "benchmarks.")):
            errors.append(f"sweep.{key}: sweeps pipeline.* or benchmarks.*.settings.*")
    s["sweep"] = sweep

    if errors:
        raise SpecError("\n".join(errors))
    return s


# ------------------------------------------------------------------ jobs
@dataclass
class Job:
    benchmark: str
    bag: str | None
    repeat: int
    point: dict[str, Any]  # sweep values of this job ({} without a sweep)
    only: list[str] | None
    settings: dict[str, Any]
    pipeline: dict[str, dict[str, Any]]
    spec_id: str

    def label(self) -> str:
        bits = [self.benchmark, self.bag or ""]
        bits += [f"{k.rsplit('.', 1)[-1]}={v}" for k, v in self.point.items()]
        return " ".join(b for b in bits if b) + f" #{self.repeat}"

    def record(self, spec: dict[str, Any]) -> dict[str, Any]:
        """What goes into the run (spec.json): this job, and the spec it came from."""
        return {
            "schema": SCHEMA,
            "spec_id": self.spec_id,
            "name": spec.get("name"),
            "notes": spec.get("notes"),
            "benchmark": self.benchmark,
            "bag": self.bag,
            "repeat": self.repeat,
            "sweep_point": self.point,
            "only": self.only,
            "settings": self.settings,
            "pipeline": self.pipeline,
            "compare_to": spec.get("compare_to"),
            "spec": spec,
        }


def spec_id(
    benchmark: str,
    only: list[str] | None,
    settings: dict[str, Any],
    pipeline: dict[str, dict[str, Any]],
) -> str:
    if not only and not settings and not pipeline:
        return DEFAULT_ID
    basis = {
        "benchmark": benchmark,
        "only": sorted(only) if only else None,
        "settings": settings,
        "pipeline": pipeline,
    }
    blob = json.dumps(basis, sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha1(blob.encode()).hexdigest()[:8]


def expand(spec: dict[str, Any], m: Manifest) -> list[Job]:
    """A validated spec -> its jobs, in the order they run."""
    sweep = spec.get("sweep") or {}
    keys = list(sweep)
    points = [dict(zip(keys, vals)) for vals in itertools.product(*sweep.values())]
    jobs = []
    for point in points or [{}]:
        s = copy.deepcopy(spec)
        for k, v in point.items():
            set_path(s, k, v)
        s.pop("sweep", None)
        s = validate(s, m)  # sweep values are checked like any other
        for name, b in s["benchmarks"].items():
            bench = m.benchmarks[name]
            only = b.get("only")
            comps = bench.components(only)
            pipeline = {c: v for c, v in s["pipeline"].items() if c in comps}
            sid = spec_id(name, only, b["settings"], pipeline)
            for bag in b.get("bags") or [None]:
                for r in range(1, b["repeats"] + 1):
                    jobs.append(
                        Job(name, bag, r, point, only, b["settings"], pipeline, sid)
                    )
    return jobs
