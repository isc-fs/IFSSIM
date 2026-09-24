"""Read benchmark runs out of MLflow.

MLflow stays the system of record: runs, params, metrics, tags and artifacts
live there and its UI keeps working. This module is the viewer's only way in:

* :class:`Catalog`: one ``search_runs`` over the bench experiments gives every
  run's identity, config params, tags and summary metrics (≈1 s for 125 runs).
  Listing, filtering, KPI tables and seed statistics need nothing else.
* :func:`Catalog.bundle`: the full run (series, tables, events) for the plots.
  It reads the ``bundle/`` artifact (Parquet, full resolution) and caches it on
  disk. Runs logged without a bundle fall back to MLflow's own data: metric
  history (integer steps, decoded with the ``series_steps`` tag) and the
  ``tables/*.json`` artifacts.
* :func:`Catalog.pin_baseline`: the one write. It sets a tag on the runs so
  the baseline choice is stored in MLflow and visible to everyone.
"""

from __future__ import annotations

import json
import math
import os
import re
import threading
from collections import OrderedDict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

os.environ.setdefault("MLFLOW_DISABLE_AGENT_HINT", "1")

import mlflow  # noqa: E402
import numpy as np  # noqa: E402
from mlflow.tracking import MlflowClient  # noqa: E402

from .. import registry  # noqa: E402
from ..bundle import Event, RunBundle, Series, Table  # noqa: E402
from ..store import BUNDLE_DIR, read_bundle  # noqa: E402

EXPERIMENTS = [
    "ifssim-bench/replay",
    "ifssim-bench/sim-matrix",
    "ifssim-bench/sim-nightly",
    "ifssim-bench/sim-sweep",
]
CACHE = Path(
    os.environ.get(
        "BENCH_VIEW_CACHE", Path.home() / ".cache" / "bench_tracking" / "viewer"
    )
)
PIN_TAG = "bench.pinned_baseline"
SUFFIXES = (".std", ".min", ".max")

COLLECTIONS = {
    "replay": "Bag replays",
    "matrix": "Sim matrix",
    "nightly": "Nightly",
    "sweep": "Sweep",
}


def _unkey(k: str) -> str:
    return k.replace("_at_", "@")


@dataclass
class RunRow:
    """What the catalog knows about one run without downloading anything."""

    run_id: str
    name: str
    job_type: str
    collection: str | None  # replay | matrix | nightly | sweep | None (seeds)
    group: str
    scenario_id: str
    scenario: str  # human label: bag name or sim scenario name
    status: str
    started: datetime
    tags: frozenset[str]
    parent_id: str | None
    commit: str  # pipeline sha, or "?" for backfilled replays
    branch: str
    message: str
    params: dict[str, str]
    summary: dict[str, float]
    has_bundle: bool
    pinned: bool
    experiment_id: str

    @property
    def short(self) -> str:
        if self.collection == "replay":
            return self.name.split("/")[-1] + (
                " (live)" if self.job_type == "live_replay" else ""
            )
        if self.collection == "nightly":
            n = self.params.get("nightly.night")
            date = self.params.get("nightly.date") or self.started.strftime("%m-%d")
            return f"night {n} · {date} · {self.commit}" if n is not None else self.name
        if self.collection == "sweep":
            la = self.params.get("params.control.lookahead_gain")
            lat = self.params.get("params.control.max_lat_acc")
            return f"t{self.params.get('sweep.trial', '?')} · la={la} · lat={lat}"
        if self.collection == "matrix":
            return f"{self.scenario} · {self.commit}"
        return self.name.split("/", 1)[-1]

    @property
    def is_baseline_tagged(self) -> bool:
        return "baseline" in self.tags


def _collection(job: str, tags: frozenset[str]) -> str | None:
    if job in ("onboard_replay", "live_replay"):
        return "replay"
    if job == "sim_sweep_trial":
        return "sweep"
    if job == "sim_aggregate":
        return (
            "nightly" if "nightly" in tags else "matrix" if "matrix" in tags else None
        )
    return None


class Catalog:
    def __init__(self, uri: str | None = None) -> None:
        self.uri = uri or os.environ.get("MLFLOW_TRACKING_URI", "http://127.0.0.1:5005")
        mlflow.set_tracking_uri(self.uri)
        self.client = MlflowClient(self.uri)
        self._lock = threading.Lock()
        self._bundles: OrderedDict[str, RunBundle] = OrderedDict()
        self.rows: dict[str, RunRow] = {}
        self.loaded_at: datetime | None = None
        self.refresh()

    # ------------------------------------------------------------------ listing
    def refresh(self) -> None:
        df = mlflow.search_runs(experiment_names=EXPERIMENTS, max_results=50000)
        rows: dict[str, RunRow] = {}
        if not df.empty:
            mcols = [c for c in df.columns if c.startswith("metrics.")]
            pcols = [c for c in df.columns if c.startswith("params.")]
            tcols = [c for c in df.columns if c.startswith("tags.tag.")]
            for rec in df.to_dict("records"):
                tags = frozenset(
                    c[len("tags.tag.") :].replace(".", ":", 1)
                    for c in tcols
                    if rec.get(c) == "1"
                )
                job = rec.get("tags.job_type") or ""
                params = {
                    c[len("params.") :]: rec[c]
                    for c in pcols
                    if isinstance(rec.get(c), str)
                }
                summary = {}
                for c in mcols:
                    v = rec.get(c)
                    if v is None or (isinstance(v, float) and math.isnan(v)):
                        continue
                    k = _unkey(c[len("metrics.") :])
                    base = next((k[: -len(s)] for s in SUFFIXES if k.endswith(s)), k)
                    if registry.spec(base) is not None:
                        summary[k] = float(v)
                coll = _collection(job, tags)
                scen = (
                    params.get("scenario.name")
                    or params.get("scenario.bag.name", "").replace("_indexed", "")
                    or "?"
                )
                started = rec.get("start_time")
                started = (
                    started.to_pydatetime()
                    if hasattr(started, "to_pydatetime")
                    else datetime.now(timezone.utc)
                )
                rows[rec["run_id"]] = RunRow(
                    run_id=rec["run_id"],
                    name=rec.get("tags.mlflow.runName") or rec["run_id"],
                    job_type=job,
                    collection=coll,
                    group=rec.get("tags.group") or "",
                    scenario_id=rec.get("tags.scenario_id") or "",
                    scenario=scen,
                    status=rec.get("tags.status") or "finished",
                    started=started,
                    tags=tags,
                    parent_id=rec.get("tags.mlflow.parentRunId")
                    if isinstance(rec.get("tags.mlflow.parentRunId"), str)
                    else None,
                    commit=(params.get("code.pipeline.sha") or "?")[:7]
                    if params.get("code.pipeline.sha") not in (None, "", "None")
                    else "?",
                    branch=params.get("code.pipeline.branch") or "",
                    message=params.get("code.pipeline.message") or "",
                    params=params,
                    summary=summary,
                    has_bundle=rec.get("tags.bench.bundle") == "1",
                    pinned=rec.get(f"tags.{PIN_TAG}") == "1",
                    experiment_id=rec["experiment_id"],
                )
        with self._lock:
            self.rows = rows
            self.loaded_at = datetime.now(timezone.utc)

    def collection(self, name: str) -> list[RunRow]:
        return sorted(
            (r for r in self.rows.values() if r.collection == name),
            key=lambda r: (r.scenario, r.started),
        )

    def seeds(self, agg: RunRow) -> list[RunRow]:
        return sorted(
            (r for r in self.rows.values() if r.parent_id == agg.run_id),
            key=lambda r: int(r.params.get("scenario.seed") or 0),
        )

    def default_baseline(self, rows: list[RunRow]) -> RunRow | None:
        """Pinned (tag in MLflow) > tagged at import > earliest finished."""
        for pick in (
            lambda r: r.pinned,
            lambda r: r.is_baseline_tagged,
            lambda r: r.status == "finished",
        ):
            c = [r for r in rows if pick(r)]
            if c:
                return min(c, key=lambda r: r.started)
        return None

    def pin_baseline(self, run_id: str) -> None:
        """Store the choice in MLflow: tag this run, untag the others of its scenario and collection."""
        row = self.rows[run_id]
        for r in self.rows.values():
            if (
                r.collection == row.collection
                and r.scenario_id == row.scenario_id
                and r.pinned
                and r.run_id != run_id
            ):
                self.client.delete_tag(r.run_id, PIN_TAG)
        self.client.set_tag(run_id, PIN_TAG, "1")
        self.refresh()

    def url(self, run_id: str) -> str:
        r = self.rows.get(run_id)
        return f"{self.uri}/#/experiments/{r.experiment_id if r else 0}/runs/{run_id}"

    # ------------------------------------------------------------------ bundles
    def bundle(self, run_id: str) -> RunBundle:
        with self._lock:
            if run_id in self._bundles:
                self._bundles.move_to_end(run_id)
                return self._bundles[run_id]
        b = self._load(run_id)
        with self._lock:
            self._bundles[run_id] = b
            while len(self._bundles) > 80:
                self._bundles.popitem(last=False)
        return b

    def bundles(self, run_ids: list[str]) -> list[RunBundle]:
        from concurrent.futures import ThreadPoolExecutor

        with ThreadPoolExecutor(8) as ex:
            return list(ex.map(self.bundle, run_ids))

    def _load(self, run_id: str) -> RunBundle:
        d = CACHE / run_id / BUNDLE_DIR
        row = self.rows.get(run_id)
        if not (d / "meta.json").is_file():
            if row is None or row.has_bundle:
                try:
                    mlflow.artifacts.download_artifacts(
                        run_id=run_id,
                        artifact_path=BUNDLE_DIR,
                        dst_path=str(CACHE / run_id),
                        tracking_uri=self.uri,
                    )
                except Exception:  # noqa: BLE001  (no bundle artifact after all)
                    pass
        b = read_bundle(d) if (d / "meta.json").is_file() else self._from_native(run_id)
        b.run_id = run_id  # type: ignore[attr-defined]  (figures key colours and callbacks by MLflow run id)
        return b

    def _from_native(self, run_id: str) -> RunBundle:
        """Rebuild a (decimated) bundle from plain MLflow data: metric history + table artifacts."""
        run = self.client.get_run(run_id)
        tags, params = run.data.tags, run.data.params
        started = datetime.fromtimestamp(run.info.start_time / 1000, timezone.utc)
        b = RunBundle(
            job_type=tags.get("job_type", "unknown"),
            name=tags.get("mlflow.runName", run_id),
            group=tags.get("group", ""),
            source_dir=None,
            started_at=started,
            status=tags.get("status", "finished"),
            tags=[
                k[4:].replace(".", ":", 1)
                for k, v in tags.items()
                if k.startswith("tag.") and v == "1"
            ],
            config=_unflatten(params),
            notes=tags.get("mlflow.note.content", ""),
        )
        steps = {
            f: (s, float(k))
            for f, s, k in re.findall(
                r"(\w+): step = (\S+) × ([\d.e+]+)", tags.get("series_steps", "")
            )
        }
        fams: dict[str, dict[str, dict[int, float]]] = {}
        for key, val in run.data.metrics.items():
            fam, _, sub = key.partition("/")
            if fam in steps:
                hist = self.client.get_metric_history(run_id, key)
                fams.setdefault(fam, {})[_unkey(sub)] = {m.step: m.value for m in hist}
            else:
                b.summary[_unkey(key)] = val
        for fam, cols in fams.items():
            step_name, scale = steps[fam]
            xs = sorted({s for c in cols.values() for s in c})
            b.add_series(
                Series(
                    fam,
                    step_name,
                    np.array(xs, float) / scale,
                    {
                        k: np.array([c.get(s, np.nan) for s in xs], float)
                        for k, c in cols.items()
                    },
                )
            )
        try:
            for f in self.client.list_artifacts(run_id, "tables"):
                p = Path(
                    mlflow.artifacts.download_artifacts(
                        run_id=run_id,
                        artifact_path=f.path,
                        dst_path=str(CACHE / run_id / "native"),
                        tracking_uri=self.uri,
                    )
                )
                data = json.loads(p.read_text())
                name = p.stem
                t = Table(name, data["columns"], data["data"])
                if name == "events":
                    b.events = [Event(*r) for r in t.rows]
                else:
                    b.add_table(t)
        except Exception:  # noqa: BLE001
            pass
        b.tags.append("native-mlflow")
        return b


def _unflatten(params: dict[str, str]) -> dict:
    out: dict = {}
    for k, v in params.items():
        cur = out
        parts = k.split(".")
        for p in parts[:-1]:
            nxt = cur.setdefault(p, {})
            if not isinstance(nxt, dict):
                break
            cur = nxt
        else:
            try:
                cur[parts[-1]] = json.loads(v)
            except (ValueError, TypeError):
                cur[parts[-1]] = v
    return out
