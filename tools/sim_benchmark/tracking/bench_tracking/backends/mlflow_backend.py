"""MLflow backend.

Mapping decisions (MLflow has fewer primitives than W&B/ClearML):

* experiments: ``ifssim-bench/<family>``: replay, sim-matrix, sim-nightly, sim-sweep
  (+ ``ifssim-bench/dashboards`` written by dashboards/mlflow_dash.py).
* seeds are **nested child runs** of their aggregate run (``mlflow.parentRunId``).
  That's MLflow's only grouping primitive, and it gives the collapsible tree in the UI.
* params: flattened config (values truncated at MLflow's 6000-char limit).
* summary: ``log_metrics`` (final values). Series: metric history. MLflow
  steps must be ints, so the x axis is encoded in the step (ms for time, dm for
  distance). The timestamp is also set to ``started_at + x`` so the UI's
  "Relative time" axis reads in real seconds for time series.
* tables -> ``log_table`` (JSON artifacts, rendered as tables in the artifact viewer).
* figures -> ``log_figure`` (plotly HTML artifacts, interactive in the viewer).
* the whole bundle at full resolution -> ``bundle/`` (Parquet + JSON, see store.py).
  This is what bench-view reads; the metric history above stays for MLflow's own UI.
"""

from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path
from typing import Any

import mlflow
from mlflow.entities import Metric
from mlflow.tracking import MlflowClient

from ..bundle import RunBundle, flatten
from ..store import BUNDLE_DIR, write_bundle
from .base import MAX_TABLE_ROWS_PER_GROUP, Backend, UploadResult, record, series_rows

# prefix -> multiplier to int step
STEP_SCALE = {
    "t/": 1000.0,
    "track/": 10.0,
    "lap": 1.0,
}


def experiment_for(b: RunBundle) -> str:
    if b.job_type in ("onboard_replay", "live_replay"):
        return "ifssim-bench/replay"
    if "sweep" in b.tags:
        return "ifssim-bench/sim-sweep"
    if "nightly" in b.tags:
        return "ifssim-bench/sim-nightly"
    return "ifssim-bench/sim-matrix"


def _scale(step_name: str) -> float:
    return next((v for k, v in STEP_SCALE.items() if step_name.startswith(k)), 1.0)


def _key(k: str) -> str:
    # MLflow keys: alphanumerics, _ - . space / :  -> '@' (recall@20m) isn't allowed
    return k.replace("@", "_at_")


class MlflowBackend(Backend):
    name = "mlflow"

    def __init__(self, tracking_uri: str | None = None) -> None:
        self.uri = tracking_uri or os.environ.get(
            "MLFLOW_TRACKING_URI", "http://127.0.0.1:5005"
        )
        mlflow.set_tracking_uri(self.uri)
        self.client = MlflowClient(self.uri)
        self._exp: dict[str, str] = {}

    def _experiment(self, name: str) -> str:
        if name not in self._exp:
            e = self.client.get_experiment_by_name(name)
            self._exp[name] = (
                e.experiment_id
                if e
                else self.client.create_experiment(
                    name, tags={"mlflow.note.content": EXPERIMENT_NOTES.get(name, "")}
                )
            )
        return self._exp[name]

    def upload(
        self,
        b: RunBundle,
        *,
        figures: dict[str, Any],
        parent: UploadResult | None = None,
    ) -> UploadResult:
        exp = self._experiment(experiment_for(b))
        start_ms = int(b.started_at.timestamp() * 1000)
        tags = {
            "job_type": b.job_type,
            "group": b.group,
            "scenario_id": b.scenario_id,
            "status": b.status,
            "mlflow.runName": b.name,
            "mlflow.note.content": b.notes,
            **{f"tag.{t.replace(':', '.')}": "1" for t in b.tags},
        }
        if parent:
            tags["mlflow.parentRunId"] = parent.run_id
        run = self.client.create_run(
            exp, start_time=start_ms, tags=tags, run_name=b.name
        )
        rid = run.info.run_id
        c = self.client

        # params: flattened config
        params = {
            k[:250]: ("" if v is None else str(v))[:6000]
            for k, v in flatten(b.config).items()
        }
        items = list(params.items())
        for i in range(0, len(items), 100):
            c.log_batch(
                rid, params=[mlflow.entities.Param(k, v) for k, v in items[i : i + 100]]
            )

        # summary metrics (step 0)
        ms = [
            Metric(_key(k), float(v), start_ms, 0)
            for k, v in b.summary.items()
            if isinstance(v, (int, float))
            and not isinstance(v, bool)
            and math.isfinite(float(v))
        ]
        for i in range(0, len(ms), 1000):
            c.log_batch(rid, metrics=ms[i : i + 1000])

        # series -> metric history
        for fam, s in b.series.items():
            steps, vals = series_rows(s)
            sc = _scale(s.step_name)
            batch: list[Metric] = []
            for k, arr in vals.items():
                key = _key(f"{fam}/{k}")
                for x, v in zip(steps, arr):
                    if v is None:
                        continue
                    ts = (
                        start_ms + int(x * 1000)
                        if s.step_name.startswith("t/")
                        else start_ms
                    )
                    batch.append(Metric(key, v, ts, int(round(x * sc))))
            for i in range(0, len(batch), 1000):
                c.log_batch(rid, metrics=batch[i : i + 1000])
        c.set_tag(
            rid,
            "series_steps",
            "; ".join(
                f"{f}: step = {s.step_name} × {_scale(s.step_name):g}"
                for f, s in b.series.items()
            )[:5000],
        )

        # tables
        tables = dict(b.tables)
        if b.events:
            tables["events"] = b.events_table()
        for name, t in tables.items():
            t2 = t.decimated(
                MAX_TABLE_ROWS_PER_GROUP, by="source" if "source" in t.columns else None
            )
            data = {
                col: [row[i] for row in t2.rows] for i, col in enumerate(t2.columns)
            }
            c.log_table(rid, data=data, artifact_file=f"tables/{name}.json")

        # figures (interactive plotly html, plotly.js from the CDN so each file stays small)
        for key, fig in figures.items():
            c.log_text(
                rid,
                fig.to_html(include_plotlyjs="cdn", full_html=True),
                f"figures/{key}.html",
            )

        attach_bundle(c, rid, b)

        # files
        if b.html_report:
            c.log_artifact(rid, str(b.html_report), "report")
        for p, kind in b.files:
            if p.is_file() and p != b.html_report:
                c.log_artifact(rid, str(p), f"files/{kind}")

        c.set_terminated(
            rid,
            status={"finished": "FINISHED", "failed": "FAILED", "aborted": "KILLED"}[
                b.status
            ],
            end_time=start_ms
            + int(
                float(
                    b.summary.get("run/replay_duration_s")
                    or (b.series["sim_time"].step[-1] if "sim_time" in b.series else 0)
                )
                * 1000
            ),
        )
        res = UploadResult(
            self.name, rid, f"{self.uri}/#/experiments/{exp}/runs/{rid}", b.name
        )
        record(
            b.source_dir
            if b.job_type not in ("sim_aggregate", "sim_sweep_trial")
            else None,
            res,
        )
        return res


def attach_bundle(c: MlflowClient, run_id: str, b: RunBundle) -> None:
    """Log the full-resolution bundle under ``bundle/`` and tag the run so viewers can find it."""
    with tempfile.TemporaryDirectory() as tmp:
        c.log_artifacts(
            run_id, str(write_bundle(b, Path(tmp) / BUNDLE_DIR)), BUNDLE_DIR
        )
    c.set_tag(run_id, "bench.bundle", "1")


EXPERIMENT_NOTES = {
    "ifssim-bench/replay": "Onboard bag replays through the live pipeline (bag benchmarks + --live runs). "
    "Series steps: t/* in ms, track/* in dm.",
    "ifssim-bench/sim-matrix": "MOCK simulator-in-the-loop scenario x commit x seed matrix. "
    "Aggregate runs are parents; seeds are nested children.",
    "ifssim-bench/sim-nightly": "MOCK nightly regression suite (3 seeds / night).",
    "ifssim-bench/sim-sweep": "MOCK parameter sweep (lookahead gain x max lateral acceleration).",
}
