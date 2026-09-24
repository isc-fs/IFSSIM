"""Weights & Biases backend.

Mapping decisions:

* one project (``WANDB_PROJECT``, default ``ifssim-bench``). ``job_type``, ``group``
  (= scenario × commit) and tags carry the structure. Seeds of one group share
  the group, so native line plots can draw mean ± min/max bands per group.
* config: the nested config as-is (W&B flattens it into filterable columns).
* summary: every scalar, incl. ``.delta`` vs baseline.
* series: each family gets its own step metric via ``define_metric``
  (``t/replay_s``, ``track/s_m``, ``lap``...). Native panels overlay any
  selected runs against that x axis.
* tables: ``wandb.Table``. The XY overlays (routes, cone maps) are custom
  Vega charts over these tables (dashboards/wandb_dash.py). Each table
  carries ``run`` and ``series`` columns so rows from many runs can be merged
  and coloured without joins.
* figures: ``wandb.Plotly`` (interactive per-run deep dives).
* files: one ``run-files`` artifact per run. Track CSVs are ``track`` artifacts,
  bags are reference-only ``bag`` artifacts (``add_reference``, nothing uploaded).
* alerts: ``run.alert`` when a nightly aggregate regresses vs baseline.
"""

from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
from typing import Any

import wandb

from ..bundle import RunBundle
from .base import MAX_TABLE_ROWS_PER_GROUP, Backend, UploadResult, record, series_rows

PROJECT = os.environ.get("WANDB_PROJECT", "ifssim-bench")


def _run_id(b: RunBundle) -> str:
    return hashlib.sha1(
        f"{b.run_key}:{os.environ.get('BENCH_UPLOAD_SALT', '')}".encode()
    ).hexdigest()[:16]


class WandbBackend(Backend):
    name = "wandb"

    def __init__(
        self,
        *,
        entity: str | None = None,
        project: str = PROJECT,
        tracks_dir: Path | None = None,
    ) -> None:
        self.entity = entity or os.environ.get("WANDB_ENTITY")
        self.project = project
        self.tracks_dir = tracks_dir

    def upload(
        self,
        b: RunBundle,
        *,
        figures: dict[str, Any],
        parent: UploadResult | None = None,
    ) -> UploadResult:
        run = wandb.init(
            entity=self.entity,
            project=self.project,
            id=_run_id(b),
            name=b.name,
            group=b.group,
            job_type=b.job_type,
            tags=sorted(set(b.tags))[:60],
            config=b.config,
            notes=b.notes,
            reinit="create_new",
            resume="allow",
            settings=wandb.Settings(
                console="off",
                x_disable_stats=True,
                x_disable_meta=True,
                save_code=False,
            ),
        )
        try:
            self._log(run, b, figures)
            if parent:
                run.config.update({"parent_run": parent.run_id}, allow_val_change=True)
            if b.status != "finished":
                run.summary["run/status"] = b.status
            self._alerts(run, b)
        finally:
            run.finish(exit_code=0 if b.status == "finished" else 1)
        url = run.url if not run.offline else None
        res = UploadResult(self.name, run.id, url, b.name)
        record(
            b.source_dir
            if b.job_type in ("onboard_replay", "live_replay", "sim_e2e")
            else None,
            res,
        )
        return res

    # ------------------------------------------------------------------ logging
    def _log(self, run: "wandb.Run", b: RunBundle, figures: dict[str, Any]) -> None:
        # series: one step metric per family
        for fam, s in b.series.items():
            run.define_metric(s.step_name, hidden=False)
            run.define_metric(f"{fam}/*", step_metric=s.step_name)
        for fam, s in b.series.items():
            steps, vals = series_rows(s)
            for i, x in enumerate(steps):
                row = {s.step_name: x}
                for k, arr in vals.items():
                    if arr[i] is not None:
                        row[f"{fam}/{k}"] = arr[i]
                run.log(row)

        # tables (+ run/series columns so a merged multi-run table can be coloured)
        tables = dict(b.tables)
        if b.events:
            tables["events"] = b.events_table()
        for name, t in tables.items():
            t2 = t.decimated(
                MAX_TABLE_ROWS_PER_GROUP, by="source" if "source" in t.columns else None
            )
            cols = [*t2.columns, "run"]
            rows = [[*r, b.name] for r in t2.rows]
            if "source" in t2.columns:
                si = t2.columns.index("source")
                cols.append("series")
                rows = [[*r, f"{b.name.split('/', 1)[-1]} · {r[si]}"] for r in rows]
            run.log({f"tables/{name}": wandb.Table(columns=cols, data=rows)})

        # per-run deep-dive figures
        for key, fig in figures.items():
            run.log({f"figures/{key}": wandb.Plotly(fig)})
        if b.html_report:
            run.log(
                {
                    "report/onboard_html": wandb.Html(
                        b.html_report.read_text(), inject=False
                    )
                }
            )

        # summary (after history so history-derived summaries don't overwrite it)
        for k, v in b.summary.items():
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                run.summary[k] = float(v) if math.isfinite(float(v)) else None
            elif v is not None:
                run.summary[k] = v

        # files + lineage
        art = wandb.Artifact(
            f"files-{_run_id(b)}",
            type="run-files",
            description=f"Local run files of {b.name}",
            metadata={"run": b.name},
        )
        for p, kind in b.files:
            if p.is_file():
                art.add_file(str(p), name=f"{kind}/{p.name}")
        run.log_artifact(art)
        self._lineage(run, b)

    def _lineage(self, run: "wandb.Run", b: RunBundle) -> None:
        if run.offline:  # use_artifact needs the server
            return
        sc = b.config.get("scenario", {})
        bag = sc.get("bag") or {}
        if bag.get("id") and bag.get("path") and Path(bag["path"]).is_dir():
            # Reference only (checksum=False: nothing is read or uploaded). The URI
            # becomes the shared-storage one once Q4 (bag storage) is decided.
            a = wandb.Artifact(
                f"bag-{bag['name']}",
                type="bag",
                description="Reference only: bags are never uploaded",
                metadata={k: v for k, v in bag.items() if k != "topic_counts"},
            )
            a.add_reference(f"file://{bag['path']}", name="bag", checksum=False)
            run.use_artifact(a)
        if self.tracks_dir and sc.get("track_csv"):
            p = self.tracks_dir / sc["track_csv"]
            if p.is_file():
                a = wandb.Artifact(
                    f"track-{p.stem}",
                    type="track",
                    metadata={"length_m": sc.get("track_length_m")},
                )
                a.add_file(str(p))
                run.use_artifact(a)

    def _alerts(self, run: "wandb.Run", b: RunBundle) -> None:
        if "nightly" not in b.tags or b.job_type != "sim_aggregate":
            return
        rows = b.tables.get("baseline_comparison")
        bad = [r for r in (rows.rows if rows else []) if r[7] == "regression"]
        if bad and not run.offline:
            text = "\n".join(
                f"{r[0]}: {r[1]:.3g} vs {r[2]:.3g} (Δ {r[3]:+.3g}, tol {r[6]})"
                for r in bad[:12]
            )
            run.alert(
                title=f"Nightly regression: {len(bad)} metrics — {b.name}",
                text=text,
                level=wandb.AlertLevel.WARN,
                wait_duration=0,
            )
