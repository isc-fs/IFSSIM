"""ClearML backend (self-hosted server, see deploy/clearml).

Mapping decisions:

* projects: ``IFSSIM Bench/Bag replay``, ``IFSSIM Bench/Sim matrix`` (+ ``/Seeds``),
  ``IFSSIM Bench/Sim nightly`` (+ ``/Seeds``), ``IFSSIM Bench/Sim sweep`` (+ ``/Seeds``).
  Aggregates sit in the main project, their seeds in the ``Seeds`` subproject with
  ``parent`` set to the aggregate task.
* tasks are created offline-style (``Task.create`` + ``mark_started``) so the
  harness's own git state is never captured as the run's code.
* hyperparameters: sections ``Scenario`` / ``Code`` / ``Params`` / ``Env`` / ``Provenance``.
  The compare view diffs these, and the parallel-coordinates widget reads them.
* summary -> ``report_single_value`` (the project table columns + compare "Summary").
* series -> ``report_scalar``. The iteration is the x axis, encoded as an int: 0.1 s for
  time, 1 m for distance, 1 for laps. The unit is in the graph title. The compare view
  overlays these natively.
* tables -> ``report_table``, figures -> ``report_plotly``, report.html -> debug sample,
  files -> artifacts on the fileserver.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import pandas as pd

from ..bundle import RunBundle, flatten
from .base import MAX_TABLE_ROWS_PER_GROUP, Backend, UploadResult, record, series_rows

ROOT = "IFSSIM Bench"
X_ENCODING = {
    "t/": (10.0, "x = 0.1 s"),
    "track/": (1.0, "x = m"),
    "lap": (1.0, "x = lap"),
}


def project_for(b: RunBundle, *, seed: bool = False) -> str:
    if b.job_type in ("onboard_replay", "live_replay"):
        return f"{ROOT}/Bag replay"
    fam = (
        "Sim sweep"
        if "sweep" in b.tags
        else "Sim nightly"
        if "nightly" in b.tags
        else "Sim matrix"
    )
    return f"{ROOT}/{fam}" + ("/Seeds" if seed else "")


def x_encoding(step_name: str) -> tuple[float, str]:
    return next(
        (v for k, v in X_ENCODING.items() if step_name.startswith(k)), (1.0, "x = step")
    )


def scalar_title(family: str, step_name: str) -> str:
    return f"{family} [{x_encoding(step_name)[1]}]"


def configure_from_env_file() -> None:
    """Point the SDK at the local server in deploy/clearml unless already configured."""
    env = Path(__file__).resolve().parents[2] / "deploy" / "clearml" / ".env"
    vals = {}
    if env.is_file():
        for line in env.read_text().splitlines():
            if "=" in line and not line.lstrip().startswith("#"):
                k, v = line.split("=", 1)
                vals[k.strip()] = v.strip()
    os.environ.setdefault(
        "CLEARML_API_HOST", f"http://localhost:{vals.get('CLEARML_API_PORT', '8008')}"
    )
    os.environ.setdefault(
        "CLEARML_WEB_HOST", f"http://localhost:{vals.get('CLEARML_WEB_PORT', '8090')}"
    )
    os.environ.setdefault(
        "CLEARML_FILES_HOST",
        f"http://localhost:{vals.get('CLEARML_FILES_PORT', '8081')}",
    )
    for k in ("CLEARML_API_ACCESS_KEY", "CLEARML_API_SECRET_KEY"):
        if k in vals:
            os.environ.setdefault(k, vals[k])


class ClearMLBackend(Backend):
    name = "clearml"

    def __init__(self) -> None:
        configure_from_env_file()
        self.web = os.environ["CLEARML_WEB_HOST"]

    def upload(
        self,
        b: RunBundle,
        *,
        figures: dict[str, Any],
        parent: UploadResult | None = None,
    ) -> UploadResult:
        from clearml import Task  # after configure_from_env_file()

        is_seed = b.job_type == "sim_e2e"
        task = Task.create(
            project_name=project_for(b, seed=is_seed),
            task_name=b.name,
            task_type=Task.TaskTypes.qc
            if b.job_type in ("sim_aggregate", "sim_sweep_trial")
            else Task.TaskTypes.testing,
        )
        task.mark_started(force=True)
        if parent:
            task.set_parent(parent.run_id)
        task.add_tags(
            sorted(set(b.tags) | {f"job:{b.job_type}", f"scenario:{b.scenario_id}"})
        )
        task.set_comment(
            f"{b.notes}\n\ngroup: {b.group}\nstarted_at: {b.started_at.isoformat()}"
        )
        cfg = b.config
        sections = {
            "Scenario": flatten(cfg.get("scenario", {})),
            "Code": flatten(cfg.get("code", {})),
            "Params": flatten(cfg.get("params", {})) or {"(none)": "unknown"},
            "Env": flatten(cfg.get("env", {})),
            "Provenance": flatten(
                {
                    k: cfg[k]
                    for k in ("provenance", "compare", "sweep", "nightly", "referee")
                    if cfg.get(k) is not None
                }
            ),
        }
        task.set_parameters_as_dict(
            {
                s: {k: ("" if v is None else v) for k, v in d.items()}
                for s, d in sections.items()
            }
        )
        log = task.get_logger()

        for k, v in b.summary.items():
            if (
                isinstance(v, (int, float))
                and not isinstance(v, bool)
                and math.isfinite(float(v))
            ):
                log.report_single_value(k, float(v))

        for fam, s in b.series.items():
            steps, vals = series_rows(s)
            scale, _ = x_encoding(s.step_name)
            title = scalar_title(fam, s.step_name)
            last_it = None
            for i, x in enumerate(steps):
                it = int(round(x * scale))
                if it == last_it:  # two samples in one iteration bucket: keep the first
                    continue
                last_it = it
                for k, arr in vals.items():
                    if arr[i] is not None:
                        log.report_scalar(title, k, arr[i], iteration=it)

        tables = dict(b.tables)
        if b.events:
            tables["events"] = b.events_table()
        for name, t in tables.items():
            t2 = t.decimated(
                MAX_TABLE_ROWS_PER_GROUP if name != "trajectory" else 600,
                by="source" if "source" in t.columns else None,
            )
            log.report_table(
                f"table/{name}",
                t.description or name,
                iteration=0,
                table_plot=pd.DataFrame(t2.rows, columns=t2.columns),
            )

        for key, fig in figures.items():
            section, name = key.split("/", 1)
            log.report_plotly(title=section, series=name, figure=fig, iteration=0)

        if b.html_report:
            log.report_media(
                "report",
                "onboard report.html",
                iteration=0,
                local_path=str(b.html_report),
            )
        for p, kind in b.files:
            if p.is_file():
                task.upload_artifact(
                    f"{kind}/{p.name}", artifact_object=p, wait_on_upload=False
                )

        log.flush(wait=True)
        task.flush(wait_for_uploads=True)
        if b.status == "finished":
            task.mark_completed(force=True)
        elif b.status == "failed":
            task.mark_failed(status_reason="run failed (DNF / crash)", force=True)
        else:
            task.mark_stopped(force=True, status_message="aborted before bag play")
        res = UploadResult(
            self.name, task.id, f"{self.web}/projects/*/experiments/{task.id}", b.name
        )
        record(
            b.source_dir
            if is_seed or b.job_type in ("onboard_replay", "live_replay")
            else None,
            res,
        )
        return res
