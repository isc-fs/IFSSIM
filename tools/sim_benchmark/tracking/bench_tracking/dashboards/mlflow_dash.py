"""MLflow "dashboards".

MLflow has no dashboard or report object that can be built from code. The
closest shareable thing is a run, so each dashboard is a run in the
``ifssim-bench/dashboards`` experiment whose artifacts are:

* ``dashboard.html``: one scrollable page with a TOC, a runs table linking to each MLflow run
  (deltas vs baseline coloured), and every comparison figure (interactive plotly),
* ``figures/<section>/<key>.html``: the same figures one by one (the artifact viewer renders them),
* ``tables/runs.json``: the runs table as an MLflow table artifact.

The native MLflow compare view (select runs → Compare) adds parallel coordinates, scatter
and contour plots of params/metrics, and overlaid metric histories. Those are configured
in the UI, not from code.
"""

from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path

from mlflow.tracking import MlflowClient

from .. import figures as F
from ..suite import Suite
from .common import all_dashboards, dashboard_html

EXPERIMENT = "ifssim-bench/dashboards"


def build(suite: Suite, state: dict[str, dict]) -> list[str]:
    uri = os.environ.get("MLFLOW_TRACKING_URI", "http://127.0.0.1:5005")
    c = MlflowClient(uri)
    exp = c.get_experiment_by_name(EXPERIMENT)
    exp_id = (
        exp.experiment_id
        if exp
        else c.create_experiment(
            EXPERIMENT,
            tags={
                "mlflow.note.content": "One run per dashboard. Open a run → Artifacts → dashboard.html."
            },
        )
    )
    links = {k: v.get("url") for k, v in state.items()}
    out = []
    for d in all_dashboards(suite):
        name = f"Dashboard · {d.title}"
        for old in c.search_runs([exp_id], f"tags.dashboard = '{d.slug}'"):
            c.delete_run(old.info.run_id)
        run = c.create_run(
            exp_id,
            run_name=name,
            tags={"dashboard": d.slug, "mlflow.note.content": d.intro},
        )
        rid = run.info.run_id
        with tempfile.TemporaryDirectory() as tmp:
            page = Path(tmp) / "dashboard.html"
            page.write_text(dashboard_html(d, links, "MLflow"))
            c.log_artifact(rid, str(page))
        for p in d.panels:
            c.log_text(
                rid,
                p.fig.to_html(include_plotlyjs="cdn", full_html=True),
                f"figures/{p.section}/{p.key}.html",
            )
        metrics = F.headline_metrics(d.runs)
        c.log_table(
            rid,
            data={
                "run": [r.name for r in d.runs],
                "link": [links.get(r.run_key) for r in d.runs],
                "status": [r.status for r in d.runs],
                **{
                    m: [
                        (
                            r.summary.get(m)
                            if isinstance(r.summary.get(m), (int, float))
                            and math.isfinite(r.summary.get(m))
                            else None
                        )
                        for r in d.runs
                    ]
                    for m in metrics
                },
            },
            artifact_file="tables/runs.json",
        )
        c.set_terminated(rid)
        url = f"{uri}/#/experiments/{exp_id}/runs/{rid}/artifacts/dashboard.html"
        out.append(f"[mlflow] {d.title}: {url}  ({len(d.panels)} panels)")
    return out
