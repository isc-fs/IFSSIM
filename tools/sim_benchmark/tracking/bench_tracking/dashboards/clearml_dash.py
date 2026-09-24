"""ClearML dashboards: real ClearML **Reports** built from code.

For each dashboard:

1. a *dashboard task* in ``IFSSIM Bench/Dashboards`` holds the precomputed
   comparison figures (route overlays, cone maps, delta heatmaps, CIs...) as plotly
   plots, plus the runs table;
2. a *Report* (``reports.create`` in the server API) mixes markdown with embedded
   widgets (``/widgets/?type=...``):
   * ``type=scalar`` with many ``objects``: live overlay of the same series across
     tasks (ClearML's native compare, embedded). New runs can be added by editing the list.
   * ``type=plot``: a figure from the dashboard task.

The native compare view (select tasks → Compare) covers scalars, plots,
hyperparameters (incl. parallel coordinates) and single values. The project
dashboard ("Overview") can chart one summary metric over the project's tasks.
"""

from __future__ import annotations

import os
from urllib.parse import quote

import pandas as pd

from .. import figures as F
from ..backends.clearml_backend import ROOT, configure_from_env_file, scalar_title
from ..bundle import RunBundle
from ..suite import Suite
from .common import Dashboard, all_dashboards

PROJECT = f"{ROOT}/Dashboards"


def _widget(
    web: str,
    typ: str,
    objects: list[str],
    metric: str | None = None,
    variant: str | None = None,
    height: int = 420,
    xaxis: str | None = None,
) -> str:
    q = f"type={typ}&objectType=task"
    if xaxis:
        q += f"&xaxis={xaxis}"
    q += "".join(f"&objects={o}" for o in objects)
    if metric is not None:
        q += f"&metrics={quote(metric)}"
    if variant is not None:
        q += f"&variants={quote(variant)}"
    return f'<iframe src="{web}/widgets/?{q}" width="100%" height="{height}"></iframe>'


def _ids(state: dict[str, dict], runs: list[RunBundle]) -> list[str]:
    return [state[r.run_key]["run_id"] for r in runs if r.run_key in state]


def _live_overlays(
    d: Dashboard, suite: Suite, state: dict[str, dict], web: str
) -> list[str]:
    md: list[str] = []

    def scalar(
        title: str, runs: list[RunBundle], fam: str, key: str, step: str, note: str = ""
    ) -> None:
        ids = _ids(state, [r for r in runs if fam in r.series])
        if ids:
            md.append(
                f"\n\n### {title}\n\n{note}\n\n"
                + _widget(
                    web, "scalar", ids, scalar_title(fam, step), key, xaxis="iter"
                )
            )

    if d.slug == "bag-replay":
        played = [r for r in d.runs if r.status == "finished"]
        onboard = [r for r in played if r.job_type == "onboard_replay"]
        scalar(
            "Raw cone detections per scan",
            onboard,
            "replay_perception",
            "n_cones_raw",
            "t/replay_s",
            "Onboard (--report) runs, x = replay time in 0.1 s.",
        )
        scalar(
            "Cone count Δ vs baseline",
            onboard,
            "replay_perception_delta",
            "n_cones_raw_delta",
            "t/replay_s",
        )
        scalar(
            "SLAM − odom gap", onboard, "replay_pose", "slam_odom_gap_m", "t/replay_s"
        )
        scalar(
            "Autonomy steering", onboard, "replay_control", "steering_rad", "t/replay_s"
        )
        scalar(
            "Autonomy − pilot steering",
            onboard,
            "replay_control",
            "steer_residual_rad",
            "t/replay_s",
        )
        scalar(
            "CONE_FILTER accepted (all played runs incl. live)",
            played,
            "log_perception_filter",
            "accepted",
            "t/log_s",
        )
        scalar("SLAM processing time", played, "log_slam_latency", "proc_ms", "t/log_s")
        scalar("SLAM map size", played, "log_slam_profile", "map", "t/log_s")
        scalar("/Path publish rate", played, "log_planning_rate", "pub_hz", "t/log_s")
        scalar(
            "Speed seen by control", played, "log_control_status", "v_mps", "t/log_s"
        )
    elif d.slug == "sim-matrix":
        for scen in sorted({a.config["scenario"]["name"] for a in d.runs}):
            sa = [a for a in d.runs if a.config["scenario"]["name"] == scen]
            scalar(
                f"{scen}: mean |cross-track| vs lap distance (baseline vs candidate)",
                sa,
                "agg_track_profile",
                "cte_m_mean",
                "track/s_lap_m",
            )
            scalar(
                f"{scen}: mean speed vs lap distance",
                sa,
                "agg_track_profile",
                "speed_mps_mean",
                "track/s_lap_m",
            )
            seeds = [x for a in sa for x in suite.seeds_by_group().get(a.group, [])]
            scalar(
                f"{scen}: lap time per lap, every seed of both commits",
                seeds,
                "sim_laps",
                "lap_time_s",
                "lap",
            )
    elif d.slug == "nightly":
        scalar(
            "Mean |cross-track| vs lap distance per night",
            d.runs,
            "agg_track_profile",
            "cte_m_mean",
            "track/s_lap_m",
        )
    # type=single with several objects renders empty on server 2.x, so the runs table carries the summaries
    elif d.slug == "sweep":
        pass
    return md


def build(suite: Suite, state: dict[str, dict]) -> list[str]:
    configure_from_env_file()
    from clearml import Task
    from clearml.backend_api import Session

    web = os.environ["CLEARML_WEB_HOST"]
    session = Session()
    out = []
    for d in all_dashboards(suite):
        name = f"Dashboard · {d.title}"
        for old in Task.get_tasks(
            project_name=PROJECT, task_name=f"^{name}$", allow_archived=True
        ):
            old.delete(raise_on_error=False)
        task = Task.create(
            project_name=PROJECT, task_name=name, task_type=Task.TaskTypes.qc
        )
        task.mark_started(force=True)
        task.add_tags(["dashboard", d.slug])
        log = task.get_logger()
        metrics = F.headline_metrics(d.runs)
        runs_df = pd.DataFrame(
            [
                {
                    "run": r.name,
                    "status": r.status,
                    "task": state.get(r.run_key, {}).get("run_id"),
                    **{m: r.summary.get(m) for m in metrics},
                    **{
                        f"{m} Δ": r.summary.get(f"{m}.delta")
                        for m in metrics
                        if any(f"{m}.delta" in x.summary for x in d.runs)
                    },
                }
                for r in d.runs
            ]
        )
        log.report_table("overview", "runs table", iteration=0, table_plot=runs_df)
        for p in d.panels:
            log.report_plotly(title=p.section, series=p.key, figure=p.fig, iteration=0)
        log.flush(wait=True)
        task.flush(wait_for_uploads=True)
        task.mark_completed(force=True)

        md = [
            f"# {d.title}\n",
            _strip_html(d.intro),
            "\n## Runs\n",
            _widget(web, "plot", [task.id], "overview", "runs table", height=460),
            "\n## Live overlays (native ClearML widgets)\n",
            "These query the run tasks directly: they are ClearML's own compare charts, embedded.\n",
            *_live_overlays(d, suite, state, web),
            "\n## Comparisons\n",
            "Figures computed by bench_tracking from all runs (stored on the dashboard task).\n",
        ]
        section = None
        for p in d.panels:
            if p.section != section:
                md.append(f"\n\n### {p.section}\n\n")
                section = p.section
            md.append(
                _widget(
                    web,
                    "plot",
                    [task.id],
                    p.section,
                    p.key,
                    height=int(p.fig.layout.height or 450) + 30,
                )
                + "\n"
            )
        report_md = "\n".join(md)

        proj_id = task.project
        existing = session.send_request(
            "reports", "get_all_ex", json={"name": f"^{name}$", "only_fields": ["id"]}
        ).json()
        for r in existing.get("data", {}).get("tasks", []):
            session.send_request(
                "reports", "delete", json={"task": r["id"], "force": True}
            )
        resp = session.send_request(
            "reports",
            "create",
            json={
                "name": name,
                "project": proj_id,
                "comment": _strip_html(d.intro)[:500],
                "tags": ["dashboard", d.slug],
                "report": report_md,
            },
        ).json()
        rid = resp["data"]["id"]
        # the server ignores `report` on create; the body has to be set with update
        upd = session.send_request(
            "reports", "update", json={"task": rid, "report": report_md}
        ).json()
        if upd.get("meta", {}).get("result_code") != 200:
            raise RuntimeError(f"reports.update failed: {upd.get('meta')}")
        out.append(
            f"[clearml] {d.title}: {web}/reports/{proj_id}/{rid}  (task {task.id}, {len(d.panels)} panels)"
        )
    return out


def _strip_html(s: str) -> str:
    import re

    return re.sub(r"<[^>]+>", "", s).strip()
