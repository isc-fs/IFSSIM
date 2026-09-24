"""W&B dashboards: saved workspace views + Reports, built from code.

* **Workspace views** (``wandb_workspaces.workspaces``): one per benchmark family,
  with sections of *native* panels that overlay whichever runs are selected:
  line plots on the family's step metric (replay time / track distance / lap),
  bar charts of summary metrics, run comparer, parallel coordinates, media
  browser for the per-run plotly deep dives, and custom Vega charts over the
  logged tables for XY overlays (routes, cone maps, penalty locations).
  The runset has the baseline pinned (``baseline_run``), so the runs table
  shows native delta columns.
* **Reports** (``wandb_workspaces.reports.v2``): curated, commented comparisons with
  fixed run sets. These are the artefacts to share in a PR or a meeting.
* **Dashboard runs** (``job_type=dashboard``): the precomputed comparison
  figures (delta heatmaps, bootstrap CIs, ...) that no native panel can draw.
  They are the same figures MLflow/ClearML get, shown in the Reports through a media browser.

Custom chart presets are registered through the same GraphQL ``upsertView``
mutation wandb-workspaces uses for views (type ``vega2-panel``). If the server
refuses, the panels fall back to the built-in ``wandb/scatter/v0`` preset.
"""

from __future__ import annotations

import json
import os

import wandb
import wandb_workspaces.reports.v2 as wr
import wandb_workspaces.workspaces as ws
from wandb_workspaces import expr
from wandb_workspaces._graphql import execute_graphql

from ..suite import Suite
from .common import (
    NIGHTLY_METRICS,
    REPLAY_BAR_METRICS,
    SIM_MATRIX_METRICS,
    all_dashboards,
)

PROJECT = os.environ.get("WANDB_PROJECT", "ifssim-bench")

# ------------------------------------------------------------------ vega presets
XY_LINES = {
    "$schema": "https://vega.github.io/schema/vega-lite/v4.json",
    "description": "XY paths of many runs, one line per series, drawn in time order",
    "data": {"name": "wandb"},
    "title": "${string:title}",
    "width": "container",
    "height": 520,
    "mark": {"type": "line", "strokeWidth": 1.4, "clip": True},
    "encoding": {
        "x": {
            "field": "${field:x}",
            "type": "quantitative",
            "scale": {"zero": False},
            "title": "x [m]",
        },
        "y": {
            "field": "${field:y}",
            "type": "quantitative",
            "scale": {"zero": False},
            "title": "y [m]",
        },
        "color": {
            "field": "${field:series}",
            "type": "nominal",
            "legend": {"orient": "bottom", "columns": 2},
        },
        "order": {"field": "${field:order}", "type": "quantitative"},
        "tooltip": [
            {"field": "${field:series}", "type": "nominal"},
            {"field": "${field:order}", "type": "quantitative"},
        ],
    },
}
XY_POINTS = {
    "$schema": "https://vega.github.io/schema/vega-lite/v4.json",
    "description": "XY points of many runs, colour = series, shape = kind",
    "data": {"name": "wandb"},
    "title": "${string:title}",
    "width": "container",
    "height": 520,
    "mark": {"type": "point", "filled": True, "size": 40, "opacity": 0.75},
    "encoding": {
        "x": {"field": "${field:x}", "type": "quantitative", "scale": {"zero": False}},
        "y": {"field": "${field:y}", "type": "quantitative", "scale": {"zero": False}},
        "color": {
            "field": "${field:color}",
            "type": "nominal",
            "legend": {"orient": "bottom", "columns": 2},
        },
        "shape": {"field": "${field:shape}", "type": "nominal"},
        "tooltip": [
            {"field": "${field:color}", "type": "nominal"},
            {"field": "${field:shape}", "type": "nominal"},
            {"field": "${field:detail}", "type": "nominal"},
        ],
    },
}


def register_presets(api: wandb.Api, entity: str) -> dict[str, str]:
    q = """mutation UpsertPreset($entityName: String, $name: String, $displayName: String, $spec: String!, $type: String) {
      upsertView(input: {entityName: $entityName, name: $name, displayName: $displayName, type: $type, spec: $spec,
                         createdUsing: WANDB_SDK}) { view { id name } inserted } }"""
    out = {}
    for name, spec in (("ifssim-xy-lines", XY_LINES), ("ifssim-xy-points", XY_POINTS)):
        try:
            execute_graphql(
                api,
                q,
                {
                    "entityName": entity,
                    "name": name,
                    "displayName": name,
                    "type": "vega2-panel",
                    "spec": json.dumps(
                        {
                            "name": name,
                            "description": spec["description"],
                            "spec": json.dumps(spec),
                            "access": "PRIVATE",
                        }
                    ),
                },
            )
            out[name] = f"{entity}/{name}"
        except Exception as e:  # noqa: BLE001 — fall back, but say so
            print(
                f"  [wandb] preset {name} not registered ({e}); falling back to wandb/scatter/v0"
            )
            out[name] = "wandb/scatter/v0"
    return out


def xy_chart(
    presets,
    table: str,
    title: str,
    *,
    lines: bool = True,
    shape: str = "source",
    color: str = "series",
):
    if lines and presets["ifssim-xy-lines"] != "wandb/scatter/v0":
        return wr.CustomChart(
            query={"summaryTable": {"tableKey": table}},
            chart_name=presets["ifssim-xy-lines"],
            chart_fields={"x": "x", "y": "y", "series": "series", "order": "t_s"},
            chart_strings={"title": title},
        )
    if presets["ifssim-xy-points"] != "wandb/scatter/v0":
        return wr.CustomChart(
            query={"summaryTable": {"tableKey": table}},
            chart_name=presets["ifssim-xy-points"],
            chart_fields={
                "x": "x",
                "y": "y",
                "color": color,
                "shape": shape,
                "detail": "run",
            },
            chart_strings={"title": title},
        )
    return wr.CustomChart(
        query={"summaryTable": {"tableKey": table}},
        chart_name="wandb/scatter/v0",
        chart_fields={"x": "x", "y": "y"},
        chart_strings={"title": title},
    )


def lp(title, x, ys, **kw):
    return wr.LinePlot(title=title, x=x, y=ys, **kw)


def bars(title, metrics):
    return wr.BarPlot(
        title=title, metrics=metrics, orientation="v", max_runs_to_show=30
    )


# ------------------------------------------------------------------ dashboard runs
def log_dashboard_runs(suite: Suite, entity: str) -> dict[str, list[str]]:
    keys = {}
    for d in all_dashboards(suite):
        run = wandb.init(
            entity=entity,
            project=PROJECT,
            job_type="dashboard",
            name=f"dashboard/{d.slug}",
            id=f"dashboard-{d.slug}",
            resume="allow",
            reinit="create_new",
            config={"dashboard": d.slug, "job_type": "dashboard"},
            tags=["dashboard"],
            settings=wandb.Settings(console="off", x_disable_stats=True),
        )
        ks = []
        for p in d.panels:
            k = f"cmp/{p.section}/{p.key}"
            run.log({k: wandb.Plotly(p.fig)})
            ks.append(k)
        run.finish()
        keys[d.slug] = ks
    return keys


# ------------------------------------------------------------------ workspace views
def replay_view(entity, presets, baseline_id):
    sec = lambda n, ps, open_=True: ws.Section(name=n, panels=ps, is_open=open_)  # noqa: E731
    return ws.Workspace(
        entity=entity,
        project=PROJECT,
        name="Bag replay",
        settings=ws.WorkspaceSettings(
            x_axis="t/replay_s", max_runs=20, tooltip_number_of_runs="all_runs"
        ),
        runset_settings=ws.RunsetSettings(
            filters="Config('job_type') in ['onboard_replay', 'live_replay']",
            baseline_run=baseline_id,
            pinned_columns=[f"summary:{m}" for m in REPLAY_BAR_METRICS[:10]],
            order=[expr.Ordering(expr.Config("env.started_at"), ascending=True)],
        ),
        sections=[
            sec(
                "Overview",
                [
                    wr.MarkdownPanel(
                        markdown=(
                            "**Bag replays** of `manual_20260920_154527` (same bag → routes overlay directly). "
                            "Runs table: the pinned baseline gives Δ columns. Live runs have log-derived data only."
                        )
                    ),
                    bars(
                        "Perception",
                        [
                            "perception/n_cones_mean",
                            "perception/empty_frame_rate_frac",
                            "perception/filter_rate_mean_hz",
                            "perception/n_dbscan_guard_trips",
                        ],
                    ),
                    bars(
                        "SLAM",
                        [
                            "slam/n_map_cones",
                            "slam/end_gap_vs_odom_m",
                            "slam/odom_divergence_max_m",
                            "slam/n_pose_jump_rejected",
                            "slam/n_da_failure_skips",
                        ],
                    ),
                    bars(
                        "SLAM compute",
                        [
                            "slam/proc_p50_ms",
                            "slam/proc_p95_ms",
                            "slam/prof_total_p95_ms",
                        ],
                    ),
                    bars(
                        "Planning / control",
                        [
                            "planning/pub_rate_mean_hz",
                            "planning/n_plan_empty",
                            "control/steer_residual_mean_abs_rad",
                            "control/steer_rate_rms_radps",
                        ],
                    ),
                    bars(
                        "Health",
                        [
                            "run/startup_s",
                            "odom/t_calibrated_s",
                            "run/n_warnings",
                            "run/n_errors",
                            "run/n_crashes",
                            "compare/n_regressions",
                        ],
                    ),
                    wr.RunComparer(diff_only="split"),
                ],
            ),
            sec(
                "Trajectory & map",
                [
                    xy_chart(
                        presets,
                        "tables/trajectory",
                        "Routes: odom + SLAM of selected runs",
                    ),
                    xy_chart(
                        presets,
                        "tables/map_cones",
                        "Final SLAM maps",
                        lines=False,
                        color="run",
                        shape="source",
                    ),
                    lp(
                        "SLAM − odom gap", "t/replay_s", ["replay_pose/slam_odom_gap_m"]
                    ),
                    lp(
                        "SLAM − odom heading",
                        "t/replay_s",
                        ["replay_pose/slam_odom_dyaw_deg"],
                    ),
                    lp(
                        "Odom speed",
                        "t/replay_s",
                        ["replay_pose/odom_speed_mps"],
                        smoothing_type="gaussian",
                        smoothing_factor=0.3,
                    ),
                    wr.MediaBrowser(
                        title="Per-run route with events",
                        media_keys=["figures/trajectory/replay_route"],
                        mode="gallery",
                        num_columns=2,
                    ),
                ],
            ),
            sec(
                "Perception",
                [
                    lp(
                        "Raw cones per scan",
                        "t/replay_s",
                        ["replay_perception/n_cones_raw"],
                        smoothing_type="average",
                        smoothing_factor=0.6,
                        smoothing_show_original=True,
                    ),
                    lp(
                        "Δ cones vs baseline (1 s mean)",
                        "t/replay_s",
                        ["replay_perception_delta/n_cones_raw_delta"],
                    ),
                    lp(
                        "CONE_FILTER accepted",
                        "t/log_s",
                        ["log_perception_filter/accepted"],
                    ),
                    lp(
                        "Funnel: clusters → shape → accepted",
                        "t/log_s",
                        [
                            "log_perception_filter/clusters",
                            "log_perception_filter/shape_pass",
                            "log_perception_filter/accepted",
                            "log_perception_filter/far_dropped",
                        ],
                    ),
                    lp(
                        "Accepted by side",
                        "t/log_s",
                        ["log_perception_filter/left", "log_perception_filter/right"],
                    ),
                    lp("Detection node Hz", "t/log_s", ["log_perception_filter/hz"]),
                    lp(
                        "DBSCAN guard: points dropped",
                        "t/log_s",
                        ["log_perception_guard/pts_dropped"],
                    ),
                    wr.MediaBrowser(
                        title="Per-run perception figures",
                        media_keys=[
                            "figures/perception/replay_cone_counts",
                            "figures/perception/replay_funnel",
                            "figures/perception/replay_cone_histogram",
                        ],
                        mode="gallery",
                        num_columns=2,
                    ),
                ],
            ),
            sec(
                "SLAM",
                [
                    lp("Processing time [ms]", "t/log_s", ["log_slam_latency/proc_ms"]),
                    lp("Message age [ms]", "t/log_s", ["log_slam_latency/age_ms"]),
                    lp(
                        "Update cost breakdown [ms]",
                        "t/log_s",
                        [
                            "log_slam_profile/pre_ms",
                            "log_slam_profile/assoc_ms",
                            "log_slam_profile/commit_ms",
                            "log_slam_profile/pub_ms",
                        ],
                        plot_type="stacked-area",
                    ),
                    lp("Map size", "t/log_s", ["log_slam_profile/map"]),
                    lp(
                        "Mode (0 map, 1 loc, 2 skip)",
                        "t/log_s",
                        ["log_slam_profile/mode"],
                    ),
                    lp(
                        "Data association per scan",
                        "t/log_s",
                        [
                            "log_slam_obs/obs",
                            "log_slam_obs/assoc",
                            "log_slam_obs/new",
                            "log_slam_obs/vetoed",
                        ],
                    ),
                    wr.WeavePanelSummaryTable(table_name="tables/events"),
                    wr.MediaBrowser(
                        title="Per-run SLAM figures",
                        media_keys=[
                            "figures/slam/replay_slam_profile",
                            "figures/slam/replay_slam_latency",
                        ],
                        mode="gallery",
                    ),
                ],
            ),
            sec(
                "Planning & control",
                [
                    lp(
                        "/Path rates",
                        "t/log_s",
                        ["log_planning_rate/cb_hz", "log_planning_rate/pub_hz"],
                    ),
                    lp(
                        "Planning problems / window",
                        "t/log_s",
                        [
                            "log_planning_rate/no_cones",
                            "log_planning_rate/plan_empty",
                            "log_planning_rate/tf_miss",
                        ],
                    ),
                    lp(
                        "Steering: autonomy vs pilot",
                        "t/replay_s",
                        [
                            "replay_control/steering_rad",
                            "replay_control/pilot_steering_rad",
                        ],
                    ),
                    lp(
                        "Steering residual",
                        "t/replay_s",
                        ["replay_control/steer_residual_rad"],
                    ),
                    lp(
                        "Throttle / brake",
                        "t/replay_s",
                        ["replay_control/throttle", "replay_control/brake"],
                    ),
                    lp(
                        "Control log: speed and path length",
                        "t/log_s",
                        ["log_control_status/v_mps", "log_control_status/path_len_m"],
                    ),
                ],
            ),
            sec(
                "Health & tables",
                [
                    wr.WeavePanelSummaryTable(table_name="tables/baseline_comparison"),
                    wr.WeavePanelSummaryTable(table_name="tables/lifecycle"),
                    wr.WeavePanelSummaryTable(table_name="tables/warn_catalogue"),
                    wr.MediaBrowser(
                        title="Startup / events",
                        media_keys=[
                            "figures/health/replay_lifecycle",
                            "figures/health/event_timeline",
                        ],
                    ),
                    wr.MediaBrowser(
                        title="Original onboard report.html",
                        media_keys=["report/onboard_html"],
                    ),
                ],
                open_=False,
            ),
        ],
    )


def sim_view(entity, presets):
    return ws.Workspace(
        entity=entity,
        project=PROJECT,
        name="Sim matrix (MOCK)",
        settings=ws.WorkspaceSettings(x_axis="track/s_m", max_runs=40),
        runset_settings=ws.RunsetSettings(
            filters="Config('job_type') == 'sim_e2e' and Tags in ['matrix']",
            groupby=[expr.Metric("Group")],
            pinned_columns=[f"summary:{m}" for m in SIM_MATRIX_METRICS],
        ),
        sections=[
            ws.Section(
                name="Outcome (grouped: scenario × commit, band = min–max over seeds)",
                is_open=True,
                panels=[
                    bars(
                        "Race",
                        [
                            "race/finished",
                            "race/lap_time_mean_s",
                            "race/n_doo",
                            "race/n_off_track",
                        ],
                    ),
                    bars(
                        "Quality",
                        [
                            "control/cross_track_rms_m",
                            "perception/recall_frac",
                            "slam/pos_err_rms_m",
                        ],
                    ),
                    bars(
                        "Latency",
                        [
                            "latency/perception_p95_ms",
                            "latency/slam_p95_ms",
                            "latency/e2e_p95_ms",
                        ],
                    ),
                    wr.ScatterPlot(
                        title="Lap time vs cones hit",
                        x=wr.SummaryMetric("race/lap_time_mean_s"),
                        y=wr.SummaryMetric("race/n_doo"),
                    ),
                    lp(
                        "Lap time per lap",
                        "lap",
                        ["sim_laps/lap_time_s"],
                        groupby_aggfunc="mean",
                        groupby_rangefunc="minmax",
                    ),
                ],
            ),
            ws.Section(
                name="Along the track (x = distance travelled)",
                is_open=True,
                panels=[
                    lp(
                        "Cross-track error",
                        "track/s_m",
                        ["sim_track/cte_m"],
                        groupby_aggfunc="mean",
                        groupby_rangefunc="minmax",
                    ),
                    lp(
                        "Speed",
                        "track/s_m",
                        ["sim_track/speed_mps"],
                        groupby_aggfunc="mean",
                        groupby_rangefunc="minmax",
                    ),
                    lp("Lateral acceleration", "track/s_m", ["sim_track/lat_acc_mps2"]),
                    lp(
                        "SLAM / odom error vs GT",
                        "track/s_m",
                        ["sim_track/slam_err_m", "sim_track/odom_err_m"],
                    ),
                    xy_chart(
                        presets,
                        "tables/trajectory",
                        "Driven line (GT) and SLAM estimate",
                    ),
                    xy_chart(
                        presets,
                        "tables/events",
                        "Penalty / failure locations",
                        lines=False,
                        color="run",
                        shape="kind",
                    ),
                ],
            ),
            ws.Section(
                name="Perception & latency (x = sim time)",
                is_open=True,
                panels=[
                    lp(
                        "Recall per scan",
                        "t/sim_s",
                        ["sim_perception/recall"],
                        smoothing_type="average",
                        smoothing_factor=0.8,
                    ),
                    lp(
                        "Precision per scan",
                        "t/sim_s",
                        ["sim_perception/precision"],
                        smoothing_type="average",
                        smoothing_factor=0.8,
                    ),
                    lp("False positives / scan", "t/sim_s", ["sim_perception/n_fp"]),
                    lp(
                        "End-to-end latency [ms]",
                        "t/sim_s",
                        ["sim_latency/e2e_ms"],
                        groupby_aggfunc="mean",
                        groupby_rangefunc="minmax",
                    ),
                    lp(
                        "Per-node latency [ms]",
                        "t/sim_s",
                        [
                            "sim_latency/perception_ms",
                            "sim_latency/slam_ms",
                            "sim_latency/planning_ms",
                            "sim_latency/control_ms",
                        ],
                    ),
                ],
            ),
            ws.Section(
                name="Per-run deep dives",
                is_open=False,
                panels=[
                    wr.MediaBrowser(
                        title="Track map + events",
                        media_keys=["figures/race/sim_track_map"],
                        mode="gallery",
                    ),
                    wr.MediaBrowser(
                        title="Failure snapshot",
                        media_keys=["figures/race/sim_failure_zoom"],
                        mode="gallery",
                    ),
                    wr.MediaBrowser(
                        title="Cross-track heatmap",
                        media_keys=["figures/control/sim_cte_heatmap"],
                    ),
                    wr.MediaBrowser(
                        title="Perception vs GT",
                        media_keys=["figures/perception/sim_perception"],
                    ),
                    wr.WeavePanelSummaryTable(table_name="tables/laps"),
                    wr.WeavePanelSummaryTable(table_name="tables/events"),
                ],
            ),
        ],
    )


def agg_view(entity, name, filters, metrics, extra_panels=()):
    return ws.Workspace(
        entity=entity,
        project=PROJECT,
        name=name,
        settings=ws.WorkspaceSettings(x_axis="track/s_lap_m", max_runs=40),
        runset_settings=ws.RunsetSettings(
            filters=filters,
            pinned_columns=[f"summary:{m}" for m in metrics],
            order=[expr.Ordering(expr.Config("env.started_at"), ascending=True)],
        ),
        sections=[
            ws.Section(
                name="Summary",
                is_open=True,
                panels=[
                    *(bars(m, [m, f"{m}.delta"]) for m in metrics),
                    *extra_panels,
                    lp(
                        "Mean |cross-track| vs lap distance",
                        "track/s_lap_m",
                        ["agg_track_profile/cte_m_mean"],
                    ),
                    lp(
                        "Mean speed vs lap distance",
                        "track/s_lap_m",
                        ["agg_track_profile/speed_mps_mean"],
                    ),
                    lp(
                        "Δ |cross-track| vs baseline",
                        "track/s_lap_m",
                        ["agg_track_profile_delta/cte_m_mean_delta"],
                    ),
                    wr.WeavePanelSummaryTable(table_name="tables/seeds"),
                    wr.WeavePanelSummaryTable(table_name="tables/baseline_comparison"),
                    wr.MediaBrowser(
                        title="Seed spread / hotspots",
                        media_keys=[
                            "figures/reliability/agg_seeds",
                            "figures/race/agg_hotspots",
                        ],
                        mode="gallery",
                    ),
                ],
            )
        ],
    )


def sweep_panels():
    return [
        wr.ParallelCoordinatesPlot(
            title="Sweep",
            columns=[
                wr.ParallelCoordinatesPlotColumn(
                    metric=wr.Config("params.control.lookahead_gain")
                ),
                wr.ParallelCoordinatesPlotColumn(
                    metric=wr.Config("params.control.max_lat_acc")
                ),
                wr.ParallelCoordinatesPlotColumn(
                    metric=wr.SummaryMetric("race/lap_time_mean_s")
                ),
                wr.ParallelCoordinatesPlotColumn(metric=wr.SummaryMetric("race/n_doo")),
                wr.ParallelCoordinatesPlotColumn(
                    metric=wr.SummaryMetric("control/cross_track_rms_m")
                ),
                wr.ParallelCoordinatesPlotColumn(
                    metric=wr.SummaryMetric("race/finish_rate_frac")
                ),
            ],
        ),
        wr.ParameterImportancePlot(with_respect_to="race/lap_time_mean_s"),
        wr.ScatterPlot(
            title="Lap time vs DOO",
            x=wr.SummaryMetric("race/lap_time_mean_s"),
            y=wr.SummaryMetric("race/n_doo"),
        ),
    ]


# ------------------------------------------------------------------ reports
def _runset(entity, name, filters, **kw):
    return wr.Runset(entity=entity, project=PROJECT, name=name, filters=filters, **kw)


def build_reports(entity, presets, fig_keys, suite: Suite) -> list[str]:
    urls = []

    def dash(slug):
        return _runset(
            entity,
            "precomputed",
            f"Config('job_type') == 'dashboard' and Config('dashboard') == '{slug}'",
        )

    replay_rs = _runset(
        entity, "onboard replays", "Config('job_type') == 'onboard_replay'"
    )
    all_rs = _runset(
        entity, "all replays", "Config('job_type') in ['onboard_replay', 'live_replay']"
    )

    r = wr.Report(
        project=PROJECT,
        entity=entity,
        title="Bag replays: how the pipeline behaved on manual_20260920_154527",
        description="Backfilled onboard + live replays of one bag. Same bag, so routes overlay directly.",
        width="fluid",
        blocks=[
            wr.TableOfContents(),
            wr.H1("Outcome"),
            wr.P(
                "Headline KPIs of every replay. The earliest onboard run is the baseline (Δ columns in the runs table)."
            ),
            wr.PanelGrid(
                runsets=[all_rs],
                panels=[
                    bars(t, ms)
                    for t, ms in (
                        (
                            "Perception",
                            [
                                "perception/n_cones_mean",
                                "perception/empty_frame_rate_frac",
                                "perception/n_dbscan_guard_trips",
                            ],
                        ),
                        (
                            "SLAM",
                            [
                                "slam/n_map_cones",
                                "slam/end_gap_vs_odom_m",
                                "slam/n_pose_jump_rejected",
                                "slam/n_da_failure_skips",
                            ],
                        ),
                        (
                            "Compute",
                            ["slam/proc_p95_ms", "slam/age_p95_ms", "run/startup_s"],
                        ),
                    )
                ],
            ),
            wr.H1("Where did it drive?"),
            wr.PanelGrid(
                runsets=[replay_rs],
                panels=[
                    xy_chart(presets, "tables/trajectory", "odom + SLAM routes"),
                    xy_chart(
                        presets,
                        "tables/map_cones",
                        "SLAM maps",
                        lines=False,
                        color="run",
                        shape="source",
                    ),
                ],
            ),
            wr.H1("Perception over time"),
            wr.PanelGrid(
                runsets=[all_rs],
                panels=[
                    lp(
                        "Raw cones per scan",
                        "t/replay_s",
                        ["replay_perception/n_cones_raw"],
                        smoothing_type="average",
                        smoothing_factor=0.6,
                    ),
                    lp(
                        "Δ vs baseline",
                        "t/replay_s",
                        ["replay_perception_delta/n_cones_raw_delta"],
                    ),
                    lp(
                        "CONE_FILTER accepted (incl. live)",
                        "t/log_s",
                        ["log_perception_filter/accepted"],
                    ),
                ],
            ),
            wr.H1("SLAM and control"),
            wr.PanelGrid(
                runsets=[all_rs],
                panels=[
                    lp("SLAM processing [ms]", "t/log_s", ["log_slam_latency/proc_ms"]),
                    lp("Map size", "t/log_s", ["log_slam_profile/map"]),
                    lp(
                        "Steering residual",
                        "t/replay_s",
                        ["replay_control/steer_residual_rad"],
                    ),
                ],
            ),
            wr.H1(
                "Precomputed comparisons (same figures as the MLflow / ClearML dashboards)"
            ),
            wr.PanelGrid(
                runsets=[dash("bag-replay")],
                panels=[
                    wr.MediaBrowser(
                        title="comparisons",
                        media_keys=fig_keys.get("bag-replay", []),
                        mode="gallery",
                        num_columns=1,
                    )
                ],
            ),
        ],
    )
    r.save()
    urls.append(r.url)

    base_rs = _runset(
        entity,
        "matrix aggregates",
        "Config('job_type') == 'sim_aggregate' and Tags in ['matrix']",
    )
    seeds_rs = _runset(
        entity,
        "matrix seeds (grouped)",
        "Config('job_type') == 'sim_e2e' and Tags in ['matrix']",
        groupby=["group"],
    )
    r = wr.Report(
        project=PROJECT,
        entity=entity,
        title="Sim matrix (MOCK): c0ffee1 vs baseline a64350a",
        description="3 scenarios × 2 commits × 5 seeds. Is the candidate faster, and at what cost?",
        width="fluid",
        blocks=[
            wr.TableOfContents(),
            wr.H1("Verdict"),
            wr.CalloutBlock(
                text="MOCK data from bench_tracking.mock_sim. The layout is what matters here, not the numbers."
            ),
            wr.PanelGrid(
                runsets=[base_rs], panels=[bars(m, [m]) for m in SIM_MATRIX_METRICS]
            ),
            wr.H1("Seed spread"),
            wr.PanelGrid(
                runsets=[seeds_rs],
                panels=[
                    lp(
                        "Lap time per lap (mean, min–max over seeds)",
                        "lap",
                        ["sim_laps/lap_time_s"],
                        groupby_aggfunc="mean",
                        groupby_rangefunc="minmax",
                    ),
                    lp(
                        "Cross-track vs distance",
                        "track/s_m",
                        ["sim_track/cte_m"],
                        groupby_aggfunc="mean",
                        groupby_rangefunc="minmax",
                    ),
                    wr.ScatterPlot(
                        title="Lap time vs DOO",
                        x=wr.SummaryMetric("race/lap_time_mean_s"),
                        y=wr.SummaryMetric("race/n_doo"),
                    ),
                ],
            ),
            wr.H1("Where on the track"),
            wr.PanelGrid(
                runsets=[base_rs],
                panels=[
                    lp(
                        "Mean |cross-track| vs lap distance",
                        "track/s_lap_m",
                        ["agg_track_profile/cte_m_mean"],
                    ),
                    xy_chart(
                        presets,
                        "tables/events",
                        "Penalties / failures",
                        lines=False,
                        color="run",
                        shape="kind",
                    ),
                ],
            ),
            wr.H1("Precomputed comparisons"),
            wr.PanelGrid(
                runsets=[dash("sim-matrix")],
                panels=[
                    wr.MediaBrowser(
                        title="comparisons",
                        media_keys=fig_keys.get("sim-matrix", []),
                        mode="gallery",
                        num_columns=1,
                    )
                ],
            ),
        ],
    )
    r.save()
    urls.append(r.url)

    nightly_rs = _runset(
        entity,
        "nightly",
        "Config('job_type') == 'sim_aggregate' and Tags in ['nightly']",
    )
    r = wr.Report(
        project=PROJECT,
        entity=entity,
        title="Nightly regression (MOCK)",
        width="fluid",
        blocks=[
            wr.P(
                "One aggregate run per night (3 seeds). Alerts fire from the nightly job when a metric regresses "
                "beyond its registry tolerance."
            ),
            wr.PanelGrid(
                runsets=[nightly_rs], panels=[bars(m, [m]) for m in NIGHTLY_METRICS]
            ),
            wr.PanelGrid(
                runsets=[dash("nightly")],
                panels=[
                    wr.MediaBrowser(
                        title="trend",
                        media_keys=fig_keys.get("nightly", []),
                        mode="gallery",
                        num_columns=1,
                    )
                ],
            ),
        ],
    )
    r.save()
    urls.append(r.url)

    sweep_rs = _runset(
        entity, "sweep trials", "Config('job_type') == 'sim_sweep_trial'"
    )
    r = wr.Report(
        project=PROJECT,
        entity=entity,
        title="Sweep (MOCK): lookahead × max lateral acceleration",
        width="fluid",
        blocks=[
            wr.PanelGrid(runsets=[sweep_rs], panels=sweep_panels()),
            wr.PanelGrid(
                runsets=[dash("sweep")],
                panels=[
                    wr.MediaBrowser(
                        title="grid / pareto",
                        media_keys=fig_keys.get("sweep", []),
                        mode="gallery",
                        num_columns=1,
                    )
                ],
            ),
        ],
    )
    r.save()
    urls.append(r.url)
    return urls


def build(suite: Suite, state: dict[str, dict]) -> list[str]:
    api = wandb.Api()
    entity = os.environ.get("WANDB_ENTITY") or api.default_entity
    presets = register_presets(api, entity)
    fig_keys = log_dashboard_runs(suite, entity)
    out = [f"[wandb] presets: {presets}"]
    base = next((b for b in suite.replays if "baseline" in b.tags), None)
    base_id = state.get(base.run_key, {}).get("run_id") if base else None
    views = [
        replay_view(entity, presets, base_id),
        sim_view(entity, presets),
        agg_view(
            entity,
            "Sim aggregates (MOCK)",
            "Config('job_type') == 'sim_aggregate' and Tags in ['matrix']",
            SIM_MATRIX_METRICS,
        ),
        agg_view(
            entity,
            "Nightly (MOCK)",
            "Config('job_type') == 'sim_aggregate' and Tags in ['nightly']",
            NIGHTLY_METRICS,
        ),
        agg_view(
            entity,
            "Sweep (MOCK)",
            "Config('job_type') == 'sim_sweep_trial'",
            ["race/lap_time_mean_s", "race/n_doo", "control/cross_track_rms_m"],
            extra_panels=sweep_panels(),
        ),
    ]
    for v in views:
        v.save_as_new_view()
        out.append(f"[wandb] view {v.name}: {v.url}")
    for u in build_reports(entity, presets, fig_keys, suite):
        out.append(f"[wandb] report: {u}")
    return out
