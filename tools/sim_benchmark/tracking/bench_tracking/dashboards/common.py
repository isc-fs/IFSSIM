"""Dashboard content shared by all backends.

Each dashboard is a list of panels: (section, key, figure). Backends place them
natively where they can: W&B builds most of them from native panels, and
MLflow/ClearML show these precomputed comparison figures.
"""

from __future__ import annotations

import html
import math
from dataclasses import dataclass
from datetime import datetime, timezone

import plotly.graph_objects as go
import plotly.io as pio

from .. import figures as F
from .. import registry
from ..bundle import RunBundle
from ..suite import Suite


@dataclass
class Panel:
    section: str
    key: str
    fig: go.Figure


@dataclass
class Dashboard:
    slug: str
    title: str
    intro: str
    panels: list[Panel]
    runs: list[RunBundle]


def _p(out: list[Panel], section: str, key: str, fig: go.Figure | None) -> None:
    if fig is not None:
        out.append(Panel(section, key, fig))


REPLAY_BAR_METRICS = [
    "perception/n_cones_mean",
    "perception/empty_frame_rate_frac",
    "perception/filter_rate_mean_hz",
    "perception/n_dbscan_guard_trips",
    "slam/n_map_cones",
    "slam/end_gap_vs_odom_m",
    "slam/odom_divergence_max_m",
    "slam/proc_p95_ms",
    "slam/age_p95_ms",
    "slam/assoc_frac",
    "slam/n_pose_jump_rejected",
    "slam/n_da_failure_skips",
    "slam/t_first_loc_s",
    "planning/pub_rate_mean_hz",
    "planning/n_plan_empty",
    "control/steer_residual_mean_abs_rad",
    "control/steer_rate_rms_radps",
    "run/startup_s",
    "odom/t_calibrated_s",
    "run/n_warnings",
]
SIM_MATRIX_METRICS = [
    "race/finish_rate_frac",
    "race/lap_time_mean_s",
    "race/n_doo",
    "race/n_off_track",
    "control/cross_track_rms_m",
    "perception/recall_frac",
    "slam/pos_err_rms_m",
    "latency/e2e_p95_ms",
]
SIM_SIG_METRICS = [
    "race/lap_time_mean_s",
    "race/n_doo",
    "control/cross_track_rms_m",
    "perception/recall_frac",
    "perception/precision_frac",
    "slam/pos_err_rms_m",
    "latency/e2e_p95_ms",
    "latency/perception_p95_ms",
]
NIGHTLY_METRICS = [
    "race/finish_rate_frac",
    "race/lap_time_mean_s",
    "race/n_doo",
    "control/cross_track_rms_m",
    "latency/perception_p95_ms",
    "latency/slam_p95_ms",
    "latency/e2e_p95_ms",
    "compare/n_regressions",
]


def replay_dashboard(s: Suite) -> Dashboard:
    runs = sorted(s.replays, key=lambda b: b.started_at)
    played = [r for r in runs if r.status == "finished"]
    onboard = [r for r in played if r.job_type == "onboard_replay"]
    base = s.baselines.get(runs[0].scenario_id) if runs else None
    P: list[Panel] = []
    _p(
        P,
        "overview",
        "kpi_bars",
        F.cmp_metric_bars(
            played, REPLAY_BAR_METRICS, "Bag replay KPIs per run", base, cols=4
        ),
    )
    _p(
        P,
        "overview",
        "delta_vs_baseline",
        F.cmp_delta_heatmap(
            played,
            [m for m in REPLAY_BAR_METRICS if registry.spec(m).direction != "none"],
            f"Change vs baseline ({base.name if base else '—'})",
        ),
    )
    _p(P, "trajectory", "route_overlay", F.cmp_routes(onboard))
    _p(
        P,
        "trajectory",
        "route_overlay_slam_only",
        F.cmp_routes(onboard, sources=("slam",)),
    )
    _p(P, "trajectory", "map_overlay", F.cmp_map_cones(onboard))
    _p(
        P,
        "trajectory",
        "slam_odom_gap",
        F.cmp_series(
            onboard,
            "replay_pose",
            "slam_odom_gap_m",
            "SLAM − odom gap over time (no-GT drift proxy)",
            "m",
        ),
    )
    _p(
        P,
        "perception",
        "cone_count_overlay",
        F.cmp_series(
            onboard,
            "replay_perception",
            "n_cones_raw",
            "Raw cone detections per scan (1 s mean)",
            "cones",
            smooth=10,
        ),
    )
    _p(
        P,
        "perception",
        "cone_count_delta",
        F.cmp_series(
            onboard,
            "replay_perception_delta",
            "n_cones_raw_delta",
            "Cone count Δ vs baseline run (same bag, 1 s mean)",
            "Δ cones",
        ),
    )
    _p(
        P,
        "perception",
        "accepted_all_runs",
        F.cmp_series(
            played,
            "log_perception_filter",
            "accepted",
            "CONE_FILTER accepted cones (incl. live runs)",
            "cones / scan",
        ),
    )
    _p(
        P,
        "perception",
        "clusters_all_runs",
        F.cmp_series(
            played,
            "log_perception_filter",
            "clusters",
            "DBSCAN clusters per scan",
            "clusters",
        ),
    )
    _p(
        P,
        "perception",
        "filter_rate",
        F.cmp_series(
            played, "log_perception_filter", "hz", "Cone-detection node rate", "Hz"
        ),
    )
    _p(
        P,
        "slam",
        "latency_distributions",
        F.cmp_distributions(
            played,
            "log_slam_latency",
            ["proc_ms", "age_ms", "dt_pub_ms"],
            "SLAM latency distributions (SLAM_LAT)",
        ),
    )
    _p(
        P,
        "slam",
        "profile_distributions",
        F.cmp_distributions(
            played,
            "log_slam_profile",
            ["total_ms", "assoc_ms", "commit_ms", "pub_ms"],
            "SLAM update cost breakdown (SLAM_PROF)",
        ),
    )
    _p(
        P,
        "slam",
        "map_growth",
        F.cmp_series(
            played, "log_slam_profile", "map", "Map size over time", "landmarks"
        ),
    )
    _p(
        P,
        "slam",
        "association",
        F.cmp_series(
            played, "log_slam_obs", "assoc", "Associated cones per scan", "cones"
        ),
    )
    _p(
        P,
        "slam",
        "new_landmarks",
        F.cmp_series(played, "log_slam_obs", "new", "New landmarks per scan", "cones"),
    )
    _p(
        P,
        "planning",
        "publish_rate",
        F.cmp_series(played, "log_planning_rate", "pub_hz", "/Path publish rate", "Hz"),
    )
    _p(
        P,
        "control",
        "steering_overlay",
        F.cmp_series(
            onboard,
            "replay_control",
            "steering_rad",
            "Autonomy steering command",
            "rad",
            smooth=8,
        ),
    )
    _p(
        P,
        "control",
        "steer_residual_overlay",
        F.cmp_series(
            onboard,
            "replay_control",
            "steer_residual_rad",
            "Autonomy − pilot steering",
            "rad",
            smooth=8,
        ),
    )
    _p(
        P,
        "control",
        "speed_overlay",
        F.cmp_series(
            played,
            "log_control_status",
            "v_mps",
            "Speed seen by control (2 Hz log, incl. live runs)",
            "m/s",
        ),
    )
    _p(
        P,
        "health",
        "event_counts",
        F.cmp_event_counts(runs, "Pipeline events per run (from node logs)"),
    )
    _p(P, "health", "startup", F.cmp_startup(runs))
    _p(
        P,
        "health",
        "warn_error_bars",
        F.cmp_metric_bars(
            runs,
            [
                "run/n_warnings",
                "run/n_errors",
                "run/n_crashes",
                "run/completed",
                "run/replay_duration_s",
                "run/n_viewer_clients",
            ],
            "Run health",
            cols=3,
        ),
    )
    return Dashboard("bag-replay", "Bag replay benchmarks", REPLAY_INTRO, P, runs)


def sim_matrix_dashboard(s: Suite) -> Dashboard:
    aggs = sorted(
        s.matrix_aggs(),
        key=lambda a: (
            a.config["scenario"]["name"],
            a.config["code"]["pipeline"]["sha"],
        ),
    )
    sbg = s.seeds_by_group()
    P: list[Panel] = []
    _p(P, "overview", "scenario_x_commit", F.cmp_sim_matrix(aggs, SIM_MATRIX_METRICS))
    _p(P, "overview", "significance", F.cmp_significance(aggs, sbg, SIM_SIG_METRICS))
    _p(
        P,
        "overview",
        "delta_vs_baseline",
        F.cmp_delta_heatmap(
            aggs,
            SIM_SIG_METRICS + ["race/finish_rate_frac"],
            "Aggregate change vs baseline commit per scenario",
        ),
    )
    _p(P, "race", "lap_distributions", F.cmp_lap_distributions(aggs))
    for scen in sorted({a.config["scenario"]["name"] for a in aggs}):
        sa = [a for a in aggs if a.config["scenario"]["name"] == scen]
        seeds = [x for a in sa for x in sbg.get(a.group, [])]
        _p(
            P,
            "race",
            f"hotspots_{scen}",
            cmp_hotspots(sa, f"Penalties / failures on {scen} (all seeds, per commit)"),
        )
        _p(
            P,
            "control",
            f"cte_profile_{scen}",
            F.cmp_profiles(
                sa, "cte_m_mean", f"{scen}: mean |cross-track| vs lap distance"
            ),
        )
        _p(
            P,
            "control",
            f"speed_profile_{scen}",
            F.cmp_profiles(sa, "speed_mps_mean", f"{scen}: mean speed vs lap distance"),
        )
        _p(
            P,
            "control",
            f"route_{scen}",
            F.cmp_routes(
                [x for x in seeds if x.config["scenario"]["seed"] == 1],
                sources=("gt", "slam"),
            ),
        )
    _p(P, "perception", "recall_vs_range", F.cmp_recall_range(aggs, sbg))
    _p(P, "latency", "e2e_cdf", F.cmp_latency_cdf(aggs, sbg, "e2e_ms"))
    _p(P, "latency", "perception_cdf", F.cmp_latency_cdf(aggs, sbg, "perception_ms"))
    _p(
        P,
        "reliability",
        "event_counts",
        F.cmp_event_counts(aggs, "Events per scenario × commit (sum over seeds)"),
    )
    return Dashboard(
        "sim-matrix",
        "Simulator matrix: candidate vs baseline (MOCK data)",
        SIM_INTRO,
        P,
        aggs,
    )


def nightly_dashboard(s: Suite) -> Dashboard:
    aggs = sorted(s.nightly_aggs(), key=lambda a: a.config["nightly"]["night"])
    base = s.baselines.get(aggs[0].scenario_id) if aggs else None
    P: list[Panel] = []
    _p(P, "trend", "headline_trend", F.cmp_nightly(aggs, NIGHTLY_METRICS, base))
    _p(
        P,
        "trend",
        "delta_vs_baseline",
        F.cmp_delta_heatmap(
            aggs, SIM_SIG_METRICS + ["race/finish_rate_frac"], "Each night vs baseline"
        ),
    )
    _p(
        P,
        "trend",
        "cte_profiles",
        F.cmp_profiles(
            aggs, "cte_m_mean", "Mean |cross-track| vs lap distance, per night"
        ),
    )
    _p(
        P,
        "trend",
        "event_counts",
        F.cmp_event_counts(aggs, "Penalty events per night (sum over seeds)"),
    )
    return Dashboard(
        "nightly", "Nightly regression (MOCK data)", NIGHTLY_INTRO, P, aggs
    )


def sweep_dashboard(s: Suite) -> Dashboard:
    trials = sorted(s.sweep_trials(), key=lambda a: a.config["sweep"]["trial"])
    P = [Panel("sweep", k.split("/", 1)[1], f) for k, f in F.cmp_sweep(trials).items()]
    _p(
        P,
        "sweep",
        "cte_profiles",
        F.cmp_profiles(
            trials, "cte_m_mean", "Mean |cross-track| vs lap distance, per trial"
        ),
    )
    return Dashboard(
        "sweep",
        "Pure-pursuit sweep: lookahead × max lateral acceleration (MOCK data)",
        SWEEP_INTRO,
        P,
        trials,
    )


def all_dashboards(s: Suite) -> list[Dashboard]:
    return [
        d
        for d in (
            replay_dashboard(s),
            sim_matrix_dashboard(s),
            nightly_dashboard(s),
            sweep_dashboard(s),
        )
        if d.panels
    ]


def cmp_hotspots(aggs: list[RunBundle], title: str) -> go.Figure | None:
    if not aggs or "map_cones" not in aggs[0].tables:
        return None
    f = go.Figure()
    F._track_base(f, aggs[0].tables["map_cones"].rows, faint=True)
    for i, a in enumerate(aggs):
        for kind in ("doo", "off_course", "dnf"):
            es = [e for e in a.events if e.kind == kind and e.x is not None]
            if es:
                f.add_scatter(
                    x=[e.x for e in es],
                    y=[e.y for e in es],
                    mode="markers",
                    name=f"{a.config['code']['pipeline']['sha']} {kind} ({len(es)})",
                    marker=dict(
                        symbol=F.EVENT_STYLE[kind][0],
                        size=12,
                        color=F.PALETTE[i % 10],
                        opacity=0.8,
                        line=dict(width=1, color="white"),
                    ),
                    text=[e.detail for e in es],
                    hoverinfo="text",
                )
    f.update_layout(title=title, height=680, **F.LAYOUT)
    return F._equal(f)


# ------------------------------------------------------------------ HTML page
def runs_table_html(
    runs: list[RunBundle], links: dict[str, str], metrics: list[str]
) -> str:
    head = "".join(
        f"<th>{html.escape(m)}</th>" for m in ["run", "job", "status", *metrics]
    )
    rows = []
    for r in runs:
        name = html.escape(r.name)
        link = links.get(r.run_key)
        cell = f'<a href="{link}" target="_blank">{name}</a>' if link else name
        vals = []
        for m in metrics:
            v = r.summary.get(m)
            d = r.summary.get(f"{m}.delta")
            txt = "" if v is None else (f"{v:.4g}" if isinstance(v, float) else str(v))
            if isinstance(d, (int, float)) and math.isfinite(d):
                verdict = (
                    registry.delta(m, v, v - d) if isinstance(v, (int, float)) else {}
                )
                cls = (
                    "bad"
                    if verdict.get("regression")
                    else "good"
                    if verdict.get("improvement")
                    else ""
                )
                txt += f' <span class="d {cls}">({d:+.3g})</span>'
            vals.append(f"<td>{txt}</td>")
        rows.append(
            f"<tr><td>{cell}</td><td>{r.job_type}</td><td>{r.status}</td>{''.join(vals)}</tr>"
        )
    return f'<div class="tw"><table><thead><tr>{head}</tr></thead><tbody>{"".join(rows)}</tbody></table></div>'


def dashboard_html(d: Dashboard, links: dict[str, str], backend: str) -> str:
    metrics = F.headline_metrics(d.runs)[:14]
    sections: dict[str, list[Panel]] = {}
    for p in d.panels:
        sections.setdefault(p.section, []).append(p)
    toc = " · ".join(f'<a href="#{s}">{s}</a>' for s in sections)
    body = []
    for s, ps in sections.items():
        body.append(f'<h2 id="{s}">{s}</h2>')
        for p in ps:
            body.append(
                f'<div class="fig">{pio.to_html(p.fig, include_plotlyjs=False, full_html=False)}</div>'
            )
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    return f"""<!doctype html><html><head><meta charset="utf-8"><title>{html.escape(d.title)}</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>
body{{font-family:system-ui,sans-serif;margin:0 24px 48px;color:#222;background:#fff}}
h1{{margin:20px 0 4px}} h2{{border-bottom:2px solid #ddd;padding-top:18px;text-transform:capitalize}}
.intro{{max-width:1100px;line-height:1.45}} .toc{{position:sticky;top:0;background:#fff;padding:8px 0;border-bottom:1px solid #eee;z-index:5}}
.tw{{overflow-x:auto}} table{{border-collapse:collapse;font-size:12px}} td,th{{border:1px solid #e0e0e0;padding:3px 6px;white-space:nowrap}}
th{{background:#fafafa;position:sticky;top:0}} .d{{color:#777}} .d.bad{{color:#c62828;font-weight:600}} .d.good{{color:#2e7d32;font-weight:600}}
.fig{{margin:10px 0 18px}}
</style></head><body>
<h1>{html.escape(d.title)}</h1><div class="intro">{d.intro}<p><small>Generated {ts} by bench_tracking for <b>{backend}</b>.
Bracketed numbers in the table are deltas vs the scenario baseline; red = regression beyond the registry tolerance.</small></p></div>
<div class="toc">{toc} · <a href="#runs">runs</a></div>
<h2 id="runs">runs</h2>{runs_table_html(d.runs, links, metrics)}
{''.join(body)}
</body></html>"""


REPLAY_INTRO = """<p>Every bag replay found under <code>results/onboard</code> (all replay the same bag, so routes and cone
maps overlay directly in the odom/map frame). <b>onboard</b> runs had <code>--report</code> (bag outputs + logs).
<b>live</b> runs were <code>--live</code> without a report: everything shown for them comes from the node logs.
Two live runs never started playing (aborted). Code SHAs are unknown for all of them (imported after the fact).</p>"""
SIM_INTRO = """<p><b>MOCK DATA.</b> 3 scenarios × 2 commits × 5 seeds from the mock SIL generator. The candidate
(<code>c0ffee1</code>, adaptive lookahead) is faster but cuts corners: more DOO, one DNF per 5 seeds on two scenarios.
The significance panel uses a bootstrap over seeds so noise is not read as a change.</p>"""
NIGHTLY_INTRO = """<p><b>MOCK DATA.</b> 8 nights × 3 seeds of trackdrive nominal. A regression lands on night 4
(controller oscillation + slower perception) and is fixed on night 6. SLAM latency also creeps up ~4 %/night. A trend
the per-night threshold doesn't catch but the chart shows.</p>"""
SWEEP_INTRO = """<p><b>MOCK DATA.</b> 4 × 3 grid over <code>control.lookahead_gain</code> and <code>control.max_lat_acc</code>,
3 seeds per trial, 4 laps each. The objective is the mean lap time. The cones hit and the tracking error show the price paid.</p>"""
