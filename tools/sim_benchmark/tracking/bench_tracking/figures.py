"""Plotly figures shared by the backends.

Per-run figures explain one run in depth. Comparison figures overlay many
runs. MLflow and ClearML rely on these for anything their native UI can't
draw. W&B gets the per-run ones too, but does comparisons natively where it
can (see dashboards/wandb_dash.py).

Every builder returns ``None`` when the data it needs is missing, so callers
can loop over the builders without special cases.
"""

from __future__ import annotations

import math
from typing import Callable, Iterable

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from . import registry
from .bundle import RunBundle
from .compare import bootstrap_diff
from .geometry import interp

PALETTE = [
    "#1f77b4",
    "#d62728",
    "#2ca02c",
    "#9467bd",
    "#ff7f0e",
    "#17becf",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
]
SRC_COLOR = {"odom": "#e65100", "slam": "#2e7d32", "gt": "#1565c0"}
CONE_COLOR = {
    "blue": "#1565c0",
    "yellow": "#f9a825",
    "orange": "#ef6c00",
    "unknown": "#616161",
}
EVENT_STYLE = {
    "pose_jump_rejected": ("x", "#d62728"),
    "da_failure_skip": ("diamond", "#9467bd"),
    "cascade_recovery": ("star", "#8c564b"),
    "dbscan_guard": ("triangle-up", "#ff7f0e"),
    "stop_latched": ("square", "#000000"),
    "crash": ("x-thin", "#000000"),
    "tf_lookup_failed": ("circle-open", "#17becf"),
    "doo": ("triangle-down", "#ff7f0e"),
    "off_course": ("x", "#d62728"),
    "dnf": ("star", "#000000"),
    "lateral_disturbance": ("diamond-open", "#9467bd"),
    "latency_spike": ("circle-open", "#7f7f7f"),
    "loop_closure": ("hexagon", "#2ca02c"),
}
LAYOUT = dict(
    template="plotly_white",
    margin=dict(l=55, r=20, t=60, b=45),
    legend=dict(orientation="h", y=-0.15),
    font=dict(size=12),
)


def _fig(title: str, **kw) -> go.Figure:
    f = go.Figure()
    f.update_layout(title=title, **{**LAYOUT, **kw})
    return f


def _equal(f: go.Figure) -> go.Figure:
    f.update_yaxes(scaleanchor="x", scaleratio=1)
    return f


def _dec(n: int, max_pts: int) -> np.ndarray:
    return (
        np.arange(n)
        if n <= max_pts
        else np.unique(np.linspace(0, n - 1, max_pts).astype(int))
    )


def short(b: RunBundle) -> str:
    return b.name.split("/", 1)[-1]


# ============================================================ replay: per run
def replay_route(b: RunBundle) -> go.Figure | None:
    tr = b.tables.get("trajectory")
    if not tr:
        return None
    f = _fig(
        f"Route — odom vs SLAM with map and pipeline events<br><sup>{b.name}</sup>",
        height=650,
    )
    src = tr.column("source")
    x, y, t = (
        np.array(tr.column("x")),
        np.array(tr.column("y")),
        np.array(tr.column("t_s")),
    )
    for s in ("odom", "slam"):
        m = np.array([v == s for v in src])
        idx = np.where(m)[0][_dec(int(m.sum()), 4000)]
        f.add_scatter(
            x=x[idx],
            y=y[idx],
            mode="lines",
            name=s,
            line=dict(color=SRC_COLOR[s], width=2),
            customdata=t[idx],
            hovertemplate="t=%{customdata:.1f}s<br>x=%{x:.2f} y=%{y:.2f}",
        )
    mc = b.tables.get("map_cones")
    if mc:
        f.add_scatter(
            x=mc.column("x"),
            y=mc.column("y"),
            mode="markers",
            name="SLAM map cones",
            marker=dict(symbol="triangle-up", size=8, color="#455a64"),
        )
    # place log events on the route using odom position at the event time (log time ≈ replay time)
    pose = b.series.get("replay_pose")
    if pose is not None:
        by_kind: dict[str, list] = {}
        for e in b.events:
            if e.t_s is None or e.kind in ("dbscan_guard",):
                continue
            by_kind.setdefault(e.kind, []).append(e)
        for kind, evs in by_kind.items():
            te = np.array([e.t_s for e in evs])
            ex = (
                [e.x for e in evs]
                if all(e.x is not None for e in evs)
                else interp(pose.step, pose.values["odom_x"], te)
            )
            ey = (
                [e.y for e in evs]
                if all(e.y is not None for e in evs)
                else interp(pose.step, pose.values["odom_y"], te)
            )
            sym, col = EVENT_STYLE.get(kind, ("circle", "#333"))
            f.add_scatter(
                x=ex,
                y=ey,
                mode="markers",
                name=f"{kind} ({len(evs)})",
                marker=dict(
                    symbol=sym, size=10, color=col, line=dict(width=1, color="white")
                ),
                text=[f"t={e.t_s:.1f}s {e.detail}" for e in evs],
                hoverinfo="text",
            )
    return _equal(f)


def replay_slam_odom_gap(b: RunBundle) -> go.Figure | None:
    p = b.series.get("replay_pose")
    if p is None or "slam_odom_gap_m" not in p.values:
        return None
    f = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        subplot_titles=("SLAM − odom position gap [m]", "SLAM − odom heading [deg]"),
    )
    i = _dec(len(p), 3000)
    f.add_scatter(
        x=p.step[i],
        y=p.values["slam_odom_gap_m"][i],
        name="gap",
        line=dict(color="#2e7d32"),
        row=1,
        col=1,
    )
    f.add_scatter(
        x=p.step[i],
        y=p.values["slam_odom_dyaw_deg"][i],
        name="dyaw",
        line=dict(color="#6a1b9a"),
        row=2,
        col=1,
    )
    for e in b.events:
        if e.kind == "pose_jump_rejected" and e.t_s is not None:
            f.add_vline(
                x=e.t_s, line=dict(color="rgba(214,39,40,0.25)", width=1), row=1, col=1
            )
    f.update_layout(
        title=f"Localisation consistency (no GT) — red lines: rejected pose jumps<br><sup>{b.name}</sup>",
        height=520,
        **{k: v for k, v in LAYOUT.items() if k != "legend"},
    )
    f.update_xaxes(title_text="replay time [s]", row=2, col=1)
    return f


def replay_cone_counts(b: RunBundle) -> go.Figure | None:
    raw = b.series.get("replay_perception")
    lf = b.series.get("log_perception_filter")
    if raw is None and lf is None:
        return None
    f = _fig(
        f"Cone detections over time<br><sup>{b.name}</sup>",
        height=420,
        xaxis_title="time [s]",
        yaxis_title="cones / scan",
    )
    if raw is not None:
        f.add_scatter(
            x=raw.step,
            y=raw.values["n_cones_raw"],
            name="/Conos_raw",
            mode="lines",
            line=dict(color="#1976d2", width=1),
        )
    flt = b.series.get("replay_perception_filtered")
    if flt is not None:
        f.add_scatter(
            x=flt.step,
            y=flt.values["n_cones_filtered"],
            name="/Conos (SLAM input)",
            mode="lines",
            line=dict(color="#90caf9", width=1),
        )
    if lf is not None:
        f.add_scatter(
            x=lf.step,
            y=lf.values["accepted"],
            name="CONE_FILTER accepted (log avg)",
            mode="lines",
            line=dict(color="#0d47a1", width=2, dash="dot"),
        )
    g = b.series.get("log_perception_guard")
    if g is not None:
        f.add_scatter(
            x=g.step,
            y=np.zeros(len(g)),
            mode="markers",
            name="DBSCAN guard trip",
            marker=dict(symbol="triangle-up", color="#ff7f0e", size=9),
            text=[f"dropped {v:.0f} pts" for v in g.values["pts_dropped"]],
            hoverinfo="text+x",
        )
    d = b.series.get("replay_perception_delta")
    if d is not None:
        f.add_scatter(
            x=d.step,
            y=d.values["n_cones_raw_delta"],
            name="Δ vs baseline (1 s mean)",
            mode="lines",
            line=dict(color="#d62728", width=1.5),
            yaxis="y2",
        )
        f.update_layout(
            yaxis2=dict(
                title="Δ cones",
                overlaying="y",
                side="right",
                zeroline=True,
                showgrid=False,
            )
        )
    return f


def replay_cone_histogram(b: RunBundle) -> go.Figure | None:
    raw = b.series.get("replay_perception")
    if raw is None:
        return None
    f = _fig(
        f"Distribution of cones per scan<br><sup>{b.name}</sup>",
        height=360,
        xaxis_title="cones / scan",
        yaxis_title="scans",
        bargap=0.05,
    )
    f.add_histogram(
        x=raw.values["n_cones_raw"],
        xbins=dict(start=-0.5, end=40.5, size=1),
        marker_color="#1976d2",
    )
    return f


def replay_funnel(b: RunBundle) -> go.Figure | None:
    s = b.series.get("log_perception_filter")
    if s is None:
        return None
    stages = [
        ("clusters", "clusters"),
        ("gt3pts", ">3 pts"),
        ("shape_pass", "shape ok"),
        ("residual", "residual ok"),
        ("accepted", "accepted"),
    ]
    f = make_subplots(
        rows=1,
        cols=2,
        column_widths=[0.35, 0.65],
        subplot_titles=("mean per scan", "over time (log window averages)"),
    )
    f.add_bar(
        x=[lbl for _, lbl in stages],
        y=[float(np.nanmean(s.values[k])) for k, _ in stages],
        marker_color="#1976d2",
        name="mean",
        showlegend=False,
        row=1,
        col=1,
    )
    for i, (k, lbl) in enumerate(stages):
        f.add_scatter(
            x=s.step,
            y=s.values[k],
            name=lbl,
            mode="lines",
            line=dict(color=PALETTE[i]),
            row=1,
            col=2,
        )
    f.add_scatter(
        x=s.step,
        y=s.values["far_dropped"],
        name="far dropped",
        mode="lines",
        line=dict(color="#7f7f7f", dash="dash"),
        row=1,
        col=2,
    )
    f.update_layout(
        title=f"Cone-detection funnel (CONE_FILTER)<br><sup>{b.name}</sup>",
        height=420,
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def replay_side_balance(b: RunBundle) -> go.Figure | None:
    s = b.series.get("log_perception_filter")
    if s is None:
        return None
    f = _fig(
        f"Accepted cones by side (per scan, log windows)<br><sup>{b.name}</sup>",
        height=380,
        xaxis_title="time [s]",
        yaxis_title="cones / scan",
    )
    for k, c in (
        ("left", "#1565c0"),
        ("right", "#f9a825"),
        ("big_orange", "#ef6c00"),
        ("center", "#7f7f7f"),
    ):
        f.add_scatter(
            x=s.step,
            y=s.values[k],
            name=k,
            stackgroup="side",
            line=dict(color=c, width=0.5),
        )
    f.add_scatter(
        x=s.step,
        y=s.values["hz"],
        name="node Hz",
        yaxis="y2",
        line=dict(color="#000", dash="dot"),
    )
    f.update_layout(
        yaxis2=dict(title="Hz", overlaying="y", side="right", showgrid=False)
    )
    return f


def replay_slam_latency(b: RunBundle) -> go.Figure | None:
    s = b.series.get("log_slam_latency")
    if s is None:
        return None
    f = make_subplots(
        rows=2,
        cols=2,
        column_widths=[0.7, 0.3],
        shared_xaxes="columns",
        subplot_titles=(
            "processing time [ms]",
            "proc histogram",
            "message age at processing [ms]",
            "age histogram",
        ),
    )
    f.add_scatter(
        x=s.step,
        y=s.values["proc_ms"],
        mode="markers",
        marker=dict(size=3, color="#2e7d32"),
        name="proc",
        row=1,
        col=1,
    )
    f.add_histogram(
        y=s.values["proc_ms"], marker_color="#2e7d32", name="proc", row=1, col=2
    )
    f.add_scatter(
        x=s.step,
        y=s.values["age_ms"],
        mode="markers",
        marker=dict(size=3, color="#6a1b9a"),
        name="age",
        row=2,
        col=1,
    )
    f.add_histogram(
        y=s.values["age_ms"], marker_color="#6a1b9a", name="age", row=2, col=2
    )
    f.update_layout(
        title=f"SLAM latency (SLAM_LAT)<br><sup>{b.name}</sup>",
        height=560,
        showlegend=False,
        **{k: v for k, v in LAYOUT.items() if k != "legend"},
    )
    return f


def replay_slam_profile(b: RunBundle) -> go.Figure | None:
    s = b.series.get("log_slam_profile")
    if s is None:
        return None
    parts = ["pre_ms", "assoc_ms", "spawn_ms", "commit_ms", "db_ms", "pub_ms"]
    f = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        row_heights=[0.55, 0.25, 0.2],
        subplot_titles=(
            "per-update time breakdown [ms] (1 s bins)",
            "map size / observations",
            "mode: 0 mapping, 1 localisation, 2 skipped",
        ),
    )
    # bin to 1 s so thousands of updates stay readable
    edges = np.arange(math.floor(s.step.min()), math.ceil(s.step.max()) + 1, 1.0)
    idx = np.digitize(s.step, edges)
    centers = edges[:-1] + 0.5
    for i, p in enumerate(parts):
        v = np.nan_to_num(s.values[p])
        m = np.array(
            [
                v[idx == j].mean() if (idx == j).any() else np.nan
                for j in range(1, len(edges))
            ]
        )
        f.add_scatter(
            x=centers,
            y=m,
            name=p.replace("_ms", ""),
            stackgroup="prof",
            line=dict(color=PALETTE[i], width=0.5),
            row=1,
            col=1,
        )
    f.add_scatter(
        x=s.step,
        y=s.values["map"],
        name="map size",
        line=dict(color="#000"),
        row=2,
        col=1,
    )
    f.add_scatter(
        x=s.step,
        y=s.values["obs"],
        name="obs",
        mode="markers",
        marker=dict(size=2, color="#7f7f7f"),
        row=2,
        col=1,
    )
    f.add_scatter(
        x=s.step,
        y=s.values["mode"],
        name="mode",
        mode="markers",
        marker=dict(
            size=4,
            color=s.values["mode"],
            colorscale=[[0, "#1f77b4"], [0.5, "#2ca02c"], [1, "#d62728"]],
        ),
        row=3,
        col=1,
    )
    f.update_layout(
        title=f"SLAM compute profile (SLAM_PROF)<br><sup>{b.name}</sup>",
        height=720,
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def replay_slam_obs(b: RunBundle) -> go.Figure | None:
    s = b.series.get("log_slam_obs")
    if s is None:
        return None
    f = _fig(
        f"SLAM data association per scan (SLAM_OBS window means)<br><sup>{b.name}</sup>",
        height=400,
        xaxis_title="time [s]",
        yaxis_title="cones / scan",
    )
    for i, k in enumerate(("obs", "assoc", "new", "vetoed", "skip")):
        f.add_scatter(x=s.step, y=s.values[k], name=k, line=dict(color=PALETTE[i]))
    ev = [
        e
        for e in b.events
        if e.kind in ("da_failure_skip", "cascade_recovery") and e.t_s is not None
    ]
    if ev:
        f.add_scatter(
            x=[e.t_s for e in ev],
            y=[0] * len(ev),
            mode="markers",
            name="DA skip / cascade",
            marker=dict(
                symbol=[EVENT_STYLE[e.kind][0] for e in ev], color="#9467bd", size=10
            ),
            text=[e.detail for e in ev],
            hoverinfo="text+x",
        )
    return f


def replay_planning(b: RunBundle) -> go.Figure | None:
    s = b.series.get("log_planning_rate")
    if s is None:
        return None
    f = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        subplot_titles=("rates [Hz]", "problem counts per window"),
    )
    f.add_scatter(
        x=s.step,
        y=s.values["cb_hz"],
        name="callback",
        line=dict(color="#1f77b4"),
        row=1,
        col=1,
    )
    f.add_scatter(
        x=s.step,
        y=s.values["pub_hz"],
        name="publish",
        line=dict(color="#2ca02c"),
        row=1,
        col=1,
    )
    for i, k in enumerate(("no_cones", "tf_miss", "plan_empty")):
        f.add_bar(
            x=s.step, y=s.values[k], name=k, marker_color=PALETTE[i + 3], row=2, col=1
        )
    p = b.series.get("replay_planning")
    if p is not None:
        f.add_scatter(
            x=p.step,
            y=p.values["path_poses"],
            name="poses in /Path",
            line=dict(color="#7f7f7f", dash="dot"),
            row=1,
            col=1,
        )
    f.update_layout(
        title=f"Path planning health (PATH_RATE)<br><sup>{b.name}</sup>",
        height=520,
        barmode="stack",
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def replay_control(b: RunBundle) -> go.Figure | None:
    c = b.series.get("replay_control")
    cl = b.series.get("log_control_status")
    if c is None and cl is None:
        return None
    rows = 3 if c is not None else 2
    titles = (
        ("steering [rad]: autonomy vs pilot", "throttle / brake", "speed [m/s]")
        if c is not None
        else ("steering (control log)", "throttle / regen (control log)")
    )
    f = make_subplots(rows=rows, cols=1, shared_xaxes=True, subplot_titles=titles)
    if c is not None:
        i = _dec(len(c), 4000)
        f.add_scatter(
            x=c.step[i],
            y=c.values["steering_rad"][i],
            name="autonomy steer",
            line=dict(color="#1565c0"),
            row=1,
            col=1,
        )
        if "pilot_steering_rad" in c.values:
            f.add_scatter(
                x=c.step[i],
                y=c.values["pilot_steering_rad"][i],
                name="pilot steer",
                line=dict(color="#c62828"),
                row=1,
                col=1,
            )
            f.add_scatter(
                x=c.step[i],
                y=c.values["steer_residual_rad"][i],
                name="residual",
                line=dict(color="#7f7f7f", width=1),
                row=1,
                col=1,
            )
        f.add_scatter(
            x=c.step[i],
            y=c.values["throttle"][i],
            name="throttle",
            line=dict(color="#2e7d32"),
            row=2,
            col=1,
        )
        f.add_scatter(
            x=c.step[i],
            y=c.values["brake"][i],
            name="brake",
            line=dict(color="#d62728"),
            row=2,
            col=1,
        )
        p = b.series.get("replay_pose")
        if p is not None:
            j = _dec(len(p), 3000)
            f.add_scatter(
                x=p.step[j],
                y=p.values["odom_speed_mps"][j],
                name="odom speed",
                line=dict(color="#000"),
                row=3,
                col=1,
            )
    else:
        f.add_scatter(x=cl.step, y=cl.values["steer"], name="steer", row=1, col=1)
        f.add_scatter(x=cl.step, y=cl.values["throttle"], name="throttle", row=2, col=1)
        f.add_scatter(x=cl.step, y=cl.values["regen"], name="regen", row=2, col=1)
    f.update_layout(
        title=f"Control signals<br><sup>{b.name}</sup>",
        height=640,
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def replay_steer_residual_hist(b: RunBundle) -> go.Figure | None:
    c = b.series.get("replay_control")
    if c is None or "steer_residual_rad" not in c.values:
        return None
    f = _fig(
        f"Autonomy − pilot steering residual<br><sup>{b.name}</sup>",
        height=360,
        xaxis_title="rad",
        yaxis_title="samples",
    )
    f.add_histogram(
        x=c.values["steer_residual_rad"], nbinsx=120, marker_color="#7f7f7f"
    )
    return f


def replay_lifecycle(b: RunBundle) -> go.Figure | None:
    t = b.tables.get("lifecycle")
    if not t:
        return None
    f = _fig(
        f"Node startup timeline (0 = bag play start)<br><sup>{b.name}</sup>",
        height=340,
        xaxis_title="seconds relative to bag play",
        barmode="stack",
    )
    nodes = t.column("node")
    ready, conf, act = (
        np.array([v if v is not None else np.nan for v in t.column(c)], float)
        for c in ("ready_s", "configured_s", "activated_s")
    )
    f.add_bar(
        y=nodes,
        x=conf - ready,
        base=ready,
        orientation="h",
        name="ready → configured",
        marker_color="#90caf9",
    )
    f.add_bar(
        y=nodes,
        x=act - conf,
        base=conf,
        orientation="h",
        name="configured → active",
        marker_color="#1565c0",
    )
    w0, w1 = (
        np.array([v if v is not None else np.nan for v in t.column(c)], float)
        for c in ("warmup_start_s", "warmup_done_s")
    )
    if np.isfinite(w0).any():
        f.add_bar(
            y=nodes,
            x=w1 - w0,
            base=w0,
            orientation="h",
            name="JIT warmup",
            marker_color="#ff7f0e",
            opacity=0.6,
        )
    fo = np.array(
        [v if v is not None else np.nan for v in t.column("first_output_s")], float
    )
    f.add_scatter(
        y=nodes,
        x=fo,
        mode="markers",
        name="first output",
        marker=dict(symbol="star", size=12, color="#2e7d32"),
    )
    f.add_vline(x=0, line=dict(color="#000", dash="dash"))
    return f


def event_timeline(b: RunBundle) -> go.Figure | None:
    ev = [e for e in b.events if e.t_s is not None]
    if not ev:
        return None
    kinds = sorted({e.kind for e in ev})
    f = _fig(
        f"Event timeline<br><sup>{b.name}</sup>",
        height=120 + 40 * len(kinds),
        xaxis_title="time [s]",
    )
    for k in kinds:
        es = [e for e in ev if e.kind == k]
        sym, col = EVENT_STYLE.get(k, ("circle", "#333"))
        f.add_scatter(
            x=[e.t_s for e in es],
            y=[k] * len(es),
            mode="markers",
            name=f"{k} ({len(es)})",
            marker=dict(symbol=sym, color=col, size=9),
            text=[f"{e.node}: {e.detail}" for e in es],
            hoverinfo="text+x",
        )
    return f


# ============================================================ sim: per run
def _track_base(f: go.Figure, cones_rows, *, faint: bool = False) -> None:
    for color in ("blue", "yellow", "orange"):
        pts = [(r[0], r[1]) for r in cones_rows if r[3] == color and r[2] == "gt"]
        if pts:
            f.add_scatter(
                x=[p[0] for p in pts],
                y=[p[1] for p in pts],
                mode="markers",
                name=f"{color} cones",
                marker=dict(
                    color=CONE_COLOR[color],
                    size=5 if faint else 7,
                    opacity=0.4 if faint else 0.9,
                ),
                showlegend=not faint,
            )


def sim_track_map(b: RunBundle) -> go.Figure | None:
    tr, mc = b.tables.get("trajectory"), b.tables.get("map_cones")
    if not tr or not mc:
        return None
    f = _fig(
        f"Track, driven line coloured by speed, penalties and failures<br><sup>{b.name}</sup>",
        height=700,
    )
    _track_base(f, mc.rows)
    src = tr.column("source")
    gi = [i for i, s in enumerate(src) if s == "gt"]
    gi = [gi[j] for j in _dec(len(gi), 5000)]
    x = [tr.rows[i][2] for i in gi]
    y = [tr.rows[i][3] for i in gi]
    spd = b.series["sim_track"].values["speed_mps"] if "sim_track" in b.series else None
    s_idx = _dec(len(spd), 5000) if spd is not None else None
    f.add_scatter(
        x=x,
        y=y,
        mode="markers",
        name="GT path (colour = speed)",
        marker=dict(
            size=3,
            color=spd[s_idx] if spd is not None else "#000",
            colorscale="Turbo",
            colorbar=dict(title="m/s", x=1.02),
            showscale=spd is not None,
        ),
    )
    si = [i for i, s in enumerate(src) if s == "slam"]
    si = [si[j] for j in _dec(len(si), 3000)]
    f.add_scatter(
        x=[tr.rows[i][2] for i in si],
        y=[tr.rows[i][3] for i in si],
        mode="lines",
        name="SLAM estimate",
        line=dict(color="rgba(46,125,50,0.35)", width=1),
    )
    for kind in ("doo", "off_course", "dnf", "lateral_disturbance"):
        es = [e for e in b.events if e.kind == kind and e.x is not None]
        if es:
            sym, col = EVENT_STYLE[kind]
            f.add_scatter(
                x=[e.x for e in es],
                y=[e.y for e in es],
                mode="markers",
                name=f"{kind} ({len(es)})",
                marker=dict(
                    symbol=sym, size=14, color=col, line=dict(width=1, color="white")
                ),
                text=[f"s={e.s_m:.0f} m: {e.detail}" for e in es],
                hoverinfo="text",
            )
    return _equal(f)


def sim_cte_heatmap(b: RunBundle) -> go.Figure | None:
    s = b.series.get("sim_track")
    if s is None:
        return None
    lap, sl, cte = s.values["lap"], s.values["s_lap_m"], s.values["cte_m"]
    edges = np.arange(0, np.nanmax(sl) + 4, 4.0)
    laps = np.unique(lap).astype(int)
    Z = np.full((len(laps), len(edges) - 1), np.nan)
    for i, lp in enumerate(laps):
        m = lap == lp
        idx = np.digitize(sl[m], edges) - 1
        sums = np.bincount(idx, cte[m], len(edges))[: len(edges) - 1]
        cnt = np.bincount(idx, None, len(edges))[: len(edges) - 1]
        Z[i] = np.where(cnt > 0, sums / np.maximum(cnt, 1), np.nan)
    f = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        row_heights=[0.45, 0.55],
        subplot_titles=(
            "cross-track error per lap [m] (+ = left of centerline)",
            "lap × distance heatmap",
        ),
    )
    for lp in laps:
        m = lap == lp
        f.add_scatter(
            x=sl[m],
            y=cte[m],
            mode="lines",
            name=f"lap {lp}",
            line=dict(width=1),
            row=1,
            col=1,
        )
    f.add_heatmap(
        x=(edges[:-1] + edges[1:]) / 2,
        y=[f"lap {lp}" for lp in laps],
        z=Z,
        colorscale="RdBu",
        zmid=0,
        colorbar=dict(title="m", y=0.28, len=0.55),
        row=2,
        col=1,
    )
    f.update_xaxes(title_text="distance along lap [m]", row=2, col=1)
    f.update_layout(
        title=f"Path tracking vs track distance<br><sup>{b.name}</sup>",
        height=700,
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def sim_speed_profile(b: RunBundle) -> go.Figure | None:
    s = b.series.get("sim_track")
    if s is None:
        return None
    f = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        subplot_titles=(
            "speed [m/s]",
            "lateral acceleration [m/s²]",
            "steering [rad] / throttle / brake",
        ),
    )
    lap, sl = s.values["lap"], s.values["s_lap_m"]
    for lp in np.unique(lap).astype(int):
        m = lap == lp
        show = dict(legendgroup=f"lap{lp}")
        f.add_scatter(
            x=sl[m],
            y=s.values["speed_mps"][m],
            name=f"lap {lp}",
            line=dict(width=1),
            row=1,
            col=1,
            **show,
        )
        f.add_scatter(
            x=sl[m],
            y=s.values["lat_acc_mps2"][m],
            showlegend=False,
            line=dict(width=1),
            row=2,
            col=1,
            **show,
        )
    last = lap == np.nanmax(lap)
    f.add_scatter(
        x=sl[last],
        y=s.values["steering_rad"][last],
        name="steer (last lap)",
        line=dict(color="#000"),
        row=3,
        col=1,
    )
    f.add_scatter(
        x=sl[last],
        y=s.values["throttle"][last],
        name="throttle",
        line=dict(color="#2e7d32"),
        row=3,
        col=1,
    )
    f.add_scatter(
        x=sl[last],
        y=-s.values["brake"][last],
        name="−brake",
        line=dict(color="#d62728"),
        row=3,
        col=1,
    )
    f.update_xaxes(title_text="distance along lap [m]", row=3, col=1)
    f.update_layout(
        title=f"Speed / dynamics profile per lap<br><sup>{b.name}</sup>",
        height=720,
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def sim_laps(b: RunBundle) -> go.Figure | None:
    t = b.tables.get("laps")
    if not t or not t.rows:
        return None
    lap, time_s, doo, oc, done = (
        t.column(c) for c in ("lap", "time_s", "doo", "off_course", "completed")
    )
    f = make_subplots(specs=[[{"secondary_y": True}]])
    f.add_bar(
        x=lap,
        y=[v or 0 for v in time_s],
        name="lap time",
        marker_color=["#1f77b4" if d else "#bdbdbd" for d in done],
    )
    f.add_scatter(
        x=lap,
        y=doo,
        name="DOO",
        mode="markers+lines",
        marker=dict(color="#ff7f0e", size=10),
        secondary_y=True,
    )
    f.add_scatter(
        x=lap,
        y=oc,
        name="off-course",
        mode="markers",
        marker=dict(color="#d62728", size=12, symbol="x"),
        secondary_y=True,
    )
    f.update_yaxes(
        title_text="lap time [s]",
        secondary_y=False,
        range=[min(v for v in time_s if v) * 0.9 if any(time_s) else 0, None],
    )
    f.update_yaxes(title_text="penalties", secondary_y=True, rangemode="tozero")
    f.update_layout(
        title=f"Laps and penalties (grey = not completed)<br><sup>{b.name}</sup>",
        height=400,
        **LAYOUT,
    )
    return f


def sim_perception(b: RunBundle) -> go.Figure | None:
    p, pr = b.series.get("sim_perception"), b.tables.get("perception_range")
    if p is None or not pr:
        return None
    f = make_subplots(
        rows=2,
        cols=2,
        specs=[[{"colspan": 2}, None], [{}, {}]],
        row_heights=[0.45, 0.55],
        subplot_titles=(
            "per-scan recall / precision (5 s rolling mean)",
            "recall vs GT range",
            "position error vs GT range [m]",
        ),
    )
    k = 50
    ker = np.ones(k) / k
    for name, col in (("recall", "#1f77b4"), ("precision", "#2ca02c")):
        v = np.nan_to_num(p.values[name], nan=np.nanmean(p.values[name]))
        f.add_scatter(
            x=p.step,
            y=np.convolve(v, ker, "same"),
            name=name,
            line=dict(color=col),
            row=1,
            col=1,
        )
    f.add_scatter(
        x=p.step,
        y=p.values["n_fp"],
        name="FP / scan",
        mode="markers",
        marker=dict(size=2, color="#d62728"),
        row=1,
        col=1,
    )
    mid = [(r[0] + r[1]) / 2 for r in pr.rows]
    f.add_scatter(
        x=mid,
        y=pr.column("recall"),
        mode="lines+markers",
        name="recall",
        showlegend=False,
        line=dict(color="#1f77b4"),
        row=2,
        col=1,
    )
    f.add_bar(
        x=mid,
        y=[n / max(pr.column("n_gt")) for n in pr.column("n_gt")],
        name="GT share",
        marker_color="rgba(0,0,0,0.12)",
        row=2,
        col=1,
    )
    f.add_scatter(
        x=mid,
        y=pr.column("pos_err_mean_m"),
        mode="lines+markers",
        name="pos err",
        showlegend=False,
        line=dict(color="#d62728"),
        row=2,
        col=2,
    )
    f.update_xaxes(title_text="range [m]", row=2)
    f.update_layout(
        title=f"Perception vs simulator GT<br><sup>{b.name}</sup>",
        height=640,
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def sim_latency(b: RunBundle) -> go.Figure | None:
    s = b.series.get("sim_latency")
    if s is None:
        return None
    nodes = ["perception_ms", "slam_ms", "planning_ms", "control_ms", "e2e_ms"]
    f = make_subplots(
        rows=1,
        cols=2,
        column_widths=[0.4, 0.6],
        subplot_titles=(
            "distribution per node [ms]",
            "end-to-end latency over time [ms]",
        ),
    )
    for i, n in enumerate(nodes):
        f.add_box(
            y=s.values[n],
            name=n.replace("_ms", ""),
            marker_color=PALETTE[i],
            boxpoints=False,
            row=1,
            col=1,
        )
    f.add_scatter(
        x=s.step,
        y=s.values["e2e_ms"],
        mode="markers",
        marker=dict(size=2, color="#000"),
        name="e2e",
        row=1,
        col=2,
    )
    f.add_scatter(
        x=s.step,
        y=s.values["cpu_frac"] * 100,
        name="CPU %",
        line=dict(color="#ff7f0e", width=1),
        row=1,
        col=2,
    )
    f.update_layout(
        title=f"Latency and compute<br><sup>{b.name}</sup>",
        height=460,
        showlegend=False,
        **{k: v for k, v in LAYOUT.items() if k != "legend"},
    )
    return f


def sim_localisation(b: RunBundle) -> go.Figure | None:
    s = b.series.get("sim_track")
    if s is None:
        return None
    f = _fig(
        f"Localisation error vs distance (loop closure after lap 1)<br><sup>{b.name}</sup>",
        height=380,
        xaxis_title="distance travelled [m]",
        yaxis_title="error vs GT [m]",
    )
    i = _dec(len(s), 4000)
    f.add_scatter(
        x=s.step[i],
        y=s.values["slam_err_m"][i],
        name="SLAM",
        line=dict(color="#2e7d32"),
    )
    f.add_scatter(
        x=s.step[i],
        y=s.values["odom_err_m"][i],
        name="EKF odom",
        line=dict(color="#e65100"),
    )
    for e in b.events:
        if e.kind == "loop_closure" and e.s_m is not None:
            f.add_vline(x=e.s_m, line=dict(color="#2ca02c", dash="dash"))
    return f


def sim_map_quality(b: RunBundle) -> go.Figure | None:
    mc = b.tables.get("map_cones")
    if not mc:
        return None
    f = _fig(f"SLAM map vs GT cones<br><sup>{b.name}</sup>", height=620)
    for src, sym, op in (("gt", "circle-open", 0.9), ("slam", "x", 0.8)):
        for color in ("blue", "yellow", "orange"):
            pts = [r for r in mc.rows if r[2] == src and r[3] == color]
            if pts:
                f.add_scatter(
                    x=[p[0] for p in pts],
                    y=[p[1] for p in pts],
                    mode="markers",
                    name=f"{src} {color}",
                    marker=dict(
                        symbol=sym, color=CONE_COLOR[color], size=9, opacity=op
                    ),
                )
    return _equal(f)


def sim_failure_zoom(b: RunBundle) -> go.Figure | None:
    """Last ~15 s before the first failure, zoomed. Stand-in for a video clip."""
    fail = next(
        (e for e in b.events if e.kind in ("dnf", "off_course") and e.x is not None),
        None,
    )
    tr, mc, s = (
        b.tables.get("trajectory"),
        b.tables.get("map_cones"),
        b.series.get("sim_track"),
    )
    if fail is None or not tr or s is None:
        return None
    t_end = fail.t_s or 0.0
    rows = [r for r in tr.rows if r[1] == "gt" and t_end - 15 <= r[0] <= t_end + 1]
    f = _fig(
        f"Failure snapshot: {fail.kind} at s={fail.s_m:.0f} m — {fail.detail}<br><sup>{b.name}</sup>",
        height=560,
    )
    _track_base(f, mc.rows)
    sp = interp(s.values["t_s"], s.values["speed_mps"], np.array([r[0] for r in rows]))
    f.add_scatter(
        x=[r[2] for r in rows],
        y=[r[3] for r in rows],
        mode="markers+lines",
        name="last 15 s",
        marker=dict(
            size=5,
            color=sp,
            colorscale="Turbo",
            showscale=True,
            colorbar=dict(title="m/s"),
        ),
    )
    f.add_scatter(
        x=[fail.x],
        y=[fail.y],
        mode="markers",
        name=fail.kind,
        marker=dict(symbol="star", size=20, color="#000"),
    )
    if rows:
        xs, ys = [r[2] for r in rows], [r[3] for r in rows]
        pad = 6
        f.update_xaxes(range=[min(xs) - pad, max(xs) + pad])
        f.update_yaxes(range=[min(ys) - pad, max(ys) + pad])
    return _equal(f)


# ============================================================ aggregate (N seeds)
def agg_seeds(b: RunBundle) -> go.Figure | None:
    t = b.tables.get("seeds")
    laps = b.tables.get("laps")
    if not t or not laps:
        return None
    f = make_subplots(
        rows=1,
        cols=2,
        column_widths=[0.55, 0.45],
        subplot_titles=(
            "lap times of every seed (racing laps)",
            "penalties and outcome per seed",
        ),
    )
    for seed in sorted(set(laps.column("seed"))):
        rows = [r for r in laps.rows if r[0] == seed and r[5] and r[1] > 1]
        f.add_box(
            y=[r[2] for r in rows],
            name=f"seed {seed}",
            boxpoints="all",
            jitter=0.4,
            row=1,
            col=1,
        )
    seeds = t.column("seed")
    fin = t.column("finished")
    f.add_bar(
        x=[f"seed {s}" for s in seeds],
        y=t.column("n_doo"),
        name="DOO",
        marker_color="#ff7f0e",
        row=1,
        col=2,
    )
    f.add_bar(
        x=[f"seed {s}" for s in seeds],
        y=t.column("n_off_track"),
        name="off-course",
        marker_color="#d62728",
        row=1,
        col=2,
    )
    f.add_scatter(
        x=[f"seed {s}" for s, ok in zip(seeds, fin) if not ok],
        y=[0.2] * (len(fin) - sum(map(bool, fin))),
        mode="markers+text",
        text=["DNF"] * (len(fin) - sum(map(bool, fin))),
        textposition="top center",
        marker=dict(symbol="star", size=16, color="#000"),
        name="DNF",
        row=1,
        col=2,
    )
    f.update_layout(
        title=f"Seed spread — finish rate {b.summary.get('race/finish_rate_frac', 0):.0%}, "
        f"lap {b.summary.get('race/lap_time_mean_s', float('nan')):.2f} "
        f"± {b.summary.get('race/lap_time_mean_ci95_s') or 0:.2f} s (95% CI)<br><sup>{b.name}</sup>",
        height=460,
        barmode="stack",
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def agg_profile(b: RunBundle) -> go.Figure | None:
    s = b.series.get("agg_track_profile")
    if s is None:
        return None
    d = b.series.get("agg_track_profile_delta")
    f = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        subplot_titles=(
            "|cross-track| over seeds [m]: mean and worst seed",
            "mean speed [m/s]",
        ),
    )
    f.add_scatter(
        x=s.step,
        y=s.values["cte_m_max"],
        name="worst seed",
        line=dict(color="rgba(214,39,40,0.5)"),
        row=1,
        col=1,
    )
    f.add_scatter(
        x=s.step,
        y=s.values["cte_m_mean"],
        name="mean",
        line=dict(color="#d62728"),
        row=1,
        col=1,
    )
    f.add_scatter(
        x=s.step,
        y=s.values["speed_mps_mean"],
        name="speed",
        line=dict(color="#1f77b4"),
        row=2,
        col=1,
    )
    if d is not None:
        f.add_scatter(
            x=d.step,
            y=d.values["baseline_cte_m_mean"],
            name="baseline mean |cte|",
            line=dict(color="#000", dash="dash"),
            row=1,
            col=1,
        )
        f.add_scatter(
            x=d.step,
            y=d.values["baseline_speed_mps_mean"],
            name="baseline speed",
            line=dict(color="#000", dash="dash"),
            row=2,
            col=1,
        )
    f.update_xaxes(title_text="distance along lap [m]", row=2, col=1)
    f.update_layout(
        title=f"Where on the track: tracking error and speed (dashed = baseline)<br><sup>{b.name}</sup>",
        height=560,
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def agg_hotspots(b: RunBundle) -> go.Figure | None:
    s, mc = b.series.get("agg_track_profile"), b.tables.get("map_cones")
    if s is None or not mc:
        return None
    # centerline from GT cones isn't in the bundle; approximate each s bin's position with the first seed's path
    f = _fig(
        f"Penalty and failure locations across all seeds<br><sup>{b.name}</sup>",
        height=680,
    )
    _track_base(f, mc.rows, faint=True)
    for kind in ("doo", "off_course", "dnf", "lateral_disturbance"):
        es = [e for e in b.events if e.kind == kind and e.x is not None]
        if es:
            sym, col = EVENT_STYLE[kind]
            f.add_scatter(
                x=[e.x for e in es],
                y=[e.y for e in es],
                mode="markers",
                name=f"{kind} ({len(es)})",
                marker=dict(
                    symbol=sym,
                    size=13,
                    color=col,
                    opacity=0.8,
                    line=dict(width=1, color="white"),
                ),
                text=[e.detail for e in es],
                hoverinfo="text",
            )
    return _equal(f)


PER_RUN_REPLAY: list[Callable[[RunBundle], go.Figure | None]] = [
    replay_route,
    replay_slam_odom_gap,
    replay_cone_counts,
    replay_cone_histogram,
    replay_funnel,
    replay_side_balance,
    replay_slam_latency,
    replay_slam_profile,
    replay_slam_obs,
    replay_planning,
    replay_control,
    replay_steer_residual_hist,
    replay_lifecycle,
    event_timeline,
]
PER_RUN_SIM: list[Callable[[RunBundle], go.Figure | None]] = [
    sim_track_map,
    sim_failure_zoom,
    sim_cte_heatmap,
    sim_speed_profile,
    sim_laps,
    sim_perception,
    sim_latency,
    sim_localisation,
    sim_map_quality,
    event_timeline,
]
PER_RUN_AGG: list[Callable[[RunBundle], go.Figure | None]] = [
    agg_seeds,
    agg_profile,
    agg_hotspots,
]


def per_run(b: RunBundle) -> dict[str, go.Figure]:
    """Section-prefixed figure keys, e.g. 'perception/cone_counts'."""
    builders = {
        "onboard_replay": PER_RUN_REPLAY,
        "live_replay": PER_RUN_REPLAY,
        "sim_e2e": PER_RUN_SIM,
        "sim_aggregate": PER_RUN_AGG,
        "sim_sweep_trial": PER_RUN_AGG,
    }.get(b.job_type, [])
    out = {}
    for fn in builders:
        fig = fn(b)
        if fig is not None:
            out[f"{SECTION.get(fn.__name__, 'misc')}/{fn.__name__}"] = fig
    return out


SECTION = {
    "replay_route": "trajectory",
    "replay_slam_odom_gap": "trajectory",
    "replay_cone_counts": "perception",
    "replay_cone_histogram": "perception",
    "replay_funnel": "perception",
    "replay_side_balance": "perception",
    "replay_slam_latency": "slam",
    "replay_slam_profile": "slam",
    "replay_slam_obs": "slam",
    "replay_planning": "planning",
    "replay_control": "control",
    "replay_steer_residual_hist": "control",
    "replay_lifecycle": "health",
    "event_timeline": "health",
    "sim_track_map": "race",
    "sim_failure_zoom": "race",
    "sim_cte_heatmap": "control",
    "sim_speed_profile": "control",
    "sim_laps": "race",
    "sim_perception": "perception",
    "sim_latency": "latency",
    "sim_localisation": "slam",
    "sim_map_quality": "slam",
    "agg_seeds": "reliability",
    "agg_profile": "control",
    "agg_hotspots": "race",
}


# ============================================================ comparisons (many runs)
def _runs_color(runs: list[RunBundle]) -> dict[str, str]:
    return {r.name: PALETTE[i % len(PALETTE)] for i, r in enumerate(runs)}


def cmp_routes(runs: list[RunBundle], *, sources=("odom", "slam")) -> go.Figure | None:
    runs = [r for r in runs if "trajectory" in r.tables]
    if not runs:
        return None
    col = _runs_color(runs)
    bag_ids = {r.config.get("scenario", {}).get("bag_id") for r in runs}
    frame = (
        "raw odom/map frame (same bag)"
        if len(bag_ids) == 1
        else "RAW frames of DIFFERENT bags — not aligned"
    )
    f = _fig(
        f"Route overlay — {frame}<br><sup>solid = odom, dotted = SLAM</sup>", height=700
    )
    for r in runs:
        t = r.tables["trajectory"]
        src = t.column("source")
        for s in sources:
            idx = [i for i, v in enumerate(src) if v == s]
            idx = [idx[j] for j in _dec(len(idx), 2500)]
            f.add_scatter(
                x=[t.rows[i][2] for i in idx],
                y=[t.rows[i][3] for i in idx],
                mode="lines",
                name=f"{short(r)} · {s}",
                legendgroup=r.name,
                line=dict(
                    color=col[r.name],
                    width=1.5,
                    dash="solid" if s in ("odom", "gt") else "dot",
                ),
            )
    return _equal(f)


def cmp_map_cones(runs: list[RunBundle]) -> go.Figure | None:
    runs = [r for r in runs if "map_cones" in r.tables]
    if not runs:
        return None
    col = _runs_color(runs)
    f = _fig("Final SLAM map overlay (landmark count in legend)", height=680)
    for r in runs:
        rows = [row for row in r.tables["map_cones"].rows if row[2] == "slam"]
        f.add_scatter(
            x=[p[0] for p in rows],
            y=[p[1] for p in rows],
            mode="markers",
            name=f"{short(r)} ({len(rows)})",
            marker=dict(symbol="triangle-up", size=8, color=col[r.name], opacity=0.75),
        )
    return _equal(f)


def cmp_series(
    runs: list[RunBundle],
    family: str,
    key: str,
    title: str,
    ytitle: str,
    *,
    smooth: int = 1,
    max_pts: int = 3000,
) -> go.Figure | None:
    runs = [r for r in runs if family in r.series and key in r.series[family].values]
    if not runs:
        return None
    col = _runs_color(runs)
    f = _fig(
        title,
        height=440,
        yaxis_title=ytitle,
        xaxis_title=runs[0].series[family].step_name,
    )
    for r in runs:
        s = r.series[family]
        v = s.values[key]
        if smooth > 1:
            v = np.convolve(np.nan_to_num(v), np.ones(smooth) / smooth, "same")
        i = _dec(len(s), max_pts)
        f.add_scatter(
            x=s.step[i],
            y=v[i],
            mode="lines",
            name=short(r),
            line=dict(color=col[r.name], width=1.3),
        )
    return f


def cmp_distributions(
    runs: list[RunBundle], family: str, keys: list[str], title: str
) -> go.Figure | None:
    runs = [r for r in runs if family in r.series]
    if not runs:
        return None
    f = make_subplots(rows=1, cols=len(keys), subplot_titles=keys)
    col = _runs_color(runs)
    for j, k in enumerate(keys, 1):
        for r in runs:
            v = r.series[family].values.get(k)
            if v is not None:
                f.add_box(
                    y=v,
                    name=short(r),
                    marker_color=col[r.name],
                    boxpoints=False,
                    showlegend=j == 1,
                    legendgroup=r.name,
                    row=1,
                    col=j,
                )
    f.update_layout(title=title, height=460, **{k: v for k, v in LAYOUT.items()})
    f.update_xaxes(showticklabels=False)
    return f


def cmp_metric_bars(
    runs: list[RunBundle],
    metrics: list[str],
    title: str,
    baseline: RunBundle | None = None,
    cols: int = 3,
) -> go.Figure | None:
    metrics = [
        m
        for m in metrics
        if any(isinstance(r.summary.get(m), (int, float)) for r in runs)
    ]
    if not metrics or not runs:
        return None
    rows = math.ceil(len(metrics) / cols)

    def arrow(m):
        sp = registry.spec(m)
        return {"min": " ↓", "max": " ↑"}.get(sp.direction if sp else "", "")

    f = make_subplots(
        rows=rows,
        cols=cols,
        subplot_titles=[
            f"<span style='font-size:11px'>{m.split('/', 1)[0]}/<br>{m.split('/', 1)[-1]}{arrow(m)}</span>"
            for m in metrics
        ],
        vertical_spacing=0.12 if rows < 4 else 0.07,
        horizontal_spacing=0.06,
    )
    col = _runs_color(runs)
    for i, m in enumerate(metrics):
        rr, cc = i // cols + 1, i % cols + 1
        for r in runs:
            v = r.summary.get(m)
            e = r.summary.get(f"{m}.std")
            f.add_bar(
                x=[short(r)],
                y=[v if isinstance(v, (int, float)) else None],
                name=short(r),
                marker_color=col[r.name],
                legendgroup=r.name,
                showlegend=i == 0,
                error_y=dict(type="data", array=[e or 0], visible=e is not None),
                row=rr,
                col=cc,
            )
        if baseline is not None and isinstance(baseline.summary.get(m), (int, float)):
            f.add_hline(
                y=baseline.summary[m],
                line=dict(color="#000", dash="dash", width=1),
                row=rr,
                col=cc,
            )
    f.update_xaxes(showticklabels=False)
    f.update_layout(
        title=title
        + "<br><sup>dashed = baseline · error bars = std over seeds · colour = run (legend)</sup>",
        height=240 * rows + 160,
        **{
            **LAYOUT,
            "margin": dict(l=55, r=20, t=110, b=45),
            "legend": dict(orientation="h", y=-0.04, yanchor="top"),
        },
    )
    return f


def cmp_delta_heatmap(
    runs: list[RunBundle], metrics: list[str], title: str
) -> go.Figure | None:
    """Relative delta vs baseline, coloured by better (green) / worse (red) per registry direction."""
    runs = [r for r in runs if "baseline_comparison" in r.tables]
    if not runs:
        return None
    Z, T = [], []
    for r in runs:
        tbl = {row[0]: row for row in r.tables["baseline_comparison"].rows}
        zr, tr = [], []
        for m in metrics:
            row = tbl.get(m)
            if row is None or row[4] is None or not math.isfinite(row[4]):
                zr.append(None)
                tr.append("")
                continue
            sign = -1 if row[5] == "min" else 1
            zr.append(max(-1, min(1, sign * row[4])))
            mark = {"regression": " ✗", "improvement": " ✓"}.get(row[7], "")
            tr.append(f"{row[3]:+.3g}{mark}")
        Z.append(zr)
        T.append(tr)
    f = go.Figure(
        go.Heatmap(
            z=Z,
            x=metrics,
            y=[short(r) for r in runs],
            text=T,
            texttemplate="%{text}",
            textfont=dict(size=12),
            xgap=2,
            ygap=2,
            colorscale=[[0, "#c62828"], [0.5, "#f5f5f5"], [1, "#2e7d32"]],
            zmid=0,
            zmin=-1,
            zmax=1,
            colorbar=dict(title="better →", tickformat=".0%"),
        )
    )
    f.update_layout(
        title=title
        + "<br><sup>cell = absolute delta (✗ regression / ✓ improvement beyond registry "
        "tolerance); colour = relative change, green = better</sup>",
        height=220 + 48 * len(runs),
        **{**LAYOUT, "margin": dict(l=260, r=20, t=80, b=170)},
    )
    f.update_xaxes(tickangle=35)
    return f


def cmp_event_counts(
    runs: list[RunBundle], title: str = "Event counts per run"
) -> go.Figure | None:
    kinds = sorted({e.kind for r in runs for e in r.events})
    if not kinds:
        return None
    Z = [[sum(1 for e in r.events if e.kind == k) for k in kinds] for r in runs]
    f = go.Figure(
        go.Heatmap(
            z=Z,
            x=kinds,
            y=[short(r) for r in runs],
            text=Z,
            texttemplate="%{text}",
            colorscale="OrRd",
        )
    )
    f.update_layout(
        title=title,
        height=120 + 38 * len(runs),
        **{**LAYOUT, "margin": dict(l=260, r=20, t=60, b=110)},
    )
    return f


def cmp_startup(runs: list[RunBundle]) -> go.Figure | None:
    runs = [r for r in runs if "lifecycle" in r.tables]
    if not runs:
        return None
    f = _fig(
        "Startup: when each node became active (0 = bag play start; aborted runs: first Ready)",
        height=440,
        xaxis_title="run",
        yaxis_title="seconds relative to t0",
    )
    nodes = sorted({row[0] for r in runs for row in r.tables["lifecycle"].rows})
    for i, n in enumerate(nodes):
        ys = []
        for r in runs:
            row = next((row for row in r.tables["lifecycle"].rows if row[0] == n), None)
            ys.append(row[3] if row else None)
        f.add_scatter(
            x=[short(r) for r in runs],
            y=ys,
            mode="markers+lines",
            name=f"{n} active",
            marker=dict(color=PALETTE[i], size=9),
        )
    return f


def cmp_sim_matrix(aggs: list[RunBundle], metrics: list[str]) -> go.Figure | None:
    """Scenario × commit table with seed spread — the headline view of a sim suite."""
    if not aggs:
        return None
    scen = sorted({a.config["scenario"]["name"] for a in aggs})
    commits = sorted({a.config["code"]["pipeline"]["sha"] for a in aggs})
    f = make_subplots(
        rows=len(metrics), cols=1, subplot_titles=metrics, vertical_spacing=0.06
    )
    for i, m in enumerate(metrics, 1):
        for j, c in enumerate(commits):
            ys, es, xs = [], [], []
            for s in scen:
                a = next(
                    (
                        a
                        for a in aggs
                        if a.config["scenario"]["name"] == s
                        and a.config["code"]["pipeline"]["sha"] == c
                    ),
                    None,
                )
                xs.append(s)
                ys.append(a.summary.get(m) if a else None)
                es.append(a.summary.get(f"{m}.std") if a else None)
            f.add_bar(
                x=xs,
                y=ys,
                name=c,
                marker_color=PALETTE[j],
                showlegend=i == 1,
                legendgroup=c,
                text=[f"{v:.3g}" if v is not None else "" for v in ys],
                textposition="inside",
                error_y=dict(type="data", array=[e or 0 for e in es]),
                row=i,
                col=1,
            )
    f.update_layout(
        title="Scenario × commit (mean over seeds, error bar = std)",
        barmode="group",
        height=200 * len(metrics) + 60,
        **{
            **LAYOUT,
            "legend": dict(
                orientation="h", y=1.02, x=1, xanchor="right", yanchor="bottom"
            ),
        },
    )
    return f


def cmp_lap_distributions(aggs: list[RunBundle]) -> go.Figure | None:
    aggs = [a for a in aggs if "laps" in a.tables]
    if not aggs:
        return None
    f = _fig(
        "Racing-lap time distribution per scenario × commit (all laps of all seeds)",
        height=480,
        yaxis_title="lap time [s]",
        boxmode="group",
    )
    commits = sorted({a.config["code"]["pipeline"]["sha"] for a in aggs})
    for j, c in enumerate(commits):
        xs, ys = [], []
        for a in aggs:
            if a.config["code"]["pipeline"]["sha"] != c:
                continue
            for r in a.tables["laps"].rows:
                if r[5] and r[1] > 1 and r[2]:
                    xs.append(a.config["scenario"]["name"])
                    ys.append(r[2])
        f.add_box(x=xs, y=ys, name=c, marker_color=PALETTE[j], boxpoints="outliers")
    return f


def cmp_profiles(
    aggs: list[RunBundle], key: str = "cte_m_mean", title: str = ""
) -> go.Figure | None:
    aggs = [a for a in aggs if "agg_track_profile" in a.series]
    if not aggs:
        return None
    col = _runs_color(aggs)
    f = _fig(
        title or f"{key} vs lap distance",
        height=440,
        xaxis_title="distance along lap [m]",
        yaxis_title=key,
    )
    for a in aggs:
        s = a.series["agg_track_profile"]
        f.add_scatter(
            x=s.step, y=s.values[key], name=short(a), line=dict(color=col[a.name])
        )
    return f


def cmp_recall_range(
    aggs_or_runs: list[RunBundle], seeds_by_group: dict[str, list[RunBundle]]
) -> go.Figure | None:
    f = _fig(
        "Perception recall vs GT range (pooled over seeds)",
        height=420,
        xaxis_title="range [m]",
        yaxis_title="recall",
    )
    any_ = False
    for i, a in enumerate(aggs_or_runs):
        seeds = seeds_by_group.get(a.group, [])
        tabs = [
            s.tables["perception_range"]
            for s in seeds
            if "perception_range" in s.tables
        ]
        if not tabs:
            continue
        any_ = True
        rows0 = tabs[0].rows
        gt = np.sum([[r[2] for r in t.rows] for t in tabs], 0)
        tp = np.sum([[r[3] for r in t.rows] for t in tabs], 0)
        f.add_scatter(
            x=[(r[0] + r[1]) / 2 for r in rows0],
            y=np.where(gt > 0, tp / np.maximum(gt, 1), np.nan),
            mode="lines+markers",
            name=short(a),
            line=dict(color=PALETTE[i % len(PALETTE)]),
        )
    return f if any_ else None


def cmp_latency_cdf(
    aggs: list[RunBundle],
    seeds_by_group: dict[str, list[RunBundle]],
    key: str = "e2e_ms",
) -> go.Figure | None:
    f = _fig(
        f"{key} CDF (pooled over seeds)",
        height=420,
        xaxis_title="ms",
        yaxis_title="fraction ≤ x",
    )
    any_ = False
    for i, a in enumerate(aggs):
        v = np.concatenate(
            [
                s.series["sim_latency"].values[key]
                for s in seeds_by_group.get(a.group, [])
                if "sim_latency" in s.series
            ]
            or [np.array([])]
        )
        if not v.size:
            continue
        any_ = True
        v = np.sort(v)
        j = _dec(len(v), 800)
        f.add_scatter(
            x=v[j],
            y=(j + 1) / len(v),
            name=short(a),
            line=dict(color=PALETTE[i % len(PALETTE)]),
        )
    return f if any_ else None


def cmp_significance(
    aggs: list[RunBundle],
    seeds_by_group: dict[str, list[RunBundle]],
    metrics: list[str],
) -> go.Figure | None:
    """Candidate vs baseline per scenario: bootstrap CI of the difference of means over seeds."""
    rows = []
    by_scen: dict[str, list[RunBundle]] = {}
    for a in aggs:
        by_scen.setdefault(a.scenario_id, []).append(a)
    for sid, group in by_scen.items():
        base = next((a for a in group if "baseline" in a.tags), None)
        if base is None:
            continue
        for a in group:
            if a is base:
                continue
            for m in metrics:
                bs = [s.summary.get(m) for s in seeds_by_group.get(base.group, [])]
                cs = [s.summary.get(m) for s in seeds_by_group.get(a.group, [])]
                d = bootstrap_diff(bs, cs)
                if d["diff"] is None:
                    continue
                # effect size: difference in units of the pooled seed std, so ms, m and counts share one axis
                pool = [
                    np.std([x for x in v if x is not None], ddof=1) for v in (bs, cs)
                ]
                ref = float(np.sqrt(np.nanmean(np.square(pool)))) or 1e-9
                d = {**d, **{k: d[k] / ref for k in ("diff", "lo", "hi")}}
                rows.append(
                    (f"{a.config['scenario']['name']} · {m}", d, registry.spec(m))
                )
    if not rows:
        return None
    f = _fig(
        "Candidate vs baseline: 95% bootstrap CI of the change over seeds (red = significantly worse, green = better)",
        height=140 + 26 * len(rows),
        xaxis_title="change of the mean, in pooled seed-std units (effect size)",
    )
    for i, (label, d, sp) in enumerate(rows):
        worse = (
            (d["lo"] > 0)
            if sp and sp.direction == "min"
            else (d["hi"] < 0)
            if sp and sp.direction == "max"
            else False
        )
        better = (
            (d["hi"] < 0)
            if sp and sp.direction == "min"
            else (d["lo"] > 0)
            if sp and sp.direction == "max"
            else False
        )
        c = "#c62828" if worse else "#2e7d32" if better else "#757575"
        f.add_scatter(
            x=[d["lo"], d["hi"]],
            y=[i, i],
            mode="lines",
            line=dict(color=c, width=4),
            showlegend=False,
            hoverinfo="skip",
        )
        f.add_scatter(
            x=[d["diff"]],
            y=[i],
            mode="markers",
            marker=dict(color=c, size=9),
            showlegend=False,
            hovertext=f"{label}: P(diff>0)={d['p_gt0']:.2f}",
            hoverinfo="text",
        )
    f.add_vline(x=0, line=dict(color="#000"))
    f.update_yaxes(
        tickmode="array",
        tickvals=list(range(len(rows))),
        ticktext=[r[0] for r in rows],
        autorange="reversed",
    )
    f.update_layout(margin=dict(l=380, r=20, t=60, b=50))
    return f


def cmp_nightly(
    aggs: list[RunBundle], metrics: list[str], baseline: RunBundle | None
) -> go.Figure | None:
    aggs = sorted(
        [a for a in aggs if a.config.get("nightly")],
        key=lambda a: a.config["nightly"]["night"],
    )
    if not aggs:
        return None
    f = make_subplots(
        rows=len(metrics),
        cols=1,
        shared_xaxes=True,
        subplot_titles=metrics,
        vertical_spacing=0.05,
    )
    x = [
        a.config["nightly"]["date"] + " " + a.config["code"]["pipeline"]["sha"]
        for a in aggs
    ]
    for i, m in enumerate(metrics, 1):
        y = np.array([a.summary.get(m, np.nan) or np.nan for a in aggs], float)
        lo = np.array([a.summary.get(f"{m}.min", np.nan) for a in aggs], float)
        hi = np.array([a.summary.get(f"{m}.max", np.nan) for a in aggs], float)
        f.add_scatter(
            x=x + x[::-1],
            y=np.r_[hi, lo[::-1]],
            fill="toself",
            fillcolor="rgba(31,119,180,0.15)",
            mode="lines",
            line=dict(width=0),
            showlegend=False,
            hoverinfo="skip",
            row=i,
            col=1,
        )
        bad = []
        for a, v in zip(aggs, y):
            bv = baseline.summary.get(m) if baseline else None
            d = registry.delta(m, v, bv) if bv is not None else {"regression": False}
            bad.append(bool(d["regression"]))
        f.add_scatter(
            x=x,
            y=y,
            mode="lines+markers",
            name=m,
            showlegend=False,
            marker=dict(size=10, color=["#c62828" if b_ else "#1f77b4" for b_ in bad]),
            row=i,
            col=1,
        )
        if baseline is not None and m in baseline.summary:
            f.add_hline(
                y=baseline.summary[m],
                line=dict(dash="dash", color="#000"),
                row=i,
                col=1,
            )
    f.update_layout(
        title="Nightly regression trend (band = min–max over seeds, dashed = baseline, red = regression)",
        height=210 * len(metrics) + 100,
        **{k: v for k, v in LAYOUT.items()},
    )
    return f


def cmp_sweep(
    trials: list[RunBundle], objective: str = "race/lap_time_mean_s"
) -> dict[str, go.Figure]:
    if not trials:
        return {}
    keys = sorted({k for t in trials for k in t.config.get("params", {})})
    out: dict[str, go.Figure] = {}
    dims = [
        dict(label=k.split(".")[-1], values=[t.config["params"].get(k) for t in trials])
        for k in keys
    ]
    for m in (
        objective,
        "race/n_doo",
        "control/cross_track_rms_m",
        "race/finish_rate_frac",
    ):
        dims.append(
            dict(label=m.split("/")[-1], values=[t.summary.get(m) for t in trials])
        )
    pc = go.Figure(
        go.Parcoords(
            line=dict(
                color=[t.summary.get(objective) for t in trials],
                colorscale="Viridis_r",
                showscale=True,
                colorbar=dict(title=objective.split("/")[-1]),
            ),
            dimensions=dims,
        )
    )
    pc.update_layout(
        title="Sweep: parameters → outcomes (parallel coordinates)",
        height=480,
        **LAYOUT,
    )
    out["sweep/parallel_coordinates"] = pc
    if len(keys) == 2:
        a, b = keys
        av, bv = (
            sorted({t.config["params"][a] for t in trials}),
            sorted({t.config["params"][b] for t in trials}),
        )
        fig = make_subplots(
            rows=1,
            cols=3,
            subplot_titles=(objective, "race/n_doo", "control/cross_track_rms_m"),
        )
        for j, m in enumerate(
            (objective, "race/n_doo", "control/cross_track_rms_m"), 1
        ):
            Z = [
                [
                    next(
                        (
                            t.summary.get(m)
                            for t in trials
                            if t.config["params"][a] == x and t.config["params"][b] == y
                        ),
                        None,
                    )
                    for x in av
                ]
                for y in bv
            ]
            fig.add_heatmap(
                x=[str(v) for v in av],
                y=[str(v) for v in bv],
                z=Z,
                texttemplate="%{z:.3g}",
                colorscale="Viridis_r",
                showscale=False,
                row=1,
                col=j,
            )
        fig.update_xaxes(title_text=a)
        fig.update_yaxes(title_text=b, col=1)
        fig.update_layout(title="Sweep grid", height=420, **LAYOUT)
        out["sweep/grid"] = fig
    pareto = _fig(
        "Sweep trade-off: lap time vs cones hit (colour = cross-track RMS)",
        height=440,
        xaxis_title="mean lap time [s]",
        yaxis_title="DOO per run",
    )
    pareto.add_scatter(
        x=[t.summary.get(objective) for t in trials],
        y=[t.summary.get("race/n_doo") for t in trials],
        mode="markers+text",
        text=[", ".join(f"{t.config['params'][k]}" for k in keys) for t in trials],
        textposition="top center",
        marker=dict(
            size=14,
            color=[t.summary.get("control/cross_track_rms_m") for t in trials],
            colorscale="Reds",
            showscale=True,
        ),
        error_x=dict(
            type="data", array=[t.summary.get(f"{objective}.std") or 0 for t in trials]
        ),
    )
    out["sweep/pareto"] = pareto
    return out


def headline_metrics(runs: Iterable[RunBundle]) -> list[str]:
    names = {k for r in runs for k in r.summary}
    return sorted(
        registry.headline(names), key=lambda m: list(registry.registry()).index(m)
    )
