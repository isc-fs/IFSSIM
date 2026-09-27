"""Figures for simulator bag sessions: perception and SLAM scored against the simulator's ground truth.

Time charts reuse ``replay.series_overlay`` (the series share the bag-replay time
axis, so the playhead works); the ones here need the per-match and per-source
tables, or draw the truth next to the estimate.
"""

from __future__ import annotations

import numpy as np
import plotly.graph_objects as go

from ..bundle import RunBundle
from .replay import dec, label, smooth_time
from .theme import CONE, base_layout, empty, equal

T = "time since bag start [s]"


def detections(
    runs: list[RunBundle], col: dict[str, str], height: int = 250
) -> go.Figure:
    """Cones detected per scan next to the true cones that were in view (dashed)."""
    f = go.Figure()
    for b in runs:
        s = b.series.get("gt_perception")
        if s is None:
            continue
        j = dec(len(s), 3000)
        for key, name, dash, w in (
            ("n_cones", "detected", "solid", 1.6),
            ("n_gt", "in view", "dash", 1.1),
        ):
            if key not in s.values:
                continue
            f.add_trace(
                go.Scatter(
                    x=s.step[j],
                    y=smooth_time(s.step, s.values[key], 1.0)[j],
                    mode="lines",
                    name=f"{label(b)} · {name}",
                    line=dict(color=col[b.run_id], width=w, dash=dash),
                )
            )
    if not f.data:
        return go.Figure()
    return base_layout(f, height=height, xtitle=T, ytitle="cones", legend=False)


def missed(runs: list[RunBundle], col: dict[str, str], height: int = 250) -> go.Figure:
    f = go.Figure()
    for b in runs:
        s = b.series.get("gt_perception")
        if s is None:
            continue
        j = dec(len(s), 3000)
        for key, name, dash in (("n_fn", "missed", "solid"), ("n_fp", "false", "dot")):
            if key in s.values:
                f.add_trace(
                    go.Scatter(
                        x=s.step[j],
                        y=smooth_time(s.step, s.values[key], 1.0)[j],
                        mode="lines",
                        name=f"{label(b)} · {name}",
                        line=dict(color=col[b.run_id], width=1.4, dash=dash),
                    )
                )
    if not f.data:
        return go.Figure()
    return base_layout(f, height=height, xtitle=T, ytitle="cones", legend=False)


def _matches(b: RunBundle) -> tuple[np.ndarray, np.ndarray] | None:
    t = b.tables.get("perception_matches")
    if not t or not t.rows:
        return None
    return (
        np.array(t.column("range_m"), float),
        np.array(t.column("err_m"), float),
    )


def error_vs_range(
    runs: list[RunBundle], col: dict[str, str], height: int = 340
) -> go.Figure:
    """Median position error per 1 m of range (line), p90 as a dotted line; single run adds the points."""
    f = go.Figure()
    one = len(runs) == 1
    for b in runs:
        m = _matches(b)
        if m is None:
            continue
        rg, err = m
        if one:
            j = dec(len(rg), 4000)
            f.add_trace(
                go.Scattergl(
                    x=rg[j],
                    y=err[j],
                    mode="markers",
                    name="each matched cone",
                    marker=dict(color=col[b.run_id], size=3, opacity=0.18),
                    hoverinfo="skip",
                )
            )
        edges = np.arange(0.0, np.ceil(np.nanmax(rg)) + 1.0, 1.0)
        mid, med, p90 = [], [], []
        for lo, hi in zip(edges[:-1], edges[1:]):
            e = err[(rg >= lo) & (rg < hi)]
            if e.size >= 5:
                mid.append((lo + hi) / 2)
                med.append(float(np.median(e)))
                p90.append(float(np.percentile(e, 90)))
        f.add_trace(
            go.Scatter(
                x=mid,
                y=med,
                mode="lines+markers",
                name=f"{label(b)} · median",
                line=dict(color=col[b.run_id], width=2),
                marker=dict(size=4),
                hovertemplate="%{x:.1f} m: median %{y:.3f} m<extra>"
                + label(b)
                + "</extra>",
            )
        )
        f.add_trace(
            go.Scatter(
                x=mid,
                y=p90,
                mode="lines",
                name=f"{label(b)} · p90",
                line=dict(color=col[b.run_id], width=1.2, dash="dot"),
                hovertemplate="%{x:.1f} m: p90 %{y:.3f} m<extra>"
                + label(b)
                + "</extra>",
            )
        )
    if not f.data:
        return go.Figure()
    base_layout(
        f, height=height, xtitle="range to the cone [m]", ytitle="position error [m]"
    )
    # legend above the plot: below it collides with the axis title
    f.update_layout(
        hovermode="closest",
        legend=dict(orientation="h", y=1.02, yanchor="bottom", x=0, xanchor="left"),
    )
    return f


def error_cdf(
    runs: list[RunBundle], col: dict[str, str], height: int = 320
) -> go.Figure:
    f = go.Figure()
    for b in runs:
        m = _matches(b)
        if m is None:
            continue
        v = np.sort(m[1][np.isfinite(m[1])])
        if not v.size:
            continue
        j = dec(len(v), 800)
        f.add_trace(
            go.Scatter(
                x=v[j],
                y=(j + 1) / len(v),
                mode="lines",
                name=f"{label(b)} · p95 {np.percentile(v, 95):.3f} m",
                line=dict(color=col[b.run_id], width=1.8),
            )
        )
    if not f.data:
        return go.Figure()
    base_layout(f, height=height, xtitle="position error [m]", ytitle="fraction ≤ x")
    f.add_hline(y=0.95, line=dict(color="#9ca3af", dash="dot", width=1))
    f.update_layout(hovermode="closest")
    return f


def pose_sources(
    runs: list[RunBundle], col: dict[str, str], height: int = 300
) -> go.Figure:
    """Mean position error of every pose source against the truth (bar), p95 as the whisker."""
    f = go.Figure()
    for b in runs:
        t = b.tables.get("pose_error_sources")
        if not t or not t.rows:
            continue
        src = t.column("source")
        mean = np.array(t.column("mean_err_m"), float)
        p95 = np.array(t.column("p95_err_m"), float)
        f.add_trace(
            go.Bar(
                y=src,
                x=mean,
                orientation="h",
                name=label(b),
                marker_color=col[b.run_id],
                error_x=dict(
                    type="data",
                    symmetric=False,
                    array=np.nan_to_num(p95 - mean),
                    arrayminus=np.zeros_like(mean),
                    thickness=1,
                    width=3,
                    color="#6b7280",
                ),
                hovertemplate="%{y}: mean %{x:.3f} m<extra>" + label(b) + "</extra>",
            )
        )
    if not f.data:
        return go.Figure()
    base_layout(f, height=height, legend=len(f.data) > 1)
    f.update_layout(barmode="group", hovermode="closest", margin=dict(l=110))
    f.update_xaxes(
        type="log",
        title_text="position error vs truth [m] (log) · whisker = p95",
        tickvals=[0.001, 0.01, 0.1, 1, 10, 100],
        ticktext=["1 mm", "1 cm", "10 cm", "1 m", "10 m", "100 m"],
    )
    f.update_yaxes(autorange="reversed", showspikes=False)
    return f


def final_map(
    runs: list[RunBundle], col: dict[str, str], height: int = 560
) -> go.Figure:
    """The true track cones (rings, in their colour) and each run's final SLAM landmarks (triangles)."""
    f = go.Figure()
    truth = next((b.tables["map_cones"] for b in runs if "map_cones" in b.tables), None)
    gt = [r for r in truth.rows if r[2] == "gt"] if truth else []
    if gt:
        f.add_trace(
            go.Scatter(
                x=[r[0] for r in gt],
                y=[r[1] for r in gt],
                mode="markers",
                name=f"true cones ({len(gt)})",
                marker=dict(
                    symbol="circle-open",
                    size=9,
                    color=[CONE.get(r[3], CONE["unknown"]) for r in gt],
                    line=dict(width=1.5),
                ),
            )
        )
    for b in runs:
        mc = b.tables.get("map_cones")
        if not mc:
            continue
        rows = [r for r in mc.rows if r[2] == "slam"]
        f.add_trace(
            go.Scatter(
                x=[r[0] for r in rows],
                y=[r[1] for r in rows],
                mode="markers",
                name=f"{label(b)} · SLAM map ({len(rows)})",
                marker=dict(
                    symbol="triangle-up",
                    size=7,
                    color=col[b.run_id],
                    opacity=0.85,
                    line=dict(width=0.5, color="#111"),
                ),
            )
        )
    if not f.data:
        return empty("No map in these runs")
    base_layout(f, height=height)
    f.update_layout(hovermode="closest")
    f.update_xaxes(showspikes=False)
    f.update_yaxes(showspikes=False)
    return equal(f)


def stages(runs: list[RunBundle], col: dict[str, str], height: int = 300) -> go.Figure:
    f = go.Figure()
    for b in runs:
        t = b.tables.get("perception_stages")
        if not t or not t.rows:
            continue
        f.add_trace(
            go.Bar(
                y=t.column("stage"),
                x=t.column("mean_ms"),
                orientation="h",
                name=label(b),
                marker_color=col[b.run_id],
                customdata=t.column("p95_ms"),
                hovertemplate="%{y}: mean %{x:.2f} ms · p95 %{customdata:.2f} ms<extra>"
                + label(b)
                + "</extra>",
            )
        )
    if not f.data:
        return go.Figure()
    base_layout(f, height=height, legend=len(f.data) > 1)
    f.update_layout(barmode="group", hovermode="closest", margin=dict(l=110))
    f.update_xaxes(title_text="mean time per scan [ms]")
    f.update_yaxes(autorange="reversed", showspikes=False)
    return f
