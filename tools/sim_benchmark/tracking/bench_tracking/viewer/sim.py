"""Simulator figures and statistics for the viewer (matrix, nightly, sweep).

Statistics come from the catalog: every seed is an MLflow child run with its
summary metrics, so candidate-vs-baseline confidence intervals need no
downloads. Plots along the track use the aggregate's ``agg_track_profile``
series plus one seed's ground-truth trajectory for the centreline geometry.
"""

from __future__ import annotations

import math

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from .. import registry
from ..bundle import RunBundle
from ..compare import bootstrap_diff
from .data import Catalog, RunRow
from .theme import (
    BAD,
    BASELINE,
    CONE,
    EVENT_SYMBOL,
    GOOD,
    NEUTRAL,
    PALETTE,
    base_layout,
    empty,
    equal,
    human,
)

FAIL_KINDS = ("doo", "off_course", "dnf", "lateral_disturbance", "latency_spike")


def short_scen(s: str) -> str:
    """trackdrive_T23_highnoise -> T23 highnoise: fits a subplot title or a tick."""
    return s.replace("trackdrive_", "").replace("_", " ")


def commit_colors(commits: list[str], baseline: str | None) -> dict[str, str]:
    out, i = {}, 0
    for c in commits:
        if c == baseline:
            out[c] = BASELINE
        else:
            out[c] = PALETTE[i % len(PALETTE)]
            i += 1
    return out


# ---------------------------------------------------------------- statistics
def verdict(metric: str, lo: float | None, hi: float | None, diff: float | None) -> str:
    sp = registry.spec(metric)
    if lo is None or sp is None or sp.direction == "none":
        return "n/a" if lo is None else "context"
    worse = lo > 0 if sp.direction == "min" else hi < 0
    better = hi < 0 if sp.direction == "min" else lo > 0
    tol = sp.threshold or 0.0
    if worse:
        return "worse" if abs(diff or 0) > tol else "worse (within tol.)"
    if better:
        return "better" if abs(diff or 0) > tol else "better (within tol.)"
    return "no clear change"


def scorecard(
    cat: Catalog,
    aggs: list[RunRow],
    base_commit: str,
    metrics: list[str],
    key=lambda a: a.commit,
) -> list[dict]:
    """One row per (scenario, candidate, metric): means ± std over seeds, Δ, 95% bootstrap CI.

    ``key`` says which report an aggregate belongs to (a commit for the matrix, the run itself for
    nightly and sweep); ``base_commit`` is the baseline's key.
    """
    out = []
    by_scen: dict[str, list[RunRow]] = {}
    for a in aggs:
        by_scen.setdefault(a.scenario, []).append(a)
    for scen, group in sorted(by_scen.items()):
        base = next((a for a in group if key(a) == base_commit), None)
        if base is None:
            continue
        bseeds = cat.seeds(base)
        for cand in group:
            if cand is base:
                continue
            cseeds = cat.seeds(cand)
            for m in metrics:
                bv = [s.summary.get(m) for s in bseeds]
                cv = [s.summary.get(m) for s in cseeds]
                bv = [v for v in bv if v is not None and math.isfinite(v)]
                cv = [v for v in cv if v is not None and math.isfinite(v)]
                if not bv or not cv:
                    continue
                d = bootstrap_diff(bv, cv)
                sp = registry.spec(m)
                p_better = None
                if (
                    d["p_gt0"] is not None
                    and sp
                    and sp.direction != "none"
                    and (d["lo"], d["hi"]) != (0.0, 0.0)
                ):
                    p_better = 1 - d["p_gt0"] if sp.direction == "min" else d["p_gt0"]
                out.append(
                    {
                        "scenario": scen,
                        "candidate": key(cand),
                        "metric": m,
                        "unit": sp.unit if sp else "",
                        "better": {"min": "lower", "max": "higher"}.get(
                            sp.direction if sp else "", "—"
                        ),
                        "baseline": float(np.mean(bv)),
                        "baseline_std": float(np.std(bv, ddof=1))
                        if len(bv) > 1
                        else 0.0,
                        "value": float(np.mean(cv)),
                        "value_std": float(np.std(cv, ddof=1)) if len(cv) > 1 else 0.0,
                        "delta": d["diff"]
                        if d["diff"] is not None
                        else float(np.mean(cv) - np.mean(bv)),
                        "ci_lo": d["lo"],
                        "ci_hi": d["hi"],
                        "p_better": p_better,
                        "n": f"{len(cv)} vs {len(bv)}",
                        "verdict": verdict(m, d["lo"], d["hi"], d["diff"]),
                        "desc": sp.desc if sp else "",
                        "base_id": base.run_id,
                        "cand_id": cand.run_id,
                    }
                )
    return out


def seed_strip(
    cat: Catalog,
    aggs: list[RunRow],
    metric: str,
    ccol: dict[str, str],
    height: int = 420,
    key=lambda a: a.commit,
    names: dict[str, str] | None = None,
) -> go.Figure:
    """One panel per scenario with its own y axis (lap times differ ×2 between scenarios), a box per commit."""
    scen = sorted({a.scenario for a in aggs})
    if not scen:
        return empty("No aggregates selected")
    f = make_subplots(
        rows=1, cols=len(scen), subplot_titles=scen, horizontal_spacing=0.06
    )
    for j, sc in enumerate(scen, 1):
        for commit in ccol:
            ys, txt = [], []
            for a in aggs:
                if key(a) != commit or a.scenario != sc:
                    continue
                for s in cat.seeds(a):
                    v = s.summary.get(metric)
                    if v is not None:
                        ys.append(v)
                        txt.append(f"seed {s.params.get('scenario.seed')}")
            if ys:
                nm = (names or {}).get(commit, commit)
                f.add_trace(
                    go.Box(
                        x=[nm] * len(ys),
                        y=ys,
                        name=nm,
                        marker_color=ccol[commit],
                        boxpoints="all",
                        jitter=0.35,
                        pointpos=0,
                        line=dict(width=1.5),
                        fillcolor="rgba(0,0,0,0)",
                        text=txt,
                        legendgroup=commit,
                        showlegend=j == 1,
                        marker=dict(size=8),
                        hovertemplate=sc
                        + "<br>%{text}: %{y:.4g}<extra>"
                        + nm
                        + "</extra>",
                    ),
                    row=1,
                    col=j,
                )
    if not f.data:
        return empty(f"No seed values for {metric}")
    base_layout(f, height=height)
    f.update_layout(hovermode="closest")
    f.update_annotations(font=dict(size=12))
    return f


# ---------------------------------------------------------------- along-track view (linked)
def centreline(
    seed: RunBundle | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    if seed is None or "trajectory" not in seed.tables:
        return None
    t = seed.tables["trajectory"]
    rows = [r for r in t.rows if r[1] == "gt" and r[5] == 1]
    if len(rows) < 10:
        return None
    s = np.array([r[4] for r in rows], float)
    o = np.argsort(s)
    return (
        s[o],
        np.array([r[2] for r in rows], float)[o],
        np.array([r[3] for r in rows], float)[o],
    )


def xy_at(cl, s_query) -> tuple[np.ndarray, np.ndarray]:
    s, x, y = cl
    q = (
        np.mod(np.asarray(s_query, float), s[-1])
        if s[-1] > 0
        else np.asarray(s_query, float)
    )
    return np.interp(q, s, x), np.interp(q, s, y)


PROFILE_LANES = [
    (
        "cte",
        "Mean |cross-track| [m] (dotted = max over seeds)",
        "cte_m_mean",
        "cte_m_max",
    ),
    ("cte_d", "Δ mean |cross-track| vs baseline [m]", None, None),
    ("speed", "Mean speed [m/s] (dotted = max)", "speed_mps_mean", "speed_mps_max"),
    ("speed_d", "Δ mean speed vs baseline [m/s]", None, None),
    ("slam", "Mean SLAM error [m]", "slam_err_m_mean", "slam_err_m_max"),
    ("fail", "Penalties & incidents (all seeds)", None, None),
]


def track_profiles(
    aggs: list[RunBundle],
    base: RunBundle | None,
    col: dict[str, str],
    lap_len: float,
    names: dict[str, str] | None = None,
) -> go.Figure:
    """Stacked along-track lanes. The Δ lanes only appear when there is a baseline and something to compare."""
    deltas = base is not None and any(a is not base for a in aggs)
    lanes = [la for la in PROFILE_LANES if deltas or la[0] not in ("cte_d", "speed_d")]
    hmap = {"cte": 190, "cte_d": 120, "speed": 190, "speed_d": 120, "slam": 150}
    heights = [hmap.get(la[0], 40 + 28 * len(aggs)) for la in lanes]
    total = sum(heights)
    f = make_subplots(
        rows=len(lanes),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=30 / total,
        row_heights=[h / total for h in heights],
    )
    names = {a.run_id: (names or {}).get(a.run_id) or _agg_label(a) for a in aggs}  # type: ignore[attr-defined]
    for i, (key, title, mean_k, max_k) in enumerate(lanes, 1):
        for a in aggs:
            rid = a.run_id  # type: ignore[attr-defined]
            p = a.series.get("agg_track_profile")
            if key == "fail":
                by: dict[str, list] = {}
                for e in a.events:
                    if e.kind in FAIL_KINDS and e.s_m is not None:
                        by.setdefault(e.kind, []).append(e)
                for kind, es in by.items():
                    f.add_trace(
                        go.Scatter(
                            x=[math.fmod(e.s_m, lap_len) for e in es],
                            y=[names[rid]] * len(es),
                            mode="markers",
                            showlegend=False,
                            marker=dict(
                                symbol=EVENT_SYMBOL.get(kind, "circle"),
                                size=11,
                                color=col[rid],
                                line=dict(width=1, color="#111"),
                            ),
                            customdata=[[kind, e.detail] for e in es],
                            hovertemplate="%{customdata[0]}: %{customdata[1]}<extra>"
                            + names[rid]
                            + "</extra>",
                        ),
                        row=i,
                        col=1,
                    )
                continue
            if p is None:
                continue
            if mean_k is None:
                if base is None or a is base or "agg_track_profile" not in base.series:
                    continue
                pb = base.series["agg_track_profile"]
                k = "cte_m_mean" if key == "cte_d" else "speed_mps_mean"
                v = p.values[k] - np.interp(p.step, pb.step, pb.values[k])
                f.add_trace(
                    go.Scatter(
                        x=p.step,
                        y=v,
                        mode="lines",
                        name=names[rid],
                        legendgroup=rid,
                        showlegend=False,
                        line=dict(color=col[rid], width=1.6),
                        fill="tozeroy",
                        fillcolor=_alpha(col[rid], 0.12),
                        hovertemplate="%{y:+.3f}<extra>" + names[rid] + "</extra>",
                    ),
                    row=i,
                    col=1,
                )
                continue
            f.add_trace(
                go.Scatter(
                    x=p.step,
                    y=p.values[mean_k],
                    mode="lines",
                    name=names[rid],
                    legendgroup=rid,
                    showlegend=i == 1,
                    line=dict(color=col[rid], width=2.4 if a is base else 1.6),
                    hovertemplate="%{y:.3f}<extra>" + names[rid] + "</extra>",
                ),
                row=i,
                col=1,
            )
            if max_k and max_k in p.values:
                f.add_trace(
                    go.Scatter(
                        x=p.step,
                        y=p.values[max_k],
                        mode="lines",
                        legendgroup=rid,
                        showlegend=False,
                        line=dict(color=col[rid], width=1, dash="dot"),
                        hoverinfo="skip",
                    ),
                    row=i,
                    col=1,
                )
        if key in ("cte_d", "speed_d"):
            f.add_hline(y=0, line=dict(color="#6b7280", width=1), row=i, col=1)
        f.add_annotation(
            text=f"<b>{title}</b>",
            xref="paper",
            yref=f"y{'' if i == 1 else i} domain",
            x=0,
            y=1,
            xanchor="left",
            yanchor="bottom",
            showarrow=False,
            font=dict(size=12, color="#374151"),
        )
    f.update_yaxes(type="category", row=len(lanes), col=1, tickfont=dict(size=10))
    base_layout(f, height=total + 90 + 30 * len(lanes))
    f.update_layout(
        hovermode="x unified",
        margin=dict(l=64, r=18, t=64, b=40),
        legend=dict(
            orientation="h", yanchor="bottom", y=1.0 + 28 / total, x=0, xanchor="left"
        ),
    )
    f.update_xaxes(title_text="distance along the lap [m]", row=len(lanes), col=1)
    return f


def track_map(
    aggs: list[RunBundle],
    base: RunBundle | None,
    cand: RunBundle | None,
    col: dict[str, str],
    cl,
    metric: str = "cte_m_mean",
    height: int = 660,
    names: dict[str, str] | None = None,
) -> tuple[go.Figure, list[int]]:
    """Track cones + centreline coloured by candidate − baseline, incidents, and one cursor trace (last)."""
    f = go.Figure()
    ref = base or (aggs[0] if aggs else None)
    if ref is None or cl is None:
        return empty("No track geometry for this scenario"), []
    mc = ref.tables.get("map_cones")
    if mc:
        rows = [r for r in mc.rows if r[2] == "gt"]
        f.add_trace(
            go.Scatter(
                x=[r[0] for r in rows],
                y=[r[1] for r in rows],
                mode="markers",
                name="track cones",
                marker=dict(
                    symbol="triangle-up",
                    size=7,
                    color=[CONE.get(r[3], "#999") for r in rows],
                    line=dict(width=0.5, color="#374151"),
                ),
                hoverinfo="skip",
            )
        )
    s, x, y = cl
    if (
        base is not None
        and cand is not None
        and "agg_track_profile" in cand.series
        and "agg_track_profile" in base.series
    ):
        p, pb = cand.series["agg_track_profile"], base.series["agg_track_profile"]
        d = p.values[metric] - np.interp(p.step, pb.step, pb.values[metric])
        sp = registry.spec(
            "control/cross_track_rms_m"
            if metric.startswith("cte")
            else "control/speed_mean_mps"
        )
        worse_pos = sp.direction == "min" if sp else True
        lim = float(np.nanmax(np.abs(d))) or 1.0
        px, py = xy_at(cl, p.step)
        f.add_trace(
            go.Scatter(
                x=px,
                y=py,
                mode="markers",
                name=f"{(names or {}).get(cand.run_id) or _agg_label(cand)} − reference",  # type: ignore[attr-defined]
                marker=dict(
                    size=9,
                    color=d,
                    cmin=-lim,
                    cmax=lim,
                    colorscale="RdBu" if worse_pos else "RdBu_r",
                    reversescale=True,
                    colorbar=dict(
                        title=dict(text="Δ", side="right"), thickness=12, x=1.0
                    ),
                ),
                customdata=np.c_[p.step, d],
                hovertemplate="s=%{customdata[0]:.0f} m<br>Δ=%{customdata[1]:+.3f}<extra></extra>",
            )
        )
    else:
        f.add_trace(
            go.Scatter(
                x=x,
                y=y,
                mode="lines",
                name="centreline",
                line=dict(color="#9ca3af", width=2),
                customdata=s,
                hovertemplate="s=%{customdata:.0f} m<extra></extra>",
            )
        )
    for a in aggs:
        rid = a.run_id  # type: ignore[attr-defined]
        by: dict[str, list] = {}
        for e in a.events:
            if e.kind in FAIL_KINDS and e.s_m is not None:
                by.setdefault(e.kind, []).append(e)
        for kind, es in by.items():
            ex = [
                e.x if e.x is not None else float(xy_at(cl, [e.s_m])[0][0]) for e in es
            ]
            ey = [
                e.y if e.y is not None else float(xy_at(cl, [e.s_m])[1][0]) for e in es
            ]
            f.add_trace(
                go.Scatter(
                    x=ex,
                    y=ey,
                    mode="markers",
                    name=f"{kind} · {(names or {}).get(rid) or _agg_label(a)} ({len(es)})",
                    legendgroup=rid,
                    marker=dict(
                        symbol=EVENT_SYMBOL.get(kind, "circle"),
                        size=13,
                        color=col[rid],
                        line=dict(width=1, color="white"),
                    ),
                    text=[e.detail for e in es],
                    hovertemplate="%{text}<extra></extra>",
                )
            )
    # start/finish
    f.add_trace(
        go.Scatter(
            x=[x[0]],
            y=[y[0]],
            mode="markers+text",
            text=["start"],
            textposition="top center",
            marker=dict(symbol="square", size=11, color="#111"),
            showlegend=False,
            hoverinfo="skip",
        )
    )
    f.add_trace(
        go.Scatter(
            x=[],
            y=[],
            mode="markers",
            showlegend=False,
            hoverinfo="skip",
            marker=dict(
                size=20, color="rgba(0,0,0,0)", line=dict(width=3, color="#111")
            ),
        )
    )
    base_layout(f, height=height)
    f.update_layout(
        hovermode="closest",
        margin=dict(l=40, r=10, t=10, b=30),
        legend=dict(
            orientation="h",
            x=0,
            y=-0.05,
            xanchor="left",
            yanchor="top",
            font=dict(size=10),
        ),
    )
    f.update_xaxes(showspikes=False)
    return equal(f), [len(f.data) - 1]


def _agg_label(a: RunBundle) -> str:
    return a.config.get("code", {}).get("pipeline", {}).get("sha", a.name)[:7]


def _alpha(hex_color: str, a: float) -> str:
    h = hex_color.lstrip("#")
    r, g, b = (int(h[i : i + 2], 16) for i in (0, 2, 4))
    return f"rgba({r},{g},{b},{a})"


# ---------------------------------------------------------------- laps, perception, compute
def lap_times(
    cat: Catalog,
    aggs: list[RunRow],
    bundles: dict[str, RunBundle],
    ccol: dict[str, str],
    height: int = 440,
    key=lambda a: a.commit,
    names: dict[str, str] | None = None,
) -> go.Figure:
    """One panel per scenario (own y axis). Multi-lap events drop lap 1, which includes the standing start."""
    scen = sorted({a.scenario for a in aggs})
    f = make_subplots(
        rows=1, cols=max(len(scen), 1), subplot_titles=scen, horizontal_spacing=0.06
    )
    for j, sc in enumerate(scen, 1):
        for commit in ccol:
            ys, txt = [], []
            for a in aggs:
                if (
                    key(a) != commit
                    or a.scenario != sc
                    or a.run_id not in bundles
                    or "laps" not in bundles[a.run_id].tables
                ):
                    continue
                rows = bundles[a.run_id].tables["laps"].rows
                multi = max((r[1] for r in rows), default=1) > 1
                for seed, lap, t, doo, off, done in rows:
                    if done and t and not (multi and lap == 1):
                        ys.append(t)
                        txt.append(
                            f"seed {seed} lap {lap}"
                            + (f" · {doo} DOO" if doo else "")
                            + (" · OFF" if off else "")
                        )
            if ys:
                nm = (names or {}).get(commit, commit)
                f.add_trace(
                    go.Box(
                        x=[nm] * len(ys),
                        y=ys,
                        name=nm,
                        marker_color=ccol[commit],
                        boxpoints="all",
                        jitter=0.4,
                        pointpos=0,
                        fillcolor="rgba(0,0,0,0)",
                        text=txt,
                        legendgroup=commit,
                        showlegend=j == 1,
                        hovertemplate="%{text}: %{y:.2f} s<extra>" + nm + "</extra>",
                    ),
                    row=1,
                    col=j,
                )
    if not f.data:
        return empty("No lap data")
    base_layout(
        f,
        title="Lap times, every completed lap of every seed (multi-lap events: lap 1 excluded)",
        height=height,
    )
    f.update_layout(hovermode="closest")
    f.update_yaxes(title_text="s", col=1)
    return f


def reliability(
    cat: Catalog,
    aggs: list[RunRow],
    ccol: dict[str, str],
    height: int = 380,
    key=lambda a: a.commit,
    names: dict[str, str] | None = None,
) -> go.Figure:
    metrics = [
        ("race/finish_rate_frac", "finish rate"),
        ("race/n_doo", "cones hit / run"),
        ("race/n_off_track", "off-track / run"),
        ("race/n_dnf", "DNFs"),
    ]
    f = make_subplots(
        rows=1,
        cols=len(metrics),
        subplot_titles=[t for _, t in metrics],
        horizontal_spacing=0.06,
    )
    for j, (m, _) in enumerate(metrics, 1):
        for commit in ccol:
            rows = [a for a in aggs if key(a) == commit]
            f.add_bar(
                x=[short_scen(a.scenario) for a in rows],
                y=[a.summary.get(m) for a in rows],
                name=(names or {}).get(commit, commit),
                marker_color=ccol[commit],
                showlegend=j == 1,
                legendgroup=commit,
                error_y=dict(
                    type="data", array=[a.summary.get(f"{m}.std") or 0 for a in rows]
                ),
                row=1,
                col=j,
            )
    base_layout(
        f, title="Reliability per scenario (mean over seeds, bar = std)", height=height
    )
    f.update_layout(
        barmode="group",
        hovermode="closest",
        margin=dict(t=80, b=70),
        legend=dict(orientation="h", y=1.12, yanchor="bottom", x=1, xanchor="right"),
    )
    f.update_xaxes(tickangle=-35, tickfont=dict(size=10))
    return f


def recall_range(
    aggs: list[RunRow],
    seed_bundles: dict[str, list[RunBundle]],
    ccol: dict[str, str],
    height: int = 400,
    key=lambda a: a.commit,
    names: dict[str, str] | None = None,
) -> go.Figure:
    f = go.Figure()
    for a in aggs:
        tabs = [
            s.tables["perception_range"]
            for s in seed_bundles.get(a.run_id, [])
            if "perception_range" in s.tables
        ]
        if not tabs:
            continue
        r0 = tabs[0].rows
        gt = np.sum([[r[2] for r in t.rows] for t in tabs], 0)
        tp = np.sum([[r[3] for r in t.rows] for t in tabs], 0)
        f.add_trace(
            go.Scatter(
                x=[(r[0] + r[1]) / 2 for r in r0],
                y=np.where(gt > 0, tp / np.maximum(gt, 1), np.nan),
                mode="lines+markers",
                name=f"{a.scenario} · {(names or {}).get(key(a), key(a))}",
                line=dict(color=ccol.get(key(a), "#999"), dash=_dash(a.scenario)),
            )
        )
    if not f.data:
        return empty("No perception range tables")
    base_layout(
        f,
        title="Perception recall vs range (pooled over seeds) · line style = scenario",
        height=height,
        xtitle="range [m]",
        ytitle="recall",
    )
    return f


def latency(
    aggs: list[RunRow],
    seed_bundles: dict[str, list[RunBundle]],
    ccol: dict[str, str],
    height: int = 400,
    key=lambda a: a.commit,
    names: dict[str, str] | None = None,
) -> tuple[go.Figure, go.Figure]:
    cdf = go.Figure()
    stages = ["perception_ms", "slam_ms", "planning_ms", "control_ms"]
    shades = ["#2a9d8f", "#e9c46a", "#f4a261", "#e76f51"]
    bars = go.Figure()
    labels, means = [], {s: [] for s in stages}
    for a in aggs:
        ss = [
            s.series["sim_latency"]
            for s in seed_bundles.get(a.run_id, [])
            if "sim_latency" in s.series
        ]
        if not ss:
            continue
        v = np.sort(np.concatenate([s.values["e2e_ms"] for s in ss]))
        j = np.unique(np.linspace(0, len(v) - 1, min(len(v), 800)).astype(int))
        cdf.add_trace(
            go.Scatter(
                x=v[j],
                y=(j + 1) / len(v),
                mode="lines",
                name=f"{a.scenario} · {(names or {}).get(key(a), key(a))} · p95 {np.percentile(v, 95):.0f} ms",
                line=dict(color=ccol.get(key(a), "#999"), dash=_dash(a.scenario)),
            )
        )
        labels.append(
            f"{a.scenario.replace('trackdrive_', 'td_')}<br>{(names or {}).get(key(a), key(a))}"
        )
        for s_ in stages:
            means[s_].append(float(np.mean(np.concatenate([s.values[s_] for s in ss]))))
    for s_, c in zip(stages, shades):
        bars.add_bar(x=labels, y=means[s_], name=s_.replace("_ms", ""), marker_color=c)
    if not cdf.data:
        return empty("No latency series"), empty("No latency series")
    base_layout(
        cdf,
        title="End-to-end latency CDF (pooled over seeds)",
        height=height,
        xtitle="ms",
        ytitle="fraction ≤ x",
    )
    cdf.update_layout(hovermode="closest")
    cdf.add_hline(y=0.95, line=dict(color="#9ca3af", dash="dot", width=1))
    base_layout(
        bars,
        title="Mean latency per pipeline stage (stacked)",
        height=height,
        ytitle="ms",
    )
    bars.update_layout(barmode="stack", hovermode="closest")
    return cdf, bars


# ---------------------------------------------------------------- nightly
def nightly_trend(
    nights: list[RunRow],
    metrics: list[str],
    base: RunRow | None,
    selected: list[str],
    col: dict[str, str],
) -> go.Figure:
    nights = sorted(nights, key=lambda r: int(r.params.get("nightly.night") or 0))
    if not nights:
        return empty("No nightly runs")
    x = [f"{r.params.get('nightly.date', '')}<br>{r.commit}" for r in nights]
    f = make_subplots(
        rows=len(metrics),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.035,
        subplot_titles=[
            f"<b>{human(m)}</b>  <span style='color:#9ca3af'>"
            f"({'lower' if (registry.spec(m) and registry.spec(m).direction == 'min') else 'higher'}"
            " is better)</span>"
            for m in metrics
        ],
    )
    for i, m in enumerate(metrics, 1):
        y = np.array([r.summary.get(m, np.nan) for r in nights], float)
        lo = np.array([r.summary.get(f"{m}.min", np.nan) for r in nights], float)
        hi = np.array([r.summary.get(f"{m}.max", np.nan) for r in nights], float)
        f.add_trace(
            go.Scatter(
                x=x + x[::-1],
                y=np.r_[hi, lo[::-1]],
                fill="toself",
                fillcolor="rgba(0,114,178,0.12)",
                mode="lines",
                line=dict(width=0),
                showlegend=False,
                hoverinfo="skip",
            ),
            row=i,
            col=1,
        )
        bv = base.summary.get(m) if base else None
        colors, sizes = [], []
        for r, v in zip(nights, y):
            d = (
                registry.delta(m, v, bv)
                if bv is not None
                else {"regression": None, "improvement": None}
            )
            colors.append(
                BAD if d["regression"] else GOOD if d["improvement"] else "#0072B2"
            )
            sizes.append(15 if r.run_id in selected else 9)
        f.add_trace(
            go.Scatter(
                x=x,
                y=y,
                mode="lines+markers",
                showlegend=False,
                line=dict(color="#0072B2", width=1.5),
                marker=dict(
                    size=sizes,
                    color=colors,
                    line=dict(
                        width=[3 if r.run_id in selected else 0 for r in nights],
                        color=[col.get(r.run_id, "#111") for r in nights],
                    ),
                ),
                customdata=[[r.run_id, r.summary.get(f"{m}.std")] for r in nights],
                hovertemplate="%{y:.4g} ± %{customdata[1]:.3g}<extra>"
                + human(m)
                + "</extra>",
            ),
            row=i,
            col=1,
        )
        if bv is not None:
            f.add_hline(
                y=bv, line=dict(color=BASELINE, dash="dash", width=1), row=i, col=1
            )
    base_layout(f, height=190 * len(metrics) + 80, legend=False)
    f.update_layout(hovermode="closest", margin=dict(l=64, r=18, t=40, b=60))
    f.update_annotations(font=dict(size=12), x=0, xanchor="left")
    return f


# ---------------------------------------------------------------- sweep
def sweep_grid(trials: list[RunRow], metric: str, selected: list[str]) -> go.Figure:
    a, b = "params.control.lookahead_gain", "params.control.max_lat_acc"
    av = sorted({float(t.params[a]) for t in trials if a in t.params})
    bv = sorted({float(t.params[b]) for t in trials if b in t.params})
    if not av or not bv:
        return empty("Sweep trials have no 2-D parameter grid")
    Z, S, ID = [], [], []
    for yb in bv:
        zr, sr, ir = [], [], []
        for xa in av:
            t = next(
                (
                    t
                    for t in trials
                    if float(t.params.get(a, "nan")) == xa
                    and float(t.params.get(b, "nan")) == yb
                ),
                None,
            )
            zr.append(t.summary.get(metric) if t else None)
            sr.append(t.summary.get(f"{metric}.std") if t else None)
            ir.append(t.run_id if t else "")
        Z.append(zr)
        S.append(sr)
        ID.append(ir)
    sp = registry.spec(metric)
    txt = [
        [
            ("" if z is None else f"{z:.3g}") + ("" if s is None else f"<br>±{s:.2g}")
            for z, s in zip(zr, sr)
        ]
        for zr, sr in zip(Z, S)
    ]
    f = go.Figure(
        go.Heatmap(
            x=[str(v) for v in av],
            y=[str(v) for v in bv],
            z=Z,
            text=txt,
            texttemplate="%{text}",
            customdata=ID,
            colorscale="Viridis" if sp and sp.direction == "max" else "Viridis_r",
            colorbar=dict(
                title=dict(text=sp.unit if sp else "", side="right"), thickness=12
            ),
            hovertemplate="lookahead %{x} · max_lat_acc %{y}<br>%{z:.4g}<extra></extra>",
        )
    )
    for i, row in enumerate(ID):
        for j, rid in enumerate(row):
            if rid in selected:
                f.add_shape(
                    type="rect",
                    x0=j - 0.5,
                    x1=j + 0.5,
                    y0=i - 0.5,
                    y1=i + 0.5,
                    line=dict(color="#fff", width=3),
                )
    base_layout(
        f,
        height=460,
        xtitle="lookahead gain",
        ytitle="max lateral acceleration [m/s²]",
        legend=False,
    )
    f.update_layout(hovermode="closest")
    f.update_xaxes(showspikes=False, type="category")
    f.update_yaxes(type="category")
    return f


def sweep_pareto(
    trials: list[RunRow], xm: str, ym: str, cm: str, selected: list[str]
) -> go.Figure:
    f = go.Figure(
        go.Scatter(
            x=[t.summary.get(xm) for t in trials],
            y=[t.summary.get(ym) for t in trials],
            mode="markers+text",
            text=[t.short.split(" · ", 1)[1] for t in trials],
            textposition="top center",
            textfont=dict(size=10),
            customdata=[t.run_id for t in trials],
            error_x=dict(
                type="data",
                array=[t.summary.get(f"{xm}.std") or 0 for t in trials],
                color="#9ca3af",
            ),
            marker=dict(
                size=[20 if t.run_id in selected else 13 for t in trials],
                color=[t.summary.get(cm) for t in trials],
                colorscale="Reds",
                showscale=True,
                colorbar=dict(
                    title=dict(text=human(cm, unit=True), side="right"), thickness=12
                ),
                line=dict(
                    width=[3 if t.run_id in selected else 0.5 for t in trials],
                    color="#111",
                ),
            ),
        )
    )
    base_layout(
        f,
        height=460,
        xtitle=human(xm, unit=True),
        ytitle=human(ym, unit=True),
        legend=False,
    )
    f.update_layout(hovermode="closest")
    return f


def sweep_parcoords(trials: list[RunRow], objective: str) -> go.Figure:
    dims = []
    for k, lab in (
        ("params.control.lookahead_gain", "lookahead"),
        ("params.control.max_lat_acc", "max_lat_acc"),
    ):
        dims.append(
            dict(label=lab, values=[float(t.params.get(k, "nan")) for t in trials])
        )
    for m in (
        objective,
        "race/n_doo",
        "control/cross_track_rms_m",
        "race/finish_rate_frac",
        "latency/e2e_p95_ms",
    ):
        dims.append(
            dict(
                label=human(m, short=True),
                values=[t.summary.get(m, np.nan) for t in trials],
            )
        )
    f = go.Figure(
        go.Parcoords(
            line=dict(
                color=[t.summary.get(objective) for t in trials],
                colorscale="Viridis_r",
                showscale=True,
            ),
            dimensions=dims,
        )
    )
    base_layout(
        f,
        title="Parameters → outcomes (drag on an axis to filter)",
        height=460,
        legend=False,
    )
    f.update_layout(margin=dict(l=70, r=40, t=90, b=30))
    return f


def equalize(f: go.Figure) -> go.Figure:
    return equal(f)


__all__ = [
    "scorecard",
    "seed_strip",
    "track_profiles",
    "track_map",
    "centreline",
    "xy_at",
    "lap_times",
    "reliability",
    "recall_range",
    "latency",
    "nightly_trend",
    "sweep_grid",
    "sweep_pareto",
    "sweep_parcoords",
    "commit_colors",
    "NEUTRAL",
]


def _dash(scenario: str) -> str:
    import zlib

    return ["solid", "dash", "dot", "dashdot"][zlib.crc32(scenario.encode()) % 4]
