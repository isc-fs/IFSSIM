"""Colours and plotly defaults shared by every viewer figure.

One rule everywhere: the baseline is always near-black, and the other selected runs
take the palette in selection order. So a run keeps its colour on every tab.
"""

from __future__ import annotations

import plotly.graph_objects as go

BASELINE = "#222222"
PALETTE = [
    "#0072B2",
    "#D55E00",
    "#009E73",
    "#CC79A7",
    "#E69F00",
    "#56B4E9",
    "#8c564b",
    "#7f3b8f",
    "#b8860b",
    "#17becf",
    "#e7298a",
    "#66a61e",
]
GOOD, BAD, NEUTRAL = "#1b7a3a", "#c62828", "#6b7280"
CONE = {
    "blue": "#1565c0",
    "yellow": "#f2b90c",
    "orange": "#ef6c00",
    "big_orange": "#bf360c",
    "unknown": "#9e9e9e",
}
EVENT_SYMBOL = {
    "pose_jump_rejected": "x",
    "da_failure_skip": "diamond",
    "cascade_recovery": "star",
    "dbscan_guard": "triangle-up",
    "stop_latched": "square",
    "crash": "x-thin-open",
    "tf_lookup_failed": "circle-open",
    "doo": "triangle-down",
    "off_course": "x",
    "dnf": "star",
    "lateral_disturbance": "diamond-open",
    "latency_spike": "circle-open",
    "loop_closure": "hexagon",
    "proximity_veto": "cross",
    "lifecycle": "line-ns-open",
}

FONT = "Inter, system-ui, -apple-system, Segoe UI, Roboto, sans-serif"


SHORT = {
    "race/lap_time_mean_s": "lap time",
    "race/lap_time_best_s": "best lap",
    "race/n_doo": "cones hit",
    "race/finish_rate_frac": "finish rate",
    "control/cross_track_rms_m": "cross-track RMS",
    "latency/e2e_p95_ms": "latency p95",
    "race/n_off_track": "off-track",
    "perception/recall_frac": "recall",
}


def human(metric: str, *, unit: bool = False, short: bool = False) -> str:
    """A metric as people say it: the registry description (or a short name), not the storage key."""
    from .. import registry

    sp = registry.spec(metric)
    text = (SHORT.get(metric) if short else None) or (
        sp.desc if sp and sp.desc else metric.split("/")[-1].replace("_", " ")
    )
    if unit and sp and sp.unit and sp.unit not in ("count", "frac"):
        text += f" [{sp.unit}]"
    return text


def colors(run_ids: list[str], baseline: str | None) -> dict[str, str]:
    out, i = {}, 0
    for r in run_ids:
        if r == baseline:
            out[r] = BASELINE
        else:
            out[r] = PALETTE[i % len(PALETTE)]
            i += 1
    if baseline and baseline not in out:
        out[baseline] = BASELINE
    return out


def base_layout(
    f: go.Figure,
    *,
    title: str | None = None,
    height: int = 420,
    legend: bool = True,
    xtitle: str | None = None,
    ytitle: str | None = None,
) -> go.Figure:
    f.update_layout(
        template="plotly_white",
        height=height,
        font=dict(family=FONT, size=11.5, color="#374151"),
        margin=dict(l=60, r=18, t=46 if title else 18, b=44),
        hovermode="x unified" if xtitle else "closest",
        title=dict(text=title, x=0.01, xanchor="left", font=dict(size=14))
        if title
        else None,
        showlegend=legend,
        legend=dict(
            orientation="h",
            yanchor="top",
            y=-0.16 if xtitle else -0.08,
            x=0,
            xanchor="left",
            font=dict(size=11),
        ),
        hoverlabel=dict(font=dict(family=FONT, size=11)),
        uirevision="keep",
    )
    if xtitle:
        f.update_xaxes(title_text=xtitle)
    if ytitle:
        f.update_yaxes(title_text=ytitle)
    f.update_xaxes(
        showgrid=True,
        gridcolor="#f0f1f4",
        zeroline=False,
        showline=True,
        linecolor="#d1d5db",
        ticks="outside",
        ticklen=3,
        tickcolor="#d1d5db",
        showspikes=True,
        spikemode="across",
        spikesnap="cursor",
        spikethickness=1,
        spikecolor="#9ca3af",
        spikedash="solid",
    )
    f.update_yaxes(
        showgrid=True,
        gridcolor="#f0f1f4",
        zeroline=False,
        title_font=dict(size=11, color="#6b7280"),
    )
    return f


def empty(msg: str, height: int = 260) -> go.Figure:
    f = go.Figure()
    f.add_annotation(
        text=msg,
        showarrow=False,
        font=dict(size=13, color="#6b7280"),
        xref="paper",
        yref="paper",
        x=0.5,
        y=0.5,
    )
    f.update_layout(
        template="plotly_white",
        height=height,
        xaxis=dict(visible=False),
        yaxis=dict(visible=False),
        margin=dict(l=10, r=10, t=10, b=10),
    )
    return f


def equal(f: go.Figure) -> go.Figure:
    f.update_yaxes(scaleanchor="x", scaleratio=1)
    return f
