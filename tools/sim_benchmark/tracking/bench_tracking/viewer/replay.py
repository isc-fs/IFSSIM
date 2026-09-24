"""Bag-replay figures for the viewer.

The centre of the replay view is the *synchronised timeline*: stacked lanes on
one time axis (0 = bag play start, which is also the log time base). It is
linked to a route map. Everything else here is a comparison figure sized for a
full-width card.

Onboard (``--report``) runs have sampled topic streams (``replay_*``). Live runs
only have what the node logs say (``log_*``). A lane lists its sources in order
of preference, so a live run still shows up where its logs carry the quantity.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from ..bundle import RunBundle
from ..geometry import interp
from .theme import BASELINE, CONE, EVENT_SYMBOL, base_layout, empty, equal

SLAM_MODE = {0: "mapping", 1: "localisation", 2: "skip"}


def label(b: RunBundle) -> str:
    """The name people see: the viewer sets ``display`` (e.g. "Tue 22 Sep 17:38"); else the run dir name."""
    shown = getattr(b, "display", None)
    if shown:
        return shown
    return b.name.split("/")[-1] + (" (live)" if b.job_type == "live_replay" else "")


def dec(n: int, max_pts: int) -> np.ndarray:
    return (
        np.arange(n)
        if n <= max_pts
        else np.unique(np.linspace(0, n - 1, max_pts).astype(int))
    )


def smooth_time(t: np.ndarray, v: np.ndarray, window_s: float) -> np.ndarray:
    """Trailing mean over ``window_s`` seconds (sample rate agnostic, NaN-aware)."""
    if window_s <= 0 or len(v) < 2:
        return v
    ok = np.isfinite(v)
    cs = np.r_[0, np.cumsum(np.where(ok, v, 0))]
    cn = np.r_[0, np.cumsum(ok)]
    lo = np.searchsorted(t, t - window_s, side="left")
    hi = np.arange(1, len(v) + 1)
    n = cn[hi] - cn[lo]
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(n > 0, (cs[hi] - cs[lo]) / n, np.nan)


@dataclass
class Lane:
    key: str
    title: str
    sources: list[tuple[str, str]] = field(default_factory=list)
    smooth_s: float = 0.0  # applied when the "smooth" toggle is on
    height: int = 150
    fmt: str = ".2f"

    def pick(self, b: RunBundle) -> tuple[np.ndarray, np.ndarray, str] | None:
        for fam, key in self.sources:
            s = b.series.get(fam)
            if s is not None and key in s.values and np.isfinite(s.values[key]).any():
                return s.step, s.values[key], f"{fam}.{key}"
        return None


LANES: list[Lane] = [
    Lane(
        "cones",
        "Cone detections / scan",
        [("replay_perception", "n_cones_raw"), ("log_perception_filter", "accepted")],
        smooth_s=1.0,
        fmt=".1f",
    ),
    Lane("cones_delta", "Cones Δ vs baseline (1 s mean)", fmt="+.1f"),
    Lane("map", "SLAM map size [landmarks]", [("log_slam_profile", "map")], fmt=".0f"),
    Lane("gap", "SLAM − odom gap [m]", [("replay_pose", "slam_odom_gap_m")]),
    Lane(
        "slam_ms",
        "SLAM processing [ms]",
        [("log_slam_latency", "proc_ms"), ("log_slam_profile", "total_ms")],
        smooth_s=1.0,
        fmt=".1f",
    ),
    Lane("assoc", "SLAM associated / observed", fmt=".2f"),
    Lane(
        "speed",
        "Speed [m/s]",
        [("replay_pose", "odom_speed_mps"), ("log_control_status", "v_mps")],
        smooth_s=0.5,
    ),
    Lane(
        "steer",
        "Steering (dashed grey = pilot)",
        [("replay_control", "steering_rad"), ("log_control_status", "steer")],
        fmt=".3f",
    ),
    Lane(
        "throttle",
        "Throttle",
        [("replay_control", "throttle"), ("log_control_status", "throttle")],
        fmt=".2f",
    ),
    Lane(
        "path_hz",
        "/Path publish rate [Hz]",
        [("log_planning_rate", "pub_hz")],
        fmt=".1f",
    ),
    Lane("events", "Pipeline events"),
]
LANE = {lane.key: lane for lane in LANES}
DEFAULT_LANES = [
    "cones",
    "cones_delta",
    "map",
    "gap",
    "slam_ms",
    "speed",
    "steer",
    "events",
]


def lane_data(lane: Lane, b: RunBundle, base: RunBundle | None, smooth: bool):
    """(t, v, source) for a lane and run, or None when the run has no data for it."""
    if lane.key == "cones_delta":
        if base is None or b is base:
            return None
        a, c = LANE["cones"].pick(b), LANE["cones"].pick(base)
        if a is None or c is None or a[2] != c[2]:
            return None
        va, vc = smooth_time(a[0], a[1], 1.0), smooth_time(c[0], c[1], 1.0)
        return a[0], va - interp(c[0], vc, a[0]), a[2]
    if lane.key == "assoc":
        s = b.series.get("log_slam_obs")
        if s is None:
            return None
        with np.errstate(invalid="ignore", divide="ignore"):
            return s.step, s.values["assoc"] / s.values["obs"], "log_slam_obs.assoc/obs"
    got = lane.pick(b)
    if got is None:
        return None
    t, v, src = got
    if smooth and lane.smooth_s:
        v = smooth_time(t, v, lane.smooth_s)
    return t, v, src


def timeline(
    runs: list[RunBundle],
    base: RunBundle | None,
    col: dict[str, str],
    lanes: list[str],
    smooth: bool,
    cursor: float | None = None,
) -> go.Figure:
    lanes = [LANE[k] for k in lanes if k in LANE] or [LANE["cones"]]
    ev_h = 40 + 26 * len(runs)
    heights = [ev_h if la.key == "events" else la.height for la in lanes]
    total = sum(heights)
    f = make_subplots(
        rows=len(lanes),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=30 / total,
        row_heights=[h / total for h in heights],
    )
    shown: set[str] = set()
    for i, la in enumerate(lanes, 1):
        if la.key == "events":
            _events_lane(f, runs, col, i)
            continue
        any_ = False
        for b in runs:
            got = lane_data(la, b, base, smooth)
            if got is None:
                continue
            t, v, src = got
            # WebGL copes; stride decimation would alias the 40 Hz control signals
            j = dec(len(t), 12000)
            rid = b.run_id  # type: ignore[attr-defined]
            f.add_trace(
                go.Scattergl(
                    x=t[j],
                    y=v[j],
                    mode="lines",
                    name=label(b),
                    legendgroup=rid,
                    showlegend=rid not in shown,
                    line=dict(
                        color=col.get(rid, "#999"), width=2.2 if b is base else 1.4
                    ),
                    hovertemplate=f"%{{y:{la.fmt}}}<extra>{label(b)}</extra>",
                ),
                row=i,
                col=1,
            )
            shown.add(rid)
            any_ = True
        if la.key == "steer":
            pil = next((b for b in runs if "replay_control" in b.series), None)
            if pil is not None:
                s = pil.series["replay_control"]
                j = dec(len(s), 2500)
                f.add_trace(
                    go.Scattergl(
                        x=s.step[j],
                        y=s.values["pilot_steering_rad"][j],
                        mode="lines",
                        name="pilot",
                        line=dict(color="#9ca3af", width=1.2, dash="dash"),
                        showlegend=False,
                        hovertemplate="%{y:.3f}<extra>pilot</extra>",
                    ),
                    row=i,
                    col=1,
                )
        if la.key == "cones_delta" and any_:
            f.add_hline(y=0, line=dict(color="#6b7280", width=1), row=i, col=1)
        f.add_annotation(
            text=f"<b>{la.title}</b>"
            + ("" if any_ else "  <i>(no data for the selected runs)</i>"),
            xref="paper",
            yref=f"y{'' if i == 1 else i} domain",
            x=0,
            y=1.0,
            xanchor="left",
            yanchor="bottom",
            showarrow=False,
            font=dict(size=12, color="#374151"),
        )
    base_layout(f, height=total + 90 + 30 * len(lanes), legend=True)
    f.update_layout(
        hovermode="x unified",
        margin=dict(l=64, r=18, t=64, b=40),
        legend=dict(
            orientation="h", yanchor="bottom", y=1.0 + 28 / total, x=0, xanchor="left"
        ),
    )
    f.update_xaxes(title_text="time since bag play start [s]", row=len(lanes), col=1)
    if cursor is not None:
        f.add_shape(
            type="line",
            x0=cursor,
            x1=cursor,
            xref="x",
            yref="paper",
            y0=0,
            y1=1,
            line=dict(color="#111", width=1.5, dash="dot"),
            name="cursor",
        )
    return f


def _events_lane(
    f: go.Figure, runs: list[RunBundle], col: dict[str, str], row: int
) -> None:
    names = [label(b) for b in runs]
    kinds_seen: set[str] = set()
    for yi, b in enumerate(runs):
        by: dict[str, list] = {}
        for e in b.events:
            if e.t_s is not None:
                by.setdefault(e.kind, []).append(e)
        for kind, es in by.items():
            kinds_seen.add(kind)
            f.add_trace(
                go.Scatter(
                    x=[e.t_s for e in es],
                    y=[names[yi]] * len(es),
                    mode="markers",
                    name=kind,
                    legendgroup=f"ev:{kind}",
                    showlegend=False,
                    marker=dict(
                        symbol=EVENT_SYMBOL.get(kind, "circle"),
                        size=9,
                        color=col.get(b.run_id, "#999"),  # type: ignore[attr-defined]
                        line=dict(width=1, color="#111"),
                    ),
                    customdata=[[kind, e.node, e.detail[:140]] for e in es],
                    hovertemplate="%{customdata[0]} · %{customdata[1]}<br>%{customdata[2]}<extra>"
                    + names[yi]
                    + "</extra>",
                ),
                row=row,
                col=1,
            )
    f.update_yaxes(
        type="category",
        categoryorder="array",
        categoryarray=names[::-1],
        row=row,
        col=1,
        tickfont=dict(size=10),
    )
    f.add_annotation(
        text="symbols: " + " · ".join(f"{k}" for k in sorted(kinds_seen)),
        xref="paper",
        yref=f"y{'' if row == 1 else row} domain",
        x=1,
        y=1.0,
        xanchor="right",
        yanchor="bottom",
        showarrow=False,
        font=dict(size=10, color="#6b7280"),
    )


# ---------------------------------------------------------------- route map (linked to the timeline)
@dataclass
class MapIndex:
    """Trace indices the callbacks patch: per run, base routes, window overlay and the cursor markers."""

    base: list[int] = field(default_factory=list)
    window: dict[str, int] = field(default_factory=dict)
    cursor_odom: dict[str, int] = field(default_factory=dict)
    cursor_slam: dict[str, int] = field(default_factory=dict)


def route_map(
    runs: list[RunBundle],
    base: RunBundle | None,
    col: dict[str, str],
    *,
    all_maps: bool,
    show_events: bool,
    height: int = 640,
) -> tuple[go.Figure, MapIndex]:
    f = go.Figure()
    idx = MapIndex()
    posed = [b for b in runs if "replay_pose" in b.series]
    if not posed:
        return empty(
            "None of the selected runs has a pose stream.<br>Live (--live) runs only have node logs.",
            height,
        ), idx
    map_src = [b for b in posed if all_maps] or (
        [base] if base is not None and base in posed else posed[:1]
    )
    for b in map_src:
        mc = b.tables.get("map_cones")
        if not mc:
            continue
        rows = [r for r in mc.rows if r[2] == "slam"]
        f.add_trace(
            go.Scatter(
                x=[r[0] for r in rows],
                y=[r[1] for r in rows],
                mode="markers",
                name=f"final SLAM map · {label(b)} ({len(rows)})",
                marker=dict(
                    symbol="triangle-up",
                    size=7 if b is base else 6,
                    color=[CONE.get(r[3], CONE["unknown"]) for r in rows]
                    if b is base or not all_maps
                    else col[b.run_id],  # type: ignore[attr-defined]
                    opacity=0.85 if b is base else 0.55,
                    line=dict(width=0.5, color="#374151"),
                ),
                hovertemplate="%{x:.1f}, %{y:.1f}<extra>landmark</extra>",
            )
        )
    for b in posed:
        s = b.series["replay_pose"]
        rid = b.run_id  # type: ignore[attr-defined]
        j = dec(len(s), 2500)
        for src, dash in (("odom", "solid"), ("slam", "dot")):
            f.add_trace(
                go.Scatter(
                    x=s.values[f"{src}_x"][j],
                    y=s.values[f"{src}_y"][j],
                    mode="lines",
                    name=f"{label(b)} · {src}",
                    legendgroup=rid,
                    showlegend=False,
                    opacity=0.55,
                    line=dict(
                        color=col[rid], width=1.6 if b is base else 1.2, dash=dash
                    ),
                    customdata=s.step[j],
                    hovertemplate=f"{label(b)} {src}<br>t=%{{customdata:.1f}} s<extra></extra>",
                )
            )
            idx.base.append(len(f.data) - 1)
    if show_events:
        for b in posed:
            s = b.series["replay_pose"]
            by: dict[str, list] = {}
            for e in b.events:
                if e.t_s is not None and e.kind != "dbscan_guard":
                    by.setdefault(e.kind, []).append(e)
            for kind, es in by.items():
                te = np.array([e.t_s for e in es])
                f.add_trace(
                    go.Scatter(
                        x=interp(s.step, s.values["odom_x"], te),
                        y=interp(s.step, s.values["odom_y"], te),
                        mode="markers",
                        name=f"{kind}",
                        legendgroup=f"ev:{kind}",
                        showlegend=b is posed[0],
                        marker=dict(
                            symbol=EVENT_SYMBOL.get(kind, "circle"),
                            size=9,
                            color=col[b.run_id],  # type: ignore[attr-defined]
                            line=dict(width=1, color="#111"),
                        ),
                        customdata=np.c_[te, [e.detail[:120] for e in es]],
                        hovertemplate=f"{kind} · {label(b)}<br>t=%{{customdata[0]:.1f}} s<br>%{{customdata[1]}}<extra></extra>",
                    )
                )
    for b in posed:
        rid = b.run_id  # type: ignore[attr-defined]
        # zoomed stretch, or the trail behind the playhead
        f.add_trace(
            go.Scatter(
                x=[],
                y=[],
                mode="lines",
                line=dict(color=col[rid], width=4.5),
                showlegend=False,
                hoverinfo="skip",
            )
        )
        idx.window[rid] = len(f.data) - 1
    for b in posed:
        rid = b.run_id  # type: ignore[attr-defined]
        f.add_trace(
            go.Scatter(
                x=[],
                y=[],
                mode="markers",
                showlegend=False,
                hoverinfo="skip",
                marker=dict(
                    size=16,
                    color=col[rid],
                    line=dict(width=2, color="white"),
                    symbol="circle",
                ),
            )
        )
        idx.cursor_odom[rid] = len(f.data) - 1
        f.add_trace(
            go.Scatter(
                x=[],
                y=[],
                mode="markers",
                showlegend=False,
                hoverinfo="skip",
                marker=dict(
                    size=16,
                    color="rgba(0,0,0,0)",
                    line=dict(width=2.5, color=col[rid]),
                    symbol="circle",
                ),
            )
        )
        idx.cursor_slam[rid] = len(f.data) - 1
    base_layout(f, height=height, legend=True)
    f.update_layout(
        hovermode="closest",
        margin=dict(l=40, r=10, t=30, b=30),
        title=dict(
            text="solid = odom · dotted = SLAM · colour = run",
            x=0.01,
            font=dict(size=12),
        ),
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
    f.update_yaxes(showspikes=False)
    return equal(f), idx


def pose_at(b: RunBundle, t: float) -> dict[str, float] | None:
    s = b.series.get("replay_pose")
    if s is None or t < s.step[0] - 1 or t > s.step[-1] + 1:
        return None
    return {
        k: float(np.interp(t, s.step, s.values[k]))
        for k in ("odom_x", "odom_y", "slam_x", "slam_y")
    }


def window_xy(b: RunBundle, t0: float, t1: float) -> tuple[list, list]:
    s = b.series.get("replay_pose")
    if s is None:
        return [], []
    m = (s.step >= t0) & (s.step <= t1)
    j = np.where(m)[0]
    j = j[dec(len(j), 1500)]
    return s.values["odom_x"][j].tolist(), s.values["odom_y"][j].tolist()


def value_at(
    t_arr: np.ndarray, v: np.ndarray, t: float, stale_s: float = 2.0
) -> float | None:
    if len(t_arr) == 0:
        return None
    i = int(np.searchsorted(t_arr, t, side="right")) - 1
    if i < 0 or t - t_arr[i] > stale_s or not np.isfinite(v[i]):
        return None
    return float(v[i])


SNAPSHOT_COLS = [
    ("speed", "speed m/s", ".1f"),
    ("steer", "steer", "+.3f"),
    ("cones", "cones", ".0f"),
    ("map", "map", ".0f"),
    ("gap", "gap m", ".2f"),
    ("slam_ms", "SLAM ms", ".1f"),
]


def snapshot(
    runs: list[RunBundle], base: RunBundle | None, t: float
) -> tuple[list[dict], list[dict]]:
    """Per-run values at time t and events within ±2 s, for the side panel."""
    rows, events = [], []
    for b in runs:
        r = {"run": label(b), "run_id": b.run_id}  # type: ignore[attr-defined]
        for key, _, _ in SNAPSHOT_COLS:
            got = lane_data(LANE[key], b, base, smooth=False)
            r[key] = value_at(got[0], got[1], t) if got else None
        prof = b.series.get("log_slam_profile")
        mode = (
            value_at(prof.step, prof.values["mode"], t, 3.0)
            if prof is not None
            else None
        )
        r["mode"] = SLAM_MODE.get(int(mode), "?") if mode is not None else None
        rows.append(r)
        for e in b.events:
            if e.t_s is not None and abs(e.t_s - t) <= 2.0:
                events.append(
                    {
                        "run_id": b.run_id,
                        "run": label(b),
                        "t": e.t_s,
                        "kind": e.kind,  # type: ignore[attr-defined]
                        "detail": e.detail[:110],
                    }
                )
    events.sort(key=lambda e: e["t"])
    return rows, events


# ---------------------------------------------------------------- comparison figures (tabs)
def overlay(
    runs: list[RunBundle],
    col: dict[str, str],
    lane_key: str,
    *,
    base: RunBundle | None,
    smooth: bool,
    title: str,
    ytitle: str = "",
    height: int = 380,
) -> go.Figure:
    la = LANE[lane_key]
    f = go.Figure()
    for b in runs:
        got = lane_data(la, b, base, smooth)
        if got is None:
            continue
        j = dec(len(got[0]), 3000)
        f.add_trace(
            go.Scattergl(
                x=got[0][j],
                y=got[1][j],
                mode="lines",
                name=label(b),
                line=dict(color=col[b.run_id], width=2.2 if b is base else 1.3),
            )
        )
    if not f.data:
        return empty(f"{title}: no data for the selected runs")
    return base_layout(
        f,
        title=title,
        height=height,
        xtitle="time since bag play start [s]",
        ytitle=ytitle,
    )


def series_overlay(
    runs: list[RunBundle],
    col: dict[str, str],
    fam: str,
    key: str,
    *,
    base: RunBundle | None,
    title: str,
    ytitle: str = "",
    smooth_s: float = 0.0,
    height: int = 380,
) -> go.Figure:
    f = go.Figure()
    for b in runs:
        s = b.series.get(fam)
        if s is None or key not in s.values:
            continue
        v = smooth_time(s.step, s.values[key], smooth_s) if smooth_s else s.values[key]
        j = dec(len(s), 3000)
        f.add_trace(
            go.Scattergl(
                x=s.step[j],
                y=v[j],
                mode="lines",
                name=label(b),
                line=dict(color=col[b.run_id], width=2.2 if b is base else 1.3),
            )
        )
    if not f.data:
        return empty(f"{title}: no data for the selected runs")
    return base_layout(
        f,
        title=title,
        height=height,
        xtitle="time since bag play start [s]",
        ytitle=ytitle,
    )


def distribution(
    runs: list[RunBundle],
    col: dict[str, str],
    lane_key: str,
    *,
    base: RunBundle | None,
    title: str,
    height: int = 380,
) -> go.Figure:
    f = go.Figure()
    for b in runs:
        got = lane_data(LANE[lane_key], b, base, smooth=False)
        if got is None:
            continue
        v = got[1][np.isfinite(got[1])]
        f.add_trace(
            go.Violin(
                y=v,
                name=label(b),
                line_color=col[b.run_id],
                box_visible=True,
                meanline_visible=True,  # type: ignore[attr-defined]
                points=False,
                spanmode="hard",
            )
        )
    if not f.data:
        return empty(f"{title}: no data")
    base_layout(f, title=title, height=height, legend=False)
    f.update_layout(hovermode="closest")
    return f


def cdf(
    runs: list[RunBundle],
    col: dict[str, str],
    fam: str,
    key: str,
    *,
    title: str,
    xtitle: str,
    base: RunBundle | None = None,
    height: int = 380,
    log_x: bool = False,
) -> go.Figure:
    f = go.Figure()
    for b in runs:
        s = b.series.get(fam)
        if s is None or key not in s.values:
            continue
        v = np.sort(s.values[key][np.isfinite(s.values[key])])
        if not v.size:
            continue
        j = dec(len(v), 800)
        p95 = np.percentile(v, 95)
        f.add_trace(
            go.Scatter(
                x=v[j],
                y=(j + 1) / len(v),
                mode="lines",
                name=f"{label(b)} · p95 {p95:.1f}",
                line=dict(color=col[b.run_id], width=2.2 if b is base else 1.4),
            )
        )
    if not f.data:
        return empty(f"{title}: no data")
    base_layout(f, title=title, height=height, xtitle=xtitle, ytitle="fraction ≤ x")
    f.update_layout(hovermode="closest")
    f.add_hline(y=0.95, line=dict(color="#9ca3af", dash="dot", width=1))
    if log_x:
        f.update_xaxes(type="log")
    return f


def slam_breakdown(
    runs: list[RunBundle], col: dict[str, str], height: int | None = None
) -> go.Figure:
    """Mean cost per SLAM stage as one horizontal stacked bar per run; the black tick is the p95 of the total."""
    parts = [
        "pre_ms",
        "assoc_ms",
        "est_ms",
        "upd_ms",
        "spawn_ms",
        "commit_ms",
        "db_ms",
        "pub_ms",
    ]
    shades = [
        "#264653",
        "#2a9d8f",
        "#8ab17d",
        "#e9c46a",
        "#f4a261",
        "#e76f51",
        "#9c6644",
        "#6d597a",
    ]
    runs = [b for b in runs if "log_slam_profile" in b.series]
    if not runs:
        return empty("No SLAM_PROF lines in the selected runs")
    f = go.Figure()
    names = [label(b) for b in runs]
    for p, c in zip(parts, shades):
        vals = []
        for b in runs:
            v = b.series["log_slam_profile"].values.get(p)
            vals.append(
                float(np.nanmean(v)) if v is not None and np.isfinite(v).any() else 0.0
            )
        f.add_bar(
            y=names,
            x=vals,
            name=p.replace("_ms", ""),
            marker_color=c,
            orientation="h",
            hovertemplate="%{y}<br>"
            + p.replace("_ms", "")
            + ": mean %{x:.2f} ms<extra></extra>",
            text=[f"{x:.1f}" if x >= 0.6 else "" for x in vals],
            textposition="inside",
            insidetextfont=dict(color="#fff", size=10),
        )
    p95 = [
        float(np.nanpercentile(b.series["log_slam_profile"].values["total_ms"], 95))
        for b in runs
    ]
    f.add_scatter(
        y=names,
        x=p95,
        mode="markers",
        name="p95 of total",
        marker=dict(symbol="line-ns-open", size=26, color="#111", line=dict(width=3)),
        hovertemplate="%{y}<br>p95 total %{x:.1f} ms<extra></extra>",
    )
    base_layout(f, height=height or (110 + 46 * len(runs)), xtitle="ms per update")
    f.update_layout(
        barmode="stack",
        hovermode="closest",
        bargap=0.35,
        legend=dict(
            orientation="h",
            y=1.02,
            yanchor="bottom",
            x=0,
            xanchor="left",
            font=dict(size=10),
        ),
    )
    f.update_yaxes(autorange="reversed", showgrid=False)
    f.update_xaxes(showspikes=False)
    return f


def funnel(runs: list[RunBundle], col: dict[str, str], height: int = 380) -> go.Figure:
    stages = [
        ("clusters", "DBSCAN clusters"),
        ("shape_pass", "pass shape test"),
        ("accepted", "accepted"),
        ("far_dropped", "dropped (range)"),
    ]
    runs = [b for b in runs if "log_perception_filter" in b.series]
    if not runs:
        return empty("No CONE_FILTER lines (runs before 22 Sep 17:00 don't log it)")
    f = go.Figure()
    for b in runs:
        s = b.series["log_perception_filter"]
        f.add_bar(
            x=[n for _, n in stages],
            y=[float(np.nanmean(s.values[k])) for k, _ in stages],
            name=label(b),
            marker_color=col[b.run_id],
        )
    base_layout(
        f,
        title="Cone-detection funnel (mean per scan)",
        height=height,
        ytitle="per scan",
    )
    f.update_layout(barmode="group", hovermode="x unified")
    return f


def side_balance(
    runs: list[RunBundle], col: dict[str, str], height: int = 360
) -> go.Figure:
    f = go.Figure()
    for b in runs:
        s = b.series.get("log_perception_filter")
        if s is None:
            continue
        with np.errstate(invalid="ignore", divide="ignore"):
            v = s.values["left"] / (s.values["left"] + s.values["right"])
        f.add_trace(
            go.Scattergl(
                x=s.step,
                y=smooth_time(s.step, v, 3.0),
                mode="lines",
                name=label(b),
                line=dict(color=col[b.run_id], width=1.4),
            )
        )
    if not f.data:
        return empty("No CONE_FILTER lines in the selected runs")
    base_layout(
        f,
        title="Left / (left + right) accepted cones, 3 s mean — 0.5 = balanced",
        height=height,
        xtitle="time since bag play start [s]",
    )
    f.add_hline(y=0.5, line=dict(color="#6b7280", dash="dot"))
    return f


def guard_trips(
    runs: list[RunBundle], col: dict[str, str], height: int = 340
) -> go.Figure:
    f = go.Figure()
    for b in runs:
        s = b.series.get("log_perception_guard")
        if s is None:
            continue
        f.add_trace(
            go.Scatter(
                x=s.step,
                y=s.values["pts_dropped"],
                mode="markers",
                name=f"{label(b)} ({len(s)})",
                marker=dict(color=col[b.run_id], size=7, opacity=0.8),
            )
        )
    if not f.data:
        return empty("No DBSCAN guard trips")
    base_layout(
        f,
        title="DBSCAN guard trips: points dropped per trip",
        height=height,
        xtitle="time since bag play start [s]",
        ytitle="points dropped",
    )
    f.update_layout(hovermode="closest")
    return f


def map_overlay(
    runs: list[RunBundle],
    col: dict[str, str],
    base: RunBundle | None,
    height: int = 620,
) -> go.Figure:
    f = go.Figure()
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
                name=f"{label(b)} ({len(rows)} landmarks)",
                marker=dict(
                    symbol="triangle-up",
                    size=9 if b is base else 7,
                    opacity=0.75,
                    color=col[b.run_id],
                    line=dict(width=0.5, color="#111"),
                ),
            )
        )
    if not f.data:
        return empty("No final SLAM maps (live runs don't save one)")
    base_layout(
        f, title="Final SLAM maps overlaid (same bag, same frame)", height=height
    )
    f.update_layout(hovermode="closest")
    f.update_xaxes(showspikes=False)
    return equal(f)


def residual_hist(
    runs: list[RunBundle], col: dict[str, str], height: int = 360
) -> go.Figure:
    f = go.Figure()
    for b in runs:
        s = b.series.get("replay_control")
        if s is None:
            continue
        v = s.values["steer_residual_rad"]
        v = v[np.isfinite(v)]
        f.add_trace(
            go.Histogram(
                x=v,
                name=f"{label(b)} · rms {np.sqrt(np.mean(v ** 2)):.3f}",
                histnorm="probability",
                opacity=0.55,
                marker_color=col[b.run_id],
                nbinsx=80,
            )
        )
    if not f.data:
        return empty("No sampled control stream (--report runs only)")
    base_layout(
        f,
        title="Autonomy − pilot steering residual",
        height=height,
        xtitle="rad",
        ytitle="fraction",
    )
    f.update_layout(barmode="overlay", hovermode="closest")
    return f


def startup(runs: list[RunBundle], col: dict[str, str], height: int = 400) -> go.Figure:
    f = go.Figure()
    for b in runs:
        t = b.tables.get("lifecycle")
        if not t:
            continue
        nodes = t.column("node")
        for c, sym, nm in (
            ("ready_s", "circle-open", "ready"),
            ("activated_s", "circle", "active"),
            ("first_output_s", "star", "first output"),
        ):
            f.add_trace(
                go.Scatter(
                    x=t.column(c),
                    y=nodes,
                    mode="markers",
                    name=f"{label(b)} · {nm}",
                    legendgroup=b.run_id,
                    showlegend=c == "activated_s",  # type: ignore[attr-defined]
                    marker=dict(symbol=sym, size=11, color=col[b.run_id]),  # type: ignore[attr-defined]
                    hovertemplate=f"{label(b)}<br>%{{y}} {nm} at %{{x:.1f}} s<extra></extra>",
                )
            )
    if not f.data:
        return empty("No lifecycle data")
    base_layout(
        f,
        title="Node startup (○ ready · ● active · ★ first output), 0 = bag play start",
        height=height,
        xtitle="seconds relative to bag play",
    )
    f.update_layout(hovermode="closest")
    f.add_vline(x=0, line=dict(color="#111", dash="dash", width=1))
    return f


def event_matrix(runs: list[RunBundle], height: int = 360) -> go.Figure:
    kinds = sorted({e.kind for b in runs for e in b.events})
    if not kinds:
        return empty("No events")
    names = [label(b) for b in runs]
    z = [[sum(e.kind == k for e in b.events) for b in runs] for k in kinds]
    f = go.Figure(
        go.Heatmap(
            z=z,
            x=names,
            y=kinds,
            colorscale="OrRd",
            texttemplate="%{z}",
            showscale=False,
            hovertemplate="%{y} · %{x}: %{z}<extra></extra>",
        )
    )
    base_layout(
        f, title="Event counts", height=max(height, 70 + 34 * len(kinds)), legend=False
    )
    f.update_layout(hovermode="closest")
    f.update_xaxes(showspikes=False)
    return f


def lane_label(k: str) -> str:
    return LANE[k].title


# ---------------------------------------------------------------- panels for the playback view
SLAM_STATE_COLOUR = {"mapping": "#f59e0b", "localisation": "#10b981", "skip": "#9ca3af"}


def lane_panel(
    lane_key: str,
    runs: list[RunBundle],
    base: RunBundle | None,
    col: dict[str, str],
    *,
    smooth: bool,
    height: int = 190,
) -> go.Figure:
    """One signal as its own compact panel (the playback view stacks these, Foxglove-style)."""
    if lane_key == "events":
        return states(runs, col, height=None)
    la = LANE[lane_key]
    f = go.Figure()
    for b in runs:
        got = lane_data(la, b, base, smooth)
        if got is None:
            continue
        j = dec(len(got[0]), 2500)
        f.add_trace(
            go.Scatter(
                x=got[0][j],
                y=got[1][j],
                mode="lines",
                name=label(b),
                line=dict(color=col[b.run_id], width=1.6),
            )
        )
    if f.data and lane_key == "steer":
        pil = next((b for b in runs if "replay_control" in b.series), None)
        if pil is not None:
            s = pil.series["replay_control"]
            j = dec(len(s), 2500)
            f.add_trace(
                go.Scatter(
                    x=s.step[j],
                    y=s.values["pilot_steering_rad"][j],
                    mode="lines",
                    name="pilot",
                    line=dict(color="#9ca3af", width=1.2, dash="dash"),
                )
            )
    if f.data and lane_key == "cones_delta":
        f.add_hline(y=0, line=dict(color="#9ca3af", width=1))
    if not f.data:
        return go.Figure()
    return base_layout(f, height=height, legend=False)


def states(
    runs: list[RunBundle], col: dict[str, str], height: int | None = None
) -> go.Figure:
    """Foxglove-style state transitions: one row per run, SLAM mode as coloured bands, events as symbols."""
    f = go.Figure()
    names = [label(b) for b in runs]
    seen: set[str] = set()
    for b, nm in zip(runs, names):
        prof = b.series.get("log_slam_profile")
        if prof is not None and "mode" in prof.values:
            t, m = prof.step, prof.values["mode"]
            ok = np.isfinite(m)
            t, m = t[ok], m[ok].astype(int)
            if len(t):
                cut = np.r_[0, np.where(np.diff(m) != 0)[0] + 1, len(m)]
                for a, z in zip(cut[:-1], cut[1:]):
                    st = SLAM_MODE.get(int(m[a]), "?")
                    t1 = t[z] if z < len(t) else t[-1]
                    f.add_trace(
                        go.Scatter(
                            x=[t[a], t1],
                            y=[nm, nm],
                            mode="lines",
                            name=st,
                            legendgroup=st,
                            showlegend=st not in seen,
                            line=dict(
                                color=SLAM_STATE_COLOUR.get(st, "#9ca3af"), width=16
                            ),
                            hoverinfo="skip",
                        )
                    )
                    seen.add(st)
        by: dict[str, list] = {}
        for e in b.events:
            if e.t_s is not None and e.kind != "lifecycle":
                by.setdefault(e.kind, []).append(e)
        for kind, es in by.items():
            f.add_trace(
                go.Scatter(
                    x=[e.t_s for e in es],
                    y=[nm] * len(es),
                    mode="markers",
                    name=kind,
                    legendgroup=f"ev:{kind}",
                    showlegend=False,
                    marker=dict(
                        symbol=EVENT_SYMBOL.get(kind, "circle"),
                        size=9,
                        color=col.get(b.run_id, "#999"),
                        line=dict(width=1, color="#fff"),
                    ),
                    customdata=[[kind, e.detail[:140]] for e in es],
                    hovertemplate="%{customdata[0]}<br>%{customdata[1]}<extra>"
                    + nm
                    + "</extra>",
                )
            )
    if not f.data:
        return go.Figure()
    base_layout(f, height=height or (70 + 34 * len(runs)), legend=True)
    f.update_yaxes(
        type="category",
        categoryorder="array",
        categoryarray=names[::-1],
        showgrid=False,
        tickfont=dict(size=11),
    )
    f.update_layout(
        hovermode="closest",
        legend=dict(
            orientation="h",
            y=1.02,
            yanchor="bottom",
            x=1,
            xanchor="right",
            font=dict(size=10),
        ),
    )
    return f


def pose_payload(runs: list[RunBundle], max_pts: int = 1500) -> dict:
    """Decimated odom/SLAM poses per run, for moving the map markers in the browser without a server round trip."""
    out = {}
    for b in runs:
        s = b.series.get("replay_pose")
        if s is None:
            continue
        j = dec(len(s), max_pts)
        out[b.run_id] = {
            "t": _json_list(s.step[j]),  # type: ignore[attr-defined]
            **{
                k: _json_list(s.values[f"{a}_{c}"][j])
                for k, a, c in (
                    ("ox", "odom", "x"),
                    ("oy", "odom", "y"),
                    ("sx", "slam", "x"),
                    ("sy", "slam", "y"),
                )
            },
        }
    return out


def _json_list(a: np.ndarray) -> list:
    """Rounded floats with NaN as null (JSON has no NaN; SLAM has no pose before it starts)."""
    return [None if not np.isfinite(x) else round(float(x), 2) for x in a]


def event_payload(runs: list[RunBundle], col: dict[str, str]) -> list[dict]:
    out = [
        {
            "t": round(e.t_s, 2),
            "run": label(b),
            "c": col.get(b.run_id, "#999"),
            "kind": e.kind,  # type: ignore[attr-defined]
            "detail": e.detail[:140],
        }
        for b in runs
        for e in b.events
        if e.t_s is not None and e.kind != "lifecycle"
    ]
    return sorted(out, key=lambda e: e["t"])


def time_span(runs: list[RunBundle]) -> tuple[float, float]:
    lo, hi = [], []
    for b in runs:
        for s in b.series.values():
            if s.step_name in ("t/replay_s", "t/log_s") and len(s):
                lo.append(float(np.nanmin(s.step)))
                hi.append(float(np.nanmax(s.step)))
    return (min(lo), max(hi)) if lo else (0.0, 1.0)


__all__ = [
    "BASELINE",
    "LANES",
    "DEFAULT_LANES",
    "timeline",
    "route_map",
    "snapshot",
    "pose_at",
    "window_xy",
]
