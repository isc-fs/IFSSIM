"""What each report page shows, in each of the three viewing modes.

The viewer is organised the way people ask questions:

1. *What kind of report?* A page per kind (bag benchmarks, live runs, sim
   benchmarks, nightly, sweeps), picked in the left navigation.
2. *Which part of it?* A section of that page (summary, perception, SLAM…),
   also in the navigation, under the kind.
3. *How many reports?* One of three modes, which change the same charts:

   * **single**: one report, every chart about that run only;
   * **overlay**: the report plus up to three others drawn on the same axes;
   * **side**: two reports in two columns, chart for chart, with the same
     axis ranges on both sides and linked zoom.

A section is a list of :class:`Chart`. Each chart is one builder that takes a
list of runs, so the three modes are three ways of calling it. Charts are
full width, one per row, with their title and a one-line explanation above the
plot rather than inside it. The main charts come first; the rest sit behind a
"more charts" fold.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Callable

import dash_ag_grid as dag
import numpy as np
import plotly.graph_objects as go
from dash import dcc, html

from .. import registry
from ..bundle import RunBundle
from . import replay as R
from . import sim as S
from .data import Catalog, RunRow
from .theme import BASELINE, PALETTE, base_layout, empty, human

MODES = {"single": "One report", "overlay": "Overlay", "side": "Side by side"}
MAX_COMPARE = 3


# ======================================================================== kinds of report
@dataclass(frozen=True)
class Kind:
    key: str
    title: str
    blurb: str
    sections: tuple[tuple[str, str], ...]
    noun: str  # what one "report" is called on this page


SIM_SECTIONS = (
    ("summary", "Summary"),
    ("laps", "Laps & reliability"),
    ("track", "Along the track"),
    ("perf", "Perception & compute"),
    ("all", "Everything"),
)
KINDS: dict[str, Kind] = {
    k.key: k
    for k in (
        Kind(
            "bag",
            "Bag benchmarks",
            "Offline replays of a recorded bag with --report: sampled topics, pose, the final SLAM map and node logs.",
            (
                ("summary", "Summary"),
                ("trajectory", "Trajectory"),
                ("perception", "Perception"),
                ("slam", "SLAM"),
                ("control", "Control"),
                ("events", "Events & logs"),
                ("all", "Everything"),
            ),
            "report",
        ),
        Kind(
            "live",
            "Live runs",
            "A bag played into the live stack (--live). Only node logs are recorded: no pose, no sampled topics.",
            (
                ("summary", "Summary"),
                ("trajectory", "Timeline"),
                ("perception", "Perception"),
                ("slam", "SLAM"),
                ("control", "Control"),
                ("events", "Events & logs"),
                ("all", "Everything"),
            ),
            "run",
        ),
        Kind(
            "sim",
            "Sim benchmarks",
            "Every scenario of the matrix, several seeds each, for one pipeline commit. MOCK data.",
            SIM_SECTIONS,
            "commit",
        ),
        Kind(
            "nightly",
            "Nightly",
            "One aggregate per night on the nominal scenario. MOCK data.",
            (("trend", "Trend"),) + SIM_SECTIONS,
            "night",
        ),
        Kind(
            "sweep",
            "Parameter sweeps",
            "Controller parameter grid, three seeds per trial. MOCK data.",
            (("explore", "Explore"),) + SIM_SECTIONS,
            "trial",
        ),
    )
}
SIM_KINDS = ("sim", "nightly", "sweep")


def parse_path(pathname: str | None) -> tuple[str, str]:
    parts = [p for p in (pathname or "").split("/") if p]
    kind = parts[0] if parts and parts[0] in KINDS else "bag"
    secs = dict(KINDS[kind].sections)
    sec = (
        parts[1] if len(parts) > 1 and parts[1] in secs else KINDS[kind].sections[0][0]
    )
    return kind, sec


def rows_of(c: Catalog, kind: str) -> list[RunRow]:
    """The runs a page lists, newest first."""
    if kind in ("bag", "live"):
        job = "onboard_replay" if kind == "bag" else "live_replay"
        rows = [r for r in c.collection("replay") if r.job_type == job]
    else:
        rows = c.collection({"sim": "matrix"}.get(kind, kind))
    return sorted(rows, key=lambda r: r.started, reverse=True)


def keyof(kind: str, r: RunRow) -> str:
    """What one report is: a commit on the sim page (it spans every scenario), a run everywhere else."""
    return r.commit if kind == "sim" else r.run_id


def baseline_keys(c: Catalog, kind: str) -> set[str]:
    rows = rows_of(c, kind)
    out = set()
    for scen in {r.scenario for r in rows}:
        b = c.default_baseline([r for r in rows if r.scenario == scen])
        if b is not None:
            out.add(keyof(kind, b))
    return out


def options(c: Catalog, kind: str) -> list[dict]:
    base = baseline_keys(c, kind)
    out, seen = [], set()
    for r in rows_of(c, kind):
        k = keyof(kind, r)
        if k in seen:
            continue
        seen.add(k)
        star = "  ★ baseline" if k in base else ""
        if kind in ("bag", "live"):
            lab = (
                f"{r.started:%a %d %b %H:%M} · {r.scenario}"
                + ("  ⚠ aborted" if r.status != "finished" else "")
                + star
            )
        elif kind == "sim":
            lab = f"{r.commit} · {r.branch} · {r.message[:48]}" + star
        else:
            lab = r.short + star
        out.append({"label": lab, "value": k, "search": f"{lab} {r.name} {r.run_id}"})
    return out


def names(c: Catalog, kind: str, keys: list[str]) -> dict[str, str]:
    out = {}
    for k in keys:
        if kind == "sim":
            r = next((r for r in rows_of(c, kind) if r.commit == k), None)
            out[k] = f"{k}" + (f" ({r.branch})" if r and r.branch else "")
        else:
            r = c.rows.get(k)
            if r is None:
                out[k] = k[:8]
            elif kind in ("bag", "live"):
                out[k] = f"{r.started:%a %d %b %H:%M}"
            elif kind == "nightly":
                out[k] = f"night {r.params.get('nightly.night', '?')} · {r.commit}"
            else:
                out[k] = r.short
    return out


def default_report(c: Catalog, kind: str) -> str | None:
    rows = rows_of(c, kind)
    if not rows:
        return None
    if kind == "sweep":
        return min(
            rows, key=lambda r: r.summary.get("race/lap_time_mean_s", 1e9)
        ).run_id
    base = baseline_keys(c, kind)
    fin = [r for r in rows if r.status == "finished"] or rows
    # newest that is not the baseline
    pick = next((r for r in fin if keyof(kind, r) not in base), fin[0])
    return keyof(kind, pick)


def default_ref(c: Catalog, kind: str, r: str | None) -> str | None:
    """What to compare with when nobody said: the baseline, else the report just before this one."""
    base = [k for k in baseline_keys(c, kind) if k != r]
    if base:
        return base[0]
    keys = [o["value"] for o in options(c, kind)]
    if r in keys:
        i = keys.index(r)
        rest = keys[i + 1 :] + keys[:i][::-1]
        return rest[0] if rest else None
    return next((k for k in keys if k != r), None)


def colours(keys: list[str], baseline: set[str]) -> dict[str, str]:
    """Baseline is always black; the rest take the palette in order (the report is blue unless it is the baseline)."""
    out, i = {}, 0
    for k in keys:
        if k in baseline:
            out[k] = BASELINE
        else:
            out[k] = PALETTE[i % len(PALETTE)]
            i += 1
    return out


@dataclass
class View:
    kind: str
    section: str
    mode: str
    keys: list[str]  # report first, then what it is compared with
    col: dict[str, str]
    names: dict[str, str]
    base: set[str]
    scen: str = "*"

    @property
    def r(self) -> str:
        return self.keys[0]

    @property
    def vs(self) -> list[str]:
        return self.keys[1:]

    @property
    def ref(self) -> str | None:
        return self.keys[1] if len(self.keys) > 1 else None

    def to_store(self) -> dict:
        return {
            "kind": self.kind,
            "section": self.section,
            "mode": self.mode,
            "keys": self.keys,
            "col": self.col,
            "names": self.names,
            "base": sorted(self.base),
            "scen": self.scen,
        }

    @classmethod
    def from_store(cls, d: dict) -> "View":
        return cls(
            d["kind"],
            d["section"],
            d["mode"],
            d["keys"],
            d["col"],
            d["names"],
            set(d["base"]),
            d.get("scen", "*"),
        )


def make_view(
    c: Catalog,
    kind: str,
    section: str,
    mode: str,
    r: str,
    vs: list[str],
    scen: str | None,
) -> View:
    mode = "single" if not vs else mode if mode in ("overlay", "side") else "overlay"
    keys = [r] + (
        [] if mode == "single" else vs[:1] if mode == "side" else vs[:MAX_COMPARE]
    )
    base = baseline_keys(c, kind)
    return View(
        kind,
        section,
        mode,
        keys,
        colours(keys, base),
        names(c, kind, keys),
        base,
        scen or "*",
    )


# ======================================================================== small components
def fmt(v, spec: str = ".4g") -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "–"
    return format(v, spec) if isinstance(v, (int, float)) else str(v)


def dot(colour: str) -> html.Span:
    return html.Span(className="dot", style={"background": colour})


def who(v: View, k: str) -> html.Span:
    return html.Span([dot(v.col[k]), v.names.get(k, k)], className="who")


def note(text, cls: str = "") -> html.Div:
    return html.Div(text, className=f"callout {cls}".strip())


def heading(text: str, sub: str | None = None) -> html.Div:
    return html.Div([html.H2(text), html.P(sub) if sub else None], className="sec-head")


def tidy(
    f: go.Figure,
    *,
    legend: bool,
    top: int = 14,
    height: int | None = None,
    svg: bool = True,
) -> go.Figure:
    """Title goes above the plot in HTML; run legends go (the colour key sits in the sticky header)."""
    f.update_layout(title_text="", showlegend=legend)
    if svg and any(tr.type == "scattergl" for tr in f.data):
        # browsers cap WebGL contexts (~16 a page); a section of charts, doubled side by side, runs past that.
        # The chart builders keep ≤3000 points a run, which SVG draws fine. Only the timeline stays on WebGL.
        traces = [
            go.Scatter(
                {k: val for k, val in tr.to_plotly_json().items() if k != "type"}
            )
            if tr.type == "scattergl"
            else tr
            for tr in f.data
        ]
        f.data = []
        f.add_traces(traces)
    m = f.layout.margin
    f.update_layout(
        margin=dict(
            t=top,
            l=m.l if m.l is not None else 60,
            r=m.r if m.r is not None else 18,
            b=m.b if m.b is not None else 44,
        )
    )
    if height:
        f.update_layout(height=height)
    return f


def graph(fig: go.Figure, gid: str) -> dcc.Graph:
    return dcc.Graph(
        id={"type": "g", "index": gid},
        figure=fig,
        className="plot",
        config={
            "displaylogo": False,
            "scrollZoom": False,
            "toImageButtonOptions": {"scale": 2},
            "modeBarButtonsToRemove": ["lasso2d", "select2d"],
        },
    )


def expand_btn(gid: str) -> html.Button:
    return html.Button(
        "⤢",
        className="btn small expand",
        id={"type": "expand", "index": gid},
        title="Full screen",
    )


def time_style(f: go.Figure) -> go.Figure:
    """Time charts read like a Foxglove plot panel: no hover boxes (the header shows the values under the
    playhead instead), the unit on the ticks rather than a repeated axis title, tight margins."""
    for tr in f.data:
        if tr.type in ("scatter", "scattergl") and (tr.mode or "").startswith("lines"):
            tr.hoverinfo = "none"
            tr.hovertemplate = None
    f.update_layout(
        hovermode="x", margin=dict(l=56, r=16, t=f.layout.margin.t or 8, b=28)
    )
    f.update_xaxes(title_text=None, showspikes=False, ticksuffix=" s")
    f.update_yaxes(showspikes=False)
    return f


def stats_caption(fig: go.Figure) -> list:
    """What the bag reports print under each chart: per run, the spread of the signal."""
    out = []
    for tr in fig.data:
        if (
            tr.type not in ("scatter", "scattergl")
            or not (tr.mode or "").startswith("lines")
            or not tr.name
        ):
            continue
        try:
            y = np.asarray(tr.y, dtype=float)
        except (TypeError, ValueError):
            continue
        y = y[np.isfinite(y)]
        if y.size < 3:
            continue
        colour = tr.line.color if tr.line and isinstance(tr.line.color, str) else "#999"
        out.append(
            html.Span(
                [
                    dot(colour),
                    html.B(tr.name),
                    f"  median {fmt(float(np.median(y)), '.3g')} · "
                    f"p95 {fmt(float(np.percentile(y, 95)), '.3g')} · min {fmt(float(y.min()), '.3g')} · "
                    f"max {fmt(float(y.max()), '.3g')}",
                ],
                className="cap-item",
            )
        )
    return out


def _head(title: str, desc: str | None, time: bool) -> html.Div:
    return html.Div(
        [
            html.Div(
                [html.H3(title), html.P(desc) if desc else None], className="chead-l"
            ),
            html.Div(className="readout") if time else None,
        ],
        className="chead",
    )


def chart_card(
    title: str,
    desc: str | None,
    fig: go.Figure,
    gid: str,
    *,
    time: bool = False,
    caption: bool | None = None,
) -> html.Div:
    if time:
        time_style(fig)
    cap = stats_caption(fig) if (time if caption is None else caption) else []
    return html.Div(
        [
            _head(title, desc, time),
            expand_btn(gid),
            graph(fig, gid),
            html.Div(cap, className="caption") if cap else None,
        ],
        className="card" + (" tsync" if time else ""),
    )


def pair_card(
    title: str,
    desc: str | None,
    left: go.Figure | None,
    right: go.Figure | None,
    gid: str,
    missing: str = "No data for this run",
    *,
    time: bool = False,
) -> html.Div:
    def side(f, s):
        if f is None or not f.data:
            return html.Div(missing, className="pair-empty")
        if time:
            time_style(f)
        cap = stats_caption(f) if time else []
        return html.Div(
            [
                html.Div(className="readout") if time else None,
                expand_btn(f"{gid}-{s}"),
                graph(f, f"{gid}-{s}"),
                html.Div(cap, className="caption") if cap else None,
            ],
            className="pair-cell" + (" tsync" if time else ""),
        )

    return html.Div(
        [
            _head(title, desc, False),
            html.Div([side(left, "L"), side(right, "R")], className="pair"),
        ],
        className="card",
    )


def match_axes(figs: list[go.Figure | None]) -> None:
    """Give every figure the same numeric axis ranges, so side-by-side charts can be read against each other."""
    figs = [f for f in figs if f is not None and f.data]
    if len(figs) < 2:
        return
    rng: dict[str, list[float]] = {}
    zero: set[str] = set()
    for f in figs:
        for tr in f.data:
            t = tr.type
            dims = {
                "scatter": "xy",
                "scattergl": "xy",
                "box": "y",
                "violin": "y",
                "bar": "y",
            }.get(t, "")
            for d in dims:
                vals = getattr(tr, d, None)
                if vals is None:
                    continue
                try:
                    a = np.asarray(vals, dtype=float)
                except (TypeError, ValueError):
                    continue
                a = a[np.isfinite(a)]
                if not a.size:
                    continue
                ax = (getattr(tr, f"{d}axis", None) or d).replace(d, f"{d}axis", 1)
                lo, hi = rng.get(ax, [math.inf, -math.inf])
                rng[ax] = [min(lo, float(a.min())), max(hi, float(a.max()))]
                if t == "bar":
                    zero.add(ax)
    for ax, (lo, hi) in rng.items():
        if ax in zero:
            lo, hi = min(lo, 0.0), max(hi, 0.0)
        pad = (hi - lo) * 0.04 or abs(hi) * 0.05 or 1.0
        for f in figs:
            lay = f.layout[ax] if ax in f.layout else None
            if lay is None or lay.type in ("category", "log", "date"):
                continue
            f.layout[ax].range = [lo - pad, hi + pad]
            f.layout[ax].autorange = False


# ======================================================================== charts
@dataclass
class Chart:
    key: str
    title: str
    desc: str
    build: Callable[..., go.Figure]  # (runs, col, ref, height) -> figure
    time: bool = True  # x = time: zoom is linked across these charts
    main: bool = True  # shown before the "more charts" fold
    compare: bool = False  # only makes sense against a reference (overlay mode)
    legend: bool = False  # the legend says something other than which run is which
    height: int = 250


def render_charts(
    v: View,
    charts: list[Chart],
    runs: list[RunBundle | None],
    ref: RunBundle | None,
    col: dict,
    prefix: str,
    *,
    expand_all: bool = False,
    what: str = "",
) -> list:
    """Cards for a section. ``runs`` is aligned with ``v.keys`` (None = run with no data)."""
    main, more, missing = [], [], []
    played = [b for b in runs if b is not None]
    for ch in charts:
        if ch.compare and (v.mode != "overlay" or ref is None):
            continue
        gid = f"{prefix}-{ch.key}"
        if v.mode == "side":
            figs = [
                ch.build([b], col, None, ch.height) if b is not None else None
                for b in runs[:2]
            ]
            figs = [
                tidy(f, legend=ch.legend, top=34 if ch.legend else 14)
                if f is not None and f.data
                else None
                for f in figs
            ]
            if not any(figs):
                missing.append(ch.title)
                continue
            match_axes(figs)
            card = pair_card(
                ch.title,
                ch.desc,
                figs[0],
                figs[1] if len(figs) > 1 else None,
                gid,
                time=ch.time,
            )
        else:
            f = ch.build(played, col, ref, ch.height)
            if not f.data:
                missing.append(ch.title)
                continue
            if ch.time and played:
                f.update_xaxes(range=list(R.time_span(played)), autorange=False)
            card = chart_card(
                ch.title,
                ch.desc,
                tidy(f, legend=ch.legend, top=34 if ch.legend else 14),
                gid,
                time=ch.time,
            )
        (main if ch.main or expand_all else more).append(card)
    out = main
    if more:
        out.append(
            html.Details(
                [html.Summary(f"More {what} charts ({len(more)})".replace("  ", " "))]
                + more,
                className="more",
            )
        )
    if missing:
        out.append(
            html.Div(
                "Not available for the selected run(s): " + " · ".join(missing),
                className="missing",
            )
        )
    return out


# ---------------------------------------------------------------- bag replay charts
T = "time since bag play start [s]"


def _ov(lane: str, ytitle: str, smooth: bool = True):
    return lambda B, col, ref, h: R.overlay(
        B, col, lane, base=None, smooth=smooth, title="", ytitle=ytitle, height=h
    )


def _so(fam: str, key: str, ytitle: str, smooth_s: float = 0.0):
    return lambda B, col, ref, h: R.series_overlay(
        B,
        col,
        fam,
        key,
        base=None,
        title="",
        ytitle=ytitle,
        smooth_s=smooth_s,
        height=h,
    )


def _steering(B, col, ref, h):
    f = R.overlay(
        B, col, "steer", base=None, smooth=False, title="", ytitle="rad", height=h
    )
    pil = next((b for b in B if "replay_control" in b.series), None)
    if f.data and pil is not None:
        s = pil.series["replay_control"]
        j = R.dec(len(s), 4000)
        f.add_trace(
            go.Scattergl(
                x=s.step[j],
                y=s.values["pilot_steering_rad"][j],
                mode="lines",
                name="pilot",
                line=dict(color="#9ca3af", width=1.2, dash="dash"),
                hovertemplate="%{y:.3f}<extra>pilot</extra>",
            )
        )
    return f


def cumulative_events(B, col, kind: str, h: int = 300) -> go.Figure:
    f = go.Figure()
    for b in B:
        t = np.sort([e.t_s for e in b.events if e.kind == kind and e.t_s is not None])
        if not len(t):
            continue
        f.add_trace(
            go.Scatter(
                x=np.r_[0, t],
                y=np.r_[0, np.arange(1, len(t) + 1)],
                mode="lines",
                line_shape="hv",
                name=f"{R.label(b)} ({len(t)})",
                line=dict(color=col[b.run_id], width=1.8),
            )
        )
    if not f.data:
        return go.Figure()
    return base_layout(f, height=h, xtitle=T, ytitle="cumulative count")


def _nodata(f: go.Figure) -> go.Figure:
    """The builders return a message figure when there is nothing to draw; here that means 'skip the chart'."""
    return f if f.data else go.Figure()


PERCEPTION = [
    Chart(
        "cones",
        "Cones detected per scan",
        "Raw cone detections per LiDAR scan, 1 s mean. Live runs: cones accepted by the filter (CONE_FILTER log).",
        _ov("cones", "cones"),
    ),
    Chart(
        "cones-delta",
        "Cones detected: difference to the reference",
        "Each run minus the reference (the first run in 'Compare with'), 1 s mean. Below zero = fewer cones.",
        lambda B, col, ref, h: R.overlay(
            B,
            col,
            "cones_delta",
            base=ref,
            smooth=True,
            title="",
            ytitle="Δ cones",
            height=h,
        ),
        compare=True,
    ),
    Chart(
        "funnel",
        "Detection funnel",
        "Mean per scan through the cone filter: DBSCAN clusters → pass the shape test → accepted, and cones "
        "dropped for range. From the CONE_FILTER log line (not in runs before 22 Sep 17:00).",
        lambda B, col, ref, h: _nodata(R.funnel(B, col, height=340)),
        time=False,
        height=340,
    ),
    Chart(
        "dist",
        "Cones per scan: distribution",
        "Violin of raw detections per scan, with box and mean.",
        lambda B, col, ref, h: _nodata(
            R.distribution(B, col, "cones", base=None, title="", height=340)
        ),
        time=False,
        main=False,
        height=340,
    ),
    Chart(
        "balance",
        "Left / right balance",
        "Share of accepted cones on the left, 3 s mean. 0.5 = balanced.",
        lambda B, col, ref, h: _nodata(R.side_balance(B, col, height=h)),
        main=False,
    ),
    Chart(
        "guard",
        "DBSCAN guard trips",
        "Each point is a trip; y = points dropped to keep clustering bounded.",
        lambda B, col, ref, h: _nodata(R.guard_trips(B, col, height=h)),
        main=False,
    ),
    Chart(
        "hz",
        "Cone-detection node rate",
        "Processing rate of the node, from its log.",
        _so("log_perception_filter", "hz", "Hz"),
        main=False,
    ),
    Chart(
        "points",
        "Points per scan entering the filter",
        "2 s mean.",
        _so("log_perception_filter", "points", "points", 2.0),
        main=False,
    ),
]

SLAM = [
    Chart(
        "state",
        "SLAM state",
        "Mapping → localisation per run, with the events SLAM logged.",
        lambda B, col, ref, h: R.states(B, col),
        legend=True,
    ),
    Chart(
        "map",
        "SLAM map size",
        "Landmarks in the map. The jump from ≈18 to 80–170 at 45–60 s is in every "
        "played replay of this bag.",
        _ov("map", "landmarks", smooth=False),
    ),
    Chart(
        "gap",
        "SLAM − odometry position gap",
        "How far the SLAM pose is from odometry. Needs the sampled pose stream (--report runs).",
        _so("replay_pose", "slam_odom_gap_m", "m"),
    ),
    Chart("ms", "SLAM processing time", "Per update, 1 s mean.", _ov("slam_ms", "ms")),
    Chart(
        "stages",
        "Where SLAM spends its time",
        "Mean cost of each stage per update, stacked; the black tick is the p95 of the total.",
        lambda B, col, ref, h: _nodata(R.slam_breakdown(B, col)),
        time=False,
        legend=True,
    ),
    Chart(
        "final-map",
        "Final SLAM map",
        "Landmarks at the end of the run, same bag, same frame.",
        lambda B, col, ref, h: _nodata(R.map_overlay(B, col, None, height=560)),
        time=False,
        height=560,
    ),
    Chart(
        "dyaw",
        "SLAM − odometry heading difference",
        "",
        _so("replay_pose", "slam_odom_dyaw_deg", "deg"),
        main=False,
    ),
    Chart(
        "cdf",
        "SLAM processing time: distribution",
        "Cumulative distribution per run (log x). The dotted line is the 95th percentile.",
        lambda B, col, ref, h: _nodata(
            R.cdf(
                B,
                col,
                "log_slam_latency",
                "proc_ms",
                title="",
                xtitle="ms",
                log_x=True,
                height=340,
            )
        ),
        time=False,
        main=False,
        legend=True,
        height=340,
    ),
    Chart(
        "age",
        "Age of the cone message when SLAM processed it",
        "",
        _so("log_slam_latency", "age_ms", "ms"),
        main=False,
    ),
    Chart(
        "assoc",
        "Associated / observed cones per update",
        "",
        _ov("assoc", "fraction", smooth=False),
        main=False,
    ),
    Chart(
        "new",
        "New landmarks per update",
        "",
        _so("log_slam_obs", "new", "count"),
        main=False,
    ),
    Chart(
        "corr",
        "Pose correction per update",
        "",
        _so("log_slam_latency", "corr_m", "m"),
        main=False,
    ),
    Chart(
        "jumps",
        "Rejected pose jumps (cumulative)",
        "",
        lambda B, col, ref, h: cumulative_events(B, col, "pose_jump_rejected", h),
        main=False,
    ),
]

CONTROL = [
    Chart(
        "speed",
        "Speed",
        "Odometry speed (live runs: from the control status log line), 0.5 s mean.",
        _ov("speed", "m/s"),
    ),
    Chart(
        "steer",
        "Steering command",
        "Autonomy steering command; dashed grey = what the pilot did in the bag.",
        _steering,
    ),
    Chart(
        "resid",
        "Autonomy − pilot steering",
        "Positive = the stack steers more to the left than the pilot did.",
        _so("replay_control", "steer_residual_rad", "rad"),
    ),
    Chart("throttle", "Throttle", "", _ov("throttle", "0–1", smooth=False)),
    Chart(
        "resid-hist",
        "Steering residual: distribution",
        "Legend gives the RMS per run.",
        lambda B, col, ref, h: _nodata(R.residual_hist(B, col, height=340)),
        time=False,
        main=False,
        legend=True,
        height=340,
    ),
    Chart(
        "rate",
        "Steering rate",
        "0.5 s mean.",
        _so("replay_control", "steer_rate_radps", "rad/s", 0.5),
        main=False,
    ),
    Chart(
        "pathlen",
        "Path length seen by control",
        "",
        _so("log_control_status", "path_len_m", "m"),
        main=False,
    ),
    Chart(
        "pathhz",
        "/Path publish rate",
        "",
        _ov("path_hz", "Hz", smooth=False),
        main=False,
    ),
    Chart(
        "cbhz",
        "Path-planning callback rate",
        "",
        _so("log_planning_rate", "cb_hz", "Hz"),
        main=False,
    ),
    Chart(
        "poses",
        "Poses in each /Path message",
        "",
        _so("replay_planning", "path_poses", "poses"),
        main=False,
    ),
    Chart(
        "trav",
        "Distance travelled",
        "From the control log.",
        _so("log_control_status", "travelled_m", "m"),
        main=False,
    ),
]

EVENTS = [
    Chart(
        "strip",
        "SLAM state and pipeline events",
        "One row per run: the band is the SLAM mode, symbols are events (hover one for its log line).",
        lambda B, col, ref, h: R.states(B, col),
        legend=True,
    ),
    Chart(
        "counts",
        "Event counts",
        "",
        lambda B, col, ref, h: _nodata(R.event_matrix(B, height=320)),
        time=False,
        height=320,
    ),
    Chart(
        "startup",
        "Node startup",
        "○ ready · ● active · ★ first output, 0 = bag play start.",
        lambda B, col, ref, h: _nodata(R.startup(B, col, height=360)),
        time=False,
        height=360,
    ),
]

BAG_SECTIONS = {
    "perception": PERCEPTION,
    "slam": SLAM,
    "control": CONTROL,
    "events": EVENTS,
}


# ======================================================================== bag / live pages
def bag_data(
    c: Catalog, v: View
) -> tuple[list[RunRow], list[RunBundle | None], RunBundle | None]:
    rows = [c.rows[k] for k in v.keys if k in c.rows]
    played = [r.run_id for r in rows if r.status == "finished"]
    loaded = dict(zip(played, c.bundles(played)))
    for k, b in loaded.items():
        b.display = v.names.get(k)  # type: ignore[attr-defined]  (charts label runs the way the page does)
    runs = [loaded.get(k) for k in v.keys]
    ref = loaded.get(v.ref) if v.ref else None
    return rows, runs, ref


def bag_page(c: Catalog, v: View) -> list:
    rows, runs, ref = bag_data(c, v)
    warn = []
    dead = [r for r in rows if r.status != "finished"]
    if dead:
        warn.append(
            note(
                [
                    "Never played the bag, so there is nothing to plot: ",
                    html.B(", ".join(v.names[r.run_id] for r in dead)),
                    ". Its summary and logs are still there.",
                ]
            )
        )
    secs = [
        s
        for s, _ in KINDS[v.kind].sections
        if s not in ("summary", "trajectory", "all")
    ]
    if v.section == "summary":
        return warn + bag_summary(c, v, rows)
    if v.section == "trajectory":
        return warn + trajectory(v, runs, ref)
    B = [b for b in runs if b is not None]
    if v.section in BAG_SECTIONS:
        return (
            warn
            + render_charts(
                v,
                BAG_SECTIONS[v.section],
                runs,
                ref,
                v.col,
                v.section,
                what=dict(KINDS[v.kind].sections)[v.section],
            )
            + (event_tables(v, runs) if v.section == "events" else [])
            + ([player(B)] if B else [])
        )
    out = (
        warn
        + [heading("Summary")]
        + bag_summary(c, v, rows)
        + [heading(dict(KINDS[v.kind].sections)["trajectory"])]
        + trajectory(v, runs, ref)
    )
    for s in secs:
        out += [heading(dict(KINDS[v.kind].sections)[s])] + render_charts(
            v, BAG_SECTIONS[s], runs, ref, v.col, s, expand_all=True
        )
        if s == "events":
            out += event_tables(v, runs)
    return out


def _metric_label(m: str) -> html.Td:
    sp = registry.spec(m)
    return html.Td(
        [
            html.Div(sp.desc if sp and sp.desc else m),
            html.Small(m + (f" · {sp.unit}" if sp and sp.unit else "")),
        ],
        className="mname",
    )


def _better(m: str) -> str:
    sp = registry.spec(m)
    return {"min": "lower is better", "max": "higher is better"}.get(
        sp.direction if sp else "", ""
    )


def _val(summary: dict, m: str) -> str:
    v, s = summary.get(m), summary.get(f"{m}.std")
    return fmt(v) + (f" ± {fmt(s, '.2g')}" if s is not None and v is not None else "")


def headline_metrics(rows: list[RunRow]) -> tuple[list[str], list[str]]:
    reg = registry.registry()
    have = {k for r in rows for k in r.summary if k in reg}
    ms = [m for m in reg if m in have]
    return [m for m in ms if reg[m].headline], ms


def tiles(summary: dict, metrics: list[str]) -> html.Div:
    out = []
    for m in metrics:
        sp = registry.spec(m)
        out.append(
            html.Div(
                [
                    html.Div(sp.desc if sp else m, className="tl"),
                    html.Div(
                        [
                            html.Span(_val(summary, m), className="tv"),
                            html.Span(
                                f" {sp.unit}" if sp and sp.unit else "", className="tu"
                            ),
                        ]
                    ),
                    html.Div(_better(m), className="tb"),
                ],
                className="tile",
                title=m,
            )
        )
    return html.Div(out, className="tiles")


def values_table(
    metrics: list[str], cols: list[tuple], by_domain: bool = True
) -> html.Table:
    """cols: (header component, summary dict). One value column per entry."""
    head = html.Tr([html.Th("metric")] + [html.Th(h) for h, _ in cols] + [html.Th("")])
    body, dom = [], None
    for m in metrics:
        d = m.split("/")[0]
        if by_domain and d != dom:
            body.append(html.Tr(html.Td(d, colSpan=len(cols) + 2), className="dom"))
            dom = d
        body.append(
            html.Tr(
                [_metric_label(m)]
                + [html.Td(_val(s, m), className="num") for _, s in cols]
                + [html.Td(_better(m), className="dim small")]
            )
        )
    return html.Table([html.Thead(head), html.Tbody(body)], className="mt")


def compare_table(
    v: View, metrics: list[str], summaries: dict[str, dict], by_domain: bool = True
) -> html.Table:
    """Report vs each comparison: value columns plus Δ = report − that run, coloured by the registry."""
    r = v.r
    head = [html.Th("metric"), html.Th(who(v, r))]
    for k in v.vs:
        head += [
            html.Th(who(v, k)),
            html.Th(
                ["Δ vs ", html.Span(v.names[k], className="nowrap")], className="dh"
            ),
        ]
    body, dom = [], None
    for m in metrics:
        d = m.split("/")[0]
        if by_domain and d != dom:
            body.append(html.Tr(html.Td(d, colSpan=len(head)), className="dom"))
            dom = d
        a = summaries[r].get(m)
        cells = [
            _metric_label(m),
            html.Td(_val(summaries[r], m), className="num strong"),
        ]
        for k in v.vs:
            b = summaries[k].get(m)
            cells.append(html.Td(_val(summaries[k], m), className="num"))
            if a is None or b is None:
                cells.append(html.Td("", className="num"))
                continue
            dd = registry.delta(m, a, b)
            if dd["delta"] is None:
                cells.append(html.Td("", className="num"))
                continue
            cls = (
                "bad"
                if dd.get("regression")
                else "good"
                if dd.get("improvement")
                else ""
            )
            rel = (
                f" ({dd['delta'] / abs(b) * 100:+.0f}%)"
                if abs(b) > 1e-9 and dd["delta"]
                else ""
            )
            cells.append(
                html.Td(fmt(dd["delta"], "+.3g") + rel, className=f"num delta {cls}")
            )
        body.append(html.Tr(cells))
    return html.Table([html.Thead(html.Tr(head)), html.Tbody(body)], className="mt")


def facts(c: Catalog, v: View, r: RunRow) -> html.Div:
    kv = [
        ("Bag / scenario", r.scenario),
        ("Started", f"{r.started:%A %d %B %Y, %H:%M} UTC"),
        ("Status", r.status),
        (
            "Kind",
            {
                "onboard_replay": "onboard replay (--report)",
                "live_replay": "live replay (--live)",
            }.get(r.job_type, r.job_type),
        ),
        (
            "Pipeline commit",
            r.commit if r.commit != "?" else "not recorded (imported run)",
        ),
        ("Branch", r.branch or "–"),
        ("Run", r.name),
    ]
    return html.Div(
        [
            html.Table(
                [html.Tr([html.Th(k), html.Td(val)]) for k, val in kv], className="kv"
            ),
            html.A(
                "Open in MLflow ↗",
                href=c.url(r.run_id),
                target="_blank",
                className="btn small",
            ),
        ],
        className="card facts",
    )


def bag_summary(c: Catalog, v: View, rows: list[RunRow]) -> list:
    head, allm = headline_metrics(rows)
    if v.mode == "single":
        r = rows[0]
        return [
            html.Div(
                [
                    facts(c, v, r),
                    html.Div(tiles(r.summary, head), className="tiles-wrap"),
                ],
                className="sum-top",
            ),
            html.Details(
                [
                    html.Summary(f"All metrics ({len(allm)})"),
                    html.Div(
                        values_table(allm, [(who(v, r.run_id), r.summary)]),
                        className="card",
                    ),
                ],
                className="more",
                open=False,
            ),
        ]
    summ = {r.run_id: r.summary for r in rows}
    return [
        runs_table(c, v, rows),
        verdict_line(v, head, summ),
        html.Div(compare_table(v, head, summ, by_domain=True), className="card"),
        html.Details(
            [
                html.Summary(f"All metrics ({len(allm)})"),
                html.Div(compare_table(v, allm, summ), className="card"),
            ],
            className="more",
        ),
    ]


def verdict_line(v: View, metrics: list[str], summ: dict[str, dict]) -> html.Div:
    out = []
    for k in v.vs:
        worse = better = 0
        for m in metrics:
            a, b = summ[v.r].get(m), summ[k].get(m)
            if a is None or b is None:
                continue
            d = registry.delta(m, a, b)
            worse += bool(d.get("regression"))
            better += bool(d.get("improvement"))
        out.append(
            html.Div(
                [
                    who(v, v.r),
                    " vs ",
                    who(v, k),
                    ": ",
                    html.Span(
                        f"{worse} worse", className="pill bad" if worse else "pill"
                    ),
                    " ",
                    html.Span(
                        f"{better} better", className="pill good" if better else "pill"
                    ),
                    html.Span(
                        " beyond the registry tolerance, headline metrics",
                        className="dim",
                    ),
                ],
                className="verdict",
            )
        )
    return html.Div(out, className="verdicts")


def runs_table(c: Catalog, v: View, rows: list[RunRow]) -> html.Div:
    role = {v.r: "left" if v.mode == "side" else "report"} | {
        k: "right" if v.mode == "side" else ("reference" if k == v.ref else "compared")
        for k in v.vs
    }
    body = [
        html.Tr(
            [
                html.Td(who(v, r.run_id)),
                html.Td(role.get(r.run_id, "")),
                html.Td(r.status, className="bad" if r.status != "finished" else ""),
                html.Td(r.commit),
                html.Td("baseline ★" if r.run_id in v.base else ""),
                html.Td(html.A("MLflow ↗", href=c.url(r.run_id), target="_blank")),
            ]
        )
        for r in rows
    ]
    return html.Div(
        html.Table(
            [
                html.Thead(
                    html.Tr(
                        [
                            html.Th(x)
                            for x in ("run", "role", "status", "commit", "", "")
                        ]
                    )
                ),
                html.Tbody(body),
            ],
            className="mt compact",
        ),
        className="card",
    )


def event_tables(v: View, runs: list[RunBundle | None]) -> list:
    B = [b for b in runs if b is not None]
    if not B:
        return []
    ev_rows = [
        {
            "run": v.names.get(b.run_id, R.label(b)),
            "t": round(e.t_s, 2) if e.t_s is not None else None,  # type: ignore[attr-defined]
            "node": e.node,
            "kind": e.kind,
            "severity": e.severity,
            "detail": e.detail,
        }
        for b in B
        for e in b.events
    ]
    pats: dict[str, dict] = {}
    for b in B:
        wc = b.tables.get("warn_catalogue")
        for lvl, node, pat, n in wc.rows if wc else []:
            d = pats.setdefault(pat, {"level": lvl, "node": node, "pattern": pat})
            d[v.names.get(b.run_id, R.label(b))] = n  # type: ignore[attr-defined]
    names_ = [v.names.get(b.run_id, R.label(b)) for b in B]  # type: ignore[attr-defined]
    return [
        html.Div(
            [
                html.Div(
                    [
                        html.H3("Every event"),
                        html.P(
                            "Filter any column; sort by time to read a run in order."
                        ),
                    ],
                    className="chead",
                ),
                dag.AgGrid(
                    rowData=ev_rows,
                    className="ag-theme-alpine",
                    columnDefs=[
                        {"field": "run", "width": 150},
                        {
                            "field": "t",
                            "headerName": "t [s]",
                            "width": 90,
                            "filter": "agNumberColumnFilter",
                        },
                        {"field": "node", "width": 160},
                        {"field": "kind", "width": 170},
                        {"field": "severity", "width": 95},
                        {"field": "detail", "flex": 1, "tooltipField": "detail"},
                    ],
                    defaultColDef={
                        "sortable": True,
                        "filter": True,
                        "resizable": True,
                        "floatingFilter": True,
                    },
                    dashGridOptions={"tooltipShowDelay": 200},
                    style={"height": "420px"},
                ),
            ],
            className="card",
        ),
        html.Div(
            [
                html.Div(
                    [html.H3("Log warnings by pattern"), html.P("Count per run.")],
                    className="chead",
                ),
                dag.AgGrid(
                    rowData=list(pats.values()),
                    className="ag-theme-alpine",
                    columnDefs=[
                        {"field": "level", "width": 80},
                        {"field": "node", "width": 150},
                        {
                            "field": "pattern",
                            "flex": 1,
                            "minWidth": 300,
                            "tooltipField": "pattern",
                        },
                    ]
                    + [{"field": n, "width": 140} for n in names_],
                    defaultColDef={"sortable": True, "resizable": True},
                    dashGridOptions={"tooltipShowDelay": 200},
                    style={"height": f"{min(460, 70 + 31 * max(len(pats), 3))}px"},
                ),
            ],
            className="card",
        )
        if pats
        else None,
    ]


# ---------------------------------------------------------------- trajectory (linked map + timeline)
def default_lanes(v: View) -> list[str]:
    lanes = ["events", "speed", "steer", "cones", "map", "gap", "slam_ms"]
    if v.mode == "overlay" and v.ref:
        lanes.insert(4, "cones_delta")
    return lanes


def lane_options(v: View) -> list[dict]:
    names = {"events": "SLAM state & events"}
    return [
        {
            "label": names.get(la.key) or la.title.split(" (")[0].split(" [")[0],
            "value": la.key,
        }
        for la in R.LANES
        if la.key != "cones_delta" or (v.mode == "overlay" and v.ref)
    ]


def lane_cards(
    v: View, B: list[RunBundle], ref: RunBundle | None, lanes: list[str], smooth: bool
) -> list:
    """The panel stack: one compact card per signal, all on the shared playhead."""
    out, missing = [], []
    span = list(R.time_span(B))
    for k in lanes:
        la = R.LANE[k]
        title = "SLAM state & events" if k == "events" else la.title
        f = R.lane_panel(k, B, ref, v.col, smooth=smooth)
        if not f.data:
            missing.append(title)
            continue
        # every panel starts on the same stretch of the bag
        f.update_xaxes(range=span, autorange=False)
        out.append(
            chart_card(
                title,
                None,
                tidy(f, legend=k == "events", top=8 if k != "events" else 26),
                f"lane-{k}",
                time=True,
                caption=k not in ("events",),
            )
        )
    if missing:
        out.append(html.Div("No data for: " + " · ".join(missing), className="missing"))
    return out


def player(B: list[RunBundle]) -> html.Div:
    """The playback bar: scrub or play through the bag; every time chart, the map and the log follow."""
    t0, t1 = R.time_span(B)
    return html.Div(
        [
            html.Button(
                "▶", id="ph-play", className="ph-btn", title="Play / pause (space)"
            ),
            html.Div(
                [
                    html.Button(
                        f"{x}×",
                        className="ph-speed" + (" on" if x == 1 else ""),
                        **{"data-speed": str(x)},
                    )
                    for x in (1, 2, 5, 10)
                ],
                className="ph-speeds",
            ),
            dcc.Input(
                id="ph-range",
                type="range",
                min=round(t0, 1),
                max=round(t1, 1),
                step=0.1,
                value=round(t0, 1),
                className="ph-range",
                debounce=False,
            ),
            html.Span("t = –", id="ph-time", className="ph-time"),
            html.Span(
                "hover a chart or drag · ← → step 1 s · space plays",
                className="ph-hint",
            ),
        ],
        id="player",
        className="player",
        **{"data-t0": str(round(t0, 2)), "data-t1": str(round(t1, 2))},
    )


def phdata(B: list[RunBundle], col: dict[str, str]) -> html.Div:
    poses = json.dumps(R.pose_payload(B), separators=(",", ":"))
    events = json.dumps(R.event_payload(B, col), separators=(",", ":"))
    return html.Div(
        id="phdata",
        style={"display": "none"},
        **{
            "data-poses": poses,
            "data-events": events,
            "data-ver": str(hash((poses, events))),
        },
    )


def map_box(
    B: list[RunBundle],
    ref: RunBundle | None,
    col: dict,
    *,
    all_maps: bool,
    show_events: bool,
) -> list:
    mp, idx = R.route_map(
        B, ref, col, all_maps=all_maps, show_events=show_events, height=520
    )
    tidy(mp, legend=False, top=8)
    mp.update_layout(margin=dict(l=40, r=10, t=8, b=28))
    return [
        html.Div(
            [
                html.Div(
                    [
                        html.Div(
                            [
                                html.H3("Route"),
                                html.P(
                                    "● odometry (solid) · ○ SLAM (dotted). Click to jump there."
                                ),
                            ],
                            className="chead-l",
                        )
                    ],
                    className="chead",
                ),
                graph(mp, "route"),
            ],
            className="map-inner",
            **{"data-idx": json.dumps(idx.__dict__)},
        )
    ]


def trajectory(v: View, runs: list[RunBundle | None], ref: RunBundle | None) -> list:
    B = [b for b in runs if b is not None]
    if not B:
        return [note("No played run selected.")]
    posed = any("replay_pose" in b.series for b in B)
    lanes = default_lanes(v)
    if v.mode == "side":
        out = []
        if posed:
            maps = [
                tidy(
                    R.route_map(
                        [b], None, v.col, all_maps=False, show_events=True, height=520
                    )[0],
                    legend=False,
                    top=8,
                )
                if b is not None and "replay_pose" in b.series
                else None
                for b in runs[:2]
            ]
            match_axes(maps)
            out.append(
                pair_card(
                    "Route and final map",
                    "Solid = odometry, dotted = SLAM. Symbols = pipeline events.",
                    maps[0],
                    maps[1] if len(maps) > 1 else None,
                    "sbs-route",
                )
            )
        for k in lanes:
            figs = [
                tidy(
                    R.lane_panel(k, [b], None, v.col, smooth=True),
                    legend=k == "events",
                    top=26 if k == "events" else 8,
                )
                if b is not None
                else None
                for b in runs[:2]
            ]
            figs = [f if f is not None and f.data else None for f in figs]
            if not any(figs):
                continue
            if k != "events":
                match_axes(figs)
            title = "SLAM state & events" if k == "events" else R.LANE[k].title
            out.append(
                pair_card(
                    title,
                    None,
                    figs[0],
                    figs[1] if len(figs) > 1 else None,
                    f"sbs-{k}",
                    time=True,
                )
            )
        return out + [player(B)]
    left = []
    if posed:
        left.append(
            html.Div(
                map_box(B, ref, v.col, all_maps=False, show_events=False),
                id="mapbox",
                className="card",
            )
        )
    left.append(
        html.Div(
            html.Div(
                "Hover a chart, drag the bar at the bottom or press play.",
                className="dim",
            ),
            id="phlog",
            className="card phlog",
        )
    )
    return [
        html.Details(
            [
                html.Summary(
                    [
                        "Panels & map",
                        html.Span(
                            f"  {len(lanes)} panels · drag on a chart to zoom, double-click "
                            "to reset",
                            className="dim",
                        ),
                    ]
                ),
                html.Div(
                    [
                        html.Span("Panels", className="lbl"),
                        dcc.Checklist(
                            id="lanes",
                            options=lane_options(v),
                            value=lanes,
                            inline=True,
                            className="pills",
                        ),
                    ],
                    className="ctl-row",
                ),
                html.Div(
                    [
                        html.Span("Map", className="lbl"),
                        dcc.Checklist(
                            id="tl-opts",
                            options=[
                                {"label": "events", "value": "events"},
                                {"label": "every run's final map", "value": "maps"},
                                {"label": "smooth noisy signals", "value": "smooth"},
                            ],
                            value=["smooth"],
                            inline=True,
                            className="pills",
                        ),
                    ],
                    className="ctl-row",
                ),
            ],
            className="controls fold",
        ),
        html.Div(
            [
                html.Div(left, className="left"),
                html.Div(
                    lane_cards(v, B, ref, lanes, smooth=True),
                    id="lane-stack",
                    className="stack",
                ),
            ],
            className="linked" if posed else "linked nomap",
        ),
        phdata(B, v.col),
        player(B),
    ]


# ======================================================================== sim-like pages (sim, nightly, sweep)
MATRIX_METRICS = [
    "race/lap_time_mean_s",
    "race/lap_time_best_s",
    "race/finish_rate_frac",
    "race/n_doo",
    "race/n_off_track",
    "control/cross_track_rms_m",
    "control/cross_track_max_m",
    "control/speed_mean_mps",
    "perception/recall_frac",
    "perception/precision_frac",
    "perception/pos_err_mean_m",
    "slam/pos_err_rms_m",
    "latency/e2e_p95_ms",
    "compute/cpu_mean_frac",
]
NIGHTLY_METRICS = [
    "race/lap_time_mean_s",
    "race/finish_rate_frac",
    "race/n_doo",
    "control/cross_track_rms_m",
    "perception/recall_frac",
    "latency/e2e_p95_ms",
]


def sim_aggs(c: Catalog, v: View, keys: list[str] | None = None) -> list[RunRow]:
    keys = v.keys if keys is None else keys
    if v.kind == "sim":
        rows = [
            r
            for r in c.collection("matrix")
            if r.commit in keys and v.scen in ("*", r.scenario)
        ]
        return sorted(rows, key=lambda r: (r.scenario, keys.index(r.commit)))
    return [c.rows[k] for k in keys if k in c.rows]


def sim_key(v: View):
    return (lambda a: a.commit) if v.kind == "sim" else (lambda a: a.run_id)


def sim_page(c: Catalog, v: View) -> list:
    secs = dict(KINDS[v.kind].sections)
    if v.section == "trend":
        return nightly_trend(c, v)
    if v.section == "explore":
        return sweep_explore(c, v)
    if v.section == "summary":
        return sim_summary(c, v)
    if v.section == "laps":
        return sim_charts(c, v, SIM_LAPS, "laps")
    if v.section == "track":
        return track_section(c, v)
    if v.section == "perf":
        return sim_charts(c, v, SIM_PERF, "perf")
    out = [heading("Summary")] + sim_summary(c, v)
    for s in ("laps", "track", "perf"):
        out += [heading(secs[s])]
        out += (
            track_section(c, v)
            if s == "track"
            else sim_charts(
                c, v, SIM_LAPS if s == "laps" else SIM_PERF, s, expand_all=True
            )
        )
    return out


@dataclass
class SimCtx:
    c: Catalog
    v: View
    keys: list[str]

    @property
    def aggs(self) -> list[RunRow]:
        return sim_aggs(self.c, self.v, self.keys)

    @property
    def ccol(self) -> dict[str, str]:
        return {k: self.v.col[k] for k in self.keys}


SimChart = Callable[[SimCtx, int], go.Figure]


def _laps(x: SimCtx, h: int) -> go.Figure:
    aggs = x.aggs
    bmap = {b.run_id: b for b in x.c.bundles([a.run_id for a in aggs])}  # type: ignore[attr-defined]
    return _nodata(
        S.lap_times(
            x.c, aggs, bmap, x.ccol, height=h, key=sim_key(x.v), names=x.v.names
        )
    )


def _strip(metric: str):
    return lambda x, h: _nodata(
        S.seed_strip(
            x.c, x.aggs, metric, x.ccol, height=h, key=sim_key(x.v), names=x.v.names
        )
    )


def _seed_bundles(x: SimCtx) -> dict[str, list[RunBundle]]:
    return {a.run_id: x.c.bundles([s.run_id for s in x.c.seeds(a)]) for a in x.aggs}


def _latency(i: int):
    def build(x: SimCtx, h: int) -> go.Figure:
        return _nodata(
            S.latency(
                x.aggs,
                _seed_bundles(x),
                x.ccol,
                height=h,
                key=sim_key(x.v),
                names=x.v.names,
            )[i]
        )

    return build


SIM_LAPS = [
    Chart(
        "laps",
        "Lap times",
        "Every completed lap of every seed, one panel per scenario. Multi-lap events leave "
        "out lap 1 (standing start).",
        _laps,
        time=False,
        height=420,
    ),
    Chart(
        "rel",
        "Reliability",
        "Mean over seeds per scenario; bar = std.",
        lambda x, h: _nodata(
            S.reliability(
                x.c, x.aggs, x.ccol, height=h, key=sim_key(x.v), names=x.v.names
            )
        ),
        time=False,
        height=380,
        legend=False,
    ),
    Chart(
        "s-lap",
        "Mean lap time per seed",
        "One point per seed.",
        _strip("race/lap_time_mean_s"),
        time=False,
        height=380,
    ),
    Chart(
        "s-doo",
        "Cones hit per seed",
        "",
        _strip("race/n_doo"),
        time=False,
        main=False,
        height=360,
    ),
    Chart(
        "s-cte",
        "Cross-track RMS per seed",
        "",
        _strip("control/cross_track_rms_m"),
        time=False,
        main=False,
        height=360,
    ),
]
SIM_PERF = [
    Chart(
        "recall",
        "Perception recall vs range",
        "Pooled over seeds. Line style = scenario.",
        lambda x, h: _nodata(
            S.recall_range(
                x.aggs,
                _seed_bundles(x),
                x.ccol,
                height=h,
                key=sim_key(x.v),
                names=x.v.names,
            )
        ),
        time=False,
        height=380,
        legend=True,
    ),
    Chart(
        "lat",
        "End-to-end latency",
        "Cumulative distribution pooled over seeds; dotted line = p95.",
        _latency(0),
        time=False,
        height=380,
        legend=True,
    ),
    Chart(
        "stages",
        "Latency per pipeline stage",
        "Mean per stage, stacked.",
        _latency(1),
        time=False,
        height=380,
        legend=True,
    ),
    Chart(
        "s-prec",
        "Perception precision per seed",
        "",
        _strip("perception/precision_frac"),
        time=False,
        main=False,
        height=360,
    ),
    Chart(
        "s-cpu",
        "CPU per seed",
        "",
        _strip("compute/cpu_mean_frac"),
        time=False,
        main=False,
        height=360,
    ),
]


def sim_charts(
    c: Catalog, v: View, charts: list[Chart], prefix: str, *, expand_all: bool = False
) -> list:
    main, more, missing = [], [], []
    for ch in charts:
        gid = f"{prefix}-{ch.key}"
        if v.mode == "side":
            figs = [ch.build(SimCtx(c, v, [k]), ch.height) for k in v.keys[:2]]
            figs = [_sim_tidy(f, ch, one=True) if f.data else None for f in figs]
            if not any(figs):
                missing.append(ch.title)
                continue
            match_axes(figs)
            card = pair_card(
                ch.title, ch.desc, figs[0], figs[1] if len(figs) > 1 else None, gid
            )
        else:
            f = ch.build(SimCtx(c, v, v.keys), ch.height)
            if not f.data:
                missing.append(ch.title)
                continue
            card = chart_card(
                ch.title, ch.desc, _sim_tidy(f, ch, one=len(v.keys) == 1), gid
            )
        (main if ch.main or expand_all else more).append(card)
    out = main
    if more:
        out.append(
            html.Details(
                [html.Summary(f"More charts ({len(more)})")] + more, className="more"
            )
        )
    if missing:
        out.append(
            html.Div("Not available: " + " · ".join(missing), className="missing")
        )
    return out


def _sim_tidy(f: go.Figure, ch: Chart, *, one: bool) -> go.Figure:
    """Per-scenario panels: short scenario titles with room above them; with one report the box ticks only
    repeat its name, so they go."""
    tidy(f, legend=ch.legend, top=46)
    for an in f.layout.annotations or ():
        if an.text:
            an.text = S.short_scen(an.text)
            an.font = dict(size=12, color="#374151")
    if one and f.data and all(tr.type == "box" for tr in f.data):
        f.update_xaxes(showticklabels=False)
    return f


def sim_facts(c: Catalog, v: View, k: str) -> html.Div:
    aggs = sim_aggs(c, v, [k])
    if not aggs:
        return note("Nothing for this report in the selected scenario.")
    a = aggs[0]
    n_seeds = sorted({len(c.seeds(x)) for x in aggs})
    kv = [
        ("Pipeline commit", a.commit),
        ("Branch", a.branch or "–"),
        ("Message", a.message or "–"),
        ("Ran", f"{a.started:%A %d %B %Y, %H:%M} UTC"),
        ("Scenarios", ", ".join(sorted({x.scenario for x in aggs}))),
        ("Seeds per scenario", ", ".join(map(str, n_seeds))),
    ]
    if v.kind == "sweep":
        kv.insert(
            0,
            (
                "Parameters",
                ", ".join(
                    f"{p.split('.')[-1]} = {val}"
                    for p, val in a.params.items()
                    if p.startswith("params.control.")
                ),
            ),
        )
    if v.kind == "nightly":
        kv.insert(
            0,
            (
                "Night",
                f"{a.params.get('nightly.night', '?')} · {a.params.get('nightly.date', '')}",
            ),
        )
    links = [
        html.A(
            f"{x.scenario} in MLflow ↗",
            href=c.url(x.run_id),
            target="_blank",
            className="btn small",
        )
        for x in aggs
    ]
    return html.Div(
        [
            html.Table(
                [html.Tr([html.Th(kk), html.Td(val)]) for kk, val in kv], className="kv"
            ),
            html.Div(links, className="links"),
        ],
        className="card facts",
    )


def sim_summary(c: Catalog, v: View) -> list:
    key = sim_key(v)
    if v.mode == "single":
        aggs = sim_aggs(c, v, [v.r])
        reg = registry.registry()
        allm = [m for m in reg if any(m in a.summary for a in aggs)]
        head = [m for m in MATRIX_METRICS if m in allm]
        cols = [(a.scenario, a.summary) for a in aggs]
        top = [sim_facts(c, v, v.r)]
        if len(aggs) == 1:
            top.append(
                html.Div(tiles(aggs[0].summary, head[:8]), className="tiles-wrap")
            )
        return [
            html.Div(top, className="sum-top"),
            html.Div(
                [
                    html.Div(
                        [
                            html.H3("Key metrics"),
                            html.P("Mean ± standard deviation over seeds."),
                        ],
                        className="chead",
                    ),
                    values_table(head, cols, by_domain=False),
                ],
                className="card",
            ),
            html.Details(
                [
                    html.Summary(f"All metrics ({len(allm)})"),
                    html.Div(values_table(allm, cols), className="card"),
                ],
                className="more",
            ),
        ]
    out = [html.Div([sim_facts(c, v, k) for k in v.keys], className="facts-row")]
    out.append(
        html.Div(
            [
                dcc.Checklist(
                    id="sc-only",
                    options=[{"label": " only show changes", "value": "on"}],
                    value=[],
                    inline=True,
                    className="pills",
                )
            ],
            className="controls",
        )
    )
    blocks = []
    for k in v.vs:
        aggs = sim_aggs(c, v, [v.r, k])
        rows = S.scorecard(c, aggs, k, MATRIX_METRICS, key=key)
        blocks.append(scorecard_block(v, k, rows))
    out.append(html.Div(blocks, id="sc-wrap"))
    return out


VERDICT_CLS = {
    "worse": "bad",
    "better": "good",
    "worse (within tol.)": "warn",
    "better (within tol.)": "okish",
}


def scorecard_block(v: View, ref: str, rows: list[dict]) -> html.Div:
    worse = sum(r["verdict"] == "worse" for r in rows)
    better = sum(r["verdict"] == "better" for r in rows)
    body, scen = [], None
    for d in rows:
        if d["scenario"] != scen:
            scen = d["scenario"]
            body.append(html.Tr(html.Td(scen, colSpan=8), className="dom"))
        changed = d["verdict"] not in ("no clear change", "context", "n/a")
        ci = (
            ""
            if d["ci_lo"] is None
            else f"[{fmt(d['ci_lo'], '+.3g')}, {fmt(d['ci_hi'], '+.3g')}]"
        )
        body.append(
            html.Tr(
                [
                    _metric_label(d["metric"]),
                    html.Td(
                        f"{fmt(d['baseline'])} ± {fmt(d['baseline_std'], '.2g')}",
                        className="num",
                    ),
                    html.Td(
                        f"{fmt(d['value'])} ± {fmt(d['value_std'], '.2g')}",
                        className="num strong",
                    ),
                    html.Td(fmt(d["delta"], "+.3g"), className="num"),
                    html.Td(ci, className="num dim"),
                    html.Td(
                        fmt(d["p_better"], ".0%") if d["p_better"] is not None else "–",
                        className="num",
                    ),
                    html.Td(
                        d["verdict"],
                        className=f"verdict-cell {VERDICT_CLS.get(d['verdict'], '')}",
                    ),
                    html.Td(d["n"], className="dim small"),
                ],
                className="" if changed else "same",
            )
        )
    head = html.Tr(
        [
            html.Th("metric"),
            html.Th(who(v, ref)),
            html.Th(who(v, v.r)),
            html.Th("Δ mean"),
            html.Th("95% CI of Δ"),
            html.Th("P(better)"),
            html.Th("verdict"),
            html.Th("seeds"),
        ]
    )
    return html.Div(
        [
            html.Div(
                [
                    html.H3([who(v, v.r), " compared with ", who(v, ref)]),
                    html.P(
                        [
                            "Bootstrap over seeds. A verdict needs the 95% CI to exclude zero; 'within tol.' = real but "
                            "smaller than the registry tolerance. ",
                            html.Span(
                                f"{worse} worse",
                                className="pill bad" if worse else "pill",
                            ),
                            " ",
                            html.Span(
                                f"{better} better",
                                className="pill good" if better else "pill",
                            ),
                        ]
                    ),
                ],
                className="chead",
            ),
            html.Table([html.Thead(head), html.Tbody(body)], className="mt"),
        ],
        className="card",
    )


# ---------------------------------------------------------------- along the track (linked map + profiles)
def track_section(c: Catalog, v: View) -> list:
    scen = sorted({a.scenario for a in sim_aggs(c, v)})
    if not scen:
        return [note("No aggregate for this selection.")]
    pick = v.scen if v.scen in scen else scen[0]
    ctl = [
        html.Span("Map colour", className="lbl"),
        dcc.RadioItems(
            id="trk-metric",
            options=[
                {"label": "cross-track error", "value": "cte_m_mean"},
                {"label": "speed", "value": "speed_mps_mean"},
            ],
            value="cte_m_mean",
            inline=True,
            className="pills",
        ),
    ]
    if len(scen) > 1:
        ctl = [
            html.Span("Scenario", className="lbl"),
            dcc.RadioItems(
                id="trk-scen", options=scen, value=pick, inline=True, className="pills"
            ),
        ] + ctl
    else:
        ctl = [
            dcc.RadioItems(
                id="trk-scen", options=scen, value=pick, style={"display": "none"}
            )
        ] + ctl
    return [
        html.Div(
            ctl
            + [
                html.Span(
                    "Hover the profiles → the spot on the map."
                    if v.mode != "side"
                    else "",
                    className="hint",
                )
            ],
            className="controls",
        ),
        html.Div(id="trk-body"),
    ]


def track_body(c: Catalog, v: View, scen: str, metric: str) -> list:
    keyf = sim_key(v)
    aggs = [a for a in sim_aggs(c, v) if a.scenario == scen]
    by_key = {keyf(a): a for a in aggs}
    order = [by_key[k] for k in v.keys if k in by_key]
    if not order:
        return [note("No aggregate of the selected reports in this scenario.")]
    bundles = dict(zip([a.run_id for a in order], c.bundles([a.run_id for a in order])))
    col = {a.run_id: v.col[keyf(a)] for a in order}
    nm = {a.run_id: v.names.get(keyf(a), keyf(a)) for a in order}
    seeds = c.seeds(order[0])
    cl = S.centreline(c.bundle(seeds[0].run_id)) if seeds else None
    lap_len = float(cl[0][-1]) if cl is not None else 1e9
    label = "|cross-track error|" if metric.startswith("cte") else "speed"
    if v.mode == "side":
        maps, profs = [], []
        for k in v.keys[:2]:
            a = by_key.get(k)
            if a is None:
                maps.append(None)
                profs.append(None)
                continue
            b = bundles[a.run_id]
            maps.append(
                tidy(
                    S.track_map([b], None, None, col, cl, metric, height=560, names=nm)[
                        0
                    ],
                    legend=True,
                )
            )
            profs.append(
                tidy(
                    S.track_profiles([b], None, col, lap_len, names=nm),
                    legend=False,
                    top=34,
                    svg=False,
                )
            )
        match_axes(maps)
        match_axes(profs)
        return [
            pair_card(
                "Track, with every incident of every seed",
                "Symbols = penalties and incidents.",
                maps[0],
                maps[1] if len(maps) > 1 else None,
                "sbs-trk-map",
            ),
            pair_card(
                "Along-track profiles",
                "Mean over seeds at each distance along the lap; dotted = max.",
                profs[0],
                profs[1] if len(profs) > 1 else None,
                "sbs-trk-prof",
            ),
        ]
    report = bundles[order[0].run_id]
    refb = (
        bundles.get(by_key[v.ref].run_id)
        if v.mode == "overlay" and v.ref in by_key
        else None
    )
    B = [bundles[a.run_id] for a in order]
    mp, cur = S.track_map(
        B,
        refb,
        report if refb is not None else None,
        col,
        cl,
        metric,
        height=600,
        names=nm,
    )
    tidy(mp, legend=True)
    prof = tidy(
        S.track_profiles(B, refb, col, lap_len, names=nm),
        legend=False,
        top=34,
        svg=False,
    )
    msg = (
        [
            "Map colour = ",
            who(v, keyf(order[0])),
            f" minus {nm[refb.run_id]} ({label}, mean over seeds): "  # type: ignore[attr-defined]
            "red = worse, blue = better.",
        ]
        if refb is not None
        else "Centreline in grey; symbols = penalties and incidents of every seed. Switch to Overlay to colour the "
        "track by the difference to another report."
    )
    return [
        html.Div(
            [
                html.Div(
                    [
                        html.Div(graph(mp, "trk-map"), className="card"),
                        note(msg, "info"),
                    ],
                    className="left",
                ),
                html.Div(
                    [expand_btn("trk-prof"), graph(prof, "trk-prof")], className="card"
                ),
            ],
            className="linked",
        ),
        dcc.Store(
            id="trk-store",
            data={
                "cursor": cur[0] if cur else None,
                "cl": [a.tolist() for a in cl] if cl is not None else None,
            },
        ),
    ]


# ---------------------------------------------------------------- nightly trend, sweep explorer
def nightly_trend(c: Catalog, v: View, metrics: list[str] | None = None) -> list:
    nights = c.collection("nightly")
    base = next((c.rows[k] for k in v.base if k in c.rows), None)
    opts = [
        {"label": human(m), "value": m}
        for m in sorted({k for r in nights for k in r.summary if "." not in k})
    ]
    return [
        note(
            "Band = min–max over the night's seeds; dashed line = the baseline night; red = worse than the baseline "
            "beyond tolerance, green = better. Big points = the nights you have open. Click a night to open its "
            "report.",
            "info",
        ),
        html.Div(
            [
                html.Span("Metrics", className="lbl"),
                dcc.Dropdown(
                    id="nt-metrics",
                    multi=True,
                    value=metrics or NIGHTLY_METRICS,
                    options=opts,
                    style={"minWidth": "600px", "flex": "1"},
                ),
            ],
            className="controls",
        ),
        html.Div(
            [
                expand_btn("nt-trend"),
                graph(
                    tidy(
                        S.nightly_trend(
                            nights, metrics or NIGHTLY_METRICS, base, v.keys, v.col
                        ),
                        legend=False,
                        top=40,
                    ),
                    "nt-trend",
                ),
            ],
            className="card",
        ),
    ]


def sweep_explore(c: Catalog, v: View) -> list:
    trials = c.collection("sweep")
    opts = [
        {"label": human(m), "value": m}
        for m in sorted({k for r in trials for k in r.summary if "." not in k})
    ]
    return [
        note(
            "12 trials × 3 seeds. The trials you have open are outlined. Click a cell or a point to open that trial.",
            "info",
        ),
        html.Div(
            [
                html.Span("Grid colour", className="lbl"),
                dcc.Dropdown(
                    id="sw-metric",
                    options=opts,
                    value="race/lap_time_mean_s",
                    clearable=False,
                    style={"minWidth": "320px"},
                ),
            ],
            className="controls",
        ),
        chart_card(
            "Parameter grid",
            "lookahead gain × max lateral acceleration. Number = mean over seeds.",
            tidy(S.sweep_grid(trials, "race/lap_time_mean_s", v.keys), legend=False),
            "sw-grid",
        ),
        chart_card(
            "Trade-off",
            "Lap time against cones hit; colour = cross-track RMS.",
            tidy(
                S.sweep_pareto(
                    trials,
                    "race/lap_time_mean_s",
                    "race/n_doo",
                    "control/cross_track_rms_m",
                    v.keys,
                ),
                legend=False,
            ),
            "sw-pareto",
        ),
        chart_card(
            "Parameters → outcomes",
            "Drag along an axis to filter the trials.",
            tidy(
                S.sweep_parcoords(trials, "race/lap_time_mean_s"), legend=False, top=60
            ),
            "sw-par",
        ),
    ]


# ======================================================================== picking runs: the browser and the selection bar
ROW_METRICS = {
    "bag": [
        "slam/end_gap_vs_odom_m",
        "slam/n_map_cones",
        "control/steer_residual_mean_abs_rad",
        "perception/n_cones_mean",
    ],
    "live": [
        "slam/n_map_cones",
        "slam/proc_p95_ms",
        "perception/n_cones_mean",
        "run/n_crashes",
    ],
    "nightly": [
        "race/lap_time_mean_s",
        "race/n_doo",
        "control/cross_track_rms_m",
        "latency/e2e_p95_ms",
    ],
    "sweep": [
        "race/lap_time_mean_s",
        "race/n_doo",
        "control/cross_track_rms_m",
        "race/finish_rate_frac",
    ],
    "sim": [
        "race/finish_rate_frac",
        "race/n_doo",
        "control/cross_track_rms_m",
        "latency/e2e_p95_ms",
    ],
}
ROW_LABEL = {
    "slam/end_gap_vs_odom_m": "SLAM drift",
    "slam/n_map_cones": "map cones",
    "control/steer_residual_mean_abs_rad": "steer vs pilot",
    "perception/n_cones_mean": "cones / scan",
    "slam/proc_p95_ms": "SLAM p95",
    "run/n_crashes": "crashes",
    "race/lap_time_mean_s": "lap time",
    "race/n_doo": "cones hit",
    "control/cross_track_rms_m": "cross-track",
    "latency/e2e_p95_ms": "latency p95",
    "race/finish_rate_frac": "finished",
}


def _unit(m: str) -> str:
    sp = registry.spec(m)
    return "" if not sp or sp.unit in ("count", "frac", "") else sp.unit


def _row_summary(c: Catalog, kind: str, key: str) -> dict:
    """Summary numbers for one browser row. A sim commit spans scenarios, so its numbers are means over them."""
    if kind != "sim":
        return c.rows[key].summary
    aggs = [a for a in c.collection("matrix") if a.commit == key]
    out = {}
    for m in ROW_METRICS["sim"]:
        vals = [a.summary[m] for a in aggs if m in a.summary]
        if vals:
            out[m] = float(np.mean(vals))
    return out


def browser(
    c: Catalog, kind: str, r: str | None, vs: list[str], col: dict[str, str]
) -> list:
    rows = rows_of(c, kind)
    base = baseline_keys(c, kind)
    base_sum = _row_summary(c, kind, next(iter(base))) if base else {}
    ms = ROW_METRICS[kind]
    groups: dict[str, list] = {}
    seen = set()
    for row in rows:
        k = keyof(kind, row)
        if k in seen:
            continue
        seen.add(k)
        summ = _row_summary(c, kind, k)
        cells = []
        for m in ms:
            val = summ.get(m)
            cls = ""
            if k not in base and val is not None and base_sum.get(m) is not None:
                d = registry.delta(m, val, base_sum[m])
                cls = (
                    "bad"
                    if d.get("regression")
                    else "good"
                    if d.get("improvement")
                    else ""
                )
            shown = (
                fmt(val * 100, ".0f") + "%"
                if (m.endswith("_frac") and val is not None)
                else fmt(val, ".3g")
            )
            cells.append(
                html.Div(
                    [
                        html.Span(shown, className=f"rv {cls}"),
                        html.Span(
                            f" {_unit(m)}" if _unit(m) and val is not None else "",
                            className="ru",
                        ),
                        html.Div(
                            ROW_LABEL.get(m, human(m, short=True)), className="rl"
                        ),
                    ],
                    className="rm",
                )
            )
        if kind in ("bag", "live"):
            title = [
                html.B(f"{row.started:%H:%M}"),
                html.Span(f" {row.scenario}", className="rr-name"),
            ]
            sub = [
                f"{'onboard replay' if row.job_type == 'onboard_replay' else 'live stack'}",
                f"commit {row.commit}" if row.commit != "?" else "",
            ]
        elif kind == "sim":
            aggs = [a for a in c.collection("matrix") if a.commit == k]
            title = [html.B(k), html.Span(f" {row.branch}", className="rr-name")]
            sub = [
                row.message,
                f"{len(aggs)} scenarios × {len(c.seeds(aggs[0])) if aggs else 0} seeds",
            ]
        elif kind == "nightly":
            title = [
                html.B(f"Night {row.params.get('nightly.night', '?')}"),
                html.Span(f" {row.commit}", className="rr-name"),
            ]
            sub = [row.scenario, f"{len(c.seeds(row))} seeds"]
        else:
            la, lat = (
                row.params.get("params.control.lookahead_gain"),
                row.params.get("params.control.max_lat_acc"),
            )
            title = [
                html.B(f"Trial {row.params.get('sweep.trial', '?')}"),
                html.Span(
                    f" lookahead {la} · max lat. acc. {lat}", className="rr-name"
                ),
            ]
            sub = [row.scenario, f"{len(c.seeds(row))} seeds"]
        badges = []
        if row.status != "finished":
            badges.append(html.Span(row.status, className="badge bad"))
        if k in base:
            badges.append(html.Span("★ baseline", className="badge base"))
        if k == r:
            act = [html.Span([dot(col.get(k, "#999")), "open"], className="state")]
        elif k in vs:
            act = [
                html.Span([dot(col.get(k, "#999")), "comparing"], className="state"),
                html.Button(
                    "Remove", id={"type": "add", "key": k}, className="btn small ghost"
                ),
            ]
        else:
            act = [
                html.Button(
                    "Open",
                    id={"type": "open", "key": k},
                    className="btn small primary js-close",
                    title="Show only this one",
                ),
                html.Button(
                    "+ Compare",
                    id={"type": "add", "key": k},
                    className="btn small",
                    title=f"Add to the comparison (up to {MAX_COMPARE + 1} at once)",
                ),
            ]
        search = " ".join(
            [
                str(row.started),
                row.scenario,
                row.commit,
                row.branch,
                row.message,
                row.name,
                row.short,
            ]
        )
        groups.setdefault(f"{row.started:%A %d %B %Y}", []).append(
            html.Div(
                [
                    html.Div(
                        [
                            html.Div(title + badges, className="rr-title"),
                            html.Div(
                                " · ".join(x for x in sub if x), className="rr-sub"
                            ),
                        ],
                        className="rr-main",
                    ),
                    html.Div(cells, className="rr-metrics"),
                    html.Div(act, className="rr-act"),
                ],
                className="run-row"
                + (" is-open" if k == r else " is-cmp" if k in vs else ""),
                **{"data-search": search.lower()},
            )
        )
    body = []
    for day, items in groups.items():
        body += [html.Div(day, className="rr-day")] + items
    k = KINDS[kind]
    return [
        html.Div(
            [
                html.Div(
                    [
                        html.B(f"{len(seen)} {k.title.lower()}"),
                        html.Span(
                            "  coloured numbers: better / worse than the baseline beyond tolerance",
                            className="dim",
                        ),
                    ]
                ),
                dcc.Input(
                    id="run-filter",
                    type="search",
                    placeholder="Filter by date, commit, branch…",
                    className="rr-filter",
                    debounce=False,
                ),
                html.Button(
                    "✕", className="btn small ghost js-close", title="Close (Esc)"
                ),
            ],
            className="rr-head",
        ),
        html.Div(body, className="rr-list"),
    ]


def selection_bar(v: View, c: Catalog, mode_pref: str) -> list:
    k = KINDS[v.kind]

    def chip(key, role, removable):
        return html.Span(
            [
                dot(v.col[key]),
                html.Span(v.names.get(key, key), className="cn"),
                html.Small(role) if role else None,
                html.Button(
                    "✕",
                    id={"type": "rm", "key": key},
                    className="x",
                    title="Remove from the view",
                )
                if removable
                else None,
            ],
            className="chip big" + (" base" if key in v.base else ""),
        )

    scen = None
    if v.kind == "sim":
        opts = [{"label": "All scenarios", "value": "*"}] + [
            {"label": x, "value": x}
            for x in sorted({a.scenario for a in c.collection("matrix")})
        ]
        scen = html.Div(
            [
                html.Span("Scenario", className="lbl"),
                dcc.Dropdown(
                    id={"type": "scen", "i": 0},
                    options=opts,
                    value=v.scen,
                    clearable=False,
                    className="dd scen",
                ),
            ],
            className="sb-scen",
        )
    if v.mode == "single":
        return [
            html.Div(
                [
                    html.Span(f"Showing {k.noun}", className="lbl"),
                    chip(v.r, "★ baseline" if v.r in v.base else "", False),
                    html.Button("Change…", className="btn js-browse"),
                ],
                className="sb-group",
            ),
            html.Div(
                [
                    html.Button(
                        f"+ Compare with another {k.noun}",
                        className="btn primary js-browse",
                    )
                ],
                className="sb-group",
            ),
            scen,
            html.Div(
                [
                    html.Button(
                        "★ Make baseline",
                        id={"type": "pin", "i": 0},
                        className="btn small ghost",
                        title="Store this as the team baseline (MLflow tag)",
                    )
                ]
                if v.r not in v.base
                else [],
                className="sb-group right",
            ),
        ]
    chips = [chip(v.r, "", True)]
    for i, key in enumerate(v.vs):
        chips += [
            html.Span("vs" if i == 0 else "&", className="vs"),
            chip(key, "", True),
        ]
    tools = [
        dcc.RadioItems(
            id={"type": "cmode", "i": 0},
            value=v.mode,
            inline=True,
            className="seg",
            options=[
                {"label": "Overlay", "value": "overlay"},
                {"label": "Side by side", "value": "side"},
            ],
        )
    ]
    if v.mode == "side":
        tools.append(
            html.Button(
                "⇄ Swap", id={"type": "swap", "i": 0}, className="btn small ghost"
            )
        )
    return [
        html.Div(
            [html.Span("Comparing", className="lbl")]
            + chips
            + (
                [html.Button("+ Add", className="btn small js-browse")]
                if len(v.keys) <= MAX_COMPARE
                else []
            ),
            className="sb-group",
        ),
        scen,
        html.Div(tools, className="sb-group right"),
    ]


def page(c: Catalog, v: View) -> list:
    if v.kind in ("bag", "live"):
        return bag_page(c, v)
    return sim_page(c, v)


__all__ = [
    "KINDS",
    "MODES",
    "View",
    "make_view",
    "page",
    "parse_path",
    "options",
    "default_report",
    "default_ref",
    "empty",
]
