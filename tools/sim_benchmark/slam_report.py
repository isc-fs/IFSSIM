from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

_PAGE_STYLE = """
body { font-family: system-ui, Arial, sans-serif; margin: 24px; max-width: 1180px; color: #1a1a1a; }
h1 { font-size: 1.35rem; } h2 { font-size: 1.1rem; margin-top: 1.75rem; border-bottom: 1px solid #e0e0e0; padding-bottom: 4px; }
h3 { font-size: 0.95rem; margin: 0.5rem 0; }
.subtitle { color: #555; } .note { font-size: 0.88rem; color: #444; line-height: 1.45; max-width: 900px; }
.warn { background: #fff8e1; border: 1px solid #ffe082; padding: 12px; border-radius: 8px; margin: 12px 0; }
.stats { display: grid; grid-template-columns: repeat(auto-fill, minmax(130px, 1fr)); gap: 8px; margin: 12px 0; }
.stat { background: #f4f6f8; border-radius: 8px; padding: 10px; }
.stat label { font-size: 0.65rem; color: #666; text-transform: uppercase; display: block; }
.stat value { font-size: 0.95rem; font-weight: 600; margin-top: 4px; }
.chart { border: 1px solid #ddd; border-radius: 8px; padding: 12px; margin: 12px 0; background: #fff; }
.chart svg { width: 100%; max-width: 960px; display: block; background: #fafafa; }
.legend { font-size: 0.82rem; margin: 6px 0 10px; }
.legend span { margin-right: 14px; }
.gt-line { color: #1565c0; } .filter-line { color: #e65100; } .slam-g { color: #2e7d32; }
.imu-only-line { color: #c62828; }
.wheel-line { color: #6a1b9a; }
.grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
@media (max-width: 900px) { .grid2 { grid-template-columns: 1fr; } }
table.data { border-collapse: collapse; font-size: 0.82rem; width: 100%; max-width: 960px; }
table.data th, table.data td { border: 1px solid #ddd; padding: 6px 8px; text-align: right; }
table.data th { background: #f0f0f0; text-align: left; }
.chart .caption { font-size: 0.8rem; color: #666; margin-top: 6px; }
.chart-guide { font-size: 0.88rem; line-height: 1.5; max-width: 960px; }
.chart-guide dt { font-weight: 600; margin-top: 10px; color: #333; }
.chart-guide dd { margin: 4px 0 0 0; color: #555; }
.section-intro { font-size: 0.88rem; color: #444; max-width: 920px; margin-bottom: 8px; }
"""

_SERIES_COLORS = {
    "gt": "#1565c0",
    "filter": "#e65100",
    "supervisor": "#6a1b9a",
    "slam": "#2e7d32",
    "filter_err": "#ef6c00",
    "slam_err": "#388e3c",
    "delta": "#c62828",
    "imu_only": "#c62828",
    "imu_only_err": "#d32f2f",
    "wheel": "#6a1b9a",
}


def _fmt(v: Any) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        if math.isnan(v):
            return "—"
        return f"{v:.3f}"
    return str(v)


def _decimate(xs: list[float], ys: list[float], max_pts: int = 1400) -> tuple[list[float], list[float]]:
    if len(xs) <= max_pts:
        return xs, ys
    step = max(1, len(xs) // max_pts)
    return xs[::step], ys[::step]


def _fmt_tick(v: float) -> str:
    av = abs(v)
    if av >= 100:
        return f"{v:.0f}"
    if av >= 10:
        return f"{v:.1f}"
    if av >= 1:
        return f"{v:.2f}"
    return f"{v:.3f}"


def _axis_grid_y(
    *,
    y0: float,
    y1: float,
    ml: float,
    plot_top: float,
    ph: float,
    plot_w: float,
    n_ticks: int = 5,
) -> tuple[list[str], list[str]]:
    """Horizontal grid lines + left-side y tick labels."""
    grid, ticks = [], []
    span = max(1e-9, y1 - y0)
    for t in range(n_ticks + 1):
        frac = t / n_ticks
        y_val = y0 + frac * span
        y_pos = plot_top + ph - frac * ph
        grid.append(
            f"<line x1='{ml}' y1='{y_pos:.1f}' x2='{ml + plot_w:.1f}' y2='{y_pos:.1f}' "
            f"stroke='#e8e8e8' stroke-width='1'/>",
        )
        ticks.append(
            f"<text x='{ml - 6}' y='{y_pos + 4:.1f}' text-anchor='end' "
            f"font-size='10' fill='#666'>{_fmt_tick(y_val)}</text>",
        )
    return grid, ticks


def _series_caption(ys: list[float], unit: str) -> str:
    if not ys:
        return ""
    ys_sorted = sorted(ys)
    med = ys_sorted[len(ys_sorted) // 2]
    mean = sum(ys) / len(ys)
    return (
        f"<p class='caption'>{len(ys)} samples · "
        f"min={_fmt_tick(min(ys))}, median={_fmt_tick(med)}, "
        f"mean={_fmt_tick(mean)}, max={_fmt_tick(max(ys))} {unit}</p>"
    )


def _stats_block(title: str, stats: dict[str, Any], keys: list[str] | None = None) -> str:
    if not stats or stats.get("steps", stats.get("scans", 0)) == 0:
        return f"<h3>{title}</h3><p class='note'>No data.</p>"
    default_keys = [
        "steps",
        "scans",
        "mean_err_m",
        "median_err_m",
        "p95_err_m",
        "max_err_m",
        "mean_yaw_err_deg",
        "max_yaw_err_deg",
        "mean_step_delta_m",
        "max_step_delta_m",
    ]
    use = keys or default_keys
    cards = "".join(
        f"<div class='stat'><label>{k.replace('_', ' ')}</label>"
        f"<div class='value'>{_fmt(stats.get(k))}</div></div>"
        for k in use
        if k in stats
    )
    return f"<h3>{title}</h3><div class='stats'>{cards}</div>"


def _svg_multi_series(
    series: list[tuple[str, list[float], list[float], str]],
    title: str,
    y_label: str = "",
    width: float = 900.0,
    height: float = 260.0,
) -> str:
    valid = [(lbl, xs, ys, c) for lbl, xs, ys, c in series if len(xs) >= 2]
    if not valid:
        return f"<p class='note'>Not enough data for {title}</p>"
    ml, mb = 62.0, 40.0
    plot_top = 14.0
    pw = width - ml - 16.0
    ph = height - mb - plot_top
    all_x, all_y = [], []
    for _, xs, ys, _ in valid:
        all_x.extend(xs)
        all_y.extend(ys)
    x0, x1 = min(all_x), max(all_x)
    y0, y1 = min(all_y), max(all_y)
    xs_span = max(1e-9, x1 - x0)
    ys_span = max(1e-9, y1 - y0)
    if y0 == y1:
        y0 -= 0.5
        y1 += 0.5
        ys_span = 1.0

    def pt(x: float, y: float) -> str:
        px = ml + (x - x0) / xs_span * pw
        py = plot_top + ph - (y - y0) / ys_span * ph
        return f"{px:.1f},{py:.1f}"

    grid, y_ticks = _axis_grid_y(
        y0=y0, y1=y1, ml=ml, plot_top=plot_top, ph=ph, plot_w=pw,
    )

    lines = []
    for lbl, xs, ys, color in valid:
        xs_d, ys_d = _decimate(xs, ys)
        pts = " ".join(pt(xs_d[i], ys_d[i]) for i in range(len(xs_d)))
        lines.append(
            f"<polyline fill='none' stroke='{color}' stroke-width='1.8' "
            f"points='{pts}'/>",
        )
    legend = "".join(
        f"<span style='color:{c}'>{lbl}</span> " for lbl, _, _, c in valid
    )
    ylab = ""
    if y_label:
        ylab = (
            f"<text x='12' y='{plot_top + ph / 2:.1f}' font-size='11' fill='#444' "
            f"transform='rotate(-90 12 {plot_top + ph / 2:.1f})' "
            f"text-anchor='middle'>{y_label}</text>"
        )
    caption = _series_caption(all_y, y_label or "")
    return (
        f"<div class='chart'><h3>{title}</h3><p class='legend'>{legend}</p>"
        f"<svg viewBox='0 0 {int(width)} {int(height)}' xmlns='http://www.w3.org/2000/svg'>"
        f"<rect width='{int(width)}' height='{int(height)}' fill='#fafafa'/>"
        f"{''.join(grid)}"
        f"<line x1='{ml}' y1='{plot_top + ph}' x2='{ml + pw}' y2='{plot_top + ph}' "
        f"stroke='#333' stroke-width='1'/>"
        f"<line x1='{ml}' y1='{plot_top}' x2='{ml}' y2='{plot_top + ph}' "
        f"stroke='#333' stroke-width='1'/>"
        f"{''.join(y_ticks)}{ylab}{''.join(lines)}"
        f"<text x='{ml}' y='{height - 10}' font-size='10' fill='#666'>{x0:.1f} s</text>"
        f"<text x='{ml + pw}' y='{height - 10}' text-anchor='end' font-size='10' fill='#666'>"
        f"{x1:.1f} s</text>"
        f"<text x='{ml + pw / 2:.1f}' y='{height - 10}' text-anchor='middle' "
        f"font-size='10' fill='#666'>time</text>"
        f"</svg>{caption}</div>"
    )


def _svg_histogram(
    values: list[float],
    title: str,
    color: str = "#1565c0",
    x_unit: str = "m",
    bins: int = 30,
    width: float = 440.0,
    height: float = 220.0,
) -> str:
    if len(values) < 3:
        return ""
    lo, hi = min(values), max(values)
    if hi <= lo:
        hi = lo + 1e-6
    counts = [0] * bins
    for v in values:
        i = min(bins - 1, int((v - lo) / (hi - lo) * bins))
        counts[i] += 1
    mx = max(counts) or 1
    ml, mb = 52.0, 36.0
    plot_top = 12.0
    ph = height - mb - plot_top
    plot_w = width - ml - 12.0
    bw = plot_w / bins
    grid: list[str] = []
    y_ticks: list[str] = []
    for t in range(6):
        frac = t / 5.0
        cnt = int(round(mx * frac))
        y_pos = plot_top + ph - frac * ph
        if t > 0:
            grid.append(
                f"<line x1='{ml}' y1='{y_pos:.1f}' x2='{ml + plot_w:.1f}' y2='{y_pos:.1f}' "
                f"stroke='#e8e8e8' stroke-width='1'/>",
            )
        y_ticks.append(
            f"<text x='{ml - 5}' y='{y_pos + 4:.1f}' text-anchor='end' "
            f"font-size='9' fill='#666'>{cnt}</text>",
        )
    bars = []
    for i, c in enumerate(counts):
        bh = (c / mx) * (ph - 4)
        x = ml + i * bw
        y = plot_top + ph - bh
        bars.append(
            f"<rect x='{x:.1f}' y='{y:.1f}' width='{bw * 0.9:.1f}' height='{bh:.1f}' "
            f"fill='{color}' opacity='0.85'/>",
        )
    caption = _series_caption(values, x_unit)
    return (
        f"<div class='chart'><h3>{title}</h3>"
        f"<svg viewBox='0 0 {int(width)} {int(height)}' xmlns='http://www.w3.org/2000/svg'>"
        f"<rect width='{int(width)}' height='{int(height)}' fill='#fafafa'/>"
        f"{''.join(grid)}{''.join(bars)}"
        f"<line x1='{ml}' y1='{plot_top + ph}' x2='{ml + plot_w}' y2='{plot_top + ph}' "
        f"stroke='#333' stroke-width='1'/>"
        f"<line x1='{ml}' y1='{plot_top}' x2='{ml}' y2='{plot_top + ph}' stroke='#333' stroke-width='1'/>"
        f"{''.join(y_ticks)}"
        f"<text x='{ml}' y='{height - 8}' font-size='9' fill='#666'>{_fmt_tick(lo)} {x_unit}</text>"
        f"<text x='{ml + plot_w}' y='{height - 8}' text-anchor='end' font-size='9' fill='#666'>"
        f"{_fmt_tick(hi)} {x_unit}</text>"
        f"<text x='12' y='{plot_top + ph / 2:.1f}' font-size='10' fill='#444' "
        f"transform='rotate(-90 12 {plot_top + ph / 2:.1f})' text-anchor='middle'>count</text>"
        f"</svg>{caption}</div>"
    )


def _svg_trajectory(
    gt_x: list[float],
    gt_y: list[float],
    paths: list[tuple[str, list[float], list[float], str, str]],
    title: str,
    size: float = 460.0,
) -> str:
    pts_x = list(gt_x)
    pts_y = list(gt_y)
    for _, xs, ys, _, _ in paths:
        pts_x.extend(xs)
        pts_y.extend(ys)
    if len(pts_x) < 2:
        return f"<p class='note'>Not enough trajectory for {title}</p>"
    pad = 1.5
    x0, x1 = min(pts_x) - pad, max(pts_x) + pad
    y0, y1 = min(pts_y) - pad, max(pts_y) + pad
    span = max(x1 - x0, y1 - y0, 1.0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    ml, mb = 48.0, 36.0
    plot = size - ml - 16.0

    def px(x: float, y: float) -> tuple[float, float]:
        sx = ml + plot / 2 + (x - cx) / span * (plot - 20)
        sy = 12 + plot / 2 - (y - cy) / span * (plot - 20)
        return sx, sy

    def poly(xs: list[float], ys: list[float], color: str, dash: str = "") -> str:
        if len(xs) < 2:
            return ""
        xs_d, ys_d = _decimate(xs, ys)
        p = " ".join(
            f"{px(xs_d[i], ys_d[i])[0]:.1f},{px(xs_d[i], ys_d[i])[1]:.1f}"
            for i in range(len(xs_d))
        )
        dash_attr = f" stroke-dasharray='{dash}'" if dash else ""
        return f"<polyline fill='none' stroke='{color}' stroke-width='2'{dash_attr} points='{p}'/>"

    # Axis ticks in metres (aligned frame).
    tick_parts = []
    for t in range(5):
        frac = t / 4.0
        xv = x0 + frac * (x1 - x0)
        yv = y0 + frac * (y1 - y0)
        sx, _ = px(xv, cy)
        _, sy = px(cx, yv)
        tick_parts.append(
            f"<text x='{sx:.1f}' y='{12 + plot + 14:.1f}' text-anchor='middle' "
            f"font-size='9' fill='#666'>{_fmt_tick(xv)}</text>",
        )
        tick_parts.append(
            f"<text x='{ml - 6}' y='{sy + 3:.1f}' text-anchor='end' "
            f"font-size='9' fill='#666'>{_fmt_tick(yv)}</text>",
        )

    parts = [
        f"<div class='chart'><h3>{title}</h3><svg viewBox='0 0 {int(size)} {int(size + 20)}' "
        f"xmlns='http://www.w3.org/2000/svg'>",
        f"<rect width='{int(size)}' height='{int(size + 20)}' fill='#fafafa'/>",
        poly(gt_x, gt_y, _SERIES_COLORS["gt"]),
    ]
    for _, xs, ys, color, dash in paths:
        parts.append(poly(xs, ys, color, dash))
    parts.append(
        f"<circle cx='{px(gt_x[0], gt_y[0])[0]:.1f}' cy='{px(gt_x[0], gt_y[0])[1]:.1f}' "
        f"r='5' fill='#1565c0'/>",
    )
    parts.extend(tick_parts)
    parts.append(
        f"<text x='{ml + plot / 2:.1f}' y='{size + 16:.1f}' text-anchor='middle' "
        f"font-size='10' fill='#444'>x (m)</text>",
    )
    parts.append(
        f"<text x='10' y='{12 + plot / 2:.1f}' font-size='10' fill='#444' "
        f"transform='rotate(-90 10 {12 + plot / 2:.1f})' text-anchor='middle'>y (m)</text>",
    )
    parts.append("</svg>")
    parts.append(
        f"<p class='caption'>GT-aligned frame · x=[{_fmt_tick(min(pts_x))}, {_fmt_tick(max(pts_x))}] m, "
        f"y=[{_fmt_tick(min(pts_y))}, {_fmt_tick(max(pts_y))}] m</p></div>",
    )
    return "".join(parts)


def _greedy_matches(
    gt_x: list[float],
    gt_y: list[float],
    slam_x: list[float],
    slam_y: list[float],
    gate_m: float = 0.5,
) -> list[tuple[int, int, float]]:
    used_gt: set[int] = set()
    out: list[tuple[int, int, float]] = []
    for si, (sx, sy) in enumerate(zip(slam_x, slam_y)):
        best_gi = None
        best_d = gate_m
        for gi, (gx, gy) in enumerate(zip(gt_x, gt_y)):
            if gi in used_gt:
                continue
            d = math.hypot(sx - gx, sy - gy)
            if d < best_d:
                best_d = d
                best_gi = gi
        if best_gi is not None:
            used_gt.add(best_gi)
            out.append((best_gi, si, best_d))
    return out


def _svg_map_compare(
    gt_x: list[float],
    gt_y: list[float],
    slam_x: list[float],
    slam_y: list[float],
    title: str,
    stats: dict[str, Any] | None = None,
    *,
    match_gate_m: float = 0.5,
    size: float = 540.0,
) -> str:
    if not gt_x and not slam_x:
        return f"<p class='note'>No map data for {title}</p>"
    pts_x = list(gt_x) + list(slam_x)
    pts_y = list(gt_y) + list(slam_y)
    pad = 2.0
    x0, x1 = min(pts_x) - pad, max(pts_x) + pad
    y0, y1 = min(pts_y) - pad, max(pts_y) + pad
    span = max(x1 - x0, y1 - y0, 5.0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    ml = 52.0
    plot = size - ml - 20.0
    plot_h = size - 48.0

    def px(x: float, y: float) -> tuple[float, float]:
        sx = ml + plot / 2 + (x - cx) / span * (plot - 24)
        sy = 16 + plot_h / 2 - (y - cy) / span * (plot_h - 24)
        return sx, sy

    parts = [
        f"<div class='chart'><h3>{title}</h3>",
        f"<p class='legend'><span class='gt-line'>● GT track layout</span> "
        f"<span class='slam-g'>● SLAM landmarks</span> "
        f"<span style='color:#999'>— matched pairs (&lt;{match_gate_m} m)</span></p>",
        f"<svg viewBox='0 0 {int(size)} {int(size + 24)}' xmlns='http://www.w3.org/2000/svg'>",
        f"<rect width='{int(size)}' height='{int(size + 24)}' fill='#fafafa'/>",
    ]

    matches = _greedy_matches(gt_x, gt_y, slam_x, slam_y, match_gate_m)
    for gi, si, _d in matches:
        x1p, y1p = px(gt_x[gi], gt_y[gi])
        x2p, y2p = px(slam_x[si], slam_y[si])
        parts.append(
            f"<line x1='{x1p:.1f}' y1='{y1p:.1f}' x2='{x2p:.1f}' y2='{y2p:.1f}' "
            f"stroke='#bbb' stroke-width='1' stroke-dasharray='3,2'/>",
        )

    for x, y in zip(gt_x, gt_y):
        sx, sy = px(x, y)
        parts.append(
            f"<circle cx='{sx:.1f}' cy='{sy:.1f}' r='4.5' fill='{_SERIES_COLORS['gt']}' "
            f"stroke='#fff' stroke-width='0.8'/>",
        )
    for x, y in zip(slam_x, slam_y):
        sx, sy = px(x, y)
        parts.append(
            f"<circle cx='{sx:.1f}' cy='{sy:.1f}' r='4' fill='{_SERIES_COLORS['slam']}' "
            f"stroke='#fff' stroke-width='0.8' opacity='0.92'/>",
        )

    for t in range(5):
        frac = t / 4.0
        xv = x0 + frac * (x1 - x0)
        yv = y0 + frac * (y1 - y0)
        sx, _ = px(xv, cy)
        _, sy = px(cx, yv)
        parts.append(
            f"<text x='{sx:.1f}' y='{16 + plot_h + 16:.1f}' text-anchor='middle' "
            f"font-size='9' fill='#666'>{_fmt_tick(xv)}</text>",
        )
        parts.append(
            f"<text x='{ml - 6}' y='{sy + 3:.1f}' text-anchor='end' "
            f"font-size='9' fill='#666'>{_fmt_tick(yv)}</text>",
        )

    parts.append(
        f"<text x='{ml + plot / 2:.1f}' y='{size + 20:.1f}' text-anchor='middle' "
        f"font-size='10' fill='#444'>x (m)</text>",
    )
    parts.append(
        f"<text x='10' y='{16 + plot_h / 2:.1f}' font-size='10' fill='#444' "
        f"transform='rotate(-90 10 {16 + plot_h / 2:.1f})' text-anchor='middle'>y (m)</text>",
    )
    parts.append("</svg>")

    cap = "GT-aligned frame · full latched track vs final SLAM landmark map"
    if stats:
        cap += (
            f" · GT={stats.get('gt_cones', '—')}, SLAM={stats.get('slam_landmarks', '—')}, "
            f"matched={stats.get('matched', '—')}, FP={stats.get('false_positive', '—')}, "
            f"FN={stats.get('false_negative', '—')}"
        )
        if stats.get("mean_match_err_m") is not None:
            cap += f", mean match err={_fmt_tick(float(stats['mean_match_err_m']))} m"
    parts.append(f"<p class='caption'>{cap}</p></div>")
    return "".join(parts)


def _load_map_csv(path: Path) -> tuple[list[float], list[float], list[float], list[float]]:
    gt_x, gt_y, slam_x, slam_y = [], [], [], []
    if not path.is_file():
        return gt_x, gt_y, slam_x, slam_y
    with path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            src = row.get("source", "")
            x, y = float(row["x"]), float(row["y"])
            if src == "gt":
                gt_x.append(x)
                gt_y.append(y)
            elif src == "slam":
                slam_x.append(x)
                slam_y.append(y)
    return gt_x, gt_y, slam_x, slam_y


def _yaw_series(
    rows: list[dict[str, str]],
    event: str,
    prefix: str,
) -> tuple[list[float], list[float], list[float]]:
    t, gy, ey = [], [], []
    for row in rows:
        if row.get("event") != event:
            continue
        ev = row.get(f"{prefix}_yaw", "")
        if not ev or ev == "nan":
            continue
        t.append(float(row["t_s"]))
        gy.append(float(row["gt_yaw"]))
        ey.append(float(ev))
    return t, gy, ey


def _cone_err_deltas(rows: list[dict[str, str]], col: str) -> tuple[list[float], list[float]]:
    """Time and step-to-step error change at cone commits."""
    prev = None
    t_out, d_out = [], []
    for row in rows:
        if row.get("event") != "cone":
            continue
        v = row.get(col, "")
        if not v or v == "nan":
            continue
        err = float(v)
        if prev is not None:
            t_out.append(float(row["t_s"]))
            d_out.append(err - prev)
        prev = err
    return t_out, d_out


def _chart_guide_html() -> str:
    items = [
        (
            "Summary stat cards",
            "Aggregate pose error (metres) and yaw error (degrees) for SLAM, the offline "
            "OdometryFilter replay, recorded <code>/odom</code>, and SLAM sampled at different rates.",
        ),
        (
            "Filter position error (IMU steps)",
            "After every IMU sample (post 3&nbsp;s calibration), Euclidean distance between "
            "the filter pose and interpolated perfect GT, in the GT-aligned frame.",
        ),
        (
            "Filter Δerror (IMU steps)",
            "How much the filter position error changed since the previous IMU tick — "
            "shows whether error is accumulating smoothly or jumping on specific events.",
        ),
        (
            "GT vs filter — x / y / yaw",
            "Side-by-side position and heading components at IMU rate (~400&nbsp;Hz). "
            "When lines diverge, check whether drift is longitudinal, lateral, or rotational.",
        ),
        (
            "Filter vx / yaw rate",
            "Internal filter state (not compared to GT here) — useful to correlate error "
            "spikes with acceleration or turning.",
        ),
        (
            "Filter error histogram",
            "Distribution of per-IMU position errors across the run.",
        ),
        (
            "SLAM position error over time",
            "<strong>Committed</strong> (green): error right after each cone/LiDAR update when "
            "SLAM optimizes. <strong>Held</strong> (light green): last committed pose sampled on "
            "the GT odom timeline — flat between commits.",
        ),
        (
            "SLAM yaw error (cone commits)",
            "Heading error at each cone injection (radians).",
        ),
        (
            "GT vs SLAM — x / y / yaw (cone commits)",
            "Position and heading components when SLAM actually updates — sparse (~10&nbsp;Hz with LiDAR trigger).",
        ),
        (
            "GT vs SLAM — x / y / yaw (held, GT odom rate)",
            "Same components sampled every GT odom message; SLAM value is held constant between "
            "cone commits (staircase vs smooth GT).",
        ),
        (
            "SLAM Δerror (cone commits)",
            "Change in SLAM position error from one cone commit to the next.",
        ),
        (
            "Filter vs SLAM error",
            "Filter error at IMU rate vs SLAM error at cone commits — compares dead-reckoning "
            "drift to graph-optimized pose.",
        ),
        (
            "Filter replay vs recorded /odom",
            "Validates offline filter synthesis against bag <code>/odom</code> when recorded.",
        ),
        (
            "SLAM error histogram",
            "Distribution of position errors at cone commits only.",
        ),
        (
            "Cone map — GT vs SLAM",
            "Bird's-eye scatter of the full latched GT track layout (blue) vs SLAM landmarks "
            "after the run (green, ≥3 observations). Dashed lines link nearest pairs within 0.5&nbsp;m.",
        ),
        (
            "Trajectory overlay (filter + SLAM)",
            "Bird's-eye path in the GT-aligned frame: GT (solid blue), EKF filter (dashed orange), "
            "SLAM (dashed green).",
        ),
        (
            "Trajectory overlay (IMU-only + wheel/steering)",
            "Separate bird's-eye view: GT vs IMU predict-only (no corrections) and "
            "kinematic-bicycle path from motor RPM + steering (vx = rpm·k, ω = −(vx/L)·tan δ_road; δ_road from bag radians or norm×0.5).",
        ),
        (
            "IMU-only position error",
            "Same EKF predict step as the full filter, but RPM/steering corrections are disabled — "
            "error after each IMU tick vs interpolated GT.",
        ),
        (
            "Longitudinal velocity (vx) — EKF vs IMU-only vs wheel",
            "Body-frame forward speed from each estimator at IMU rate. Wheel sets "
            "<code>vx = rpm·k</code>; IMU-only integrates accel only (no RPM anchor), so "
            "<code>vx</code> often fails to return to zero under braking and biases high — "
            "the usual reason the IMU-only path is longer than wheel/steering.",
        ),
        (
            "Cumulative path length (aligned frame)",
            "Integrated step distance along each reconstructed trajectory vs GT. "
            "Diverging IMU-only length with similar GT and wheel curves confirms "
            "longitudinal drift, not a scale bug on the wheel model.",
        ),
        (
            "Worst moments tables",
            "Largest errors with timestamp, event type, GT coordinates, and estimate coordinates.",
        ),
    ]
    body = "".join(f"<dt>{title}</dt><dd>{desc}</dd>" for title, desc in items)
    return f"<h2>Chart guide</h2><dl class='chart-guide'>{body}</dl>"


def _load_pose_steps(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="") as fh:
        return list(csv.DictReader(fh))


def _cumulative_path_length(
    rows: list[dict[str, str]],
    prefix: str,
    *,
    event: str = "imu",
) -> tuple[list[float], list[float]]:
    """Integrate aligned-frame step distance for one pose prefix (IMU steps only)."""
    t_out: list[float] = []
    len_out: list[float] = []
    cum = 0.0
    prev_x: float | None = None
    prev_y: float | None = None
    for row in rows:
        if row.get("event") != event:
            continue
        xv = row.get(f"{prefix}_x", "")
        yv = row.get(f"{prefix}_y", "")
        if not xv or xv == "nan" or not yv or yv == "nan":
            continue
        x, y = float(xv), float(yv)
        if prev_x is not None:
            cum += math.hypot(x - prev_x, y - prev_y)
        prev_x, prev_y = x, y
        t_out.append(float(row["t_s"]))
        len_out.append(cum)
    return t_out, len_out


def _series_from_steps(
    rows: list[dict[str, str]],
    *,
    event: str | None,
    x_col: str,
    y_col: str,
) -> tuple[list[float], list[float]]:
    xs, ys = [], []
    for row in rows:
        if event is not None and row.get("event") != event:
            continue
        yv = row.get(y_col, "")
        if not yv or yv == "nan":
            continue
        xs.append(float(row["t_s"]))
        ys.append(float(yv))
    return xs, ys


def _component_series(
    rows: list[dict[str, str]],
    event: str,
    prefix: str,
) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float]]:
    """Return t, gt_x, gt_y, est_x, est_y for aligned-frame components."""
    t, gx, gy, ex, ey = [], [], [], [], []
    for row in rows:
        if row.get("event") != event:
            continue
        exv = row.get(f"{prefix}_x", "")
        if not exv or exv == "nan":
            continue
        t.append(float(row["t_s"]))
        gx.append(float(row["gt_x"]))
        gy.append(float(row["gt_y"]))
        ex.append(float(exv))
        ey.append(float(row[f"{prefix}_y"]))
    return t, gx, gy, ex, ey


def _worst_moments_table(rows: list[dict[str, str]], col: str, n: int = 12) -> str:
    ranked = []
    for row in rows:
        v = row.get(col, "")
        if not v or v == "nan":
            continue
        ranked.append((float(v), row))
    ranked.sort(key=lambda x: x[0], reverse=True)
    if not ranked:
        return ""
    hdr = (
        "<table class='data'><thead><tr>"
        "<th>t (s)</th><th>event</th><th>error (m)</th><th>GT x,y</th><th>est x,y</th>"
        "</tr></thead><tbody>"
    )
    body = []
    for err, row in ranked[:n]:
        ev = row.get("event", "")
        if col.startswith("filter"):
            prefix = "filter"
        elif col.startswith("imu_only"):
            prefix = "imu_only"
        elif col.startswith("wheel"):
            prefix = "wheel"
        elif col.startswith("supervisor"):
            prefix = "supervisor"
        else:
            prefix = "slam"
        ex = row.get(f"{prefix}_x", "—")
        ey = row.get(f"{prefix}_y", "—")
        body.append(
            f"<tr><td>{float(row['t_s']):.2f}</td><td>{ev}</td><td>{err:.3f}</td>"
            f"<td>{float(row['gt_x']):.2f}, {float(row['gt_y']):.2f}</td>"
            f"<td>{ex}, {ey}</td></tr>",
        )
    return hdr + "".join(body) + "</tbody></table>"


def _load_traj_csv(path: Path) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {
        "t_s": [],
        "gt_x": [],
        "gt_y": [],
        "slam_x": [],
        "slam_y": [],
    }
    with path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            out["t_s"].append(float(row["t_s"]))
            out["gt_x"].append(float(row["gt_x"]))
            out["gt_y"].append(float(row["gt_y"]))
            sx, sy = row.get("slam_x", ""), row.get("slam_y", "")
            if sx and sy and sx != "nan":
                out["slam_x"].append(float(sx))
                out["slam_y"].append(float(sy))
    return out


def _load_err_series(path: Path) -> tuple[list[float], list[float]]:
    t_axis, errs = [], []
    with path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            t_axis.append(float(row["t_s"]))
            errs.append(float(row["err_m"]))
    return t_axis, errs


def render_slam_html(summary: dict[str, Any], run_dir: Path) -> str:
    strategy = summary.get("strategy", "trackdrive")
    title = f"SLAM benchmark — {strategy}"
    warns = []

    odom_src = summary.get("odom_source", "bag" if summary.get("has_supervisor_odom") else "missing")
    if odom_src == "missing":
        warns.append(
            "No <code>/odom</code> and synthesis failed — SLAM ran without the pose prior (#545). "
            "Bag needs <code>/imu</code> and <code>/motor_rpm</code>."
        )
    elif odom_src == "synthesized":
        warns.append(
            "<code>/odom</code> was rebuilt offline from IMU/RPM/steering with the "
            "production 9-state EKF (<code>OdometryFilterCpp</code>), not read from the bag."
        )
    warn_html = (
        "<div class='warn'>" + "<br>".join(warns) + "</div>" if warns else ""
    )

    trigger = summary.get("cone_trigger", "/lidar/Lidar1")
    gate_html = (
        f"<p class='subtitle'>GT cones from latched track, injected on <code>{trigger}</code>; "
        f"gate {summary.get('gt_min_range_m', 0.5)}–{summary.get('gt_range_m', 20.0)} m, "
        f"±{summary.get('gt_hfov_deg', 60.0)}° H-FOV; "
        f"scan offset {summary.get('gt_scan_center_frac', 0.0)} periods; "
        f"<code>/odom</code> source: {odom_src}</p>"
    )

    methodology = """
<p class="note"><strong>How to read this report.</strong>
<strong>GT</strong> (<code>/testing_only/odom</code>) is in sim ENU; it is rotated into the
<strong>GT-aligned frame</strong> (origin + axes at SLAM calibration — same as <code>/cone_slam/gt_aligned</code>).
<strong>Filter</strong> (<code>/odom</code>) already lives in a local odom frame (integrated from zero at filter calibration);
it is re-expressed relative to the filter pose at SLAM calibration so it can be compared to GT-aligned coordinates
(do not apply the ENU alignment transform to filter states).
<strong>IMU-only</strong> uses the same EKF predict step but skips RPM/steering — expect a
<strong>longer</strong> path than wheel/steering because <code>vx</code> is not wheel-anchored
(accel bias and residual speed integrate into extra distance). See the vx and cumulative-length charts.
<strong>Wheel + steering</strong> sets <code>vx = rpm·k</code> and integrates yaw from steering kinematics (no IMU).
<strong>SLAM</strong> poses are already in the GT-aligned frame. One row per IMU tick after filter calibration;
<code>filter_err_delta_m</code> is the change in filter position error since the previous IMU.
SLAM only commits on cone/LiDAR triggers — “held” traces sample the last commit on the GT odom timeline.
Download <code>pose_steps.csv</code> for the full log.</p>
"""

    stats_html = ""
    stats_html += _stats_block("SLAM vs GT (cone commits)", summary.get("gt_cones") or {})
    stats_html += _stats_block(
        "Odometry filter vs GT (per IMU)",
        summary.get("filter_odom") or {},
    )
    stats_html += _stats_block(
        "IMU-only predict vs GT (per IMU, no corrections)",
        summary.get("imu_only_odom") or {},
    )
    stats_html += _stats_block(
        "Wheel + steering vs GT (per IMU)",
        summary.get("wheel_odom") or {},
    )
    stats_html += _stats_block(
        "Recorded /odom vs GT",
        summary.get("supervisor_odom") or {},
    )
    stats_html += _stats_block(
        "SLAM vs GT (at cone injection)",
        summary.get("slam_at_cones") or {},
    )
    stats_html += _stats_block(
        "SLAM vs GT (held, at GT odom rate)",
        summary.get("slam_at_gt_rate") or {},
    )
    map_stats = summary.get("map") or {}
    if map_stats.get("gt_cones", 0) or map_stats.get("slam_landmarks", 0):
        stats_html += _stats_block(
            "Cone map match (0.5 m gate)",
            map_stats,
            keys=[
                "gt_cones",
                "slam_landmarks",
                "matched",
                "false_positive",
                "false_negative",
                "mean_match_err_m",
                "max_match_err_m",
            ],
        )

    charts = ""
    pose_path = run_dir / "pose_steps.csv"
    steps = _load_pose_steps(pose_path)
    t_io: list[float] = []

    if steps:
        charts += (
            "<h2>Odometry filter (IMU + RPM EKF) vs perfect GT</h2>"
            "<p class='section-intro'>Dead-reckoning replay of the production 9-state EKF "
            "(<code>odometry_filter_node</code>, ported as <code>OdometryFilterCpp</code>) — "
            "one sample per IMU tick after calibration. All errors are position "
            "distance to interpolated perfect GT in the aligned frame.</p>"
        )
        t_imu, e_imu = _series_from_steps(
            steps, event="imu", x_col="t_s", y_col="filter_err_m",
        )
        t_delta, e_delta = _series_from_steps(
            steps, event="imu", x_col="t_s", y_col="filter_err_delta_m",
        )
        charts += _svg_multi_series(
            [
                ("Filter error (m)", t_imu, e_imu, _SERIES_COLORS["filter_err"]),
            ],
            "Position error after each IMU predict step",
            "m",
        )
        if t_delta:
            charts += _svg_multi_series(
                [("Δ error since prev IMU (m)", t_delta, e_delta, _SERIES_COLORS["delta"])],
                "Per-IMU change in filter position error",
                "m",
            )

        t_c, gt_xs, gt_ys, fxs, fys = _component_series(steps, "imu", "filter")
        if t_c:
            charts += "<div class='grid2'>"
            charts += _svg_multi_series(
                [
                    ("GT x", t_c, gt_xs, _SERIES_COLORS["gt"]),
                    ("Filter x", t_c, fxs, _SERIES_COLORS["filter"]),
                ],
                "X position — GT vs filter (IMU steps)",
                "m",
            )
            charts += _svg_multi_series(
                [
                    ("GT y", t_c, gt_ys, _SERIES_COLORS["gt"]),
                    ("Filter y", t_c, fys, _SERIES_COLORS["filter"]),
                ],
                "Y position — GT vs filter (IMU steps)",
                "m",
            )
            charts += "</div>"
            gt_yaw = [float(r["gt_yaw"]) for r in steps if r.get("event") == "imu" and r.get("filter_yaw") not in ("", "nan")]
            fyaw = [float(r["filter_yaw"]) for r in steps if r.get("event") == "imu" and r.get("filter_yaw") not in ("", "nan")]
            if len(gt_yaw) == len(fyaw) and gt_yaw:
                charts += _svg_multi_series(
                    [
                        ("GT yaw", t_c, gt_yaw, _SERIES_COLORS["gt"]),
                        ("Filter yaw", t_c, fyaw, _SERIES_COLORS["filter"]),
                    ],
                    "Yaw — GT vs filter (IMU steps)",
                    "rad",
                )

        t_v, vx = _series_from_steps(steps, event="imu", x_col="t_s", y_col="filter_vx")
        t_yr, yr = _series_from_steps(steps, event="imu", x_col="t_s", y_col="filter_yaw_rate")
        if t_v or t_yr:
            charts += "<div class='grid2'>"
            if t_v:
                charts += _svg_multi_series(
                    [("Filter vx (m/s)", t_v, vx, _SERIES_COLORS["filter"])],
                    "Filter longitudinal velocity",
                    "m/s",
                )
            if t_yr:
                charts += _svg_multi_series(
                    [("Filter yaw rate (rad/s)", t_yr, yr, _SERIES_COLORS["filter"])],
                    "Filter yaw rate",
                    "rad/s",
                )
            charts += "</div>"

        filter_errs = [float(r["filter_err_m"]) for r in steps if r.get("event") == "imu" and r.get("filter_err_m") not in ("", "nan")]
        charts += "<div class='grid2'>"
        charts += _svg_histogram(filter_errs, "Filter error distribution (IMU steps)", _SERIES_COLORS["filter_err"])
        charts += "</div>"

        charts += (
            "<h2>SLAM vs perfect GT</h2>"
            "<p class='section-intro'>Cone-graph SLAM with perfect gated cones. Pose only "
            "changes when cones are injected (~LiDAR rate). Between updates the estimate is "
            "held — see both commit and held traces below.</p>"
        )
        t_cone, e_cone = _series_from_steps(steps, event="cone", x_col="t_s", y_col="slam_err_m")
        t_gto, e_gto = _series_from_steps(steps, event="gt_odom", x_col="t_s", y_col="slam_err_m")
        series_err = []
        if t_cone:
            series_err.append(("SLAM error at cone commit (m)", t_cone, e_cone, _SERIES_COLORS["slam_err"]))
        if t_gto:
            series_err.append(("SLAM error held (m)", t_gto, e_gto, "#81c784"))
        if series_err:
            charts += _svg_multi_series(
                series_err,
                "SLAM position error over time",
                "m",
            )

        t_yaw_e, e_yaw = _series_from_steps(
            steps, event="cone", x_col="t_s", y_col="slam_yaw_err_rad",
        )
        if t_yaw_e:
            charts += _svg_multi_series(
                [("SLAM yaw error (rad)", t_yaw_e, e_yaw, _SERIES_COLORS["slam_err"])],
                "SLAM yaw error at cone commits",
                "rad",
            )

        t_sc, gt_xc, gt_yc, sxc, syc = _component_series(steps, "cone", "slam")
        if t_sc:
            charts += "<p class='section-intro'><strong>At cone commits</strong> — when SLAM optimizes.</p>"
            charts += "<div class='grid2'>"
            charts += _svg_multi_series(
                [
                    ("GT x", t_sc, gt_xc, _SERIES_COLORS["gt"]),
                    ("SLAM x", t_sc, sxc, _SERIES_COLORS["slam"]),
                ],
                "X — GT vs SLAM (cone commits)",
                "m",
            )
            charts += _svg_multi_series(
                [
                    ("GT y", t_sc, gt_yc, _SERIES_COLORS["gt"]),
                    ("SLAM y", t_sc, syc, _SERIES_COLORS["slam"]),
                ],
                "Y — GT vs SLAM (cone commits)",
                "m",
            )
            charts += "</div>"
            t_y, gt_yw, slam_yw = _yaw_series(steps, "cone", "slam")
            if t_y:
                charts += _svg_multi_series(
                    [
                        ("GT yaw", t_y, gt_yw, _SERIES_COLORS["gt"]),
                        ("SLAM yaw", t_y, slam_yw, _SERIES_COLORS["slam"]),
                    ],
                    "Yaw — GT vs SLAM (cone commits)",
                    "rad",
                )

        t_h, gt_xh, gt_yh, sxh, syh = _component_series(steps, "gt_odom", "slam")
        if t_h:
            charts += (
                "<p class='section-intro'><strong>Held on GT odom timeline</strong> — SLAM "
                "value is flat between cone commits (staircase).</p>"
            )
            charts += "<div class='grid2'>"
            charts += _svg_multi_series(
                [
                    ("GT x", t_h, gt_xh, _SERIES_COLORS["gt"]),
                    ("SLAM x (held)", t_h, sxh, _SERIES_COLORS["slam"]),
                ],
                "X — GT vs SLAM (held)",
                "m",
            )
            charts += _svg_multi_series(
                [
                    ("GT y", t_h, gt_yh, _SERIES_COLORS["gt"]),
                    ("SLAM y (held)", t_h, syh, _SERIES_COLORS["slam"]),
                ],
                "Y — GT vs SLAM (held)",
                "m",
            )
            charts += "</div>"
            t_yh, gt_ywh, slam_ywh = _yaw_series(steps, "gt_odom", "slam")
            if t_yh:
                charts += _svg_multi_series(
                    [
                        ("GT yaw", t_yh, gt_ywh, _SERIES_COLORS["gt"]),
                        ("SLAM yaw (held)", t_yh, slam_ywh, _SERIES_COLORS["slam"]),
                    ],
                    "Yaw — GT vs SLAM (held)",
                    "rad",
                )

        t_slam_d, e_slam_d = _cone_err_deltas(steps, "slam_err_m")
        if t_slam_d:
            charts += _svg_multi_series(
                [("Δ SLAM error (m)", t_slam_d, e_slam_d, _SERIES_COLORS["delta"])],
                "Per-commit change in SLAM position error",
                "m",
            )

        if t_cone and t_imu:
            charts += _svg_multi_series(
                [
                    ("Filter error (IMU)", t_imu, e_imu, _SERIES_COLORS["filter_err"]),
                    ("SLAM error (cones)", t_cone, e_cone, _SERIES_COLORS["slam_err"]),
                ],
                "Filter vs SLAM position error vs GT",
                "m",
            )

        t_sup, e_sup = _series_from_steps(
            steps, event="supervisor_odom", x_col="t_s", y_col="supervisor_err_m",
        )
        if t_sup:
            charts += _svg_multi_series(
                [
                    ("Filter (IMU steps)", t_imu, e_imu, _SERIES_COLORS["filter_err"]),
                    ("Recorded /odom", t_sup, e_sup, _SERIES_COLORS["supervisor"]),
                ],
                "Filter replay vs recorded supervisor /odom (both vs GT)",
                "m",
            )

        slam_errs = [
            float(r["slam_err_m"])
            for r in steps
            if r.get("event") == "cone" and r.get("slam_err_m") not in ("", "nan")
        ]
        if slam_errs:
            charts += _svg_histogram(
                slam_errs,
                "SLAM error distribution (cone commits)",
                _SERIES_COLORS["slam_err"],
            )

        t_io, e_io = _series_from_steps(
            steps, event="imu", x_col="t_s", y_col="imu_only_err_m",
        )
        if t_io:
            charts += (
                "<h2>IMU-only dead reckoning (predict, no corrections)</h2>"
                "<p class='section-intro'>Same EKF calibration and predict step "
                "(<code>OdometryFilterCpp</code>) as the EKF replay, but <code>push_rpm</code> "
                "and steering are never fed — isolates drift from accel/yaw integration "
                "(plus the non-holonomic vy constraint) alone.</p>"
            )
            charts += _svg_multi_series(
                [("IMU-only error (m)", t_io, e_io, _SERIES_COLORS["imu_only_err"])],
                "Position error after each IMU predict step (no RPM)",
                "m",
            )
            imu_only_errs = [
                float(r["imu_only_err_m"])
                for r in steps
                if r.get("event") == "imu" and r.get("imu_only_err_m") not in ("", "nan")
            ]
            if imu_only_errs:
                charts += _svg_histogram(
                    imu_only_errs,
                    "IMU-only error distribution",
                    _SERIES_COLORS["imu_only_err"],
                )

            t_werr, e_werr = _series_from_steps(
                steps, event="imu", x_col="t_s", y_col="wheel_err_m",
            )
            if t_werr:
                charts += _svg_multi_series(
                    [("Wheel + steering error (m)", t_werr, e_werr, _SERIES_COLORS["wheel"])],
                    "Wheel + steering position error",
                    "m",
                )

            t_fvx, fvx = _series_from_steps(
                steps, event="imu", x_col="t_s", y_col="filter_vx",
            )
            t_ivx, ivx = _series_from_steps(
                steps, event="imu", x_col="t_s", y_col="imu_only_vx",
            )
            t_wvx, wvx = _series_from_steps(
                steps, event="imu", x_col="t_s", y_col="wheel_vx",
            )
            vx_series: list[tuple[str, list[float], list[float], str]] = []
            if t_fvx:
                vx_series.append(("EKF vx", t_fvx, fvx, _SERIES_COLORS["filter"]))
            if t_ivx:
                vx_series.append(("IMU-only vx", t_ivx, ivx, _SERIES_COLORS["imu_only"]))
            if t_wvx:
                vx_series.append(("Wheel vx", t_wvx, wvx, _SERIES_COLORS["wheel"]))
            if vx_series:
                charts += _svg_multi_series(
                    vx_series,
                    "Longitudinal velocity — EKF vs IMU-only vs wheel",
                    "m/s",
                )

            t_gt_len, gt_len = _cumulative_path_length(steps, "gt")
            t_io_len, io_len = _cumulative_path_length(steps, "imu_only")
            t_wh_len, wh_len = _cumulative_path_length(steps, "wheel")
            t_f_len, f_len = _cumulative_path_length(steps, "filter")
            len_series: list[tuple[str, list[float], list[float], str]] = []
            if t_gt_len:
                len_series.append(("GT path length", t_gt_len, gt_len, _SERIES_COLORS["gt"]))
            if t_io_len:
                len_series.append(
                    ("IMU-only path length", t_io_len, io_len, _SERIES_COLORS["imu_only"]),
                )
            if t_wh_len:
                len_series.append(
                    ("Wheel path length", t_wh_len, wh_len, _SERIES_COLORS["wheel"]),
                )
            if t_f_len:
                len_series.append(
                    ("EKF path length", t_f_len, f_len, _SERIES_COLORS["filter"]),
                )
            if len_series:
                charts += _svg_multi_series(
                    len_series,
                    "Cumulative path length (why IMU-only can look longer)",
                    "m",
                )
                if gt_len and io_len and wh_len:
                    charts += (
                        f"<p class='caption'>End of run — GT: {_fmt_tick(gt_len[-1])} m, "
                        f"IMU-only: {_fmt_tick(io_len[-1])} m, "
                        f"wheel: {_fmt_tick(wh_len[-1])} m"
                        + (
                            f", EKF: {_fmt_tick(f_len[-1])} m"
                            if f_len
                            else ""
                        )
                        + ".</p>"
                    )

        charts += "<h2>Worst moments</h2>"
        charts += "<h3>Filter (IMU steps)</h3>"
        charts += _worst_moments_table(
            [r for r in steps if r.get("event") == "imu"],
            "filter_err_m",
        )
        if t_io:
            charts += "<h3>IMU-only (IMU steps)</h3>"
            charts += _worst_moments_table(
                [r for r in steps if r.get("event") == "imu"],
                "imu_only_err_m",
            )
            charts += "<h3>Wheel + steering (IMU steps)</h3>"
            charts += _worst_moments_table(
                [r for r in steps if r.get("event") == "imu"],
                "wheel_err_m",
            )
        charts += "<h3>SLAM (cone commits)</h3>"
        charts += _worst_moments_table(
            [r for r in steps if r.get("event") == "cone"],
            "slam_err_m",
        )

    map_path = run_dir / "map_cones.csv"
    gtx, gty, slx, sly = _load_map_csv(map_path)
    if gtx or slx:
        charts += (
            "<h2>Cone map — GT layout vs SLAM landmarks</h2>"
            "<p class='section-intro'>Full track from latched <code>/testing_only/track</code> "
            "(transformed to the aligned frame) compared to SLAM's final landmark database "
            "(same cones published on <code>/Conos</code>, min 3 observations). "
            "This shows map geometry error — duplicated landmarks, missing cones, or global drift.</p>"
        )
        charts += _svg_map_compare(
            gtx,
            gty,
            slx,
            sly,
            "GT track layout vs SLAM map (aligned frame)",
            map_stats if map_stats else None,
        )

    traj_path = run_dir / "trajectory.csv"
    samples_path = run_dir / "samples.csv"
    if not traj_path.is_file():
        traj_path = run_dir / "trajectory_gt_cones.csv"
    if not samples_path.is_file():
        samples_path = run_dir / "samples_gt_cones.csv"

    if traj_path.is_file():
        d = _load_traj_csv(traj_path)
        charts += (
            "<h2>Trajectory (aligned frame)</h2>"
            "<p class='section-intro'>Bird's-eye view of the full path. All traces use the "
            "GT-aligned frame (origin at SLAM calibration).</p>"
        )
        charts += (
            "<p class='legend'><span class='gt-line'>— GT</span> "
            "<span class='filter-line'>— Filter (from pose_steps)</span> "
            "<span class='slam-g'>— SLAM</span></p>"
        )
        paths = []
        if d["slam_x"]:
            paths.append(("slam", d["slam_x"], d["slam_y"], _SERIES_COLORS["slam"], "6,4"))
        if steps:
            _, _, _, fxs, fys = _component_series(steps, "imu", "filter")
            if fxs:
                paths.append(("filter", fxs, fys, _SERIES_COLORS["filter"], "3,3"))
        charts += _svg_trajectory(
            d["gt_x"],
            d["gt_y"],
            paths,
            "GT vs filter vs SLAM",
        )

        if steps:
            _, gxo, gyo, iox, ioy = _component_series(steps, "imu", "imu_only")
            _, gxw, gyw, wx, wy = _component_series(steps, "imu", "wheel")
            # GT at IMU rate from pose_steps (paired with estimates); fall back to trajectory.csv.
            gt_x_dr = gxo if gxo else gxw
            gt_y_dr = gyo if gyo else gyw
            if iox or wx:
                charts += (
                    "<h2>Trajectory — IMU-only and wheel/steering dead reckoning</h2>"
                    "<p class='section-intro'>Bird's-eye paths in the GT-aligned frame: "
                    "IMU predict-only (no RPM corrections) vs open-loop kinematics: "
                    "<code>vx = rpm·k</code>, <code>ω = −(vx/L)·tan δ_road</code> "
                    "(auto-detect rad vs normalized [-1,1] on <code>/steering_angle</code>), "
                    "integrated with the same body→world position update as "
                    "<code>odometry_filter::predict_step</code> (vy = 0).</p>"
                    "<p class='legend'><span class='gt-line'>— GT</span> "
                    "<span class='imu-only-line'>— IMU-only predict</span> "
                    "<span class='wheel-line'>— Wheel + steering</span></p>"
                )
                dr_paths: list[tuple[str, list[float], list[float], str, str]] = []
                if iox:
                    dr_paths.append(
                        ("imu_only", iox, ioy, _SERIES_COLORS["imu_only"], "4,3"),
                    )
                if wx:
                    dr_paths.append(
                        ("wheel", wx, wy, _SERIES_COLORS["wheel"], "6,2"),
                    )
                charts += _svg_trajectory(
                    gt_x_dr if gt_x_dr else d["gt_x"],
                    gt_y_dr if gt_y_dr else d["gt_y"],
                    dr_paths,
                    "GT vs IMU-only vs wheel/steering",
                )

    if samples_path.is_file():
        t, e = _load_err_series(samples_path)
        if t:
            charts += _svg_multi_series(
                [("SLAM err at cones (samples.csv)", t, e, _SERIES_COLORS["slam_err"])],
                "Cone-commit errors (legacy samples.csv)",
                "m",
            )

    return f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>{title}</title>
<style>{_PAGE_STYLE}</style></head><body>
<h1>{title}</h1>
<p class="subtitle">Bag: <code>{summary.get('bag', '')}</code></p>
{warn_html}
{gate_html}
{methodology}
{_chart_guide_html()}
<h2>Summary</h2>
{stats_html}
{charts}
<details><summary>Raw JSON</summary><pre>{json.dumps(summary, indent=2)}</pre></details>
</body></html>"""
