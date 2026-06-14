from __future__ import annotations

import json
import math
from pathlib import Path
from statistics import median
from typing import Any

from perception_metrics import (
    Cone2D,
    FrameMetrics,
    MatchResult,
    summarize_error_by_range,
)

_PAGE_STYLE = """
body { font-family: system-ui, Arial, sans-serif; margin: 24px; max-width: 1100px; color: #1a1a1a; }
h1 { font-size: 1.35rem; } h2 { font-size: 1.1rem; margin-top: 1.75rem; }
.subtitle { color: #555; } .warn { background: #fff8e1; border: 1px solid #ffe082; padding: 12px; border-radius: 8px; }
.stats { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 10px; margin: 1rem 0; }
.stat { background: #f4f6f8; border-radius: 8px; padding: 10px; }
.stat label { font-size: 0.7rem; color: #666; text-transform: uppercase; display: block; }
.stat value { font-size: 1.05rem; font-weight: 600; margin-top: 4px; }
.chart { border: 1px solid #ddd; border-radius: 8px; padding: 12px; margin: 12px 0; background: #fff; }
.chart svg { width: 100%; max-width: 900px; height: auto; display: block; background: #fafafa; }
.chart .caption { font-size: 0.8rem; color: #666; margin-top: 6px; }
.legend { font-size: 0.85rem; margin-top: 6px; }
.legend span { margin-right: 14px; }
.gt { color: #1565c0; } .pred { color: #c62828; } .match { color: #2e7d32; }
.frame-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 16px; }
details pre { font-size: 0.75rem; background: #f8f8f8; padding: 10px; overflow: auto; }
"""


def _fmt(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.3f}"
    return str(v)


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
    grid, ticks = [], []
    span = max(1e-9, y1 - y0)
    for t in range(n_ticks + 1):
        frac = t / n_ticks
        y_val = y0 + frac * span
        y_pos = plot_top + ph - frac * ph
        grid.append(
            f"<line x1='{ml}' y1='{y_pos:.1f}' x2='{ml + plot_w:.1f}' y2='{y_pos:.1f}' "
            f"stroke='#e8e8e8' stroke-width='1'/>"
        )
        ticks.append(
            f"<text x='{ml - 6}' y='{y_pos + 4:.1f}' text-anchor='end' "
            f"font-size='10' fill='#666'>{_fmt_tick(y_val)}</text>"
        )
    return grid, ticks


def _axis_grid_x(
    *,
    x0: float,
    x1: float,
    ml: float,
    plot_top: float,
    ph: float,
    plot_w: float,
    y_label_y: float,
    n_ticks: int = 5,
    unit: str = "",
) -> list[str]:
    ticks: list[str] = []
    span = max(1e-9, x1 - x0)
    for t in range(n_ticks + 1):
        frac = t / n_ticks
        x_val = x0 + frac * span
        x_pos = ml + frac * plot_w
        label = _fmt_tick(x_val)
        if unit:
            label = f"{label} {unit}".strip()
        ticks.append(
            f"<text x='{x_pos:.1f}' y='{y_label_y:.1f}' text-anchor='middle' "
            f"font-size='10' fill='#666'>{label}</text>"
        )
    return ticks


def _stats_html(stats: dict[str, Any], keys: list[str]) -> str:
    cards = []
    for k in keys:
        if k not in stats:
            continue
        label = k.replace("_", " ")
        cards.append(
            f"<div class='stat'><label>{label}</label><div class='value'>{_fmt(stats[k])}</div></div>"
        )
    return "<div class='stats'>" + "".join(cards) + "</div>"


def _svg_histogram(values: list[float], title: str, bins: int = 24) -> str:
    if not values:
        return f"<p>No data for {title}</p>"
    lo, hi = min(values), max(values)
    if hi <= lo:
        hi = lo + 1e-6
    span = hi - lo
    counts = [0] * bins
    for v in values:
        idx = min(bins - 1, int((v - lo) / span * bins))
        counts[idx] += 1
    cmax = max(counts) or 1
    w, h = 640.0, 220.0
    ml, mb = 52.0, 40.0
    plot_top = 16.0
    pw, ph = w - ml - 16.0, h - mb - plot_top
    bar_w = pw / bins
    med = median(values)
    grid, y_ticks = _axis_grid_y(
        y0=0.0,
        y1=float(cmax),
        ml=ml,
        plot_top=plot_top,
        ph=ph,
        plot_w=pw,
        n_ticks=5,
    )
    parts = [
        f"<div class='chart'><h3>{title}</h3>",
        f"<svg viewBox='0 0 {int(w)} {int(h)}' xmlns='http://www.w3.org/2000/svg'>",
        f"<rect width='{int(w)}' height='{int(h)}' fill='#fafafa'/>",
        *grid,
    ]
    for i, c in enumerate(counts):
        bh = (c / cmax) * (ph - 8)
        x = ml + i * bar_w + 1.0
        y = plot_top + ph - bh
        parts.append(
            f"<rect x='{x:.1f}' y='{y:.1f}' width='{bar_w - 2:.1f}' height='{bh:.1f}' "
            f"fill='#1976d2' opacity='0.85'/>"
        )
    x_ticks = _axis_grid_x(
        x0=lo,
        x1=hi,
        ml=ml,
        plot_top=plot_top,
        ph=ph,
        plot_w=pw,
        y_label_y=h - 8,
        n_ticks=5,
        unit="m",
    )
    parts.extend(
        [
            f"<line x1='{ml}' y1='{plot_top + ph:.1f}' x2='{ml + pw:.1f}' y2='{plot_top + ph:.1f}' "
            f"stroke='#333' stroke-width='1'/>",
            f"<line x1='{ml}' y1='{plot_top}' x2='{ml}' y2='{plot_top + ph:.1f}' "
            f"stroke='#333' stroke-width='1'/>",
            *y_ticks,
            *x_ticks,
            f"<text x='8' y='{plot_top + ph / 2:.1f}' font-size='10' fill='#666' "
            f"transform='rotate(-90 8 {plot_top + ph / 2:.1f})'>count</text>",
            "</svg>",
            f"<p class='caption'>n={len(values)} matched pairs · median={_fmt_tick(med)} m · "
            f"range=[{_fmt_tick(lo)}, {_fmt_tick(hi)}] m</p></div>",
        ]
    )
    return "".join(parts)


def _full_range_bins(
    bins: list[dict[str, float | int]],
    *,
    bin_width_m: float,
    max_range_m: float,
) -> list[dict[str, float | int]]:
    """Fill in empty distance bins so the x-axis spans 0..max_range_m."""
    by_lo = {float(b["bin_lo_m"]): b for b in bins}
    n_bins = max(1, int(math.ceil(max_range_m / bin_width_m)))
    full: list[dict[str, float | int]] = []
    for i in range(n_bins):
        lo = i * bin_width_m
        if lo in by_lo:
            full.append(by_lo[lo])
        else:
            full.append(
                {
                    "bin_lo_m": lo,
                    "bin_hi_m": lo + bin_width_m,
                    "count": 0,
                    "mean_err_m": 0.0,
                    "median_err_m": 0.0,
                }
            )
    return full


def _svg_error_by_distance(
    bins: list[dict[str, float | int]],
    title: str,
    *,
    bin_width_m: float,
    max_range_m: float,
) -> str:
    full_bins = _full_range_bins(bins, bin_width_m=bin_width_m, max_range_m=max_range_m)
    populated = [b for b in full_bins if int(b["count"]) > 0]
    if not populated:
        return ""
    w, h = 720.0, 260.0
    ml, mb = 52.0, 56.0
    plot_top = 16.0
    pw, ph = w - ml - 16.0, h - mb - plot_top
    y_max = max(
        max(float(b["mean_err_m"]) for b in populated),
        max(float(b["median_err_m"]) for b in populated),
    )
    y_max = max(0.05, y_max * 1.12)
    n = len(full_bins)
    bar_w = pw / n
    grid, y_ticks = _axis_grid_y(
        y0=0.0,
        y1=y_max,
        ml=ml,
        plot_top=plot_top,
        ph=ph,
        plot_w=pw,
        n_ticks=5,
    )
    parts = [
        f"<div class='chart'><h3>{title}</h3>",
        f"<p class='legend'><span style='color:#1976d2'>■</span> mean error "
        f"<span style='color:#2e7d32;margin-left:12px'>●</span> median error "
        f"(GT range bins, {bin_width_m:g} m wide, 0–{max_range_m:g} m)</p>",
        f"<svg viewBox='0 0 {int(w)} {int(h)}' xmlns='http://www.w3.org/2000/svg'>",
        f"<rect width='{int(w)}' height='{int(h)}' fill='#fafafa'/>",
        *grid,
    ]
    med_pts: list[str] = []
    total_pairs = 0
    for i, b in enumerate(full_bins):
        cx = ml + i * bar_w + bar_w / 2
        lo, hi = float(b["bin_lo_m"]), float(b["bin_hi_m"])
        parts.append(
            f"<text x='{cx:.1f}' y='{h - 28}' text-anchor='middle' font-size='9' fill='#666'>"
            f"{lo:g}–{hi:g}</text>"
        )
        count = int(b["count"])
        total_pairs += count
        if count == 0:
            parts.append(
                f"<text x='{cx:.1f}' y='{h - 12}' text-anchor='middle' font-size='8' fill='#bbb'>"
                f"n=0</text>"
            )
            continue
        mean_err = float(b["mean_err_m"])
        med_err = float(b["median_err_m"])
        bh = (mean_err / y_max) * (ph - 8)
        x = ml + i * bar_w + 4.0
        y = plot_top + ph - bh
        parts.append(
            f"<rect x='{x:.1f}' y='{y:.1f}' width='{bar_w - 8:.1f}' height='{bh:.1f}' "
            f"fill='#1976d2' opacity='0.85'/>"
        )
        parts.append(
            f"<text x='{cx:.1f}' y='{y - 4:.1f}' text-anchor='middle' font-size='8' "
            f"fill='#1565c0'>{_fmt_tick(mean_err)}</text>"
        )
        my = plot_top + ph - (med_err / y_max) * (ph - 8)
        med_pts.append(f"{cx:.1f},{my:.1f}")
        parts.append(
            f"<text x='{cx:.1f}' y='{h - 12}' text-anchor='middle' font-size='8' fill='#888'>"
            f"n={count}</text>"
        )
    if len(med_pts) >= 2:
        parts.append(
            f"<polyline fill='none' stroke='#2e7d32' stroke-width='2' "
            f"points='{' '.join(med_pts)}'/>"
        )
    for pt in med_pts:
        px, py = pt.split(",")
        parts.append(f"<circle cx='{px}' cy='{py}' r='3' fill='#2e7d32'/>")
    parts.extend(
        [
            f"<line x1='{ml}' y1='{plot_top + ph:.1f}' x2='{ml + pw:.1f}' y2='{plot_top + ph:.1f}' "
            f"stroke='#333' stroke-width='1'/>",
            f"<line x1='{ml}' y1='{plot_top}' x2='{ml}' y2='{plot_top + ph:.1f}' "
            f"stroke='#333' stroke-width='1'/>",
            *y_ticks,
            f"<text x='8' y='{plot_top + ph / 2:.1f}' font-size='10' fill='#666' "
            f"transform='rotate(-90 8 {plot_top + ph / 2:.1f})'>error (m)</text>",
            f"<text x='{ml + pw / 2:.1f}' y='{h - 2}' text-anchor='middle' "
            f"font-size='10' fill='#666'>GT range (m)</text>",
            "</svg>",
            f"<p class='caption'>{total_pairs} matched pairs across {len(populated)} "
            f"non-empty bins (0–{max_range_m:g} m)</p></div>",
        ]
    )
    return "".join(parts)


def _svg_time_series(
    xs: list[float],
    ys: list[float],
    title: str,
    ylabel: str,
    *,
    y_min: float | None = None,
    y_max: float | None = None,
) -> str:
    if len(xs) < 2:
        return ""
    w, h = 640.0, 220.0
    ml, mb = 52.0, 40.0
    plot_top = 16.0
    pw, ph = w - ml - 16.0, h - mb - plot_top
    x0, x1 = min(xs), max(xs)
    data_y0, data_y1 = min(ys), max(ys)
    y0 = 0.0 if y_min is None and data_y0 >= 0.0 else (y_min if y_min is not None else data_y0)
    y1 = y_max if y_max is not None else data_y1
    if y1 <= y0:
        y1 = y0 + 1e-6
    xs_span = max(1e-9, x1 - x0)
    ys_span = max(1e-9, y1 - y0)
    grid, y_ticks = _axis_grid_y(
        y0=y0,
        y1=y1,
        ml=ml,
        plot_top=plot_top,
        ph=ph,
        plot_w=pw,
        n_ticks=5,
    )
    x_ticks = _axis_grid_x(
        x0=x0,
        x1=x1,
        ml=ml,
        plot_top=plot_top,
        ph=ph,
        plot_w=pw,
        y_label_y=h - 8,
        n_ticks=5,
        unit="s",
    )
    pts = []
    for i in range(len(xs)):
        px = ml + (xs[i] - x0) / xs_span * pw
        py = plot_top + ph - (ys[i] - y0) / ys_span * ph
        pts.append(f"{px:.1f},{py:.1f}")
    return (
        f"<div class='chart'><h3>{title}</h3>"
        f"<svg viewBox='0 0 {int(w)} {int(h)}' xmlns='http://www.w3.org/2000/svg'>"
        f"<rect width='{int(w)}' height='{int(h)}' fill='#fafafa'/>"
        f"{''.join(grid)}"
        f"<line x1='{ml}' y1='{plot_top + ph:.1f}' x2='{ml + pw:.1f}' y2='{plot_top + ph:.1f}' "
        f"stroke='#333' stroke-width='1'/>"
        f"<line x1='{ml}' y1='{plot_top}' x2='{ml}' y2='{plot_top + ph:.1f}' "
        f"stroke='#333' stroke-width='1'/>"
        f"{''.join(y_ticks)}{''.join(x_ticks)}"
        f"<text x='8' y='{plot_top + ph / 2:.1f}' font-size='10' fill='#666' "
        f"transform='rotate(-90 8 {plot_top + ph / 2:.1f})'>{ylabel}</text>"
        f"<polyline fill='none' stroke='#1976d2' stroke-width='2' points='{' '.join(pts)}'/>"
        f"</svg>"
        f"<p class='caption'>{len(xs)} frames · {ylabel} min={_fmt_tick(data_y0)}, "
        f"median={_fmt_tick(median(ys))}, max={_fmt_tick(data_y1)}</p></div>"
    )


def _svg_bev_frame(frame: FrameMetrics, size: float = 280.0) -> str:
    cones = frame.gt_cones + frame.pred_cones
    if not cones:
        return "<p>Empty frame</p>"
    xs = [c.x for c in cones]
    ys = [c.y for c in cones]
    pad = 2.0
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    span = max(x1 - x0, y1 - y0, 1.0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2

    def to_px(x: float, y: float) -> tuple[float, float]:
        px = size / 2 + (x - cx) / span * (size - 40)
        py = size / 2 - (y - cy) / span * (size - 40)
        return px, py

    parts = [
        f"<svg viewBox='0 0 {int(size)} {int(size)}' xmlns='http://www.w3.org/2000/svg'>",
        f"<rect width='{int(size)}' height='{int(size)}' fill='#fafafa'/>",
        f"<polygon points='{size/2},{size/2 - 8} {size/2 - 6},{size/2 + 10} {size/2 + 6},{size/2 + 10}' fill='#333'/>",
    ]
    for m in frame.matches:
        p = frame.pred_cones[m.pred_idx]
        g = frame.gt_cones[m.gt_idx]
        x1, y1 = to_px(p.x, p.y)
        x2, y2 = to_px(g.x, g.y)
        parts.append(
            f"<line x1='{x1:.1f}' y1='{y1:.1f}' x2='{x2:.1f}' y2='{y2:.1f}' "
            f"stroke='#2e7d32' stroke-width='1' opacity='0.7'/>"
        )
    matched_g = {m.gt_idx for m in frame.matches}
    matched_p = {m.pred_idx for m in frame.matches}
    for i, g in enumerate(frame.gt_cones):
        px, py = to_px(g.x, g.y)
        col = "#1565c0" if i in matched_g else "#90caf9"
        parts.append(f"<circle cx='{px:.1f}' cy='{py:.1f}' r='4' fill='{col}'/>")
    for i, p in enumerate(frame.pred_cones):
        px, py = to_px(p.x, p.y)
        col = "#c62828" if i in matched_p else "#ef9a9a"
        parts.append(
            f"<circle cx='{px:.1f}' cy='{py:.1f}' r='4' fill='{col}' "
            f"stroke='#b71c1c' stroke-width='0.5'/>"
        )
    parts.append("</svg>")
    cap = (
        f"t={frame.t_s:.1f}s · GT {frame.n_gt} · pred {frame.n_pred} · "
        f"TP {frame.n_tp} FP {frame.n_fp} FN {frame.n_fn} · err μ={frame.mean_match_err_m:.2f}m"
    )
    return f"<div class='chart'><p><small>{cap}</small>{''.join(parts)}</div>"


def _frame_from_sample(item: dict[str, Any]) -> FrameMetrics:
    fm = FrameMetrics(
        t_s=item["t_s"],
        latency_ms=item["latency_ms"],
        n_points=item["n_points"],
        n_gt=item["n_gt"],
        n_pred=item["n_pred"],
        n_tp=item["n_tp"],
        n_fp=item["n_fp"],
        n_fn=item["n_fn"],
        mean_match_err_m=item["mean_match_err_m"],
        max_match_err_m=item["max_match_err_m"],
        gt_cones=[Cone2D(**c) for c in item["gt"]],
        pred_cones=[Cone2D(**c) for c in item["pred"]],
        matches=[MatchResult(**m) for m in item.get("matches", [])],
    )
    return fm


def render_perception_html(summary: dict[str, Any], run_dir: Path) -> str:
    has_gt = bool(summary.get("gt_eval"))
    warn = ""
    if not has_gt:
        warn = (
            "<div class='warn'><strong>No GT in bag.</strong> Re-record with "
            "<code>/testing_only/track</code> and <code>/testing_only/odom</code> "
            "using <code>capture_benchmark_bag.py</code> for cone-vs-prediction plots.</div>"
        )
    elif summary.get("gt_hfov_deg") is None or summary.get("gt_scan_period_ms") is None:
        warn = (
            "<div class='warn'><strong>This run predates GT alignment fixes.</strong> "
            "Re-run <code>run_perception_benchmark.py</code> on the bag (not only "
            "<code>generate_report.py</code>). New reports use end-of-scan GT odom "
            "(LiDAR motion), ±60° FOV, and 0.5 m blind-spot gate.</div>"
        )
    else:
        bias_x = (summary.get("gt_metrics") or {}).get("mean_bias_x_m")
        bias_y = (summary.get("gt_metrics") or {}).get("mean_bias_y_m")
        if bias_x is not None and abs(bias_x) + abs(bias_y or 0) > 0.35:
            warn = (
                f"<div class='warn'><strong>Systematic pred−GT offset</strong> "
                f"(mean bias x={bias_x:+.2f} m, y={bias_y:+.2f} m). "
                "Green match lines can still look displaced when error is ~0.5–1 m "
                f"(gate {summary.get('match_gate_m', 1.5)} m). "
                "Residual bias is often LiDAR sweep motion; try "
                "<code>--gt-scan-center-frac 0.5</code> (default) or detection tuning.</div>"
            )

    perf_keys = [
        "frames",
        "mean_latency_ms",
        "median_latency_ms",
        "max_latency_ms",
        "mean_cones_per_frame",
        "median_cones_per_frame",
    ]
    gt_keys = [
        "precision",
        "recall",
        "mean_match_err_m",
        "median_match_err_m",
        "p95_match_err_m",
        "max_match_err_m",
        "mean_gt_per_frame",
        "median_gt_per_frame",
        "mean_pred_per_frame",
        "median_pred_per_frame",
        "total_tp",
        "total_fp",
        "total_fn",
        "mean_bias_x_m",
        "mean_bias_y_m",
        "median_bias_x_m",
        "median_bias_y_m",
    ]
    gt = summary.get("gt_metrics") or {}

    charts = ""
    details_path = run_dir / "frame_details.jsonl"
    if has_gt and details_path.is_file():
        errs: list[float] = []
        range_err_pairs: list[tuple[float, float]] = []
        t_axis: list[float] = []
        err_t: list[float] = []
        recall_t: list[float] = []
        with details_path.open() as fh:
            for line in fh:
                row = json.loads(line)
                frame_errs = row.get("match_errs", [])
                frame_ranges = row.get("match_ranges_m")
                errs.extend(frame_errs)
                if frame_ranges and len(frame_ranges) == len(frame_errs):
                    range_err_pairs.extend(zip(frame_ranges, frame_errs))
                t_axis.append(row["t_s"])
                err_t.append(row["mean_match_err_m"] * 100.0)
                tp, fn = row["n_tp"], row["n_fn"]
                recall_t.append(tp / (tp + fn) if (tp + fn) else 0.0)

        gt_range_m = float(summary.get("gt_range_m") or 20.0)
        bin_width_m = max(1.0, gt_range_m / 10.0)
        range_bins = summarize_error_by_range(
            range_err_pairs,
            bin_width_m=bin_width_m,
            max_range_m=gt_range_m,
        )
        if range_err_pairs:
            charts += _svg_error_by_distance(
                range_bins,
                "Match error vs GT cone distance",
                bin_width_m=bin_width_m,
                max_range_m=gt_range_m,
            )

        charts += _svg_histogram(errs, "Position error distribution (matched pairs, m)")
        charts += _svg_time_series(t_axis, err_t, "Mean match error per frame", "cm")
        charts += _svg_time_series(
            t_axis, recall_t, "Recall per frame", "recall", y_min=0.0, y_max=1.0
        )

        sample_path = run_dir / "frame_samples.json"
        if sample_path.is_file():
            samples = json.loads(sample_path.read_text())
            bev = "".join(_svg_bev_frame(_frame_from_sample(s)) for s in samples)
            charts += (
                "<h2>GT vs prediction (sample frames, base_link)</h2>"
                "<p class='legend'><span class='gt'>● GT</span> "
                "<span class='pred'>● Pred</span> <span class='match'>— match</span> ▲ ego</p>"
                f"<div class='frame-grid'>{bev}</div>"
            )

    from report_html import _render_charts_for_module

    csv_path = run_dir / "results.csv"
    if csv_path.is_file():
        charts += _render_charts_for_module("perception", csv_path)

    profile_html = ""
    profile = summary.get("profile")
    profile_path = run_dir / "profile.json"
    if not profile and profile_path.is_file():
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
        summary["profile"] = profile
    if profile:
        from perception_profiling import render_profile_html

        profile_html = render_profile_html(profile)
    elif (run_dir / "results.csv").is_file():
        profile_html = (
            "<div class='warn'><strong>No pipeline profile in this run.</strong> "
            "Re-run with <code>--profile</code> (and optionally "
            "<code>--ransac-ablation</code>) to populate the stage table and chart: "
            "<code>python run_perception_benchmark.py &lt;bag&gt; --profile "
            "--profile-frames 80</code></div>"
        )

    title = f"Perception benchmark — {summary.get('strategy', 'base')}"
    return f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>{title}</title>
<style>{_PAGE_STYLE}</style></head><body>
<h1>{title}</h1>
<p class="subtitle">Bag: <code>{summary.get('bag', '')}</code></p>
{warn}
<h2>Performance</h2>
{_stats_html(summary, perf_keys)}
{profile_html}
<h2>Detection vs sim GT</h2>
{_stats_html(gt, gt_keys) if has_gt else '<p>—</p>'}
{charts}
<details><summary>Raw JSON</summary><pre>{json.dumps(summary, indent=2)}</pre></details>
</body></html>"""
