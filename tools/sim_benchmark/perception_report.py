from __future__ import annotations

import json
from pathlib import Path
from statistics import median
from typing import Any

from perception_metrics import Cone2D, FrameMetrics, MatchResult

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
    span = max(1e-6, hi - lo)
    counts = [0] * bins
    for v in values:
        idx = min(bins - 1, int((v - lo) / span * bins))
        counts[idx] += 1
    cmax = max(counts) or 1
    w, h = 520.0, 140.0
    bar_w = w / bins
    med = median(values) if values else 0.0
    bars = []
    for i, c in enumerate(counts):
        bh = (c / cmax) * (h - 24)
        x = 40 + i * bar_w
        y = h - 20 - bh
        bars.append(
            f"<rect x='{x:.1f}' y='{y:.1f}' width='{bar_w - 2:.1f}' height='{bh:.1f}' "
            f"fill='#1976d2' opacity='0.85'/>"
        )
    return (
        f"<div class='chart'><h3>{title}</h3>"
        f"<svg viewBox='0 0 {int(w)} {int(h)}' xmlns='http://www.w3.org/2000/svg'>"
        f"<text x='40' y='14' font-size='11' fill='#666'>{lo:.2f} m</text>"
        f"<text x='{w - 50}' y='14' text-anchor='end' font-size='11' fill='#666'>{hi:.2f} m</text>"
        + "".join(bars)
        + f"<text x='40' y='{h - 4}' font-size='10' fill='#666'>n={len(values)} · median={med:.3f} m</text>"
        + "</svg></div>"
    )


def _svg_time_series(xs: list[float], ys: list[float], title: str, ylabel: str) -> str:
    if len(xs) < 2:
        return ""
    w, h, ml, mb = 640.0, 200.0, 50.0, 32.0
    pw, ph = w - ml - 12, h - mb - 12
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    xs_span = max(1e-9, x1 - x0)
    ys_span = max(1e-9, y1 - y0)
    pts = []
    for i in range(len(xs)):
        px = ml + (xs[i] - x0) / xs_span * pw
        py = 12 + ph - (ys[i] - y0) / ys_span * ph
        pts.append(f"{px:.1f},{py:.1f}")
    return (
        f"<div class='chart'><h3>{title}</h3>"
        f"<svg viewBox='0 0 {int(w)} {int(h)}'>"
        f"<polyline fill='none' stroke='#1976d2' stroke-width='2' points='{' '.join(pts)}'/>"
        f"<text x='{ml}' y='{h - 6}' font-size='10' fill='#666'>{x0:.0f}s</text>"
        f"<text x='{ml + pw}' y='{h - 6}' text-anchor='end' font-size='10' fill='#666'>{x1:.0f}s</text>"
        f"<text x='8' y='{12 + ph / 2}' font-size='10' fill='#666' "
        f"transform='rotate(-90 8 {12 + ph / 2})'>{ylabel}</text>"
        f"</svg></div>"
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
        t_axis: list[float] = []
        err_t: list[float] = []
        recall_t: list[float] = []
        with details_path.open() as fh:
            for line in fh:
                row = json.loads(line)
                errs.extend(row.get("match_errs", []))
                t_axis.append(row["t_s"])
                err_t.append(row["mean_match_err_m"] * 100.0)
                tp, fn = row["n_tp"], row["n_fn"]
                recall_t.append(tp / (tp + fn) if (tp + fn) else 0.0)

        charts += _svg_histogram(errs, "Position error distribution (matched pairs, m)")
        charts += _svg_time_series(t_axis, err_t, "Mean match error per frame", "cm")
        charts += _svg_time_series(t_axis, recall_t, "Recall per frame", "recall")

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
