from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import median
from typing import Any

MODULE_METRIC = {
    "perception": "latency_ms",
    "slam": "err_m",
    "control": "cross_track_err_m",
}

# Extra perception series (column name, chart title, y-axis label).
PERCEPTION_CHARTS = [
    ("latency_ms", "Detection latency", "ms"),
    ("n_cones", "Cones per frame", "count"),
    ("n_points", "LiDAR points per frame", "points"),
]

_PAGE_STYLE = """
body { font-family: system-ui, Arial, sans-serif; margin: 24px; max-width: 900px; color: #1a1a1a; }
h1 { font-size: 1.35rem; margin-bottom: 0.25rem; }
.subtitle { color: #555; margin-top: 0; margin-bottom: 1.5rem; }
.stats { display: grid; grid-template-columns: repeat(auto-fill, minmax(160px, 1fr)); gap: 12px; margin-bottom: 1.5rem; }
.stat { background: #f4f6f8; border-radius: 8px; padding: 12px; }
.stat label { display: block; font-size: 0.75rem; color: #666; text-transform: uppercase; letter-spacing: 0.04em; }
.stat value { font-size: 1.1rem; font-weight: 600; margin-top: 4px; }
.chart { border: 1px solid #ddd; border-radius: 8px; padding: 12px; margin-bottom: 16px; background: #fff; }
.chart h3 { margin: 0 0 8px; font-size: 0.95rem; }
.chart svg { display: block; width: 100%; max-width: 840px; height: auto; background: #fafafa; }
.chart .caption { font-size: 0.8rem; color: #666; margin-top: 6px; }
details { margin-top: 1.5rem; }
pre { background: #f8f8f8; padding: 12px; border-radius: 8px; overflow: auto; font-size: 0.8rem; }
"""


def resolve_csv_path(csv_path: str | None, results_root: Path, run_dir: Path | None = None) -> Path | None:
    if run_dir is not None:
        candidate = run_dir / "results.csv"
        if candidate.is_file():
            return candidate
    if not csv_path:
        return None
    p = Path(csv_path)
    if p.is_file():
        return p
    prefix = "/results/"
    if csv_path.startswith(prefix):
        host = results_root / csv_path[len(prefix) :]
        if host.is_file():
            return host
    return p if p.is_file() else None


def _load_csv_columns(csv_path: Path, columns: list[str], max_points: int = 2000) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {c: [] for c in columns}
    with csv_path.open(newline="") as fh:
        for row in csv.DictReader(fh):
            ok = True
            vals: dict[str, float] = {}
            for col in columns:
                if col not in row:
                    ok = False
                    break
                try:
                    vals[col] = float(row[col])
                except ValueError:
                    ok = False
                    break
            if not ok:
                continue
            for col in columns:
                out[col].append(vals[col])
            if len(out[columns[0]]) >= max_points:
                break
    return out


def _svg_chart(
    xs: list[float],
    ys: list[float],
    *,
    title: str,
    y_label: str,
    width: float = 800.0,
    height: float = 220.0,
    margin_left: float = 56.0,
    margin_bottom: float = 36.0,
) -> str:
    if len(xs) < 2 or len(ys) < 2:
        return f"<p>Not enough samples for <em>{title}</em>.</p>"

    plot_w = width - margin_left - 16.0
    plot_h = height - margin_bottom - 16.0
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_span = max(1e-9, x_max - x_min)
    y_span = max(1e-9, y_max - y_min)

    def px(i: int) -> float:
        return margin_left + ((xs[i] - x_min) / x_span) * plot_w

    def py(i: int) -> float:
        return 12.0 + plot_h - ((ys[i] - y_min) / y_span) * plot_h

    pts = " ".join(f"{px(i):.1f},{py(i):.1f}" for i in range(len(xs)))

    # Horizontal grid + y tick labels (5 ticks).
    grid_lines: list[str] = []
    y_ticks: list[str] = []
    for t in range(6):
        frac = t / 5.0
        y_val = y_min + frac * y_span
        y_pos = 12.0 + plot_h - frac * plot_h
        grid_lines.append(
            f"<line x1='{margin_left}' y1='{y_pos:.1f}' x2='{margin_left + plot_w:.1f}' "
            f"y2='{y_pos:.1f}' stroke='#e8e8e8' stroke-width='1'/>"
        )
        y_ticks.append(
            f"<text x='{margin_left - 6}' y='{y_pos + 4:.1f}' text-anchor='end' "
            f"font-size='10' fill='#666'>{y_val:.1f}</text>"
        )

    x0_label = f"{xs[0]:.1f}"
    x1_label = f"{xs[-1]:.1f}"

    body = "".join(
        [
            f"<div class='chart'><h3>{title}</h3>",
            f"<svg viewBox='0 0 {int(width)} {int(height)}' xmlns='http://www.w3.org/2000/svg'>",
            f"<rect x='0' y='0' width='{int(width)}' height='{int(height)}' fill='#fafafa'/>",
            *grid_lines,
            f"<line x1='{margin_left}' y1='{12 + plot_h}' x2='{margin_left + plot_w}' "
            f"y2='{12 + plot_h}' stroke='#333' stroke-width='1'/>",
            f"<line x1='{margin_left}' y1='12' x2='{margin_left}' y2='{12 + plot_h}' "
            f"stroke='#333' stroke-width='1'/>",
            *y_ticks,
            f"<text x='{margin_left}' y='{height - 8}' font-size='10' fill='#666'>{x0_label}</text>",
            f"<text x='{margin_left + plot_w}' y='{height - 8}' text-anchor='end' "
            f"font-size='10' fill='#666'>{x1_label}</text>",
            f"<text x='14' y='{12 + plot_h / 2}' font-size='11' fill='#444' "
            f"transform='rotate(-90 14 {12 + plot_h / 2})' text-anchor='middle'>{y_label}</text>",
            f"<polyline fill='none' stroke='#1976d2' stroke-width='2' points='{pts}'/>",
            "</svg>",
            f"<p class='caption'>{len(xs)} samples · {y_label} min={y_min:.2f}, median={median(ys):.2f}, "
            f"mean={sum(ys) / len(ys):.2f}, max={y_max:.2f}</p></div>",
        ]
    )
    return body


def _stats_grid(summary: dict[str, Any]) -> str:
    skip = {
        "module",
        "strategy",
        "csv",
        "report",
        "bag",
        "gt_eval",
        "match_gate_m",
        "gt_range_m",
        "gt_min_range_m",
        "gt_hfov_deg",
    }
    items: dict[str, Any] = {}
    for key, val in summary.items():
        if key in skip or isinstance(val, dict):
            continue
        items[key] = val
    nested = summary.get("gt_metrics") or summary.get("gt_cones")
    if isinstance(nested, dict):
        for key, val in nested.items():
            if isinstance(val, (int, float)):
                items.setdefault(key, val)
    cards = []
    for key, val in items.items():
        if isinstance(val, float):
            display = f"{val:.3f}" if abs(val) < 1000 else f"{val:.1f}"
        else:
            display = str(val)
        label = key.replace("_", " ")
        cards.append(
            f"<div class='stat'><label>{label}</label><div class='value'>{display}</div></div>"
        )
    return "<div class='stats'>" + "".join(cards) + "</div>"


def _render_charts_for_module(module: str, csv_path: Path | None) -> str:
    if csv_path is None or not csv_path.is_file():
        return "<p>No CSV found — charts unavailable.</p>"

    if module == "perception":
        cols = _load_csv_columns(csv_path, ["t_s", "latency_ms", "n_cones", "n_points"])
        if len(cols["t_s"]) < 2:
            return "<p>Not enough rows in results.csv.</p>"
        parts = []
        for col, title, ylab in PERCEPTION_CHARTS:
            parts.append(_svg_chart(cols["t_s"], cols[col], title=title, y_label=ylab))
        return "\n".join(parts)

    metric = MODULE_METRIC.get(module, "")
    if not metric:
        return ""
    cols = _load_csv_columns(csv_path, ["t_s", metric])
    if len(cols["t_s"]) < 2:
        return "<p>Not enough rows in results.csv.</p>"
    return _svg_chart(cols["t_s"], cols[metric], title=metric, y_label=metric)


def render_run_html(summary: dict[str, Any], csv_path: Path | None) -> str:
    module = summary.get("module", "unknown")
    strategy = summary.get("strategy", "n/a")
    bag = summary.get("bag", "")
    title = f"IFSSIM {module} benchmark — {strategy}"
    charts = _render_charts_for_module(module, csv_path)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title}</title>
  <style>{_PAGE_STYLE}</style>
</head>
<body>
  <h1>{title}</h1>
  <p class="subtitle">Bag: <code>{bag}</code></p>
  {_stats_grid(summary)}
  <h2>Time series</h2>
  {charts}
  <details>
    <summary>Raw JSON</summary>
    <pre>{json.dumps(summary, indent=2)}</pre>
  </details>
</body>
</html>
"""


def write_run_report(
    summary: dict[str, Any],
    run_dir: Path,
    results_root: Path | None = None,
    filename: str = "report.html",
) -> Path:
    root = results_root if results_root is not None else run_dir.parent.parent
    csv_path = resolve_csv_path(summary.get("csv"), root, run_dir)
    out = run_dir / filename
    if summary.get("module") == "perception":
        from perception_report import render_perception_html

        html = render_perception_html(summary, run_dir)
    elif summary.get("module") == "slam":
        from slam_report import render_slam_html

        html = render_slam_html(summary, run_dir)
    elif summary.get("module") == "onboard":
        from onboard_report import render_onboard_html

        samples_path = run_dir / "samples.json"
        samples = json.loads(samples_path.read_text()) if samples_path.is_file() else {}
        html = render_onboard_html(summary, samples)
    else:
        html = render_run_html(summary, csv_path)
    out.write_text(html, encoding="utf-8")
    summary["report"] = str(out)
    return out


def render_index_html(rows: list[dict[str, Any]], results_root: Path) -> str:
    sections = []
    for r in rows:
        module = r.get("module", "unknown")
        run_dir = Path(r["report"]).parent if r.get("report") else None
        csv_path = resolve_csv_path(r.get("csv"), results_root, run_dir)
        sections.append(
            f"<section style='margin-bottom:2rem;padding-bottom:1rem;border-bottom:1px solid #ddd'>"
            f"<h2>{module} — {r.get('strategy', 'n/a')}</h2>"
            f"{_stats_grid(r)}"
            f"{_render_charts_for_module(module, csv_path)}"
            f"</section>"
        )
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>IFSSIM Sim Benchmark Report</title>
  <style>{_PAGE_STYLE}</style>
</head>
<body>
  <h1>IFSSIM Sim Benchmark Report</h1>
  <p class="subtitle">{len(rows)} run(s)</p>
  {"".join(sections)}
</body>
</html>
"""
