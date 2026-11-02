"""No-GT HTML report for an onboard bag replayed through the live pipeline.

Scores are consistency / rate / coverage — there is no sim ground truth.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from statistics import mean, median
from typing import Any

from common import write_csv, write_json

_PAGE_STYLE = """
body { font-family: system-ui, Arial, sans-serif; margin: 24px; max-width: 1100px; color: #1a1a1a; }
h1 { font-size: 1.35rem; margin-bottom: 0.25rem; }
h2 { font-size: 1.1rem; margin-top: 1.75rem; border-bottom: 1px solid #e0e0e0; padding-bottom: 4px; }
.subtitle { color: #555; margin-top: 0; }
.note { font-size: 0.88rem; color: #444; line-height: 1.45; max-width: 920px; }
.warn { background: #fff8e1; border: 1px solid #ffe082; padding: 12px; border-radius: 8px; margin: 12px 0; }
.stats { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 8px; margin: 12px 0; }
.stat { background: #f4f6f8; border-radius: 8px; padding: 10px; }
.stat label { font-size: 0.65rem; color: #666; text-transform: uppercase; display: block; }
.stat .value { font-size: 0.95rem; font-weight: 600; margin-top: 4px; }
.chart { border: 1px solid #ddd; border-radius: 8px; padding: 12px; margin: 12px 0; background: #fff; }
.chart svg { width: 100%; max-width: 960px; display: block; background: #fafafa; }
.chart .caption { font-size: 0.8rem; color: #666; margin-top: 6px; }
.grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
@media (max-width: 900px) { .grid2 { grid-template-columns: 1fr; } }
details { margin-top: 1.5rem; }
pre { background: #f8f8f8; padding: 12px; border-radius: 8px; overflow: auto; font-size: 0.8rem; }
"""

_COLORS = {
    "odom": "#e65100",
    "slam": "#2e7d32",
    "path": "#6a1b9a",
    "cmd": "#1565c0",
    "pilot": "#c62828",
    "cones": "#1976d2",
}


def path_length_m(xs: list[float], ys: list[float]) -> float:
    if len(xs) < 2:
        return 0.0
    return float(
        sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))
    )


def empty_rate(counts: list[int]) -> float:
    if not counts:
        return 1.0
    return sum(1 for n in counts if n <= 0) / len(counts)


def interp_at(ts: list[float], vs: list[float], tq: float) -> float | None:
    if not ts or not vs or len(ts) != len(vs):
        return None
    if tq <= ts[0]:
        return vs[0]
    if tq >= ts[-1]:
        return vs[-1]
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= tq:
            lo = mid
        else:
            hi = mid
    span = ts[hi] - ts[lo]
    w = 0.0 if span == 0 else (tq - ts[lo]) / span
    return vs[lo] + w * (vs[hi] - vs[lo])


def steer_residual_rad(
    cmd: list[dict[str, float]],
    steering: list[dict[str, float]],
) -> list[float]:
    ts = [float(s["t_s"]) for s in steering]
    vs = [float(s["rad"]) for s in steering]
    out: list[float] = []
    for row in cmd:
        ref = interp_at(ts, vs, float(row["t_s"]))
        if ref is None:
            continue
        out.append(abs(float(row["steering"]) - ref))
    return out


def summarize(samples: dict[str, Any]) -> dict[str, Any]:
    source_counts = dict(samples.get("source_counts") or {})
    conos_raw = list(samples.get("conos_raw") or [])
    conos = list(samples.get("conos") or [])
    odom = list(samples.get("odom") or [])
    slam = list(samples.get("slam") or [])
    path = list(samples.get("path") or [])
    cmd = list(samples.get("cmd") or [])
    steering = list(samples.get("steering") or [])
    raw_ns = [int(r["n"]) for r in conos_raw]
    residuals = steer_residual_rad(cmd, steering)
    odom_xs = [float(p["x"]) for p in odom]
    odom_ys = [float(p["y"]) for p in odom]
    slam_xs = [float(p["x"]) for p in slam]
    slam_ys = [float(p["y"]) for p in slam]
    last_path = path[-1] if path else {}
    end_gap = None
    if odom and slam:
        end_gap = math.hypot(
            float(odom[-1]["x"]) - float(slam[-1]["x"]),
            float(odom[-1]["y"]) - float(slam[-1]["y"]),
        )
    return {
        "module": "onboard",
        "gt_eval": False,
        "source_duration_s": float(samples.get("source_duration_s") or 0.0),
        "source_imu": int(source_counts.get("/imu") or 0),
        "source_lidar": int(source_counts.get("/lidar_points") or 0),
        "source_rpm": int(source_counts.get("/motor_rpm") or 0),
        "n_conos_raw": len(conos_raw),
        "n_conos": len(conos),
        "n_odom": len(odom),
        "n_slam": len(slam),
        "n_path": len(path),
        "n_cmd": len(cmd),
        "mean_conos_raw": float(mean(raw_ns)) if raw_ns else 0.0,
        "median_conos_raw": float(median(raw_ns)) if raw_ns else 0.0,
        "max_conos_raw": int(max(raw_ns)) if raw_ns else 0,
        "empty_detection_rate": empty_rate(raw_ns),
        "odom_path_length_m": path_length_m(odom_xs, odom_ys),
        "slam_path_length_m": path_length_m(slam_xs, slam_ys),
        "last_path_poses": int(last_path.get("n") or 0),
        "odom_slam_end_gap_m": end_gap,
        "mean_abs_steer_residual_rad": float(mean(residuals)) if residuals else None,
        "n_map_cones": len(samples.get("map_x") or []),
    }


def write_run_files(samples: dict[str, Any], run_dir: Path, summary: dict[str, Any]) -> None:
    write_csv(
        run_dir / "perception.csv",
        [
            {"t_s": r["t_s"], "n_conos_raw": r["n"]}
            for r in samples.get("conos_raw") or []
        ],
    )
    odom_by_t = {float(p["t_s"]): p for p in samples.get("odom") or []}
    slam_by_t = {float(p["t_s"]): p for p in samples.get("slam") or []}
    traj_rows = []
    for t in sorted(set(odom_by_t) | set(slam_by_t)):
        o = odom_by_t.get(t)
        s = slam_by_t.get(t)
        traj_rows.append(
            {
                "t_s": t,
                "odom_x": "" if o is None else o["x"],
                "odom_y": "" if o is None else o["y"],
                "slam_x": "" if s is None else s["x"],
                "slam_y": "" if s is None else s["y"],
            }
        )
    write_csv(run_dir / "trajectory.csv", traj_rows)
    write_csv(run_dir / "control.csv", list(samples.get("cmd") or []))
    write_csv(
        run_dir / "map_cones.csv",
        [
            {"x": x, "y": y}
            for x, y in zip(samples.get("map_x") or [], samples.get("map_y") or [])
        ],
    )
    summary["csv"] = str(run_dir / "perception.csv")
    write_json(run_dir / "results.json", summary)


def _fmt(v: Any) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        if math.isnan(v):
            return "—"
        return f"{v:.3f}" if abs(v) < 1000 else f"{v:.1f}"
    return str(v)


def _stats_grid(summary: dict[str, Any]) -> str:
    skip = {"module", "gt_eval", "csv", "report", "bag", "strategy", "mission"}
    cards = []
    for key, val in summary.items():
        if key in skip or isinstance(val, (dict, list)):
            continue
        cards.append(
            "<div class='stat'><label>"
            + key.replace("_", " ")
            + f"</label><div class='value'>{_fmt(val)}</div></div>"
        )
    return "<div class='stats'>" + "".join(cards) + "</div>"


def _decimate(xs: list[float], ys: list[float], max_pts: int = 1400) -> tuple[list[float], list[float]]:
    if len(xs) <= max_pts:
        return xs, ys
    step = max(1, len(xs) // max_pts)
    return xs[::step], ys[::step]


def _svg_series(
    xs: list[float],
    series: list[tuple[str, list[float], str]],
    *,
    title: str,
    y_label: str,
    width: float = 800.0,
    height: float = 220.0,
) -> str:
    usable = [(name, ys, color) for name, ys, color in series if len(xs) >= 2 and len(ys) >= 2]
    if not usable:
        return f"<p class='note'>Not enough samples for {title}.</p>"
    ml, mb = 56.0, 36.0
    plot_w = width - ml - 16.0
    plot_h = height - mb - 16.0
    x_min, x_max = min(xs), max(xs)
    y_vals = [v for _, ys, _ in usable for v in ys]
    y_min, y_max = min(y_vals), max(y_vals)
    x_span = max(1e-9, x_max - x_min)
    y_span = max(1e-9, y_max - y_min)

    def py(v: float) -> float:
        return 12.0 + plot_h - ((v - y_min) / y_span) * plot_h

    parts = [
        f"<div class='chart'><h3>{title}</h3>",
        f"<svg viewBox='0 0 {int(width)} {int(height)}' xmlns='http://www.w3.org/2000/svg'>",
        f"<rect width='{int(width)}' height='{int(height)}' fill='#fafafa'/>",
    ]
    for t in range(6):
        frac = t / 5.0
        yv = y_min + frac * y_span
        yp = py(yv)
        parts.append(
            f"<line x1='{ml}' y1='{yp:.1f}' x2='{ml + plot_w:.1f}' y2='{yp:.1f}' "
            f"stroke='#e8e8e8'/>"
        )
        parts.append(
            f"<text x='{ml - 6}' y='{yp + 4:.1f}' text-anchor='end' font-size='10' "
            f"fill='#666'>{yv:.2f}</text>"
        )
    legend = []
    for name, ys, color in usable:
        xs_d, ys_d = _decimate(xs[: len(ys)], ys)
        pts = " ".join(
            f"{ml + ((xs_d[i] - x_min) / x_span) * plot_w:.1f},{py(ys_d[i]):.1f}"
            for i in range(len(xs_d))
        )
        parts.append(
            f"<polyline fill='none' stroke='{color}' stroke-width='2' points='{pts}'/>"
        )
        legend.append(f"<span style='color:{color}'>{name}</span>")
    parts.append("</svg>")
    parts.append(
        f"<p class='caption'>{' · '.join(legend)} · {y_label} · {len(xs)} samples</p></div>"
    )
    return "".join(parts)


def _svg_xy(
    paths: list[tuple[str, list[float], list[float], str]],
    *,
    title: str,
    size: float = 460.0,
) -> str:
    pts_x: list[float] = []
    pts_y: list[float] = []
    for _, xs, ys, _ in paths:
        pts_x.extend(xs)
        pts_y.extend(ys)
    if len(pts_x) < 2:
        return f"<p class='note'>Not enough trajectory for {title}.</p>"
    pad = 1.5
    x0, x1 = min(pts_x) - pad, max(pts_x) + pad
    y0, y1 = min(pts_y) - pad, max(pts_y) + pad
    span = max(x1 - x0, y1 - y0, 1.0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    ml = 48.0
    plot = size - ml - 16.0

    def px(x: float, y: float) -> tuple[float, float]:
        sx = ml + plot / 2 + (x - cx) / span * (plot - 20)
        sy = 12 + plot / 2 - (y - cy) / span * (plot - 20)
        return sx, sy

    parts = [
        f"<div class='chart'><h3>{title}</h3>",
        f"<svg viewBox='0 0 {int(size)} {int(size + 20)}' xmlns='http://www.w3.org/2000/svg'>",
        f"<rect width='{int(size)}' height='{int(size + 20)}' fill='#fafafa'/>",
    ]
    legend = []
    for name, xs, ys, color in paths:
        if len(xs) < 2:
            continue
        xs_d, ys_d = _decimate(xs, ys)
        pts = " ".join(
            f"{px(xs_d[i], ys_d[i])[0]:.1f},{px(xs_d[i], ys_d[i])[1]:.1f}"
            for i in range(len(xs_d))
        )
        parts.append(
            f"<polyline fill='none' stroke='{color}' stroke-width='2' points='{pts}'/>"
        )
        legend.append(f"<span style='color:{color}'>{name}</span>")
    parts.append("</svg>")
    parts.append(f"<p class='caption'>{' · '.join(legend)} · metres</p></div>")
    return "".join(parts)


def _svg_scatter(xs: list[float], ys: list[float], *, title: str, size: float = 460.0) -> str:
    if len(xs) < 1:
        return f"<p class='note'>No map cones for {title}.</p>"
    pad = 1.5
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    span = max(x1 - x0, y1 - y0, 1.0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    ml = 48.0
    plot = size - ml - 16.0

    def px(x: float, y: float) -> tuple[float, float]:
        sx = ml + plot / 2 + (x - cx) / span * (plot - 20)
        sy = 12 + plot / 2 - (y - cy) / span * (plot - 20)
        return sx, sy

    dots = []
    for x, y in zip(xs, ys):
        sx, sy = px(x, y)
        dots.append(f"<circle cx='{sx:.1f}' cy='{sy:.1f}' r='3' fill='#2e7d32'/>")
    return (
        f"<div class='chart'><h3>{title}</h3>"
        f"<svg viewBox='0 0 {int(size)} {int(size + 20)}' xmlns='http://www.w3.org/2000/svg'>"
        f"<rect width='{int(size)}' height='{int(size + 20)}' fill='#fafafa'/>"
        f"{''.join(dots)}</svg>"
        f"<p class='caption'>{len(xs)} landmarks · last /Conos</p></div>"
    )


def render_onboard_html(summary: dict[str, Any], samples: dict[str, Any]) -> str:
    conos_raw = list(samples.get("conos_raw") or [])
    odom = list(samples.get("odom") or [])
    slam = list(samples.get("slam") or [])
    path = list(samples.get("path") or [])
    cmd = list(samples.get("cmd") or [])
    steering = list(samples.get("steering") or [])
    last_path = path[-1] if path else {}
    det_t = [float(r["t_s"]) for r in conos_raw]
    det_n = [float(r["n"]) for r in conos_raw]
    cmd_t = [float(r["t_s"]) for r in cmd]
    charts = [
        _svg_xy(
            [
                ("odom", [float(p["x"]) for p in odom], [float(p["y"]) for p in odom], _COLORS["odom"]),
                ("slam", [float(p["x"]) for p in slam], [float(p["y"]) for p in slam], _COLORS["slam"]),
                (
                    "path",
                    list(last_path.get("xs") or []),
                    list(last_path.get("ys") or []),
                    _COLORS["path"],
                ),
            ],
            title="Trajectory (odom / SLAM / last path)",
        ),
        _svg_scatter(
            list(samples.get("map_x") or []),
            list(samples.get("map_y") or []),
            title="SLAM map cones",
        ),
        _svg_series(
            det_t,
            [("detections", det_n, _COLORS["cones"])],
            title="Perceived cones per scan (/Conos_raw)",
            y_label="count",
        ),
        _svg_series(
            cmd_t,
            [
                ("cmd steer", [float(r["steering"]) for r in cmd], _COLORS["cmd"]),
                (
                    "pilot steer",
                    [
                        interp_at(
                            [float(s["t_s"]) for s in steering],
                            [float(s["rad"]) for s in steering],
                            float(r["t_s"]),
                        )
                        or 0.0
                        for r in cmd
                    ],
                    _COLORS["pilot"],
                )
                if steering
                else ("pilot steer", [], _COLORS["pilot"]),
            ],
            title="Steering: autonomy command vs recorded pilot",
            y_label="rad",
        ),
        _svg_series(
            cmd_t,
            [("throttle", [float(r["throttle"]) for r in cmd], _COLORS["odom"])],
            title="Autonomy throttle command",
            y_label="[0,1]",
        ),
    ]
    bag = summary.get("bag", "")
    mission = summary.get("mission") or summary.get("strategy") or "n/a"
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>IFSSIM onboard replay — {mission}</title>
  <style>{_PAGE_STYLE}</style>
</head>
<body>
  <h1>Onboard pipeline replay</h1>
  <p class="subtitle">Bag: <code>{bag}</code> · mission <code>{mission}</code></p>
  <div class="warn">
    No ground truth. Charts are pipeline outputs on a recorded drive:
    detection counts, odom/SLAM trajectories, map landmarks, and
    autonomy commands vs the pilot's recorded steering.
  </div>
  {_stats_grid(summary)}
  <div class="grid2">{charts[0]}{charts[1]}</div>
  {charts[2]}
  {charts[3]}
  {charts[4]}
  <details>
    <summary>Raw JSON</summary>
    <pre>{json.dumps(summary, indent=2)}</pre>
  </details>
</body>
</html>
"""


def write_onboard_report(
    summary: dict[str, Any],
    samples: dict[str, Any],
    run_dir: Path,
) -> Path:
    write_run_files(samples, run_dir, summary)
    out = run_dir / "report.html"
    out.write_text(render_onboard_html(summary, samples), encoding="utf-8")
    summary["report"] = str(out)
    write_json(run_dir / "results.json", summary)
    write_json(run_dir / "samples.json", _samples_for_json(samples))
    return out


def _samples_for_json(samples: dict[str, Any]) -> dict[str, Any]:
    """Drop bulky per-path coordinate arrays from the archived JSON."""
    slim = dict(samples)
    slim["path"] = [
        {"t_s": p["t_s"], "n": p["n"]} for p in samples.get("path") or []
    ]
    return slim
