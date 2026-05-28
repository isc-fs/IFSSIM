"""Aggregate stage timings from perception benchmark / profile runs."""

from __future__ import annotations

from statistics import mean, median


STAGE_KEYS = (
    "decode_ms",
    "ransac_ms",
    "rotate_ms",
    "dbscan_ms",
    "cluster_prep_ms",
    "fit_ms",
    "detect_ms",
    "total_ms",
)

# Stages inside detect_ms (for breakdown chart; excludes decode / totals).
PIPELINE_STAGE_KEYS = (
    "fit_ms",
    "dbscan_ms",
    "ransac_ms",
    "rotate_ms",
    "cluster_prep_ms",
)

STAGE_COLORS = {
    "fit_ms": "#c62828",
    "dbscan_ms": "#ef6c00",
    "ransac_ms": "#1565c0",
    "rotate_ms": "#7b1fa2",
    "cluster_prep_ms": "#546e7a",
}


def _percentile(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return 0.0
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * p
    f = int(k)
    c = min(f + 1, len(sorted_vals) - 1)
    if f == c:
        return sorted_vals[f]
    return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)


def summarize_stage_rows(rows: list[dict[str, float]]) -> dict[str, dict[str, float]]:
    """Per-stage mean / median / p95 / max over profile rows."""
    out: dict[str, dict[str, float]] = {}
    for key in STAGE_KEYS:
        vals = sorted(float(r[key]) for r in rows if key in r)
        if not vals:
            continue
        out[key] = {
            "mean_ms": mean(vals),
            "median_ms": median(vals),
            "p95_ms": _percentile(vals, 0.95),
            "max_ms": max(vals),
        }
    return out


def _svg_stage_breakdown(stages: dict[str, dict[str, float]]) -> str:
    """Horizontal stacked bar of median ms per pipeline stage."""
    parts: list[tuple[str, float, str]] = []
    for key in PIPELINE_STAGE_KEYS:
        stats = stages.get(key)
        if stats:
            parts.append((key.replace("_ms", ""), stats["median_ms"], STAGE_COLORS.get(key, "#999")))
    if not parts:
        return ""
    total = sum(v for _, v, _ in parts) or 1.0
    w = 520.0
    x = 0.0
    rects = []
    legend = []
    for label, val, color in parts:
        frac = val / total
        bw = max(2.0, frac * w)
        rects.append(
            f"<rect x='{x:.1f}' y='28' width='{bw:.1f}' height='36' fill='{color}' "
            f"opacity='0.9'><title>{label}: {val:.1f} ms median</title></rect>"
        )
        pct = 100.0 * val / total
        legend.append(
            f"<span style='margin-right:12px'><span style='color:{color}'>■</span> "
            f"{label} {val:.1f} ms ({pct:.0f}%)</span>"
        )
        x += bw
    return (
        "<div class='chart'><h3>Median time per scan (pipeline stages)</h3>"
        f"<svg viewBox='0 0 {int(w)} 80' xmlns='http://www.w3.org/2000/svg'>"
        f"<rect width='{int(w)}' height='80' fill='#fafafa'/>"
        + "".join(rects)
        + f"<text x='0' y='16' font-size='11' fill='#666'>"
        f"Sum of medians ≈ {total:.0f} ms (excludes PointCloud2 decode)</text>"
        + "</svg>"
        f"<p class='legend'>{''.join(legend)}</p></div>"
    )


def render_profile_html(profile: dict) -> str:
    """HTML block for perception report (stage breakdown + optional RANSAC ablation)."""
    stages = profile.get("stages") or {}
    if not stages:
        return ""

    chart = _svg_stage_breakdown(stages)
    fit_med = (stages.get("fit_ms") or {}).get("median_ms", 0.0)
    db_med = (stages.get("dbscan_ms") or {}).get("median_ms", 0.0)
    tips = ""
    if fit_med >= db_med:
        tips = f"""
<div class='warn' style='margin-top:12px'>
<strong>Fit is the bottleneck</strong> (median {fit_med:.0f} ms vs DBSCAN {db_med:.0f} ms).
Production now uses: (1) closed-form collinear fit for line-like clusters,
(2) single-template L-BFGS-B when cluster height &lt; {profile.get('early_exit_small_m', 0.40):.2f} m
or &gt; {profile.get('early_exit_big_m', 0.50):.2f} m,
(3) capped scipy iterations. Re-run this benchmark to measure the delta.
</div>
"""

    rows_html = []
    for key, stats in stages.items():
        label = key.replace("_ms", "").replace("_", " ")
        rows_html.append(
            f"<tr><td>{label}</td>"
            f"<td>{stats['mean_ms']:.2f}</td>"
            f"<td>{stats['median_ms']:.2f}</td>"
            f"<td>{stats['p95_ms']:.2f}</td>"
            f"<td>{stats['max_ms']:.2f}</td></tr>"
        )

    counts = profile.get("point_counts") or {}
    count_bits = " · ".join(f"{k}={v:.0f}" for k, v in counts.items())

    ablation = profile.get("ransac_ablation")
    ablation_html = ""
    if ablation:
        sub = ablation.get("with_subsample_5000") or {}
        full = ablation.get("without_subsample") or {}
        speedup = ablation.get("median_speedup_x") or 0.0
        ablation_html = f"""
<h3>RANSAC ablation (median over {ablation.get('frames', 0)} frames)</h3>
<table style="border-collapse:collapse;width:100%;max-width:640px">
  <tr><th>Mode</th><th>median ms</th><th>mean ms</th><th>p95 ms</th></tr>
  <tr><td>Production (5k iter subsample)</td>
      <td>{sub.get('median_ms', 0):.2f}</td>
      <td>{sub.get('mean_ms', 0):.2f}</td>
      <td>{sub.get('p95_ms', 0):.2f}</td></tr>
  <tr><td>No iter subsample (full cloud scoring)</td>
      <td>{full.get('median_ms', 0):.2f}</td>
      <td>{full.get('mean_ms', 0):.2f}</td>
      <td>{full.get('p95_ms', 0):.2f}</td></tr>
</table>
<p>Median speedup from subsampling: <strong>{speedup:.1f}×</strong>
 (verified against live <code>ransac2(iter_subsample_max=5000)</code>).</p>
"""

    warmup_note = ""
    n_ex = profile.get("profile_frames_excluding_warmup")
    if n_ex is not None and n_ex != profile.get("profile_frames"):
        warmup_note = f" Table stats exclude frame 1 (JIT warmup); {n_ex} frames."

    return f"""
<h2>Pipeline profile</h2>
<p class="subtitle">Stage timings from <code>--profile</code> ({profile.get('profile_frames', 0)} frames).{warmup_note}
 {count_bits}</p>
{chart}
{tips}
<table style="border-collapse:collapse;width:100%;max-width:720px">
  <tr><th>Stage</th><th>mean ms</th><th>median ms</th><th>p95 ms</th><th>max ms</th></tr>
  {''.join(rows_html)}
</table>
{ablation_html}
<p class='subtitle'>Raw numbers: <code>profile.json</code> and <code>profile_stages.csv</code> in this run folder.</p>
"""


def write_profile_stages_csv(path, profile: dict) -> None:
    """One row per stage for spreadsheets / Lichtblick."""
    import csv
    from pathlib import Path

    stages = profile.get("stages") or {}
    rows = []
    for key in STAGE_KEYS:
        stats = stages.get(key)
        if not stats:
            continue
        rows.append(
            {
                "stage": key.replace("_ms", ""),
                "mean_ms": stats["mean_ms"],
                "median_ms": stats["median_ms"],
                "p95_ms": stats["p95_ms"],
                "max_ms": stats["max_ms"],
            }
        )
    p = Path(path)
    if not rows:
        p.write_text("")
        return
    with p.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def summarize_point_counts(rows: list[dict[str, float]]) -> dict[str, float]:
    n_pts = [float(r["n_points"]) for r in rows if "n_points" in r]
    n_out = [float(r["n_outliers"]) for r in rows if "n_outliers" in r]
    n_cl = [float(r["n_clusters"]) for r in rows if "n_clusters" in r]
    summary: dict[str, float] = {}
    if n_pts:
        summary["mean_n_points"] = mean(n_pts)
        summary["median_n_points"] = median(n_pts)
    if n_out:
        summary["mean_n_outliers"] = mean(n_out)
    if n_cl:
        summary["mean_n_clusters"] = mean(n_cl)
    return summary
