#!/usr/bin/env python3
"""Visual sanity check of the perception pipeline on one LiDAR scan per bag.

Complements the numeric benchmarks with something you can look at: for each
bag, the first PointCloud2 (or ``--scan-index``) goes through the production
``RealtimeConeDetector.detect`` and one HTML page shows, per bag:

1. Ground removal — the cropped scan, coloured ground / above ground / vetoed.
2. Clustering    — the above-ground points coloured by DBSCAN cluster, with
                   each cluster's fate (accepted, or which gate dropped it)
                   on hover.
3. Cone fit      — one fitted cluster at a time (dropdown): its points with
                   BOTH template fits (small and big cone) in 3D, the one the
                   pipeline chose highlighted, plus a z-vs-radius profile where a
                   good fit puts the points on its model line.

Sim bags: compare against one recorded at the datasheet LiDAR rate
(``PointsPerSecond`` 1740000, ~95k points per scan); the 300k default gives
~8k points per scan and is not representative of the real Hesai.

Nothing is re-implemented: ``perception_sanity.capture`` wraps the stage functions
``detect`` calls to record their inputs and outputs, so the plots show exactly what
production computed. Sim bag benchmarks record the same capture
(``perception_sanity.json``), which IFS-DV-BENCHWEB shows in the perception section;
this script is for any bag, including real ones. Everything is drawn in the frame the fit runs in: sensor xy origin,
rotated so the RANSAC ground plane is horizontal and shifted so it is z = 0.

Example (from the repo root; re-execs in ifssim-dv_pipeline_stack when the host
has no ROS)::

    python tools/sim_benchmark/perception_sanity_plots.py \\
        --bag ~/bags/<sim_bag> --bag tools/sim_benchmark/results/capture/<real_bag>

Output: ``tools/sim_benchmark/results/sanity/<timestamp>/perception_sanity.html``
(needs internet to load plotly.js from the CDN).
"""

from __future__ import annotations

import argparse
import html
import json
import os
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from common import _in_ros_env, dv_pipeline_ros_setup_shell, repo_root, results_dir
from perception_sanity import StageCapture
from perception_sanity import capture as run_instrumented
from perception_sanity import pipeline_path as _pipeline_path
from perception_sanity import rows_not_in as _rows_not_in
from perception_sanity import template_fits as _template_fits
from perception_sanity import to_ground_frame as _to_ground_frame

PLOTLY_JS = "https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.0/plotly.min.js"

# Preferred LiDAR topics: sim bridge, then the Hesai driver on the car.
LIDAR_TOPICS = ("/lidar/Lidar1", "/lidar_points")

CLUSTER_COLORS = (
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#bcbd22", "#17becf", "#393b79",
)
NOISE_COLOR = "#9a9a9a"


# ---------------------------------------------------------------------------
# Docker re-exec (bags may live anywhere, so mount each one explicitly)
# ---------------------------------------------------------------------------


def _reexec_in_docker(args: argparse.Namespace) -> None:
    bench = Path(__file__).resolve().parent
    cone_detection_src = repo_root() / "pipeline" / "cone_detection"
    out_dir = Path(args.out).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    image = os.environ.get("IFSSIM_DV_IMAGE", "ifssim-dv_pipeline_stack:latest")

    mounts = [
        "-v", f"{bench}:/bench:ro",
        "-v", f"{cone_detection_src.resolve()}:/dev_cone_detection:ro",
        "-v", f"{out_dir}:/out",
    ]
    inner_args = ["--out", "/out", "--scan-index", str(args.scan_index)]
    inner_args += ["--max-fit-clusters", str(args.max_fit_clusters)]
    for i, bag in enumerate(args.bag):
        host = Path(bag).expanduser().resolve()
        if not host.is_dir():
            raise SystemExit(f"bag directory not found: {host}")
        mounts += ["-v", f"{host}:/bags/{i}/{host.name}:ro"]
        inner_args += ["--bag", f"/bags/{i}/{host.name}"]
    for topic in args.topic or []:
        inner_args += ["--topic", topic]

    inner = (
        "set -eo pipefail; "
        "export IFSSIM_BENCHMARK_IN_DOCKER=1; "
        f"{dv_pipeline_ros_setup_shell()}"
        "export PYTHONPATH=/dev_cone_detection:${PYTHONPATH}; "
        "cd /bench && python3 perception_sanity_plots.py "
        + " ".join(shlex.quote(a) for a in inner_args)
    )
    cmd = [
        "docker", "run", "--rm", "--entrypoint", "bash",
        "--user", f"{os.getuid()}:{os.getgid()}",
        "-e", "NUMBA_CACHE_DIR=/tmp/numba_cache",
        *mounts, image, "-lc", inner,
    ]
    print("Host lacks ROS; running in Docker:", image)
    raise SystemExit(subprocess.call(cmd))


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------


def read_scan(bag: str, topic_override: str | None, scan_index: int):
    """Return ``(xyz, topic, stamp_ns)`` for the ``scan_index``-th cloud."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    from cone_detection.cone_detection_node import ConeDetectionNode
    from common import bag_storage_id

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag, storage_id=bag_storage_id(bag)),
        ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    clouds = [n for n, t in types.items() if t == "sensor_msgs/msg/PointCloud2"]
    if topic_override:
        topic = topic_override
    else:
        topic = next((t for t in LIDAR_TOPICS if t in clouds), clouds[0] if clouds else None)
    if topic is None or topic not in types:
        raise SystemExit(f"{bag}: no PointCloud2 topic (have {sorted(types)})")

    reader.set_filter(StorageFilter(topics=[topic]))
    msg_cls = get_message(types[topic])
    seen = 0
    while reader.has_next():
        _, raw, t_ns = reader.read_next()
        if seen == scan_index:
            msg = deserialize_message(raw, msg_cls)
            # Same decoder as the live node (handles the packed 26-byte Hesai layout).
            return ConeDetectionNode.pointcloud2_to_xyz(msg), topic, t_ns
        seen += 1
    raise SystemExit(f"{bag}: only {seen} scans on {topic}, asked for index {scan_index}")


# ---------------------------------------------------------------------------
# Plotly figure JSON
# ---------------------------------------------------------------------------


def _xyz(pts, decimals: int = 3) -> dict:
    import numpy as np

    pts = np.round(np.asarray(pts, dtype=np.float64), decimals)
    return {"x": pts[:, 0].tolist(), "y": pts[:, 1].tolist(), "z": pts[:, 2].tolist()}


def _scatter3d(pts, name: str, color: str, size: float = 1.5, **extra) -> dict:
    return {
        "type": "scatter3d", "mode": "markers", "name": name, **_xyz(pts),
        "marker": {"size": size, "color": color}, **extra,
    }


def _scene(title_z: str = "z [m] above fitted ground") -> dict:
    return {
        "aspectmode": "data",
        "xaxis": {"title": "x [m]"}, "yaxis": {"title": "y [m]"}, "zaxis": {"title": title_z},
    }


def fig_ground(cap: StageCapture) -> dict:
    ground = _to_ground_frame(cap, cap.cropped[cap.ground_mask])
    above = cap.clustered
    traces = [
        _scatter3d(ground, f"ground — RANSAC inliers ({len(ground)})", "#8c8c8c", 1.2),
        _scatter3d(above, f"above ground → DBSCAN ({len(above)})", "#d62728", 2.0),
    ]
    if len(cap.vetoed):
        traces.append(_scatter3d(cap.vetoed, f"above ground, tall-column veto ({len(cap.vetoed)})", "#ff9f1c", 1.8))
    if len(cap.guard_dropped):
        traces.append(_scatter3d(cap.guard_dropped, f"above ground, DBSCAN guard drop ({len(cap.guard_dropped)})", "#6a3d9a", 1.8))
    return {
        "data": traces,
        "layout": {
            "title": {"text": f"1 · Ground removal — fitted plane moved to z = 0 (sensor {-cap.floor_z:.2f} m above it)"},
            "scene": _scene(), "legend": {"itemsizing": "constant"},
            "margin": {"l": 0, "r": 0, "t": 40, "b": 0}, "height": 700,
        },
    }


def fig_clusters(cap: StageCapture) -> dict:
    traces = []
    cone_xy = []
    for i, info in enumerate(cap.clusters):
        noise = info.label == -1
        color = NOISE_COLOR if noise else CLUSTER_COLORS[i % len(CLUSTER_COLORS)]
        name = "DBSCAN noise (label -1)" if noise else f"cluster {info.label}"
        hover = f"{name}<br>{len(info.points)} pts<br>{html.escape(info.fate)}"
        traces.append(_scatter3d(
            info.points, name, color, 1.8 if noise else 2.5,
            text=hover, hoverinfo="text", showlegend=False,
            legendgroup="noise" if noise else "clusters",
        ))
        if info.accepted:
            cone_xy.append((info.fit["a"], info.fit["b"], info.label))
    if cone_xy:
        traces.append({
            "type": "scatter3d", "mode": "markers", "name": f"accepted cones ({len(cone_xy)})",
            "x": [c[0] for c in cone_xy], "y": [c[1] for c in cone_xy],
            "z": [0.0] * len(cone_xy),
            "text": [f"cone from cluster {c[2]}" for c in cone_xy], "hoverinfo": "text",
            "marker": {"size": 5, "symbol": "diamond", "color": "#000000"},
        })
    n_clusters = sum(1 for c in cap.clusters if c.label != -1)
    return {
        "data": traces,
        "layout": {
            "title": {"text": (
                f"2 · Clustering — {n_clusters} DBSCAN clusters, {len(cap.cones)} accepted "
                "(hover a cluster for its fate; ◆ = accepted cone at ground level)"
            )},
            "scene": _scene(), "margin": {"l": 0, "r": 0, "t": 40, "b": 0}, "height": 700,
        },
    }


def _cone_surface(a, b, c, d, z0, n_theta: int = 40, n_z: int = 12) -> dict:
    import numpy as np

    theta = np.linspace(0.0, 2.0 * np.pi, n_theta)
    z = np.linspace(min(z0, d), d, n_z)
    r = np.clip((d - z) / c, 0.0, None)
    tt, zz = np.meshgrid(theta, z)
    rr = np.repeat(r[:, None], n_theta, axis=1)
    return {
        "x": np.round(a + rr * np.cos(tt), 4).tolist(),
        "y": np.round(b + rr * np.sin(tt), 4).tolist(),
        "z": np.round(zz, 4).tolist(),
    }


TEMPLATE_COLORS = {"small": "#1f77b4", "big": "#ff7f0e", "pipeline": "#2ca02c"}


def fig_cone_fit(cap: StageCapture, max_clusters: int) -> dict | None:
    import numpy as np

    fitted = [c for c in cap.clusters if c.fit is not None]
    if not fitted:
        return None
    # Accepted cones first, nearest first — the ones SLAM actually consumes.
    fitted.sort(key=lambda c: (not c.accepted, c.fit["range"]))
    fitted = fitted[:max_clusters]

    traces, buttons, spans = [], [], []
    for k, info in enumerate(fitted):
        f = info.fit
        used = f["clean"]
        culled = _rows_not_in(info.points, used)
        vis = k == 0
        chosen = "big" if f["is_big"] else "small"

        # Both templates, refit for display. The pipeline's own (a, b) is the
        # template fit it chose unless it took the collinear branch, in which
        # case that fit is drawn as a third model.
        models = _template_fits(used)
        for m in models:
            m["chosen"] = m["kind"] == chosen and not np.isnan(f["res_other"])
            m["label"] = f"{m['kind']} template (c={m['c']:.1f}, d={m['d']:.2f})"
        if np.isnan(f["res_other"]):
            models.append({
                "kind": "pipeline", "a": f["a"], "b": f["b"], "c": f["c"], "d": f["d"], "mse": f["res_min"],
                "chosen": True, "label": f"pipeline collinear fit, {chosen} (c={f['c']:.1f}, d={f['d']:.2f})",
            })

        group = [
            _scatter3d(used, "points used by the fit", "#222222", 3.5, visible=vis),
            _scatter3d(culled, "points culled (below floor margin)", "#b0b0b0", 3.0, visible=vis),
        ]
        r_max = 0.0
        for m in models:
            color = TEMPLATE_COLORS[m["kind"]]
            rmse = 1000.0 * float(np.sqrt(m["mse"])) if np.isfinite(m["mse"]) else float("inf")
            name = f"{m['label']}: RMSE {rmse:.0f} mm{' ← pipeline' if m['chosen'] else ''}"
            r_pts = np.hypot(used[:, 0] - m["a"], used[:, 1] - m["b"])
            r_max = max(r_max, float(r_pts.max(initial=0.0)), m["d"] / m["c"])
            group += [
                {"type": "surface", "name": name, **_cone_surface(m["a"], m["b"], m["c"], m["d"], 0.0),
                 "opacity": 0.4 if m["chosen"] else 0.18, "showscale": False,
                 "colorscale": [[0, color], [1, color]], "hoverinfo": "name", "visible": vis, "showlegend": True},
                _scatter3d([[m["a"], m["b"], m["d"]]], f"{m['kind']} apex", color, 5, visible=vis, showlegend=False),
                {"type": "scatter", "mode": "markers", "name": f"points vs {m['kind']} axis",
                 "x": np.round(r_pts, 4).tolist(), "y": np.round(used[:, 2], 4).tolist(),
                 "marker": {"color": color, "size": 6, "opacity": 0.8},
                 "xaxis": "x", "yaxis": "y", "visible": vis, "showlegend": False},
            ]
        r_max *= 1.1
        for m in models:
            group.append({
                "type": "scatter", "mode": "lines", "name": f"{m['kind']}: z = {m['d']:.2f} − {m['c']:.1f}·r",
                "x": [0.0, r_max], "y": [m["d"], m["d"] - m["c"] * r_max],
                "line": {"color": TEMPLATE_COLORS[m["kind"]], "width": 3 if m["chosen"] else 1.5,
                         "dash": "solid" if m["chosen"] else "dot"},
                "xaxis": "x", "yaxis": "y", "visible": vis, "showlegend": False,
            })
        group.append({
            "type": "scatter", "mode": "lines", "name": "ground plane",
            "x": [0.0, r_max], "y": [0.0, 0.0], "line": {"color": "#8c8c8c", "dash": "dash"},
            "xaxis": "x", "yaxis": "y", "visible": vis, "showlegend": False,
        })
        spans.append((len(traces), len(traces) + len(group)))
        traces += group

        fits_txt = ", ".join(
            f"{m['kind']} {1000.0 * float(np.sqrt(m['mse'])):.0f} mm" for m in models if m["kind"] != "pipeline"
        )
        title = (
            f"3 · Cone fit — cluster {info.label} at {f['range']:.1f} m — {info.fate}<br>"
            f"pipeline: {_pipeline_path(f)} → {chosen}, RMSE {1000.0 * float(np.sqrt(f['res_min'])):.0f} mm "
            f"· template RMSE: {fits_txt}"
        )
        buttons.append({"label": f"cluster {info.label} ({f['range']:.1f} m{', ✓' if info.accepted else ''})",
                        "method": "update", "args": [None, {"title.text": title}]})
    n = len(traces)
    for btn, (lo, hi) in zip(buttons, spans):
        btn["args"][0] = {"visible": [lo <= i < hi for i in range(n)]}

    first_title = buttons[0]["args"][1]["title.text"]
    return {
        "data": traces,
        "layout": {
            "title": {"text": first_title, "font": {"size": 13}},
            "scene": {**_scene(), "domain": {"x": [0.0, 0.55], "y": [0.0, 1.0]}},
            "xaxis": {"domain": [0.63, 1.0], "title": "r = distance from each fit's axis (a, b) [m]"},
            "yaxis": {"title": "z [m] above fitted ground", "anchor": "x"},
            "updatemenus": [{"buttons": buttons, "x": 0.0, "y": 1.16, "xanchor": "left", "showactive": True}],
            "legend": {"x": 0.0, "y": -0.02, "orientation": "h"},
            "margin": {"l": 0, "r": 10, "t": 110, "b": 30}, "height": 680,
        },
    }


# ---------------------------------------------------------------------------
# HTML page
# ---------------------------------------------------------------------------


def _bag_section(idx: int, bag: str, topic: str, stamp_ns: int, cap: StageCapture, figs: list) -> str:
    n_ground = int(cap.ground_mask.sum()) if cap.ground_mask is not None else 0
    n_crop = len(cap.cropped) if cap.cropped is not None else 0
    stats = (
        f"topic <code>{html.escape(topic)}</code> · stamp {stamp_ns} ns · "
        f"{cap.n_raw} raw points ({cap.n_nonfinite} non-finite) · {n_crop} inside the range crop · "
        f"{n_ground} ground · {len(cap.clustered) if cap.clustered is not None else 0} clustered · "
        f"{len(cap.cones)} cones accepted"
    )
    divs = []
    for j, fig in enumerate(figs):
        div_id = f"b{idx}f{j}"
        if fig is None:
            divs.append(f'<p class="empty">No cluster reached the cone fit on this scan.</p>')
            continue
        divs.append(
            f'<div id="{div_id}" class="fig"></div>\n'
            f"<script>Plotly.newPlot('{div_id}', {json.dumps(fig['data'])}, "
            f"{json.dumps(fig['layout'])}, {{responsive: true}});</script>"
        )
    return (
        f"<section><h2>{html.escape(Path(bag).name)}</h2><p class=\"stats\">{stats}</p>\n"
        + "\n".join(divs) + "</section>"
    )


def write_page(out: Path, sections: list[str]) -> Path:
    page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Perception Sanity Plots</title>
<script src="{PLOTLY_JS}"></script>
<style>
  body {{ font-family: system-ui, sans-serif; margin: 16px; background: #fff; color: #222; }}
  h1 {{ font-size: 1.4rem; }} h2 {{ font-size: 1.15rem; margin-top: 2.5rem; border-bottom: 1px solid #ddd; }}
  .stats, .note {{ color: #555; font-size: 0.9rem; }} .fig {{ margin: 12px 0 28px; }}
  code {{ background: #f2f2f2; padding: 0 3px; }}
</style></head><body>
<h1>Perception sanity plots</h1>
<p class="note">One scan per bag through the production <code>RealtimeConeDetector.detect</code>.
Frame: the one clustering and the cone fit run in: sensor xy origin, rotated so the RANSAC ground
plane is horizontal and shifted so that plane is z = 0 (z = height above ground).
Drag to rotate, scroll to zoom, click legend entries to hide layers.</p>
{"".join(sections)}
</body></html>
"""
    path = out / "perception_sanity.html"
    path.write_text(page, encoding="utf-8")
    return path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--bag", action="append", required=True, help="rosbag2 directory; repeat for several bags")
    ap.add_argument("--topic", action="append", help="PointCloud2 topic per --bag (default: auto)")
    ap.add_argument("--scan-index", type=int, default=0, help="which scan of the topic (default: first)")
    ap.add_argument("--max-fit-clusters", type=int, default=12, help="clusters offered in the cone-fit dropdown")
    ap.add_argument(
        "--out",
        default=str(results_dir() / "sanity" / datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")),
    )
    ap.add_argument("--no-docker", action="store_true", help="run on the host (needs a sourced ROS shell)")
    args = ap.parse_args()
    if args.topic and len(args.topic) != len(args.bag):
        ap.error("pass --topic once per --bag, or not at all")

    if os.environ.get("IFSSIM_BENCHMARK_IN_DOCKER") != "1" and not args.no_docker and not _in_ros_env():
        _reexec_in_docker(args)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    sections = []
    for i, bag in enumerate(args.bag):
        topic = args.topic[i] if args.topic else None
        xyz, topic, stamp_ns = read_scan(bag, topic, args.scan_index)
        cap = run_instrumented(xyz)
        if cap.cropped is None:
            print(f"{bag}: scan empty after the range crop, skipped")
            continue
        figs = [fig_ground(cap), fig_clusters(cap), fig_cone_fit(cap, args.max_fit_clusters)]
        sections.append(_bag_section(i, bag, topic, stamp_ns, cap, figs))
        print(
            f"{Path(bag).name}: {len(cap.cropped)} pts in crop, {int(cap.ground_mask.sum())} ground, "
            f"{sum(1 for c in cap.clusters if c.label != -1)} clusters, {len(cap.fits)} fitted, "
            f"{len(cap.cones)} accepted"
        )
    path = write_page(out, sections)
    host_hint = os.environ.get("IFSSIM_BENCHMARK_IN_DOCKER") == "1"
    print(f"wrote {'<--out>/' + path.name if host_hint else path}")


if __name__ == "__main__":
    main()
