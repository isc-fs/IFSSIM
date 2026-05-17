"""SLAM trajectory comparison tool.

Compares two `replay_slam.py` residual CSVs (typically: today's
cone_graph_slam vs the in-progress rewrite) on a single matplotlib
figure. Three panels:

  1. Top-down trajectory — SLAM_old, SLAM_new, GT.
  2. Pose error vs time — both algorithms.
  3. Yaw error vs time — both algorithms.

The residual CSVs are produced by:

    python3 pipeline/cone_slam/scripts/replay_slam.py <bag> \\
        --csv /tmp/old_residuals.csv --quiet

Then this script:

    python3 tools/dev/slam_compare.py \\
        --old /tmp/old_residuals.csv \\
        --new /tmp/new_residuals.csv \\
        --out /tmp/slam_compare.png

The single-CSV mode (one algorithm only) is also supported — pass
--old without --new and only the new lines are omitted.

Implementation note: deliberately ROS-free + lightweight. Runs on
the host with just matplotlib; no need to dive into the
dv_pipeline_stack container to render the plot.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Optional


def _read_csv(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open() as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append({k: float(v) for k, v in r.items()})
    return rows


def _extract(rows: list[dict[str, float]]) -> dict[str, list[float]]:
    """Slice the CSV columns the plot consumes. Re-anchors t to start
    at zero so the time axis is human-readable (raw stamps are epoch
    seconds in the ~1e9 range)."""
    if not rows:
        return {"t": [], "slam_x": [], "slam_y": [], "gt_x": [], "gt_y": [],
                "err_m": [], "yaw_err_deg": []}
    t0 = rows[0]["t"]
    return {
        "t":           [r["t"] - t0 for r in rows],
        "slam_x":      [r["slam_x"] for r in rows],
        "slam_y":      [r["slam_y"] for r in rows],
        "gt_x":        [r["gt_x"] for r in rows],
        "gt_y":        [r["gt_y"] for r in rows],
        "err_m":       [r["err_m"] for r in rows],
        "yaw_err_deg": [math.degrees(r["yaw_err_rad"]) for r in rows],
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Overlay SLAM trajectories from one or two replay_slam.py "
                    "residual CSVs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--old", required=True,
                    help="Residual CSV for the 'old' algorithm "
                         "(typically current cone_graph_slam).")
    ap.add_argument("--new", default=None,
                    help="Residual CSV for the 'new' algorithm "
                         "(the in-progress rewrite). Optional — when "
                         "omitted the figure shows the 'old' run alone.")
    ap.add_argument("--out", default="/tmp/slam_compare.png",
                    help="Output PNG path. Default /tmp/slam_compare.png.")
    ap.add_argument("--show", action="store_true",
                    help="Open the figure in matplotlib's interactive "
                         "window after saving.")
    ap.add_argument("--title", default=None,
                    help="Figure title override. Default uses the CSV stems.")
    args = ap.parse_args()

    old_path = Path(args.old)
    if not old_path.is_file():
        print(f"--old not found: {old_path}", file=sys.stderr)
        return 2
    old = _extract(_read_csv(old_path))

    new: Optional[dict[str, list[float]]] = None
    if args.new:
        new_path = Path(args.new)
        if not new_path.is_file():
            print(f"--new not found: {new_path}", file=sys.stderr)
            return 2
        new = _extract(_read_csv(new_path))

    # Deferred import keeps the --help / file-presence checks cheap.
    import matplotlib
    matplotlib.use("Agg" if not args.show else "")
    import matplotlib.pyplot as plt

    fig, (ax_xy, ax_err, ax_yaw) = plt.subplots(
        3, 1, figsize=(10, 12), constrained_layout=True,
        gridspec_kw={"height_ratios": [3, 1, 1]},
    )

    # ----- Top-down trajectory -----
    # Use the OLD CSV's GT as the canonical GT line. Both CSVs come
    # from the same bag (the user runs replay_slam.py twice on the
    # same bag, one per algorithm), so GT is identical.
    ax_xy.plot(old["gt_x"], old["gt_y"], color="black",
               label="GT", linewidth=2.0)
    ax_xy.plot(old["slam_x"], old["slam_y"], color="tab:red",
               label="old", linewidth=1.3, linestyle="--")
    if new:
        ax_xy.plot(new["slam_x"], new["slam_y"], color="tab:green",
                   label="new", linewidth=1.3)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set_xlabel("x [m]"); ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Top-down trajectory (anchored at calibration end)")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best")

    # ----- Pose error -----
    ax_err.plot(old["t"], old["err_m"], color="tab:red",
                label="old", linewidth=1.0, linestyle="--")
    if new:
        ax_err.plot(new["t"], new["err_m"], color="tab:green",
                    label="new", linewidth=1.0)
    ax_err.set_xlabel("t [s]"); ax_err.set_ylabel("pose err [m]")
    ax_err.set_title("Pose error vs time")
    ax_err.axhline(1.5, color="gray", linestyle=":",
                   label="post-rewrite gate (1.5 m)")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="best", fontsize=8)

    # ----- Yaw error -----
    ax_yaw.plot(old["t"], old["yaw_err_deg"], color="tab:red",
                label="old", linewidth=1.0, linestyle="--")
    if new:
        ax_yaw.plot(new["t"], new["yaw_err_deg"], color="tab:green",
                    label="new", linewidth=1.0)
    ax_yaw.set_xlabel("t [s]"); ax_yaw.set_ylabel("yaw err [deg]")
    ax_yaw.set_title("Yaw error vs time")
    ax_yaw.axhline(5.0, color="gray", linestyle=":",
                   label="post-rewrite gate (5°)")
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.legend(loc="best", fontsize=8)

    # Title.
    if args.title:
        fig.suptitle(args.title)
    elif new:
        fig.suptitle(f"{old_path.stem} (old, dashed)  vs  {new_path.stem} (new, solid)")
    else:
        fig.suptitle(f"{old_path.stem}")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    print(f"==> wrote {out_path}")

    # ----- Compact text summary -----
    def _summary(label: str, data: dict[str, list[float]]) -> None:
        if not data["err_m"]:
            print(f"  {label}: no samples")
            return
        em = data["err_m"]
        yd = [abs(v) for v in data["yaw_err_deg"]]
        print(f"  {label}: pose mean={sum(em)/len(em):.2f}m "
              f"max={max(em):.2f}m  |  yaw mean={sum(yd)/len(yd):.2f}° "
              f"max={max(yd):.2f}°  |  N={len(em)}")
    print("Summary:")
    _summary("old", old)
    if new:
        _summary("new", new)

    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
