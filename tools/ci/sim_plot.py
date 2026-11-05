#!/usr/bin/env python3
"""A map per mission of the sim check: the car's path in every run over the
track's cones, with its speed underneath.

    python3 tools/ci/sim_plot.py <report dir> --results <results root>

Reads <report dir>/report.json (sim_report.py) and each run's telemetry.csv
and cones.csv (both ENU metres, so they plot together as they are). Writes
<report dir>/maps/<mission>.png. Needs matplotlib: the sim check runs it in
the pipeline image (tools/ci/sim_pr_check.sh).
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# The referee's cone colours (FSDSReferee.h): yellow, blue, big orange, small orange, unknown.
CONE = {0: ("#e3b505", 18), 1: ("#2563c9", 18), 2: ("#f07f13", 34), 3: ("#f07f13", 18), 4: ("#9aa0a6", 14)}
# One colour per run, kept clear of the cone colours.
RUNS = ["#7b2cbf", "#0f9d76", "#d6336c", "#5c677d", "#8f5b1e"]
INK, MUTED, GRID = "#1f2328", "#59636e", "#e6e8eb"
FAIL = "#c92a2a"


def read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    out = []
    with path.open() as f:
        for r in csv.DictReader(f):
            try:
                out.append({k: float(v) for k, v in r.items() if v not in ("", None)})
            except ValueError:
                pass
    return out


def gates(cones: list[dict]) -> list[dict]:
    """The timing gates: big orange cones paired across the track (each with its
    nearest big orange), and pairs whose middles are close merged into one gate."""
    big = [c for c in cones if int(c.get("color", -1)) == 2]

    def d2(a, b):
        return (a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2

    pairs = set()
    for i, a in enumerate(big):
        others = [j for j in range(len(big)) if j != i]
        if not others:
            continue
        j = min(others, key=lambda j: d2(a, big[j]))
        if min((k for k in range(len(big)) if k != j), key=lambda k: d2(big[j], big[k])) == i:
            pairs.add(tuple(sorted((i, j))))
    out: list[dict] = []
    for i, j in sorted(pairs):
        a, b = big[i], big[j]
        mid = ((a["x"] + b["x"]) / 2, (a["y"] + b["y"]) / 2)
        g = next((g for g in out if (g["x"] - mid[0]) ** 2 + (g["y"] - mid[1]) ** 2 < 10 ** 2), None)
        if g is None:
            g = {"x": mid[0], "y": mid[1], "lines": []}
            out.append(g)
        g["lines"].append((a, b))
        # The gate's middle: the mean of its pairs' middles.
        n = len(g["lines"])
        g["x"] = sum((p["x"] + q["x"]) / 2 for p, q in g["lines"]) / n
        g["y"] = sum((p["y"] + q["y"]) / 2 for p, q in g["lines"]) / n
    return out


def nearest(gs: list[dict], p: dict | None) -> dict | None:
    if not gs or p is None:
        return None
    return min(gs, key=lambda g: (g["x"] - p["x"]) ** 2 + (g["y"] - p["y"]) ** 2)


def plot_group(group: dict, runs: list[dict], run_dir: Path, out: Path) -> None:
    cones = read_csv(run_dir / runs[0]["dir"] / "cones.csv")
    paths = [(r, read_csv(run_dir / r["dir"] / "telemetry.csv")) for r in runs]
    xs = [c["x"] for c in cones] + [p["x"] for _, t in paths for p in t if "x" in p]
    ys = [c["y"] for c in cones] + [p["y"] for _, t in paths for p in t if "y" in p]
    if not xs:
        raise ValueError("no telemetry or cones recorded")
    # Long, thin tracks (acceleration) lie along the page: rotate them 90 degrees.
    turn = (max(ys) - min(ys)) > 1.5 * (max(xs) - min(xs))

    def xy(x, y):
        return (y, -x) if turn else (x, y)

    w, h = (max(ys) - min(ys), max(xs) - min(xs)) if turn else (max(xs) - min(xs), max(ys) - min(ys))
    map_h = min(4.5, max(1.4, 7.4 * (h + 6) / (w + 6)))  # the map's height for its shape, in inches
    fig, (ax, sp) = plt.subplots(
        2, 1, figsize=(8, map_h + 2.6), dpi=110, gridspec_kw={"height_ratios": [map_h, 2.0]}
    )
    fig.patch.set_facecolor("white")
    for c in cones:
        color, size = CONE.get(int(c.get("color", 4)), CONE[4])
        px, py = xy(c["x"], c["y"])
        ax.scatter(px, py, s=size, c=color, marker="^", linewidths=0, zorder=2)
    # Start and finish: the gate nearest where the runs start, and the one nearest
    # where finished runs stop (the same gate when the track is a loop).
    gs = gates(cones)
    firsts = [t[0] for _, t in paths if t and "x" in t[0]]
    ends = [t[-1] for r, t in paths if t and r["finished"]]
    start_g = nearest(gs, firsts[0] if firsts else None)
    finish_g = nearest(gs, ends[0] if ends else None)
    if finish_g is None and len(gs) == 1:
        finish_g = start_g  # one gate: a loop, it's both
    elif finish_g is None and start_g is not None:
        finish_g = max(gs, key=lambda g: (g["x"] - start_g["x"]) ** 2 + (g["y"] - start_g["y"]) ** 2)
    for g in gs:
        for a, b in g["lines"]:
            ax.plot(*zip(xy(a["x"], a["y"]), xy(b["x"], b["y"])), color=INK, lw=1.6, zorder=2)
    for g, text in ((start_g, "START"), (finish_g, "FINISH")):
        if g is None or (text == "FINISH" and g is start_g):
            continue
        if text == "START" and g is finish_g:
            text = "START / FINISH"
        ax.annotate(text, xy(g["x"], g["y"]), xytext=(0, 14), textcoords="offset points",
                    ha="center", fontsize=8, fontweight="bold", color=INK, zorder=6,
                    bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": GRID, "lw": 0.8})
    for i, (r, tel) in enumerate(paths):
        pts = [xy(p["x"], p["y"]) for p in tel if "x" in p and "y" in p]
        if not pts:
            continue
        color = RUNS[i % len(RUNS)]
        label = f"run {r['seed']}: " + (f"{r['lap_s']:.1f} s" if r["finished"] and r["lap_s"] else r["outcome"])
        ax.plot(*zip(*pts), color=color, lw=2, solid_capstyle="round", zorder=3,
                ls="-" if r["finished"] else "--", label=label)
        ax.scatter(*pts[-1], s=40, c=color, marker="o" if r["finished"] else "X",
                   edgecolors="white", linewidths=1.5, zorder=4)
        # Where the run started, and which way it set off.
        ax.scatter(*pts[0], s=46, facecolors="white", edgecolors=color, linewidths=2, zorder=5)
        ahead = next((q for q in pts[1:] if (q[0] - pts[0][0]) ** 2 + (q[1] - pts[0][1]) ** 2 > 2.0 ** 2), None)
        if ahead is not None:
            ax.annotate("", xy=ahead, xytext=pts[0], zorder=5,
                        arrowprops={"arrowstyle": "-|>", "color": color, "lw": 1.6, "mutation_scale": 12})
        # Where it went wrong (sim_failures.py), for a run that didn't finish.
        f = r.get("failure")
        if f and "x" in f:
            fx, fy = xy(f["x"], f["y"])
            ax.scatter(fx, fy, s=150, facecolors="none", edgecolors=FAIL, linewidths=2.2, zorder=6)
            ax.annotate(f"{f['kind']} · {f['progress_m']:.0f} m", (fx, fy), xytext=(10, -14),
                        textcoords="offset points", fontsize=8, fontweight="bold", color=FAIL, zorder=6,
                        bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": FAIL, "lw": 0.8})
        ts = [p["t"] - tel[0]["t"] for p in tel if "speed" in p]
        vs = [p["speed"] for p in tel if "speed" in p]
        sp.plot(ts, vs, color=color, lw=2, label=label)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    done = f"{group['finished']}/{group['runs']} finished"
    lap = f" · mean lap {group['lap_mean']:.1f} s" if group.get("lap_mean") else ""
    track = Path(group["track"]).stem
    name = group["mission"] if track in ("", group["mission"]) else f"{group['mission']} · {track}"
    ax.set_title(f"{name}\n{done}{lap} · {group['cones']} cones hit · {group['off_track']} off track",
                 loc="left", fontsize=11, color=INK)
    if len(paths) > 1:
        sp.legend(loc="upper center", bbox_to_anchor=(0.5, -0.32), ncol=min(len(paths), 5),
                  fontsize=8, frameon=False, labelcolor=MUTED)
    sp.set_xlabel("time since start (s)", fontsize=9, color=MUTED)
    sp.set_ylabel("speed (m/s)", fontsize=9, color=MUTED)
    sp.grid(color=GRID, lw=0.8)
    sp.tick_params(colors=MUTED, labelsize=8)
    for s in ("top", "right"):
        sp.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        sp.spines[s].set_color(GRID)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, facecolor="white")
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("report_dir", type=Path)
    ap.add_argument("--results", type=Path, required=True, help="the results root the report was built from")
    args = ap.parse_args(argv)
    report = json.loads((args.report_dir / "report.json").read_text())
    run_dir = args.results / report["run_dir"]
    n = 0
    for g in report["groups"]:
        runs = [r for r in report["runs"] if r["scenario"] == g["scenario"]]
        try:
            plot_group(g, runs, run_dir, args.report_dir / "maps" / f"{g['scenario']}.png")
            n += 1
        except Exception as e:  # one bad map shouldn't lose the others
            print(f"{g['scenario']}: no map ({e})", file=sys.stderr)
    print(f"{n} maps in {args.report_dir / 'maps'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
