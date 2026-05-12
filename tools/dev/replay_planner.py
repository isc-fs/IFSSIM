#!/usr/bin/env python3
"""Offline replay of a captured Plan_Path tick.

When the in-loop DV_PLANNER_CAPTURE env var is set on `Plan_Path`, every
cone callback dumps a JSON line with `t_ns`, `pose=[x,y,yaw]`,
`cones=[[x,y,color_int], ...]`, `n_path`, `path=[[x,y], ...]`.

This tool reads that file, picks a tick, replays it through the same
`FasttubeAdapter` the live node uses, and renders an inspection plot:

  - Cones colored by SLAM class (yellow / blue / orange-small / orange-big)
  - Car pose with a heading arrow
  - FaSTTUBe's per-side `with_virtual` chains (the library's settled
    matching, virtual cones included) — blue line for left, yellow for
    right, hollow markers for virtual cones
  - Final centerline (purple)
  - The path the live system actually published (dashed grey, from the
    capture file) for cross-checking

Usage (inside the dv_pipeline_stack container, where fsd_path_planning
is installed):

    docker compose exec dv_pipeline_stack \
        python3 /workspace/tools/dev/replay_planner.py \
        /tmp/planner.jsonl --worst --out /tmp/replay.png

To capture a session: set DV_PLANNER_CAPTURE on the Plan_Path node, drive
to the failure, then replay the resulting JSONL with --worst (the tick
where n_path is smallest non-zero), or --tick N for a specific frame.

    tools/dev/replay_planner.py /tmp/planner.jsonl                # last tick
    tools/dev/replay_planner.py /tmp/planner.jsonl --tick 412
    tools/dev/replay_planner.py /tmp/planner.jsonl --worst
    tools/dev/replay_planner.py /tmp/planner.jsonl --out plot.png

Exit code 0 = ok. Failures print to stderr.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path as _PathlibPath
from typing import List, Tuple

import numpy as np

# Make the path_planning package importable when running from the repo root.
_REPO = _PathlibPath(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / "pipeline" / "path_planning"))

# Defer the adapter import — it transitively requires `fsd_path_planning`,
# which is only installed inside the dv_pipeline_stack container. Letting
# `--help` and JSON loading work without it makes the tool friendlier
# when invoked outside the container by mistake.


# All cones rendered with the same neutral colour — the pipeline
# carries no per-cone colour signal anymore.
_CONE_COLOUR = "#f0d000"


def _load_ticks(path: str) -> List[dict]:
    out = []
    with open(path, "r") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError as ex:
                print(f"warn: skipping malformed line: {ex}", file=sys.stderr)
    return out


def _pick_tick(ticks: List[dict], spec: str) -> Tuple[int, dict]:
    if spec == "last":
        return len(ticks) - 1, ticks[-1]
    if spec == "worst":
        # Smallest non-zero n_path (planner gave up or shrank). Tie-break
        # on highest cone count — denser scene where shrink is most
        # suspicious.
        cands = [(t.get("n_path", 0), -len(t.get("cones", [])), i)
                 for i, t in enumerate(ticks)]
        cands_nonzero = [c for c in cands if c[0] > 0]
        if not cands_nonzero:
            print("warn: every captured tick had n_path=0; picking last",
                  file=sys.stderr)
            return len(ticks) - 1, ticks[-1]
        cands_nonzero.sort()
        i = cands_nonzero[0][2]
        return i, ticks[i]
    # Numeric index
    i = int(spec)
    if i < 0:
        i += len(ticks)
    if not (0 <= i < len(ticks)):
        raise IndexError(f"tick index {spec} out of range [0, {len(ticks)})")
    return i, ticks[i]


def _render(tick_idx: int, tick: dict, out_path: str | None) -> None:
    import matplotlib.pyplot as plt
    # Deferred — requires fsd_path_planning inside the pipeline container.
    from path_planning.core_types import Cone, Pose2D
    from path_planning.fasttube_adapter import FasttubeAdapter

    pose = Pose2D(x=tick["pose"][0], y=tick["pose"][1], yaw=tick["pose"][2])
    # Capture format: cones is a list of [x, y] pairs (or older [x, y, color]
    # which we ignore the third field on for backward-compat with old captures).
    cones = [Cone(x=c[0], y=c[1]) for c in tick["cones"]]

    adapter = FasttubeAdapter()
    points, debug = adapter.plan(cones, pose)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect("equal")
    ax.set_title(
        f"tick #{tick_idx}  pose=({pose.x:.2f}, {pose.y:.2f}, "
        f"{np.degrees(pose.yaw):+.1f}°)  "
        f"n_cones={len(cones)}  n_path={len(points)} (live={tick.get('n_path')})"
    )

    # Cones — single neutral colour.
    if cones:
        xs = [c.x for c in cones]
        ys = [c.y for c in cones]
        ax.scatter(xs, ys, c=_CONE_COLOUR,
                   edgecolors="black", linewidth=0.5,
                   s=80, zorder=3, label=f"cones (n={len(cones)})")

    # FaSTTUBe-settled per-side chains (with virtual cones).
    if debug.left_with_virtual.size:
        xs = debug.left_with_virtual[:, 0]
        ys = debug.left_with_virtual[:, 1]
        ax.plot(xs, ys, color="#3070f0", lw=2, alpha=0.7, zorder=2,
                label="left chain (with virtual)")
        ax.scatter(xs, ys, facecolors="none", edgecolors="#3070f0",
                   s=140, lw=1.2, zorder=4)
    if debug.right_with_virtual.size:
        xs = debug.right_with_virtual[:, 0]
        ys = debug.right_with_virtual[:, 1]
        ax.plot(xs, ys, color="#f0c000", lw=2, alpha=0.7, zorder=2,
                label="right chain (with virtual)")
        ax.scatter(xs, ys, facecolors="none", edgecolors="#f0c000",
                   s=140, lw=1.2, zorder=4)

    # Replayed centerline.
    if points:
        xs = [p.x for p in points]
        ys = [p.y for p in points]
        ax.plot(xs, ys, color="#9020e0", lw=3, zorder=5,
                label="replayed centerline")

    # Live-published centerline from capture (sanity check that replay matches live).
    live_path = tick.get("path", [])
    if live_path:
        xs = [p[0] for p in live_path]
        ys = [p[1] for p in live_path]
        ax.plot(xs, ys, color="#666666", lw=2, ls="--", zorder=5,
                label="live published centerline")

    # Pose + heading arrow.
    ax.scatter([pose.x], [pose.y], marker="*", s=300,
               c="#222", zorder=6, label="car pose")
    arrow_len = 1.5
    ax.arrow(pose.x, pose.y,
             arrow_len * np.cos(pose.yaw),
             arrow_len * np.sin(pose.yaw),
             head_width=0.3, head_length=0.3, fc="#222", ec="#222",
             zorder=6)

    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    if out_path:
        fig.savefig(out_path, dpi=120, bbox_inches="tight")
        print(f"wrote {out_path}")
    else:
        plt.show()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture_file")
    ap.add_argument("--tick", default="last",
                    help="tick index, 'last', or 'worst' (smallest non-zero "
                         "n_path). Default: last.")
    ap.add_argument("--out", default=None,
                    help="save figure to this PNG instead of showing it")
    args = ap.parse_args()

    ticks = _load_ticks(args.capture_file)
    if not ticks:
        print(f"no ticks in {args.capture_file}", file=sys.stderr)
        return 2
    print(f"loaded {len(ticks)} ticks from {args.capture_file}")

    idx, tick = _pick_tick(ticks, args.tick)
    print(f"replaying tick #{idx}: n_cones={len(tick.get('cones', []))} "
          f"n_path_live={tick.get('n_path')}")
    _render(idx, tick, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
