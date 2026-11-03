#!/usr/bin/env python3
"""Markdown summary of a sim PR check, for the job summary.

    python3 tools/ci/sim_pr_summary.py <results root>

Reads the newest headless run under <root>/headless/ and the referee's
results for each mission run in it.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def fmt(value, unit: str = "") -> str:
    return "–" if value is None else f"{value:.1f}{unit}"


def main(root: Path) -> int:
    runs = sorted((root / "headless").glob("*/run.json"))
    if not runs:
        print("### Sim check: no run recorded\n\nSee the job log.")
        return 0
    rec = json.loads(runs[-1].read_text())
    run_dir = runs[-1].parent
    print(f"### Sim check: {rec.get('outcome', 'unknown')}\n")
    if rec.get("error"):
        print(f"> {rec['error']}\n")

    rows = []
    for res in sorted(run_dir.glob("mission/*/*/seed*/results.json")):
        m = json.loads(res.read_text())
        seed = res.parent
        rows.append(
            "| {} | {} | {} | {} | {} | {} |".format(
                seed.parent.parent.name,
                seed.name.removeprefix("seed"),
                "finished" if m.get("race/finished") == 1 else "**not finished**",
                fmt(m.get("race/lap_time_best_s"), " s"),
                m.get("race/n_doo", "–"),
                m.get("race/n_off_track", "–"),
            )
        )
    if rows:
        print("| Mission · track | Seed | Result | Lap | Cones hit | Off track |")
        print("|---|---|---|---|---|---|")
        print("\n".join(rows))
    else:
        print("No mission ran. See `benchmark.log` in the artifact.")
    print("\nSim and pipeline logs are in the `sim-pr-results` artifact.")
    return 0


if __name__ == "__main__":
    sys.exit(main(Path(sys.argv[1])))
