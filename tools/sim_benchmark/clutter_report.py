#!/usr/bin/env python3
"""The P2a reproduction's verdict: the clutter runs against the control.

Reads the mission benchmark's runs of specs/clutter-*.yaml
(results/mission/autocross_clutter_<kind>_e<seed>/<commit>/seed<k>/) and, per
kind, counts the outcomes. A run is a premature stop when the pipeline declared
the mission finished before the referee saw the finish (outcome stopped_early,
run_mission_benchmark.py): the stop latch firing on false big oranges, the
failure the roadmap wants reproduced before the 2027 track tests. In autocross the
pipeline need not declare the mission finished when the latch fires early: the car
brakes to the latch's anchor and waits there until the benchmark's timeout. So a run
that didn't finish and whose car stood still for 10 s after moving 10 m (telemetry.csv,
standstill()) counts as stopped early too; benchmark_outcome keeps the benchmark's word.

    python3 tools/sim_benchmark/clutter_report.py [--results DIR] [--commit SHA] [--json]

Roadmap acceptance (docs/ENVIRONMENT_ROADMAP.md, P2a): premature stops in at
least 3 of 25 clutter runs and in 0 of 25 control runs. The per-scan false
big-orange counts need recorded bags (run_perception_benchmark.py); this report
covers the closed-loop half. The one-sided Fisher exact p says how unlikely the
clutter kind's stop count is if props made no difference.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
DIR_RE = re.compile(r"^autocross_clutter_(?P<kind>[a-z]+)_e(?P<env>\d+)$")
MIN_STOPS, CONTROL = 3, "control"
STALL_MOVED_M, STALL_S, STALL_V = 10.0, 10.0, 0.2   # standstill(): moved this far, then this long under this speed


def load(results: Path, commit: str | None = None) -> dict[str, list[dict]]:
    """{kind: [run]}; run = {env, seed, commit, outcome, doo, time_s}. The newest commit per track unless given."""
    runs: dict[str, list[dict]] = defaultdict(list)
    # results/mission/<track>, or the same under headless_run's results/**/headless/<stamp>/mission
    track_dirs = sorted(p for p in results.rglob("autocross_clutter_*") if p.is_dir() and p.parent.name == "mission")
    for track_dir in track_dirs:
        m = DIR_RE.match(track_dir.name)
        if not m:
            continue
        commits = [d for d in track_dir.iterdir() if d.is_dir()]
        if commit:
            commits = [d for d in commits if d.name.startswith(commit)]
        if not commits:
            continue
        newest = max(commits, key=lambda d: max((f.stat().st_mtime for f in d.rglob("manifest.json")), default=0))
        for seed_dir in sorted(newest.glob("seed*")):
            try:
                man = json.loads((seed_dir / "manifest.json").read_text())
                res = json.loads((seed_dir / "results.json").read_text())
            except (OSError, ValueError):
                continue
            outcome = man.get("outcome", "unknown")
            stall = standstill(seed_dir / "telemetry.csv") if outcome not in ("finished", "stopped_early") else None
            runs[m["kind"]].append({
                "env": int(m["env"]),
                "seed": seed_dir.name,
                "commit": newest.name,
                # a car parked short of the finish is a premature stop whatever ended the run
                "outcome": "stopped_early" if stall else outcome,
                "benchmark_outcome": outcome,
                "stall": stall,
                "doo": int(res.get("race/n_doo") or 0),
                "time_s": res.get("race/run_sim_time_s", res.get("race/run_time_s")),
            })
    return runs


def standstill(telemetry: Path, moved_m: float = STALL_MOVED_M, still_s: float = STALL_S,
               v_still: float = STALL_V) -> dict | None:
    """The first stop of at least still_s at under v_still m/s after the car moved moved_m, or None.

    In autocross the pipeline need not declare the mission finished when its stop latch fires
    early (the car brakes to the latch's anchor and waits there), so the benchmark scores the run
    a timeout. The referee never saw the finish, so a car parked this long short of it stopped
    early. telemetry.csv: t,x,y,speed,... (run_mission_benchmark.py)."""
    try:
        rows = [ln.split(",") for ln in telemetry.read_text().splitlines()[1:] if ln.strip()]
        pts = [(float(r[0]), float(r[1]), float(r[2]), float(r[3])) for r in rows]
    except (OSError, ValueError, IndexError):
        return None
    travelled, since, prev = 0.0, None, None
    for t, x, y, v in pts:
        if prev:
            travelled += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
        if v < v_still and travelled >= moved_m:
            since = since or (t, travelled, x, y)
            if t - since[0] >= still_s:
                return {"t_s": round(since[0], 1), "travelled_m": round(since[1], 1),
                        "x": round(since[2], 2), "y": round(since[3], 2)}
        else:
            since = None
    return None


def fisher_greater(a: int, n1: int, c: int, n2: int) -> float:
    """One-sided Fisher exact p: P(at least a of n1 stops | the same stop rate in both groups)."""
    k, n = a + c, n1 + n2
    if k == 0:
        return 1.0
    total = math.comb(n, k)
    return sum(math.comb(n1, i) * math.comb(n2, k - i) for i in range(a, min(k, n1) + 1)) / total


def summarise(runs: dict[str, list[dict]]) -> dict:
    ctrl = runs.get(CONTROL, [])
    c_stops = sum(r["outcome"] == "stopped_early" for r in ctrl)
    out = {"kinds": {}, "control_runs": len(ctrl), "control_stops": c_stops}
    for kind in sorted(runs, key=lambda k: (k != CONTROL, k)):
        rs = runs[kind]
        outcomes = Counter(r["outcome"] for r in rs)
        stops = outcomes.get("stopped_early", 0)
        row = {
            "runs": len(rs),
            "outcomes": dict(outcomes),
            "stopped_early": stops,
            "doo_mean": round(sum(r["doo"] for r in rs) / len(rs), 2) if rs else None,
            "stops_per_env_seed": dict(sorted(Counter(r["env"] for r in rs if r["outcome"] == "stopped_early").items())),
        }
        if kind != CONTROL:
            row["p_vs_control"] = round(fisher_greater(stops, len(rs), c_stops, len(ctrl)), 4) if ctrl else None
            row["reproduced"] = bool(ctrl) and stops >= MIN_STOPS and c_stops == 0
        out["kinds"][kind] = row
    return out


def markdown(s: dict) -> str:
    lines = ["| Kind | Runs | Finished | Stopped early | Emergency | Timeout | DOO / run | p vs control | P2a |",
             "|---|---|---|---|---|---|---|---|---|"]
    for kind, r in s["kinds"].items():
        o = r["outcomes"]
        verdict = "control" if kind == CONTROL else ("**reproduced**" if r.get("reproduced") else "not reproduced")
        p = r.get("p_vs_control")
        lines.append(f"| {kind} | {r['runs']} | {o.get('finished', 0)} | {r['stopped_early']} | {o.get('emergency', 0)} | "
                     f"{o.get('timeout', 0)} | {r['doo_mean']} | {'' if p is None else p} | {verdict} |")
    lines.append("")
    lines.append(f"P2a: at least {MIN_STOPS} early stops in a clutter kind's runs and none in the control's "
                 f"(control: {s['control_stops']} of {s['control_runs']}).")
    if not s["control_runs"]:
        lines.append("No control runs yet: run specs/clutter-control.yaml before reading the others.")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default=str(HERE / "results"), help="the benchmark's results root")
    ap.add_argument("--commit", help="only runs of this IFSSIM commit (prefix); default the newest per track")
    ap.add_argument("--json", action="store_true", help="print the summary as JSON")
    a = ap.parse_args(argv)
    runs = load(Path(a.results), a.commit)
    if not runs:
        print(f"no clutter runs under {a.results}/mission (run specs/clutter-*.yaml)", file=sys.stderr)
        return 2
    s = summarise(runs)
    print(json.dumps(s, indent=2) if a.json else markdown(s))
    return 0


if __name__ == "__main__":
    sys.exit(main())
