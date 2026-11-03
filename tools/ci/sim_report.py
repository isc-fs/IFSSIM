#!/usr/bin/env python3
"""The sim check's result: what ran, how it went, and whether the check passed.

    python3 tools/ci/sim_report.py <results root> --out <dir> [--baseline <report.json>]

Reads the newest headless run under <results root>/headless/ and the referee's
results for each mission run in it. Writes <dir>/report.json (read by
sim_plot.py and sim_publish.py) and <dir>/summary.md (the job summary).

Only the missions tools/ci/sim_check.json lists as `required` decide the
result; `informational` ones are reported but never fail the check.

Exit code: 0 every required run finished, 1 some didn't, 3 nothing ran
(the machine failed: see the job log).

The markdown for the PR comment and the report issue is built here too, so
the job summary, the comment and the issue always agree.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIG = HERE / "sim_check.json"
COMMENT_MARKER = "<!-- sim-check-report -->"


# ------------------------------------------------------------------ reading a run
def _num(v):
    return v if isinstance(v, (int, float)) else None


def _telemetry(path: Path) -> list[dict]:
    if not path.exists():
        return []
    rows = []
    with path.open() as f:
        for r in csv.DictReader(f):
            try:
                rows.append({k: float(v) for k, v in r.items() if v not in ("", None)})
            except ValueError:
                continue
    return rows


def read_run(seed_dir: Path, run_dir: Path) -> dict:
    res = json.loads((seed_dir / "results.json").read_text())
    man_path = seed_dir / "manifest.json"
    man = json.loads(man_path.read_text()) if man_path.exists() else {}
    scen = man.get("scenario") or {}
    wall, sim = _num(res.get("race/run_time_s")), _num(res.get("race/run_sim_time_s"))
    tel = _telemetry(seed_dir / "telemetry.csv")
    steer = [abs(r["steering"]) for r in tel if "steering" in r]
    return {
        "scenario": seed_dir.parent.parent.name,
        "mission": scen.get("event") or seed_dir.parent.parent.name.split("_")[0],
        "track": (scen.get("track") or {}).get("csv", ""),
        "seed": int(seed_dir.name.removeprefix("seed") or 0),
        "finished": res.get("race/finished") == 1,
        "outcome": man.get("outcome", "finished" if res.get("race/finished") == 1 else "not finished"),
        "lap_s": _num(res.get("race/lap_time_best_s")),
        "laps": _num(res.get("race/n_laps")),
        "cones": _num(res.get("race/n_doo")),
        "off_track": _num(res.get("race/n_off_track")),
        "penalty_s": _num(res.get("race/penalty_s")),
        "v_mean": _num(res.get("control/speed_mean_mps")),
        "v_max": _num(res.get("control/speed_max_mps")),
        # Sim seconds per real second: below ~0.95 the car is known to drive badly
        # (the pipeline is real-time sensitive), so a failure there may be the box.
        "sim_speed": sim / wall if sim and wall else None,
        "steer_max": max(steer) if steer else None,
        "dir": str(seed_dir.relative_to(run_dir)),
        "code": man.get("code") or {},
    }


def build(root: Path, config: dict) -> dict:
    runs_json = sorted((root / "headless").glob("*/run.json"))
    if not runs_json:
        return {"verdict": "error", "outcome": "no_run", "error": "no headless run recorded", "runs": [], "groups": []}
    rec = json.loads(runs_json[-1].read_text())
    run_dir = runs_json[-1].parent
    runs = [read_run(p.parent, run_dir) for p in sorted(run_dir.glob("mission/*/*/seed*/results.json"))]

    required, info = config.get("required", []), config.get("informational", [])
    groups: dict[str, dict] = {}
    for r in runs:
        g = groups.setdefault(r["scenario"], {
            "scenario": r["scenario"], "mission": r["mission"], "track": r["track"],
            "required": r["mission"] not in info, "runs": 0, "finished": 0, "laps": [],
            "cones": 0, "off_track": 0, "v_max": None, "sim_speed": [],
        })
        g["runs"] += 1
        g["finished"] += r["finished"]
        if r["finished"] and r["lap_s"] is not None:
            g["laps"].append(r["lap_s"])
        g["cones"] += r["cones"] or 0
        g["off_track"] += r["off_track"] or 0
        if r["v_max"] is not None:
            g["v_max"] = max(g["v_max"] or 0, r["v_max"])
        if r["sim_speed"] is not None:
            g["sim_speed"].append(r["sim_speed"])
    for g in groups.values():
        g["lap_mean"] = statistics.fmean(g["laps"]) if g["laps"] else None
        g["sim_speed"] = min(g["sim_speed"]) if g["sim_speed"] else None

    # A required mission that never ran (the benchmark stopped early) fails too.
    ran = {g["mission"] for g in groups.values()}
    missing = [m for m in required if m not in ran]
    failed = [g["scenario"] for g in groups.values() if g["required"] and g["finished"] < g["runs"]]
    if not runs:
        verdict = "error"
    elif failed or missing:
        verdict = "fail"
    else:
        verdict = "pass"

    code = runs[0]["code"] if runs else {}
    return {
        "verdict": verdict,
        "outcome": rec.get("outcome"),
        "error": rec.get("error"),
        "duration_s": rec.get("duration_s"),
        "missing": missing,
        "failed": failed,
        "ifssim": (code.get("ifssim") or {}).get("sha", ""),
        "pipeline": (code.get("pipeline") or {}).get("sha", ""),
        "run_dir": str(run_dir.relative_to(root)),
        "runs": [{k: v for k, v in r.items() if k != "code"} for r in runs],
        "groups": list(groups.values()),
    }


# ------------------------------------------------------------------ markdown
def f1(v, unit: str = "") -> str:
    return "–" if v is None else f"{v:.1f}{unit}"


def _delta(g: dict, baseline: dict | None) -> str:
    if not baseline:
        return "–"
    b = next((x for x in baseline.get("groups", []) if x["scenario"] == g["scenario"]), None)
    if not b or b.get("lap_mean") is None:
        return "–"
    if g["lap_mean"] is None:
        return f"dev {b['lap_mean']:.1f} s"
    d = g["lap_mean"] - b["lap_mean"]
    return f"{d:+.1f} s (dev {b['lap_mean']:.1f})"


def _name(g: dict) -> str:
    track = Path(g["track"]).stem
    name = g["mission"] if track in ("", g["mission"]) else f"{g['mission']} · {track}"
    return name if g["required"] else f"{name} *(info)*"


HEADLINE = {
    "pass": "passed",
    "fail": "failed",
    "error": "couldn't run",
}


def headline(report: dict) -> str:
    v = report["verdict"]
    parts = [f"Sim check {HEADLINE.get(v, v)}"]
    if report.get("ifssim"):
        parts.append(f"IFSSIM `{report['ifssim']}`")
    if report.get("pipeline"):
        parts.append(f"pipeline `{report['pipeline']}`")
    if report.get("duration_s"):
        parts.append(f"{report['duration_s'] / 60:.0f} min")
    return " · ".join(parts)


def tables(report: dict, baseline: dict | None) -> str:
    out = []
    if report.get("error"):
        out.append(f"> {report['error']}\n")
    if report.get("missing"):
        out.append(f"> Never ran: {', '.join(report['missing'])} (see `benchmark.log` in the artifact).\n")
    if not report["groups"]:
        out.append("No mission ran. See `benchmark.log` in the `sim-pr-results` artifact.")
        return "\n".join(out)
    out += [
        "| Mission | Finished | Lap (mean) | vs dev | Cones hit | Off track | Top speed | Sim speed |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for g in report["groups"]:
        ok = g["finished"] == g["runs"]
        out.append("| {} | {}{}/{} | {} | {} | {} | {} | {} | {} |".format(
            _name(g),
            "" if ok else "**", g["finished"], f"{g['runs']}**" if not ok else g["runs"],
            f1(g["lap_mean"], " s"), _delta(g, baseline), g["cones"], g["off_track"],
            f1(g["v_max"], " m/s"),
            "–" if g["sim_speed"] is None else f"{g['sim_speed']:.2f}×",
        ))
    out.append("")
    out.append("<details><summary>Every run</summary>\n")
    out.append("| Mission | Seed | Result | Lap | Cones hit | Off track | Mean / top speed | Max steering | Sim speed |")
    out.append("|---|---|---|---|---|---|---|---|---|")
    for r in report["runs"]:
        out.append("| {} | {} | {} | {} | {} | {} | {} / {} | {} | {} |".format(
            r["scenario"], r["seed"], "finished" if r["finished"] else f"**{r['outcome']}**",
            f1(r["lap_s"], " s"), r["cones"] if r["cones"] is not None else "–",
            r["off_track"] if r["off_track"] is not None else "–",
            f1(r["v_mean"]), f1(r["v_max"], " m/s"),
            "–" if r["steer_max"] is None else f"{r['steer_max']:.2f}",
            "–" if r["sim_speed"] is None else f"{r['sim_speed']:.2f}×",
        ))
    out.append("\n</details>\n")
    out.append("*Sim speed* is sim seconds per real second, the slowest run's. "
               "Below ~0.95× the car is known to drive badly, so check it before blaming the pipeline.")
    return "\n".join(out)


def maps(images: dict[str, str]) -> str:
    if not images:
        return ""
    cells = [f'<img src="{url}" width="420" alt="{name}">' for name, url in images.items()]
    return "\n" + "\n".join(cells) + "\n"


def comment(report: dict, baseline: dict | None, images: dict[str, str], run_url: str, base_note: str = "") -> str:
    parts = [COMMENT_MARKER, "### " + headline(report), "", tables(report, baseline), maps(images)]
    foot = f"[Run, logs and telemetry]({run_url})" if run_url else ""
    if baseline and base_note:
        foot += f" · {base_note}"
    parts.append(foot)
    return "\n".join(parts)


def summary(report: dict, baseline: dict | None) -> str:
    return "\n".join(["### " + headline(report), "", tables(report, baseline),
                      "\nSim and pipeline logs are in the `sim-pr-results` artifact.\n"])


def history_cell(report: dict) -> str:
    """One line per mission for the report issue's history table."""
    cells = []
    for g in report["groups"]:
        lap = f" {g['lap_mean']:.1f}s" if g["lap_mean"] is not None else ""
        name = g["mission"] if g["required"] else f"*{g['mission']}*"
        cells.append(f"{name} {g['finished']}/{g['runs']}{lap}")
    return "<br>".join(cells) or "–"


# ------------------------------------------------------------------ main
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path, help="the results root (<root>/headless/...)")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--baseline", type=Path, help="the last dev run's report.json, to compare with")
    ap.add_argument("--config", type=Path, default=CONFIG)
    args = ap.parse_args(argv)

    config = json.loads(args.config.read_text())
    report = build(args.root, config)
    baseline = None
    if args.baseline and args.baseline.exists():
        baseline = json.loads(args.baseline.read_text())
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "report.json").write_text(json.dumps(report, indent=2))
    (args.out / "summary.md").write_text(summary(report, baseline))
    print(summary(report, baseline))
    return {"pass": 0, "fail": 1}.get(report["verdict"], 3)


if __name__ == "__main__":
    sys.exit(main())
