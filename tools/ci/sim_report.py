#!/usr/bin/env python3
"""The sim check's result: what ran, how it went, and whether the check passed.

    python3 tools/ci/sim_report.py <results root> --out <dir> [--baseline <report.json>]

Reads the newest headless run under <results root>/headless/ and the referee's
results for each mission run in it. Writes <dir>/report.json (read by
sim_plot.py and sim_publish.py) and <dir>/summary.md (the job summary).

Only the missions tools/ci/sim_check.json lists as `required` decide the
result; `informational` ones are reported but never fail the check.

Autocross also runs on random tracks (tools/ci/gen_tracks.py, named
ci_rand_<seed>.csv): they share one row in the summary, and for every run
that didn't finish, sim_failures.py works out where it went wrong.

The car characterisation (tools/characterise), when the run had one, is read
from <run>/characterisation/: its section joins the summary and the comment,
and with `characterisation.required` in sim_check.json a failed hard check
fails the check. One that couldn't measure is shown, but fails nothing: that
is the measurement's fault, not evidence about the car.

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
sys.path.insert(0, str(HERE))
import sim_failures  # noqa: E402
import track_geometry  # noqa: E402

CONFIG = HERE / "sim_check.json"
COMMENT_MARKER = "<!-- sim-check-report -->"
RANDOM_PREFIX = "ci_rand_"
# Missions on closed loops, where "how far round the lap" means something.
LOOP_MISSIONS = ("autocross", "trackdrive")
# A corner below this radius is flagged (the car's turning circle is 4.95 m).
TIGHT_M = track_geometry.CAR_MIN_RADIUS_M
# The cone-hit table lists at most this many hits per run, and this many overall.
HITS_PER_RUN, HITS_TOTAL = 8, 24


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


def _dnf(path: Path) -> str:
    """The benchmark's reason for a run that didn't finish (events.csv)."""
    if not path.exists():
        return ""
    with path.open() as f:
        for r in csv.DictReader(f):
            if r.get("kind") == "dnf":
                return (r.get("detail") or "").strip()
    return ""


def _kv(detail: str) -> dict[str, str]:
    """events.csv's `key=value key=value` detail."""
    return dict(p.split("=", 1) for p in (detail or "").split() if "=" in p)


def _doo_events(path: Path) -> list[dict]:
    """The kind=doo rows of events.csv (one per cone the referee counted)."""
    if not path.exists():
        return []
    out = []
    with path.open() as f:
        for r in csv.DictReader(f):
            if r.get("kind") != "doo":
                continue
            kv = _kv(r.get("detail") or "")

            def num(k):
                try:
                    return float(kv[k])
                except (KeyError, ValueError):
                    return None

            try:
                t = float(r.get("t"))
            except (TypeError, ValueError):
                t = None
            out.append({"t": t, "color": kv.get("color", ""), "why": kv.get("why", ""),
                        "x": num("x"), "y": num("y"), "speed": num("car_speed")})
    return out


def random_seed(track: str) -> int | None:
    stem = Path(track).stem
    if not stem.startswith(RANDOM_PREFIX):
        return None
    try:
        return int(stem[len(RANDOM_PREFIX):])
    except ValueError:
        return None


def read_run(seed_dir: Path, run_dir: Path) -> dict:
    res = json.loads((seed_dir / "results.json").read_text())
    man_path = seed_dir / "manifest.json"
    man = json.loads(man_path.read_text()) if man_path.exists() else {}
    scen = man.get("scenario") or {}
    wall, sim = _num(res.get("race/run_time_s")), _num(res.get("race/run_sim_time_s"))
    tel = _telemetry(seed_dir / "telemetry.csv")
    steer = [abs(r["steering"]) for r in tel if "steering" in r]
    finished = res.get("race/finished") == 1
    mission = scen.get("event") or seed_dir.parent.parent.name.split("_")[0]
    track = (scen.get("track") or {}).get("csv", "")
    failure = None
    if not finished and mission in LOOP_MISSIONS:
        try:
            failure = sim_failures.locate(_telemetry(seed_dir / "cones.csv"), tel)
        except Exception as e:  # a bad file shouldn't lose the report
            print(f"{seed_dir}: couldn't locate the failure ({e})", file=sys.stderr)
    cones = [(r["x"], r["y"], int(r["color"])) for r in _telemetry(seed_dir / "cones.csv")
             if {"x", "y", "color"} <= r.keys()]
    # Corners only mean something on a lap track: the skidpad's figure eight
    # crosses itself, so its "centreline" reads as a 1 m corner.
    stats = track_geometry.corner_stats(cones if mission in LOOP_MISSIONS else [], TIGHT_M)
    hits = _doo_events(seed_dir / "events.csv")
    for h in hits:
        h["corner_m"] = (track_geometry.radius_near(cones, h["x"], h["y"])
                         if mission in LOOP_MISSIONS and h["x"] is not None and h["y"] is not None
                         else None)
    return {
        "scenario": seed_dir.parent.parent.name,
        "mission": mission,
        "track": track,
        "random_seed": random_seed(track),
        "seed": int(seed_dir.name.removeprefix("seed") or 0),
        "finished": finished,
        "outcome": man.get("outcome", "finished" if finished else "not finished"),
        "dnf": _dnf(seed_dir / "events.csv"),
        "failure": failure,
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
        # Share of the run some task on the machine waited for a CPU (Linux CPU pressure):
        # what explains a slow sim on the GPU box, where other jobs share the CPU.
        "cpu_wait": _num(res.get("box/cpu_wait")),
        "steer_max": max(steer) if steer else None,
        "corner_min_m": stats["min_radius_m"],
        "corner_tight": stats["n_tight"],
        "hits": hits,
        "dir": str(seed_dir.relative_to(run_dir)),
        "code": man.get("code") or {},
    }


def read_characterisation(run_dir: Path, rec: dict, config: dict) -> dict | None:
    """The run's car characterisation: status, its markdown, the hard checks that failed."""
    cfg = config.get("characterisation") or {}
    d = run_dir / "characterisation"
    out = {"required": bool(cfg.get("required")), "status": "error", "markdown": "", "failed": [],
           "error": None}
    try:
        res = json.loads((d / "result.json").read_text())
    except (OSError, ValueError):
        if "characterisation" not in rec:
            return None  # not asked for, or the run never got that far
        out["error"] = (rec["characterisation"] or {}).get("error") or \
            f"it ended {(rec['characterisation'] or {}).get('status', 'without a result')}"
        return out
    out["status"] = res.get("status", "error")
    out["error"] = res.get("error")
    out["failed"] = [c["name"] for c in res.get("checks", []) if c.get("hard") and c.get("status") == "fail"]
    if (d / "summary.md").exists():
        out["markdown"] = (d / "summary.md").read_text()
    return out


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
            "random": r["random_seed"] is not None,
            "required": r["mission"] not in info, "runs": 0, "finished": 0, "laps": [],
            "cones": 0, "off_track": 0, "v_max": None, "sim_speed": [], "cpu_wait": None,
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
        if r.get("cpu_wait") is not None:
            g["cpu_wait"] = max(g["cpu_wait"] or 0.0, r["cpu_wait"])
    for g in groups.values():
        g["lap_mean"] = statistics.fmean(g["laps"]) if g["laps"] else None
        g["sim_speed"] = min(g["sim_speed"]) if g["sim_speed"] else None

    # A required mission that never ran (the benchmark stopped early) fails too.
    ran = {g["mission"] for g in groups.values()}
    missing = [m for m in required if m not in ran]
    failed = [g["scenario"] for g in groups.values() if g["required"] and g["finished"] < g["runs"]]
    car = read_characterisation(run_dir, rec, config)
    car_failed = bool(car and car["required"] and car["status"] == "fail")
    if not runs:
        verdict = "error"
    elif failed or missing or car_failed:
        verdict = "fail"
    else:
        verdict = "pass"

    failures = [
        {"scenario": r["scenario"], "mission": r["mission"], "track": r["track"],
         "random_seed": r["random_seed"], "seed": r["seed"], "outcome": r["outcome"],
         "dnf": r["dnf"], **(r["failure"] or {"kind": "unknown"})}
        for r in runs if not r["finished"] and r["mission"] in LOOP_MISSIONS
    ]

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
        "failures": failures,
        "characterisation": car,
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


def display_groups(groups: list[dict]) -> list[dict]:
    """The groups as the summary shows them: the random tracks of a mission
    share one row (their laps aren't comparable, so it has no lap time)."""
    out: list[dict] = []
    merged: dict[str, dict] = {}
    for g in groups:
        if not g.get("random"):
            out.append(g)
            continue
        m = merged.get(g["mission"])
        if m is None:
            m = merged[g["mission"]] = {
                **g, "scenario": f"{g['mission']}_random", "track": "", "tracks": 0,
                "runs": 0, "finished": 0, "cones": 0, "off_track": 0, "laps": [],
                "lap_mean": None, "v_max": None, "sim_speed": None, "cpu_wait": None,
            }
            out.append(m)
        m["tracks"] += 1
        for k in ("runs", "finished", "cones", "off_track"):
            m[k] += g[k]
        if g["v_max"] is not None:
            m["v_max"] = max(m["v_max"] or 0, g["v_max"])
        if g["sim_speed"] is not None:
            m["sim_speed"] = g["sim_speed"] if m["sim_speed"] is None else min(m["sim_speed"], g["sim_speed"])
        if g.get("cpu_wait") is not None:
            m["cpu_wait"] = max(m["cpu_wait"] or 0.0, g["cpu_wait"])
    return out


def speed_cell(speed, cpu_wait) -> str:
    """Sim speed, with the machine's CPU wait when it was recorded (e.g. ``0.78× · 21 % CPU wait``)."""
    if speed is None:
        return "–"
    return f"{speed:.2f}×" + ("" if cpu_wait is None else f" · {cpu_wait * 100:.0f} % CPU wait")


def _name(g: dict) -> str:
    if g.get("random"):
        n = g.get("tracks", 1)
        name = f"{g['mission']} · {n} random track{'s' if n != 1 else ''}"
        return name if g["required"] else f"{name} *(info)*"
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


def corner_cell(r: dict) -> str:
    """Tightest centreline radius, bold with the count when it is under what the car can turn."""
    m = r.get("corner_min_m")
    if m is None:
        return "–"
    if m < TIGHT_M:
        return f"**{m:.1f} m** ({r.get('corner_tight') or 1} under {TIGHT_M:.2f})"
    return f"{m:.1f} m"


def hits_table(report: dict) -> str:
    """The cones each run knocked down: when, which side, the corner there, how fast."""
    rows = []
    for r in report["runs"]:
        hs = r.get("hits") or []
        for h in hs[:HITS_PER_RUN]:
            rows.append((r, h))
        if len(hs) > HITS_PER_RUN:
            rows.append((r, None))
    if not rows:
        return ""
    out = ["<details><summary>Cone hits</summary>\n",
           "| Run | Time | Side | Corner | Car speed | Why |", "|---|---|---|---|---|---|"]
    for r, h in rows[:HITS_TOTAL]:
        run = f"{r['scenario']} #{r['seed']}"
        if h is None:
            out.append(f"| {run} | … | | | | +{len(r['hits']) - HITS_PER_RUN} more |")
            continue
        side = {"blue": "blue (left)", "yellow": "yellow (right)"}.get(h["color"], h["color"] or "–")
        out.append("| {} | {} | {} | {} | {} | {} |".format(
            run, f1(h["t"], " s"), side,
            "–" if h.get("corner_m") is None else f"R {h['corner_m']:.1f} m",
            f1(h["speed"], " m/s"), h["why"] or "–"))
    if len(rows) > HITS_TOTAL:
        out.append(f"| … | | | | | +{len(rows) - HITS_TOTAL} more rows |")
    out.append("\n</details>\n")
    return "\n".join(out)


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
    for g in display_groups(report["groups"]):
        ok = g["finished"] == g["runs"]
        out.append("| {} | {}{}/{} | {} | {} | {} | {} | {} | {} |".format(
            _name(g),
            "" if ok else "**", g["finished"], f"{g['runs']}**" if not ok else g["runs"],
            f1(g["lap_mean"], " s"), _delta(g, baseline), g["cones"], g["off_track"],
            f1(g["v_max"], " m/s"),
            speed_cell(g["sim_speed"], g.get("cpu_wait")),
        ))
    out.append("")
    out.append("<details><summary>Every run</summary>\n")
    out.append("| Mission | Seed | Result | Lap | Cones hit | Off track | Mean / top speed | Max steering "
               "| Tightest corner | Sim speed |")
    out.append("|---|---|---|---|---|---|---|---|---|---|")
    for r in report["runs"]:
        out.append("| {} | {} | {} | {} | {} | {} | {} / {} | {} | {} | {} |".format(
            r["scenario"], r["seed"], "finished" if r["finished"] else f"**{r['outcome']}**",
            f1(r["lap_s"], " s"), r["cones"] if r["cones"] is not None else "–",
            r["off_track"] if r["off_track"] is not None else "–",
            f1(r["v_mean"]), f1(r["v_max"], " m/s"),
            "–" if r["steer_max"] is None else f"{r['steer_max']:.2f}",
            corner_cell(r),
            speed_cell(r["sim_speed"], r.get("cpu_wait")),
        ))
    out.append("\n</details>\n")
    out.append(hits_table(report))
    out.append("*Sim speed* is sim seconds per real second, the slowest run's. "
               "Below ~0.95× the car is known to drive badly, so check it before blaming the pipeline. "
               "*CPU wait* is the share of the run some task on the machine waited for a CPU, the highest "
               "run's. A slow sim with a high CPU wait means a busy machine, not a slow change.")
    return "\n".join(out)


def where(f: dict) -> str:
    """A failure's place on its track, in words."""
    if f.get("kind") == "unknown":
        return "–"
    return "{} m of {} m ({}%)".format(f"{f['progress_m']:.0f}", f"{f['lap_m']:.0f}", f["progress_pct"])


def track_cell(f: dict, track_urls: dict[str, str] | None = None) -> str:
    stem = Path(f["track"]).stem
    url = (track_urls or {}).get(stem)
    return f"[`{stem}`]({url})" if url else f"`{stem}`"


def how(f: dict) -> str:
    if f.get("kind") == "unknown":
        return f["outcome"]
    bits = [f["kind"]]
    if f.get("speed") is not None and f["kind"] == "left the track":
        bits.append(f"at {f['speed']:.1f} m/s")
    if f.get("t_s") is not None and f["kind"] != "never moved":
        bits.append(f"after {f['t_s']:.0f} s")
    if f.get("dnf"):
        bits.append(f"({f['dnf']})")
    return " ".join(bits)


def failures_table(report: dict, track_urls: dict[str, str] | None = None) -> str:
    """Where each autocross run that didn't finish went wrong."""
    fs = report.get("failures") or []
    if not fs:
        return ""
    out = ["", "#### Where it failed", "",
           "| Track | Where | Corner | What happened |", "|---|---|---|---|"]
    for f in fs:
        out.append("| {} | {} | {} | {} |".format(
            track_cell(f, track_urls), where(f), f.get("corner") or "–", how(f)))
    out.append("")
    out.append("A random track is rebuilt from its seed with `tools/ci/gen_tracks.py --seeds <seed>`, "
               "or rerun on the box with the `random_seeds` input of the workflow.")
    return "\n".join(out)


def characterisation_section(report: dict) -> str:
    """The car characterisation's table (written by tools/characterise), or why there is none."""
    car = report.get("characterisation")
    if not car:
        return ""
    if car.get("markdown"):
        text = car["markdown"].rstrip()
    else:
        text = f"#### Car characterisation · couldn't run\n\n> {car.get('error') or 'no result'}"
    if car.get("status") == "fail":
        text += ("\n\nThis fails the sim check." if car.get("required") else
                 "\n\nReported only: `characterisation.required` is off in `tools/ci/sim_check.json`.")
    return "\n" + text + "\n"


def maps(images: dict[str, str]) -> str:
    if not images:
        return ""
    cells = [f'<img src="{url}" width="420" alt="{name}">' for name, url in images.items()]
    return "\n" + "\n".join(cells) + "\n"


def comment(report: dict, baseline: dict | None, images: dict[str, str], run_url: str, base_note: str = "") -> str:
    parts = [COMMENT_MARKER, "### " + headline(report), "", tables(report, baseline),
             failures_table(report, report.get("track_urls")), characterisation_section(report), maps(images)]
    foot = f"[Run, logs and telemetry]({run_url})" if run_url else ""
    if baseline and base_note:
        foot += f" · {base_note}"
    parts.append(foot)
    return "\n".join(parts)


def summary(report: dict, baseline: dict | None) -> str:
    return "\n".join(["### " + headline(report), "", tables(report, baseline), failures_table(report),
                      characterisation_section(report),
                      "\nSim and pipeline logs are in the `sim-pr-results` artifact.\n"])


def history_cell(report: dict) -> str:
    """One line per mission for the report issue's history table."""
    cells = []
    for g in display_groups(report["groups"]):
        lap = f" {g['lap_mean']:.1f}s" if g["lap_mean"] is not None else ""
        name = g["mission"] + (" random" if g.get("random") else "")
        name = name if g["required"] else f"*{name}*"
        cells.append(f"{name} {g['finished']}/{g['runs']}{lap}")
    car = report.get("characterisation")
    if car:
        cells.append(f"car {car['status']}")
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
