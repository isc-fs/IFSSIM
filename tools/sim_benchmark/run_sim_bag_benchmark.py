"""Run every ground-truth benchmark on one simulator bag, as one session.

A simulator bag carries the true track (``/testing_only/track``) and the true
pose (``/testing_only/odom``), so perception and SLAM can be scored against
them. This runs ``run_perception_benchmark.py`` and ``run_slam_benchmark.py``
on the bag and puts both results in one folder:

    results/sim_bag/<bag>_<ts>/
        session.json            what ran, with which options, and how it went
        provenance.json (+ *.diff)   the code state (see run_provenance.py)
        perception/base_<ts>/   the perception benchmark's usual output (report.html, ...)
        slam/<strategy>_<ts>/   the SLAM benchmark's usual output

Each benchmark still writes its own ``report.html``. Once uploaded
(``bench-track sync``, tracking/README.md), ``bench-view`` shows the session as
one report under "Simulator bag benchmarks".

    python tools/sim_benchmark/run_sim_bag_benchmark.py results/capture/<bag>
    python tools/sim_benchmark/run_sim_bag_benchmark.py <bag> --only slam --motion-model imu
    python tools/sim_benchmark/run_sim_bag_benchmark.py <bag> --skip slam --profile
    python tools/sim_benchmark/run_sim_bag_benchmark.py <bag> --dry-run   (print the commands)

Anything a single benchmark accepts can be passed with ``--perception-args``
or ``--slam-args`` (one quoted string each).
"""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import pipeline_overrides  # noqa: E402
import run_provenance  # noqa: E402
from common import bag_topic_names, resolve_benchmark_path  # noqa: E402

# name -> (script, subfolder it writes, what it needs from the bag)
BENCHMARKS = {
    "perception": (
        "run_perception_benchmark.py",
        "perception",
        ("/testing_only/track", "/testing_only/odom"),
    ),
    "slam": (
        "run_slam_benchmark.py",
        "slam",
        ("/testing_only/track", "/testing_only/odom", "/imu"),
    ),
}
# which --pipeline-overrides section each benchmark applies (pipeline_overrides.py)
COMPONENT = {"perception": "cone_detection", "slam": "slam_node"}
# Options both benchmarks take, forwarded to each so their ground truth is gated the same way.
SHARED = ("gt_range_m", "gt_hfov_deg", "gt_min_range_m", "gt_scan_period_ms")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "bag", help="simulator bag (under tools/sim_benchmark/ when Docker is used)"
    )
    pick = ap.add_argument_group("which benchmarks (default: all)")
    pick.add_argument(
        "--only",
        nargs="+",
        choices=sorted(BENCHMARKS),
        metavar="NAME",
        help=f"run only these: {', '.join(BENCHMARKS)}",
    )
    pick.add_argument(
        "--skip",
        nargs="+",
        choices=sorted(BENCHMARKS),
        default=[],
        metavar="NAME",
        help="run everything except these",
    )

    gt = ap.add_argument_group("ground-truth gating (forwarded to every benchmark)")
    gt.add_argument("--gt-range-m", type=float)
    gt.add_argument("--gt-hfov-deg", type=float)
    gt.add_argument("--gt-min-range-m", type=float)
    gt.add_argument("--gt-scan-period-ms", type=float)

    per = ap.add_argument_group("perception")
    per.add_argument(
        "--profile",
        action="store_true",
        help="per-stage timings (perception --profile)",
    )
    per.add_argument("--max-frames", type=int, help="only the first N LiDAR frames")
    per.add_argument(
        "--perception-args", default="", help='anything else, e.g. "--no-vfov-gate"'
    )

    sl = ap.add_argument_group("SLAM")
    sl.add_argument(
        "--strategy", help="SLAM strategy (default: the benchmark's, trackdrive)"
    )
    sl.add_argument("--motion-model", choices=("odom", "imu"))
    sl.add_argument("--resynth-odom", action="store_true")
    sl.add_argument(
        "--slam-args", default="", help="anything else for run_slam_benchmark.py"
    )

    ap.add_argument(
        "--pipeline-overrides",
        metavar="FILE",
        help="JSON parameter overrides per component (pipeline_overrides.py); each benchmark "
        "gets its own section: perception cone_detection, SLAM slam_node",
    )

    run = ap.add_argument_group("running")
    run.add_argument(
        "--name", help="short label for the session folder (default: the bag name)"
    )
    run.add_argument(
        "--results-root",
        default=str(HERE / "results"),
        help="sessions go in <results-root>/sim_bag/ (default: tools/sim_benchmark/results)",
    )
    run.add_argument(
        "--no-docker", action="store_true", help="run in this (sourced ROS) shell"
    )
    run.add_argument(
        "--stop-on-error",
        action="store_true",
        help="stop at the first benchmark that fails (default: run the rest)",
    )
    run.add_argument(
        "--dry-run", action="store_true", help="print the commands, run nothing"
    )
    return ap.parse_args(argv)


def selected(args: argparse.Namespace) -> list[str]:
    names = args.only or list(BENCHMARKS)
    return [n for n in BENCHMARKS if n in names and n not in args.skip]


def overrides_file(session: Path, name: str) -> Path:
    return session / "overrides" / f"{name}.json"


def command(
    name: str,
    args: argparse.Namespace,
    bag: Path,
    session: Path,
    overrides: dict | None = None,
) -> list[str]:
    script = BENCHMARKS[name][0]
    cmd = [sys.executable, str(HERE / script), str(bag), "--results-root", str(session)]
    if (overrides or {}).get(COMPONENT[name]):
        cmd += ["--pipeline-overrides", str(overrides_file(session, name))]
    for k in SHARED:
        if getattr(args, k) is not None:
            cmd += [f"--{k.replace('_', '-')}", str(getattr(args, k))]
    if name == "perception":
        if args.profile:
            cmd.append("--profile")
        if args.max_frames:
            cmd += ["--max-frames", str(args.max_frames)]
        cmd += shlex.split(args.perception_args)
    if name == "slam":
        if args.strategy:
            cmd += ["--strategy", args.strategy]
        if args.motion_model:
            cmd += ["--motion-model", args.motion_model]
        if args.resynth_odom:
            cmd.append("--resynth-odom")
        cmd += shlex.split(args.slam_args)
    if args.no_docker:
        cmd.append("--no-docker")
    return cmd


def run_dirs(session: Path, sub: str) -> list[Path]:
    d = session / sub
    return sorted(p for p in d.iterdir() if p.is_dir()) if d.is_dir() else []


def write_session(session: Path, doc: dict) -> None:
    (session / "session.json").write_text(json.dumps(doc, indent=2, sort_keys=True))


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    bag = resolve_benchmark_path(args.bag)
    names = selected(args)
    if not names:
        print("Nothing to run: --only / --skip leave no benchmark.", file=sys.stderr)
        return 2
    if not bag.is_dir():
        print(f"Bag not found: {bag}", file=sys.stderr)
        return 2
    try:
        overrides = pipeline_overrides.load(args.pipeline_overrides)
        pipeline_overrides.only(overrides, *(COMPONENT[n] for n in names))
    except pipeline_overrides.OverrideError as e:
        print(f"--pipeline-overrides: {e}", file=sys.stderr)
        return 2
    topics = bag_topic_names(bag)
    if topics and not {"/testing_only/track", "/testing_only/odom"} <= topics:
        print(
            f"{bag.name} has no ground truth (/testing_only/track and /testing_only/odom). "
            "Record it in the simulator with capture_benchmark_bag.py; for bags from the car "
            "use run_onboard_replay.py.",
            file=sys.stderr,
        )
        return 2

    started = datetime.now(timezone.utc)
    label = args.name or bag.name.replace("_indexed", "")
    session = (
        Path(args.results_root).resolve()
        / "sim_bag"
        / f"{label}_{started:%Y%m%d_%H%M%S}"
    )
    cmds = {n: command(n, args, bag, session, overrides) for n in names}
    if args.dry_run:
        print(f"session: {session}")
        for n, c in cmds.items():
            print(f"[{n}] {shlex.join(c)}")
        return 0

    session.mkdir(parents=True, exist_ok=True)
    for n in names:
        if overrides.get(COMPONENT[n]):
            f = overrides_file(session, n)
            f.parent.mkdir(exist_ok=True)
            f.write_text(json.dumps({COMPONENT[n]: overrides[COMPONENT[n]]}, indent=2))
    # the session's own record; each benchmark also writes one
    run_provenance.record(session)
    doc = {
        "schema": 1,
        "kind": "sim_bag_session",
        "bag": str(bag),
        "started_at": started.isoformat(timespec="seconds"),
        "requested": names,
        "pipeline_overrides": overrides,
        "options": {k: v for k, v in vars(args).items() if k not in ("bag", "dry_run")},
        "benchmarks": {},
        "status": "running",
    }
    write_session(session, doc)
    print(f"Session {session}")

    failed = []
    for n in names:
        sub = BENCHMARKS[n][1]
        missing = [t for t in BENCHMARKS[n][2] if topics and t not in topics]
        before = set(run_dirs(session, sub))
        print(f"\n=== {n}: {shlex.join(cmds[n])}", flush=True)
        t0 = time.time()
        rc = subprocess.call(cmds[n], cwd=HERE.parent.parent)
        new = [p for p in run_dirs(session, sub) if p not in before]
        ok = rc == 0 and bool(new) and (new[-1] / "results.json").is_file()
        doc["benchmarks"][n] = {
            "status": "finished" if ok else "failed",
            "exit_code": rc,
            "duration_s": round(time.time() - t0, 1),
            "run_dir": new[-1].relative_to(session).as_posix() if new else None,
            "command": cmds[n],
            "missing_topics": missing,
        }
        write_session(session, doc)
        if not ok:
            failed.append(n)
            print(f"!!! {n} failed (exit {rc})", file=sys.stderr)
            if args.stop_on_error:
                break

    doc["finished_at"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    ran = [n for n in names if doc["benchmarks"].get(n, {}).get("status") == "finished"]
    doc["status"] = (
        "finished" if len(ran) == len(names) else "partial" if ran else "failed"
    )
    write_session(session, doc)

    print(f"\nSession {doc['status']}: {session}")
    for n in names:
        b = doc["benchmarks"].get(n)
        if b is None:
            print(f"  {n:<11} not run")
            continue
        rep = session / b["run_dir"] / "report.html" if b["run_dir"] else None
        print(
            f"  {n:<11} {b['status']:<9} {b['duration_s']:>7.1f} s  {rep if rep and rep.is_file() else ''}"
        )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
