"""Run the simulator headless: one check, one run, or runs on repeat.

    cd tools
    # Does the sim start offscreen on this machine? (starts it, asks its status, stops it)
    python3 -m headless_run check --sim ../Saved/StagedBuilds/Linux/IFSSIM.sh

    # One benchmark run; everything after -- goes to run_mission_benchmark.py
    python3 -m headless_run run --sim ../Saved/StagedBuilds/Linux/IFSSIM.sh --mc-up \\
        -- --missions acceleration,skidpad --repeats 2

    # Continuous: a fresh sim per cycle, until Ctrl-C (or --cycles N)
    python3 -m headless_run loop --sim ... --mc-up -- --missions all --start-noise

Exit codes (run): 0 every mission finished, 1 some didn't, 3 the machine
failed (sim didn't start or crashed, backend down), 130 interrupted.
See docs/HEADLESS.md.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path
from typing import List

from .orchestrate import RunConfig, TOOLS, exit_code, run_loop, run_once
from .simproc import SimError, SimProcess, resolve_command, rpc

DEFAULT_RESULTS = TOOLS / "sim_benchmark" / "results"


def _split(argv: List[str]):
    if "--" in argv:
        i = argv.index("--")
        return argv[:i], argv[i + 1:]
    return argv, []


def _parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="python3 -m headless_run", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def sim_args(p):
        p.add_argument("--sim", required=True,
                       help="packaged build (IFSSIM.sh, IFSSIM.app, IFSSIM.exe), the UnrealEditor "
                            "binary (with --project), or 'attach' for a sim that's already running")
        p.add_argument("--project", help="IFSSIM.uproject, when --sim is the editor")
        p.add_argument("--settings-dir", help="folder whose settings.json the sim should read "
                                              "(it is started from there)")
        p.add_argument("--sim-arg", action="append", default=[],
                       help="extra engine argument, repeatable (e.g. --sim-arg=-ExecCmds=\"t.MaxFPS 60\")")
        p.add_argument("--host", default="127.0.0.1")
        p.add_argument("--ready-timeout-s", type=float, default=900.0,
                       help="how long the sim may take to answer (first boots compile shaders)")

    p = sub.add_parser("check", help="start the sim offscreen, ask its status, stop it")
    sim_args(p)
    p.add_argument("--hold-s", type=float, default=10.0,
                   help="keep it up this long before judging (frame rate settles, LiDAR renders)")

    for name, hlp in (("run", "one benchmark run around a fresh sim"),
                      ("loop", "benchmark runs on repeat, a fresh sim each cycle")):
        p = sub.add_parser(name, help=hlp)
        sim_args(p)
        p.add_argument("--results-root", default=str(DEFAULT_RESULTS))
        p.add_argument("--mc-url", default="http://127.0.0.1:8000")
        p.add_argument("--mc-key", default=os.environ.get("IFSSIM_MC_API_KEY", ""))
        p.add_argument("--mc-up", action="store_true",
                       help="start Mission Control's backend (docker compose up -d mission_control_backend)")
        p.add_argument("--max-run-s", type=float, default=0.0,
                       help="hard cap on one run, on top of the benchmark's own per-mission timeouts")
        p.add_argument("--settle-s", type=float, default=120.0,
                       help="before the first mission, wait (at most this long; 0 = don't) until "
                            "the sim keeps real time: for ~30 s after boot it runs slower and the "
                            "car oscillates off the track (not applied with --sim attach)")
        if name == "loop":
            p.add_argument("--cycles", type=int, default=0, help="0 = until stopped")
            p.add_argument("--pause-s", type=float, default=10.0)
            p.add_argument("--max-infra-failures", type=int, default=3,
                           help="stop after this many machine failures in a row (0 = never)")
    return ap


def _sim_command(a):
    if a.sim == "attach":
        return None
    return resolve_command(a.sim, project=a.project, extra=a.sim_arg, settings_dir=a.settings_dir)


def _check(a, should_stop) -> int:
    sim = SimProcess(_sim_command(a), host=a.host,
                     log_path=Path(DEFAULT_RESULTS) / "headless" / "check_sim.log")
    try:
        sim.start()
        ready = sim.wait_ready(a.ready_timeout_s, should_stop=should_stop)
        t_hold = time.monotonic()
        while time.monotonic() - t_hold < a.hold_s:
            if should_stop():
                raise KeyboardInterrupt
            time.sleep(0.2)
        status = json.loads(rpc(a.host, "getSimStatus") or "{}")
        # The GPU LiDAR is the part that needs a real GPU offscreen: it logs
        # its first depth readback. No readback = a headless box that can't
        # run perception, however healthy the rest looks.
        lidar = None
        if sim.log_path and sim.log_path.exists():
            text = sim.log_path.read_text(errors="replace")
            lidar = "first readback OK" in text if "LiDAR GPU" in text else None
        ok = bool(status) and lidar is not False
        if lidar is None:
            # Shipping builds can compile these log lines out, so no line is
            # not proof the GPU works. Say so rather than pass silently.
            print("warning: no LiDAR GPU lines in the sim log, so the GPU readback is unchecked "
                  "(a Shipping build may strip them; check a perception run instead)", file=sys.stderr)
        print(json.dumps({"ok": ok, "ready_s": round(ready, 1), "status": status,
                          "lidar_gpu_readback": lidar, "sim_log": str(sim.log_path)}, indent=2))
        return 0 if ok else 3
    except KeyboardInterrupt:
        print(json.dumps({"ok": False, "error": "interrupted", "sim_log": str(sim.log_path)}, indent=2))
        return 130
    except (SimError, OSError, ValueError) as e:
        print(json.dumps({"ok": False, "error": str(e), "sim_log": str(sim.log_path)}, indent=2))
        return 3
    finally:
        sim.stop()


def main(argv=None) -> int:
    own, bench = _split(list(sys.argv[1:] if argv is None else argv))
    a = _parser().parse_args(own)
    try:
        cmd = _sim_command(a)
    except SimError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    stop = {"flag": False}

    def _on_signal(signum, _frame):
        if stop["flag"]:
            raise KeyboardInterrupt
        stop["flag"] = True
        print("stopping after cleanup (signal again to force)", file=sys.stderr)

    # SIGHUP too: a dropped ssh session or closed terminal must still stop the
    # sim and the benchmark, which run in their own sessions and would
    # otherwise be orphaned holding the RPC port.
    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            signal.signal(sig, _on_signal)
    should_stop = lambda: stop["flag"]  # noqa: E731

    if a.cmd == "check":
        return _check(a, should_stop)

    cfg = RunConfig(sim_command=cmd, results_root=Path(a.results_root), benchmark_args=bench,
                    mc_url=a.mc_url.rstrip("/"), mc_key=a.mc_key, mc_up=a.mc_up, host=a.host,
                    sim_ready_timeout_s=a.ready_timeout_s, max_run_s=a.max_run_s,
                    settle_s=a.settle_s)
    log = lambda m: print(m, flush=True)  # noqa: E731
    if a.cmd == "run":
        rec = run_once(cfg, log=log, should_stop=should_stop)
        return exit_code(rec["outcome"])
    recs = run_loop(cfg, cycles=a.cycles, pause_s=a.pause_s, max_infra_failures=a.max_infra_failures,
                    log=log, should_stop=should_stop)
    outcomes = [r["outcome"] for r in recs]
    print(json.dumps({o: outcomes.count(o) for o in sorted(set(outcomes))}))
    return 0 if outcomes and all(o == "finished" for o in outcomes) else (exit_code(outcomes[-1]) if outcomes else 3)


if __name__ == "__main__":
    sys.exit(main())
