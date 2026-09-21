#!/usr/bin/env python3
"""Run a scenario N times with different seeds, capturing a manifest per run.

WHY THIS EXISTS
---------------
Comparing two algorithms in this simulator has meant: boot the stack, drive a
lap by hand, record a bag, eyeball a plot. That is slow, unrepeatable, and the
difference you measure mixes the change under test with whatever the noise
happened to do that run.

This is the smallest thing that makes a fair comparison possible:

  * every run starts from an identical, explicitly restored state
    (resetScenario), so run 2 is not run 1 continued;
  * the seed is swept explicitly, so N repeats are N genuine samples rather
    than N draws from an unknown distribution;
  * every run writes a manifest naming the code, config and seed that produced
    it, so a result that cannot be attributed is visibly missing its provenance
    instead of silently assumed good.

WHAT IT DOES NOT DO YET
-----------------------
It does not drive the car — there is no scripted control input, so a run
exercises the sim and whatever autonomy is already attached. Closed-loop
scenario scripting is the next piece. Deliberately not faked here: a runner
that pretended to control the car would produce comparisons that look rigorous
and are not.

USAGE
    python3 tools/scenario_runner/run_scenario.py --track acceleration.csv \\
        --seeds 1 2 3 --seconds 20 --out runs/
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "mission_control" / "backend"))

try:
    from sim_client import SimConnection  # type: ignore
except Exception as exc:  # pragma: no cover
    print(f"error: could not import the sim RPC client: {exc}", file=sys.stderr)
    print("       expected tools/mission_control/backend/sim_client.py", file=sys.stderr)
    raise SystemExit(2)


def git_sha(path: Path) -> str:
    """Short SHA plus a dirty marker. Provenance is worthless if it silently
    reports a clean tree while uncommitted edits are what actually ran."""
    try:
        sha = subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "--short", "HEAD"],
            text=True, stderr=subprocess.DEVNULL).strip()
        dirty = subprocess.check_output(
            ["git", "-C", str(path), "status", "--porcelain"],
            text=True, stderr=subprocess.DEVNULL).strip()
        return sha + ("-dirty" if dirty else "")
    except Exception:
        return "unknown"


def build_manifest(seed: int, track: str, seconds: float,
                   reset_reply: dict, referee: dict) -> dict:
    return {
        "seed": seed,
        "track": track,
        "sim_seconds": seconds,
        # Both repos, because a run is the pair. A pipeline-only change with an
        # unchanged sim SHA is exactly the case you need to be able to see.
        "ifssim_sha": git_sha(REPO),
        "pipeline_sha": git_sha(REPO / "pipeline"),
        "reset": reset_reply,
        "referee": referee,
        "wall_clock_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }


def run_one(sim: SimConnection, seed: int, track: str | None, seconds: float) -> dict:
    if track:
        print(f"  loading track {track}")
        sim.load_track(track)

    # Reset FIRST, then let time pass. Reset restores the RNG generation, the
    # referee, the pose, the body velocities and the rotor speed together —
    # doing any of them separately is how a "repeat" quietly stops being one.
    raw = sim._cmd(f"resetScenario {seed}", timeout=20.0)
    try:
        reset_reply = json.loads(raw)
    except Exception:
        reset_reply = {"raw": raw}
    print(f"  reset -> {reset_reply}")

    if seconds > 0:
        # Uses the sim's own clock. Under the fixed timestep this is a
        # deterministic number of physics steps, not a wall-clock guess.
        sim._cmd(f"simContinueForTime {seconds}", timeout=seconds + 30.0)
        time.sleep(seconds + 1.0)

    referee = sim.get_referee_state()
    print(f"  referee -> {referee}")
    return build_manifest(seed, track or "(current)", seconds, reset_reply, referee)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", help="track CSV to load before each run")
    ap.add_argument("--seeds", type=int, nargs="+", default=[1],
                    help="scenario seeds; one run per seed")
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="sim seconds to advance per run (0 = reset only)")
    ap.add_argument("--out", type=Path, default=Path("runs"),
                    help="directory for per-run manifests")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=41451)
    args = ap.parse_args()

    sim = SimConnection(args.host, args.port)
    if not sim.is_connected():
        print("error: no sim on {}:{} — is the editor running and in Play?"
              .format(args.host, args.port), file=sys.stderr)
        return 1

    args.out.mkdir(parents=True, exist_ok=True)
    manifests = []

    for seed in args.seeds:
        print(f"[seed {seed}]")
        m = run_one(sim, seed, args.track, args.seconds)
        path = args.out / f"run_seed{seed}.json"
        path.write_text(json.dumps(m, indent=2))
        manifests.append(m)
        print(f"  manifest -> {path}")

    summary = args.out / "summary.json"
    summary.write_text(json.dumps(manifests, indent=2))
    print(f"\n{len(manifests)} run(s) -> {summary}")

    # Loud, because a comparison across differing code is not a comparison.
    shas = {(m["ifssim_sha"], m["pipeline_sha"]) for m in manifests}
    if len(shas) > 1:
        print("WARNING: runs span different code revisions — not comparable:", shas)
    if any("dirty" in m["ifssim_sha"] or "dirty" in m["pipeline_sha"] for m in manifests):
        print("WARNING: at least one run came from a dirty tree — its exact code "
              "cannot be recovered from the manifest.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
