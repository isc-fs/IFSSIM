#!/usr/bin/env python3
"""Path planning smoke test — does Plan_Path actually publish /Path
on the canonical replay bag?

Pass criteria:
  • Replay bag-side subscriber (within the ephemeral container) records
    /Path with at least MIN_PATH_MESSAGES non-empty messages over the
    REPLAY_DURATION_S window.
  • At least MIN_PATH_FRACTION of those messages have len(poses) >= 2
    (anything shorter isn't an actionable trajectory for the controller).

Why this exists:
  feat/30 swapped the vendored fsd_path_planning library for a pip dep
  on upstream and rewrote the cone-color classifier to match the
  cone_graph_slam RGB tuples (the old classifier dropped every blue
  cone into UNKNOWN). This test guards against regressions in either:
  the integration with /Conos (color classification, forward-cone
  gate) and the bag-replay path-publishing pipeline as a whole.

  This is intentionally a smoke test, not a path-quality regression.
  We're asserting "Plan_Path publishes paths," not "the path is good".
  Path quality is downstream of cone_slam quality, which is already
  guarded by tools/test_slam_regression.py.

Usage:
    tools/test_path_planning_smoke.py
    tools/test_path_planning_smoke.py --bag <other_bag_name>
    tools/test_path_planning_smoke.py --no-replay   # parse existing recording

Exit code 0 = pass, 1 = regression detected, 2 = harness/parse error.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Tuple


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BAG = "trackA_manual_001602"
REPLAY_DURATION_S = 145

# Empirical baseline thresholds. trackA_manual_001602 has 145 s of
# replay; with a healthy planner publishing at ~10 Hz we'd expect
# ~1450 messages. Iter-15 cone_slam cascades ~t=80 s, after which DA
# corruption can starve the cone gate; we still expect well over 100
# pre-cascade messages. Set the bar low enough to catch breakage
# without flagging the known back-stretch starvation.
MIN_PATH_MESSAGES = 100
MIN_PATH_FRACTION = 0.95   # of recorded /Path messages, ≥95% must have ≥2 poses


def die(code: int, msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(code)


def run_replay(bag_name: str) -> Path:
    """Execute tools/replay.sh; return path to the recorded sqlite3 dir."""
    bag_dir = REPO_ROOT / "tools" / "bags" / bag_name
    if not bag_dir.is_dir():
        die(2, f"bag dir not found: {bag_dir}")

    cmd = [str(REPO_ROOT / "tools" / "replay.sh"),
           bag_name, str(REPLAY_DURATION_S)]
    print(f"==> running {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, cwd=REPO_ROOT)
    if proc.returncode != 0:
        die(2, f"replay.sh exited {proc.returncode}")

    rec_dir = bag_dir / "replay_cone_slam"
    if not rec_dir.is_dir():
        die(2, f"replay produced no recording at {rec_dir}")
    return rec_dir


def count_path_messages(rec_dir: Path) -> Tuple[int, int]:
    """Open the recorded bag (sqlite3) and count /Path messages.

    Returns (total_messages, messages_with_at_least_two_poses).

    We open via the rosbag2 reader inside the running dv_pipeline_stack
    container so we don't have to depend on rosbag2_py being installed
    on the host. The container is the same image that produced the
    recording, so storage compatibility is guaranteed.
    """
    container = "ifssim-dv_pipeline_stack-1"
    db_files = sorted(rec_dir.glob("*.db3"))
    if not db_files:
        die(2, f"no .db3 file inside {rec_dir}")
    db_host = db_files[0]

    # Copy the DB and a small reader script into the container (via
    # docker cp, no shell quoting hazards), then exec python directly
    # on the script file. Trying to inline the script via `python3 -c`
    # tripped over nested quoting before.
    db_in_container = "/tmp/replay_cone_slam.db3"
    script_in_container = "/tmp/count_path_msgs.py"
    subprocess.run(
        ["docker", "cp", str(db_host), f"{container}:{db_in_container}"],
        check=True)
    script_src = (
        REPO_ROOT / "tools" / "_count_path_msgs.py")
    subprocess.run(
        ["docker", "cp", str(script_src), f"{container}:{script_in_container}"],
        check=True)

    res = subprocess.run(
        ["docker", "exec", container,
         "bash", "-lc",
         f"source /opt/ros/humble/setup.bash && python3 {script_in_container} {db_in_container}"],
        check=True, capture_output=True, text=True)
    subprocess.run(
        ["docker", "exec", container, "rm", "-f",
         db_in_container, script_in_container],
        check=False)

    parts = res.stdout.strip().splitlines()[-1].split()
    if len(parts) != 2:
        die(2, f"unexpected output from in-container counter: {res.stdout!r}")
    return int(parts[0]), int(parts[1])


def evaluate(total: int, nontrivial: int) -> int:
    print()
    print(f"  /Path messages recorded:        {total}")
    print(f"  /Path messages with ≥2 poses:   {nontrivial}")
    print(f"  required ≥{MIN_PATH_MESSAGES} total, "
          f"≥{int(MIN_PATH_FRACTION*100)}% non-trivial")
    print()

    fails = []
    if total < MIN_PATH_MESSAGES:
        fails.append(
            f"only {total} /Path messages recorded (need ≥{MIN_PATH_MESSAGES})")
    if total > 0 and (nontrivial / total) < MIN_PATH_FRACTION:
        fails.append(
            f"only {nontrivial}/{total} = {100*nontrivial/total:.1f}% have ≥2 poses "
            f"(need ≥{int(MIN_PATH_FRACTION*100)}%)")

    if fails:
        print("REGRESSION:")
        for f in fails:
            print(f"  • {f}")
        return 1
    print("✅ /Path is healthy on the canonical bag")
    return 0


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", default=DEFAULT_BAG,
                   help=f"bag-dir name under tools/bags/ (default: {DEFAULT_BAG})")
    p.add_argument("--no-replay", action="store_true",
                   help="skip the replay; reuse the existing recording dir")
    args = p.parse_args()

    if args.no_replay:
        rec_dir = REPO_ROOT / "tools" / "bags" / args.bag / "replay_cone_slam"
        if not rec_dir.is_dir():
            die(2, f"--no-replay set but {rec_dir} doesn't exist")
    else:
        rec_dir = run_replay(args.bag)

    total, nontrivial = count_path_messages(rec_dir)
    sys.exit(evaluate(total, nontrivial))


if __name__ == "__main__":
    main()
