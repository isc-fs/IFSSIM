#!/usr/bin/env python3
"""Control pipeline smoke test — does Control actually publish
/control_command on the canonical replay bag?

Pass criteria (intentionally lenient — this is a wiring guard, not a
driving-quality regression):

  • At least MIN_CONTROL_MESSAGES /control_command messages were
    recorded over the replay window.
  • At least MIN_NONTRIVIAL_FRACTION of recorded messages exit the
    planner-warm gate (i.e. throttle > 0 OR brake > 0 — anything but
    a default-zero ControlCommand).
  • At least MIN_STEERING_FRACTION of recorded messages have a non-
    zero steering value (catches the v < 0.5 m/s clamp that leaves
    steering at exactly 0 forever — the failure mode that motivated
    moving control's velocity source from /gss to /cone_slam/state).

Why this exists:
  feat/31 dropped /gss + /testing_only/odom from Control's velocity
  source and rewired it to /cone_slam/state.twist (which now carries
  body-frame velocity per nav_msgs/Odometry semantics). Without this
  test we'd miss future regressions of either:
    (a) cone_slam reverting to world-frame twist → control reads the
        wrong axis component as longitudinal speed, hits the v<0.5
        clamp, never steers
    (b) someone re-introducing a bridge-side velocity dependency that
        breaks replay (the bag has no /gss).

  Driving-quality regressions (e.g. wrong steering sign, EBS not
  firing) need a different test — this one only guards the wiring.

Usage:
    tools/smoke/test_control_smoke.py
    tools/smoke/test_control_smoke.py --bag <other_bag_name>
    tools/smoke/test_control_smoke.py --no-replay   # parse existing recording

Exit code 0 = pass, 1 = regression, 2 = harness/parse error.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Tuple


REPO_ROOT = Path(__file__).resolve().parent.parent.parent  # tools/smoke/X.py → repo root
DEFAULT_BAG = "trackA_manual_001602"
REPLAY_DURATION_S = 145

# trackA_manual_001602 has 28 s of standstill at the start, then ~50 s
# of clean driving, then back-stretch cascade. Control publishes at
# 40 Hz → ~5800 messages over the full replay; require well above the
# standstill-only count (~1100) so we know motion was reached.
MIN_CONTROL_MESSAGES = 2000

# Of those messages, the fraction that should leave the planner-warm
# gate (throttle > 0 or brake > 0). Standstill-only would be near 1.0
# (small brake-tap during the warm gate), but we want this to also
# catch a "stuck publishing zeroes" failure where the gate never
# releases. 0.5 is comfortably between the two regimes.
MIN_NONTRIVIAL_FRACTION = 0.5

# Of all messages, fraction that should have steering ≠ 0. Pre-motion
# the velocity-gate clamps steering to zero, so on a bag with 28 s of
# standstill the floor is roughly (145 − 28) / 145 ≈ 0.80 of total
# messages; require 0.40 to leave headroom for cascade-induced
# clamping at the end. Below this and we're back to the v<0.5
# never-releases bug that motivated the GSS removal.
MIN_STEERING_FRACTION = 0.40


def die(code: int, msg: str) -> None:
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(code)


def run_replay(bag_name: str) -> Path:
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


def count_control_messages(rec_dir: Path) -> Tuple[int, int, int]:
    """Open the recorded bag and count /control_command messages.

    Returns (total, nontrivial, steering_nonzero).
    """
    container = "ifssim-dv_pipeline_stack-1"
    db_files = sorted(rec_dir.glob("*.db3"))
    if not db_files:
        die(2, f"no .db3 file inside {rec_dir}")
    db_host = db_files[0]

    db_in_container = "/tmp/replay_cone_slam.db3"
    script_in_container = "/tmp/count_control_msgs.py"
    subprocess.run(
        ["docker", "cp", str(db_host), f"{container}:{db_in_container}"],
        check=True)
    script_src = REPO_ROOT / "tools" / "smoke" / "_count_control_msgs.py"
    subprocess.run(
        ["docker", "cp", str(script_src), f"{container}:{script_in_container}"],
        check=True)

    res = subprocess.run(
        ["docker", "exec", container,
         "bash", "-lc",
         f"source /opt/ros/humble/setup.bash && "
         f"source /dv_pipeline_stack_ws/install/setup.bash && "
         f"python3 {script_in_container} {db_in_container}"],
        check=True, capture_output=True, text=True)
    subprocess.run(
        ["docker", "exec", container, "rm", "-f",
         db_in_container, script_in_container],
        check=False)

    parts = res.stdout.strip().splitlines()[-1].split()
    if len(parts) != 3:
        die(2, f"unexpected output from in-container counter: {res.stdout!r}")
    return int(parts[0]), int(parts[1]), int(parts[2])


def evaluate(total: int, nontrivial: int, steering_nonzero: int) -> int:
    print()
    print(f"  /control_command total:        {total}")
    print(f"  with throttle>0 or brake>0:    {nontrivial} "
          f"({100 * nontrivial / max(1, total):.1f}%)")
    print(f"  with steering != 0:            {steering_nonzero} "
          f"({100 * steering_nonzero / max(1, total):.1f}%)")
    print()
    print(f"  required ≥{MIN_CONTROL_MESSAGES} total, "
          f"≥{int(MIN_NONTRIVIAL_FRACTION * 100)}% non-trivial, "
          f"≥{int(MIN_STEERING_FRACTION * 100)}% steering")
    print()

    fails = []
    if total < MIN_CONTROL_MESSAGES:
        fails.append(
            f"only {total} /control_command messages (need ≥{MIN_CONTROL_MESSAGES})")
    if total > 0 and (nontrivial / total) < MIN_NONTRIVIAL_FRACTION:
        fails.append(
            f"only {100 * nontrivial / total:.1f}% non-trivial "
            f"(need ≥{int(MIN_NONTRIVIAL_FRACTION * 100)}%)")
    if total > 0 and (steering_nonzero / total) < MIN_STEERING_FRACTION:
        fails.append(
            f"only {100 * steering_nonzero / total:.1f}% with steering "
            f"(need ≥{int(MIN_STEERING_FRACTION * 100)}%)")

    if fails:
        print("REGRESSION:")
        for f in fails:
            print(f"  • {f}")
        return 1
    print("✅ /control_command pipeline is healthy on the canonical bag")
    return 0


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", default=DEFAULT_BAG)
    p.add_argument("--no-replay", action="store_true",
                   help="skip the replay; reuse the existing recording dir")
    args = p.parse_args()

    if args.no_replay:
        rec_dir = REPO_ROOT / "tools" / "bags" / args.bag / "replay_cone_slam"
        if not rec_dir.is_dir():
            die(2, f"--no-replay set but {rec_dir} doesn't exist")
    else:
        rec_dir = run_replay(args.bag)

    total, nontrivial, steering_nz = count_control_messages(rec_dir)
    sys.exit(evaluate(total, nontrivial, steering_nz))


if __name__ == "__main__":
    main()
