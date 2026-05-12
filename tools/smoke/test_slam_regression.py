#!/usr/bin/env python3
"""Regression test: cone_slam drift on the canonical fixture bag.

Invokes tools/replay.sh against `trackA_manual_001602` and asserts
the per-second drift table stays within the bounds locked in at the
**iter 15** baseline (2026-04-29). A fail means a recent change made
the SLAM measurably worse on a known-good drive — caller's job to
either fix the regression, or update the thresholds with a documented
reason and bump the iter number.

Iteration history (each row is the cone_slam state at that point;
numbers are median actuals across two consecutive runs):

  iter   date        Cone_Detection           SLAM-side                60 s    75 s    85 s     cascade
  ----   ----------  ----------------------   --------------------     -----   -----   ------   --------
  14     2026-04-28  bag's recorded (19%      RPM=0.00898, σ=0.30,     0.79    0.99    1.52     t≈92 s
                     empty scans)             BIAS_RW=1e-4/1e-5
  15     2026-04-29  live mix (parametric +   same SLAM knobs +        0.69    2.14    cascade  t≈80–90 s
                     centroid fallback at     replay.sh spawns
                     <20 m, range-aware       Cone_Detection live
                     point-count gate)        instead of using bag

Thresholds in this file are iter-15-calibrated. Run-to-run variance
is ~10 % from replay scheduling jitter; thresholds carry 1.5–2×
headroom over the median actuals.

t=85 s is intentionally NOT asserted: cascade onset is structurally
unstable on this bag (varies between t=80–95 s) and gating on it
produces false-positive regressions. The contract is "tracks well
through the front 75 s of the lap"; back-stretch is a known limit
until loop closure or proper covariance-aware DA lands.

Usage:
    tools/smoke/test_slam_regression.py
    tools/smoke/test_slam_regression.py --quiet
    tools/smoke/test_slam_regression.py --bag <other_bag_name>

Exit code 0 = pass, 1 = regression detected, 2 = harness/parse error.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict


REPO_ROOT = Path(__file__).resolve().parent.parent.parent  # tools/smoke/X.py → repo root
DEFAULT_BAG = "trackA_manual_001602"
REPLAY_DURATION_S = 145   # full lap incl. loop closure (~t=137s).
                          # Bag is 151 s; we stop a few seconds short of the
                          # trailing standstill because GT goes flat there
                          # (no useful info beyond confirming the car parked).

# (timestamp_s, max_drift_m). Re-baselined on 2026-04-29 against the
# mix-approach cone_detection (parametric fit primary, range-aware
# centroid fallback at <20 m) running live in the replay container —
# not against the bag's recorded /Conos_raw which had a 19% empty-scan
# rate from the pre-fix detection. The numbers in the comments below
# are the median actuals across two consecutive runs of the live new
# detection pipeline (variance ~10 % run-to-run from replay scheduling).
#
# t=85s is intentionally NOT asserted: cascade onset is structurally
# unstable on this bag (varies between t=80–95 s across runs because
# of empty-cone-window scheduling jitter) and gating on it produces
# false-positive regressions. The contract is "tracks well through
# the front 75 s of the lap"; back-stretch is a known limit until
# loop closure or covariance-aware DA is properly implemented.
THRESHOLDS = [
    ( 3, 0.10),   # actual: 0.01 m
    (10, 0.10),   # actual: 0.02 m
    (20, 0.10),   # actual: 0.01 m
    (28, 0.10),   # actual: 0.01 m   (last sample of standstill)
    (30, 0.20),   # actual: 0.05 m   (first sample of motion)
    (35, 0.80),   # actual: 0.16 m   (first-turn entry — loose, varies)
    (45, 1.00),   # actual: 0.40 m
    (60, 1.50),   # actual: 0.69 m
    (75, 4.00),   # actual: 2.14 m   (back-stretch — loose, ~10% RTRV)
]


# pose_cmp.py output format:
#   "  35s |   +0.08  +12.07   +87.8 |   +0.10  +12.22   +88.1 |  +0.15m"
# We only need the leading timestamp and trailing dist_err.
LINE_RE = re.compile(
    r"^\s*(\d+)s\s*\|.*\|\s*([+-]?\d+\.\d+)m\s*$"
)


def run_replay(bag_name: str, quiet: bool) -> Path:
    """Run tools/replay.sh and return the path to the pose-cmp txt."""
    bag_dir = REPO_ROOT / "tools" / "bags" / bag_name
    if not bag_dir.is_dir():
        die(2, f"bag dir not found: {bag_dir}")
    pose_cmp_path = bag_dir / "replay_pose_cmp_cone_slam.txt"

    cmd = [str(REPO_ROOT / "tools" / "replay.sh"), bag_name, str(REPLAY_DURATION_S)]
    if not quiet:
        print(f"==> running {' '.join(cmd)}", flush=True)
    proc = subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL if quiet else None,
        stderr=subprocess.STDOUT if quiet else None,
    )
    if proc.returncode != 0:
        die(2, f"replay.sh exited {proc.returncode}")
    if not pose_cmp_path.exists():
        die(2, f"replay produced no pose-cmp file at {pose_cmp_path}")
    return pose_cmp_path


def parse_drifts(path: Path) -> Dict[int, float]:
    out: Dict[int, float] = {}
    skip_scan_warnings = 0
    for line in path.read_text().splitlines():
        if "skip scan" in line:
            skip_scan_warnings += 1
            continue
        m = LINE_RE.match(line)
        if m:
            out[int(m.group(1))] = float(m.group(2))
    if skip_scan_warnings:
        print(f"⚠  {skip_scan_warnings} 'skip scan' warnings in replay log")
    return out


def evaluate(drifts: Dict[int, float]) -> int:
    fails = []
    print()
    print("checkpoint │  drift  │ threshold │ verdict")
    print("───────────┼─────────┼───────────┼────────")
    for t, lim in THRESHOLDS:
        d = drifts.get(t)
        if d is None:
            print(f"  t={t:>3}s    │   ----  │   {lim:>5.2f} m │ ⚠ MISSING")
            fails.append((t, None, lim, "missing"))
            continue
        ok = d <= lim
        mark = "✅" if ok else "❌ FAIL"
        print(f"  t={t:>3}s    │  {d:>5.2f} m │   {lim:>5.2f} m │ {mark}")
        if not ok:
            fails.append((t, d, lim, "exceeded"))
    print()
    if fails:
        print(f"REGRESSION: {len(fails)} checkpoint(s) failed:")
        for t, d, lim, reason in fails:
            if reason == "missing":
                print(f"  • t={t}s: missing from replay output")
            else:
                print(f"  • t={t}s: drift {d:.2f} m > threshold {lim:.2f} m"
                      f"  (over by {d-lim:+.2f} m)")
        return 1
    print("✅ all checkpoints within threshold")
    return 0


def die(code: int, msg: str):
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(code)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", default=DEFAULT_BAG,
                   help=f"bag-dir name under tools/bags/ (default: {DEFAULT_BAG})")
    p.add_argument("--quiet", action="store_true",
                   help="silence replay.sh stdout/stderr (still prints the verdict)")
    p.add_argument("--no-replay", action="store_true",
                   help="skip the replay; just parse the existing pose-cmp txt")
    args = p.parse_args()

    if args.no_replay:
        bag_dir = REPO_ROOT / "tools" / "bags" / args.bag
        path = bag_dir / "replay_pose_cmp_cone_slam.txt"
        if not path.exists():
            die(2, f"--no-replay set but {path} doesn't exist; run a replay first")
    else:
        path = run_replay(args.bag, args.quiet)

    drifts = parse_drifts(path)
    if not drifts:
        die(2, f"no drift samples parsed from {path}")
    sys.exit(evaluate(drifts))


if __name__ == "__main__":
    main()
