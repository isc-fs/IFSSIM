#!/usr/bin/env python3
"""
Validate every track CSV under Content/tracks/ — runs as a CI gate.

A track CSV is a simple text file: each line is `cone_type,x,y` (in
metres, ENU). cone_type ∈ {blue, yellow, big_orange, small_orange}.
Anything malformed (missing fields, non-numeric coordinates, unknown
cone type, blank file) is a regression that would silently break
loadTrack at runtime.

It also checks where the sim will spawn the car (envgen.startpose, a port
of FSDSConeSpawner::ComputeStartGatePose):
  - the car faces the way the track goes, with BLUE ON ITS LEFT (FS rules;
    the planner's left chain is blue). The CSV frame is right-handed:
    +y is left of +x (the loader maps CSV +y to UE -Y).
  - on a one-gate track the sim puts blue on the left from the nearest blue
    and yellow cones; that offset must point clearly across the gate, not
    along it. Without both colours near the gate the sim falls back to the
    four track cones nearest it, which must then all be clearly ahead.
    tools/envgen/corridor.py's start zone satisfies both by construction:
    pairs at +2.0 and +3.5 m, nothing within 5 m behind.

Exit code:
  0 — every CSV under the search root parses clean
  1 — at least one file failed validation; details on stderr
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
from envgen.startpose import K_DIRECTION, compute_start_pose  # noqa: E402


VALID_CONE_TYPES = frozenset({"blue", "yellow", "big_orange", "small_orange"})

# Sanity bounds on cone coordinates — tracks larger than 1 km square
# are almost certainly a unit-confusion bug (cm vs m). Tighten if real
# tracks ever push closer to this.
MAX_COORD_M = 500.0

# Start-pose margins.
MAX_SIDE_SKEW = 0.5      # side test: |along| / |across| of the yellow->blue offset
MIN_AHEAD_M = 1.0        # each of the K nearest cones is at least this far ahead
SET_MARGIN_M = 1.0       # the nearest cone not ahead is this much farther than the K-th
SIDE_WINDOW_AHEAD_M = 5.0   # cones 0..this far ahead decide which side blue is on (skidpad's outer circle crosses the lane at ~6.8 m)
SIDE_WINDOW_LAT_M = 4.0
MIN_SIDE_LAT_M = 0.5

Cone = Tuple[str, float, float]


def check_start_pose(path: Path, cones: List[Cone]) -> List[str]:
    """Errors in where the sim will spawn the car on this track."""
    if not any(t == "big_orange" for t, _, _ in cones):
        return []
    pose = compute_start_pose(cones)
    if pose is None:
        return [f"{path}: the sim cannot compute a start pose "
                f"(needs >= 4 big_orange and >= 1 blue/yellow)"]

    (ax, ay), (fx, fy) = pose.anchor, pose.forward

    def frame(x: float, y: float) -> Tuple[float, float, float]:
        dx, dy = x - ax, y - ay
        return dx * fx + dy * fy, -dx * fy + dy * fx, (dx * dx + dy * dy) ** 0.5

    track = [(t, *frame(x, y)) for t, x, y in cones if t in ("blue", "yellow")]
    where = f"{path}: start pose ({pose.method}, facing ({fx:+.2f}, {fy:+.2f}) from ({ax:.2f}, {ay:.2f}))"
    errors: List[str] = []

    if pose.method == "pca-sides":
        sx, sy = pose.side_offset
        along, across = sx * fx + sy * fy, -sx * fy + sy * fx
        if abs(along) > MAX_SIDE_SKEW * abs(across):
            errors.append(f"{where}: the nearest blue and yellow cones, which set the heading, "
                          f"are offset {along:+.2f} m along the gate and {across:+.2f} m across it; "
                          f"put them in pairs (lay out the start zone as envgen/corridor.py does).")
    elif pose.method == "pca":
        by_dist = sorted(track, key=lambda c: c[3])
        nearest = by_dist[:K_DIRECTION]
        offsets = ", ".join(f"{c[0]} {c[1]:+.2f}" for c in nearest)
        if any(c[1] < MIN_AHEAD_M for c in nearest):
            errors.append(f"{where}: with no blue and yellow near the gate, the {K_DIRECTION} cones "
                          f"nearest it pick forward, "
                          f"and not all are >= {MIN_AHEAD_M} m ahead (along: {offsets}). "
                          f"Lay out the start zone as envgen/corridor.py does.")
        else:
            not_ahead: Optional[float] = min((c[3] for c in track if c[1] < MIN_AHEAD_M), default=None)
            if not_ahead is not None and not_ahead - nearest[-1][3] < SET_MARGIN_M:
                errors.append(f"{where}: a cone not ahead of the gate is within {SET_MARGIN_M} m of "
                              f"joining the {K_DIRECTION} nearest ({not_ahead:.2f} m vs "
                              f"{nearest[-1][3]:.2f} m); the heading is fragile.")

    window = [c for c in track if 0.0 <= c[1] <= SIDE_WINDOW_AHEAD_M and abs(c[2]) <= SIDE_WINDOW_LAT_M]
    blue = [c[2] for c in window if c[0] == "blue"]
    yellow = [c[2] for c in window if c[0] == "yellow"]
    if not blue or not yellow:
        errors.append(f"{where}: no blue and yellow cones in the {SIDE_WINDOW_AHEAD_M:.0f} m ahead "
                      f"of the gate (blue {len(blue)}, yellow {len(yellow)})")
    elif min(blue) < MIN_SIDE_LAT_M or max(yellow) > -MIN_SIDE_LAT_M:
        side = "RIGHT" if sum(blue) / len(blue) < 0 else "both sides"
        errors.append(f"{where}: blue is on the car's {side} "
                      f"(lateral, + = left: blue {sorted(round(v, 2) for v in blue)}, "
                      f"yellow {sorted(round(v, 2) for v in yellow)}). FS has blue on the left.")
    return errors


def validate_one(path: Path) -> List[str]:
    """Return a list of human-readable error strings; empty list = clean."""
    errors: List[str] = []
    try:
        text = path.read_text()
    except Exception as e:
        return [f"{path}: cannot read: {e}"]

    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    if not lines:
        return [f"{path}: file has no non-blank lines"]

    cone_count = 0
    cones: List[Cone] = []
    for lineno, line in enumerate(lines, start=1):
        # Tolerate trailing fields (some generators emit a 4th colour
        # column or similar) — we only require the first three.
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 3:
            errors.append(f"{path}:{lineno}: only {len(parts)} field(s); need at least 3 (type,x,y)")
            continue

        cone_type = parts[0].lower()
        if cone_type not in VALID_CONE_TYPES:
            # Skip header rows (e.g., "type,x,y") quietly — a header is
            # a one-off zeroth line some authoring tools add.
            if lineno == 1 and cone_type in ("type", "cone_type", "color", "colour"):
                continue
            errors.append(
                f"{path}:{lineno}: unknown cone type {cone_type!r} "
                f"(allowed: {sorted(VALID_CONE_TYPES)})"
            )
            continue

        try:
            x = float(parts[1])
            y = float(parts[2])
        except ValueError:
            errors.append(f"{path}:{lineno}: non-numeric coordinates {parts[1]!r}, {parts[2]!r}")
            continue

        if abs(x) > MAX_COORD_M or abs(y) > MAX_COORD_M:
            errors.append(
                f"{path}:{lineno}: coordinates ({x}, {y}) outside ±{MAX_COORD_M} m sanity bound — "
                f"unit-confusion bug (cm vs m)?"
            )
        cone_count += 1
        cones.append((cone_type, x, y))

    if cone_count == 0:
        errors.append(f"{path}: no valid cone rows parsed")
    elif not errors:
        errors.extend(check_start_pose(path, cones))
    return errors


def find_csvs(root: Path) -> Iterable[Path]:
    return sorted(root.rglob("*.csv"))


def main(argv: List[str]) -> int:
    root = Path(argv[1]) if len(argv) > 1 else Path("Content/tracks")
    if not root.is_dir():
        print(f"track-validator: search root does not exist: {root}", file=sys.stderr)
        return 1

    paths = list(find_csvs(root))
    if not paths:
        print(f"track-validator: no CSV files under {root}", file=sys.stderr)
        return 1

    all_errors: List[Tuple[Path, List[str]]] = []
    for p in paths:
        errs = validate_one(p)
        if errs:
            all_errors.append((p, errs))

    if all_errors:
        print(f"track-validator: {len(all_errors)} file(s) failed:", file=sys.stderr)
        for p, errs in all_errors:
            for e in errs:
                print(f"  {e}", file=sys.stderr)
        return 1

    print(f"track-validator: {len(paths)} file(s) clean")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
