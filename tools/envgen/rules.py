"""Check a built track against the FS rules and against itself.

Two severities:
- "error": the track is broken and should not be loaded: the corridor folds
  or crosses itself, or the sim would spawn the car facing the wrong way.
- "warn": the track breaks an FS-DV layout rule (FS Rules 2026, DV handbook):
  width under 3 m, cones more than 5 m apart, a straight over 80 m, a hairpin
  under 9 m outside diameter, a lap outside 200-500 m. These are warnings,
  not errors, because a test track may break them on purpose -- a missing-
  cone section breaks the 5 m rule by design. `strict=True` makes them errors.

Every issue carries where it is (`s`, metres along the centreline from the
start gate, and `xy`), so the drawing UI can point at it.
"""

from __future__ import annotations

import math
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

from .corridor import Track
from .startpose import compute_start_pose, heading_error_deg

MIN_WIDTH_M = 3.0
MAX_SPACING_M = 5.0
MAX_STRAIGHT_M = 80.0
STRAIGHT_KAPPA = 1.0 / 100.0     # radius over 100 m counts as straight
MIN_OUTSIDE_DIAMETER_M = 9.0
LAP_MIN_M, LAP_MAX_M = 200.0, 500.0
CLEARANCE_M = 1.5                # between two separate parts of the corridor
FLOOR_HALF_M = 100.0             # customMap's floor is about 200 m x 250 m
HEADING_TOL_DEG = 30.0


def _issue(code, severity, message, track: Track, i: Optional[int] = None) -> Dict:
    d = {"code": code, "severity": severity, "message": message}
    if i is not None:
        d["s"] = round(track.s[i], 1)
        d["xy"] = [round(track.centreline[i][0], 2), round(track.centreline[i][1], 2)]
    return d


def check(track: Track, strict: bool = False) -> List[Dict]:
    issues: List[Dict] = []
    rule = "error" if strict else "warn"
    spec = track.spec
    n = len(track.centreline)
    closed = spec.closed

    # --- geometry that cannot be built --------------------------------------
    # A boundary folds where the turn radius is under half the width: the
    # inner cone line would run backwards.
    worst = max(range(n), key=lambda i: abs(track.kappa[i]) * track.width[i] / 2)
    if abs(track.kappa[worst]) * track.width[worst] / 2 >= 0.95:
        r = 1 / abs(track.kappa[worst])
        issues.append(_issue("fold", "error",
                             f"a corner of radius {r:.1f} m is too tight for a "
                             f"{track.width[worst]:.1f} m wide track: the inside cone line folds",
                             track, worst))

    # Two parts of the corridor that are far apart along the track but close
    # in space: the track crosses or touches itself.
    cell = 4.0
    grid = defaultdict(list)
    for i, (x, y) in enumerate(track.centreline):
        grid[(int(math.floor(x / cell)), int(math.floor(y / cell)))].append(i)
    worst_gap: Tuple[float, int] = (float("inf"), -1)
    for i, (x, y) in enumerate(track.centreline):
        gx, gy = int(math.floor(x / cell)), int(math.floor(y / cell))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in grid[(gx + dx, gy + dy)]:
                    if j <= i:
                        continue
                    along = abs(track.s[j] - track.s[i])
                    if closed:
                        along = min(along, track.length - along)
                    need = (track.width[i] + track.width[j]) / 2 + CLEARANCE_M
                    if along < 3 * need:
                        continue
                    d = math.hypot(track.centreline[j][0] - x, track.centreline[j][1] - y)
                    gap = d - (track.width[i] + track.width[j]) / 2
                    if gap < worst_gap[0]:
                        worst_gap = (gap, i)
    if worst_gap[1] >= 0:
        gap, i = worst_gap
        if gap < 0:
            issues.append(_issue("crossing", "error",
                                 "the track crosses or overlaps itself", track, i))
        elif gap < CLEARANCE_M:
            issues.append(_issue("clearance", rule,
                                 f"two parts of the track are only {gap:.1f} m apart "
                                 f"(cone lines of one part next to another)", track, i))

    # The sim must spawn the car facing the way the track was drawn.
    pose = compute_start_pose(track.cones)
    if pose is None:
        issues.append(_issue("start_pose", "error",
                             "the sim cannot derive a start pose from these cones", track, 0))
    else:
        tx = track.centreline[1][0] - track.centreline[0][0]
        ty = track.centreline[1][1] - track.centreline[0][1]
        err = heading_error_deg(pose, (tx, ty))
        if err > HEADING_TOL_DEG:
            issues.append(_issue("start_pose", "error",
                                 f"the sim would spawn the car {err:.0f} deg off the drawn "
                                 f"direction ({pose.method})", track, 0))

    if not closed and track.length < 3 * 10.0:
        issues.append(_issue("too_short", "error",
                             f"an open track needs at least 30 m between its start and "
                             f"finish gates, this one is {track.length:.0f} m", track))

    # --- FS layout rules ------------------------------------------------------
    i_narrow = min(range(n), key=lambda i: track.width[i])
    if track.width[i_narrow] < MIN_WIDTH_M:
        issues.append(_issue("width", rule,
                             f"{track.width[i_narrow]:.1f} m wide, under the {MIN_WIDTH_M:.0f} m minimum",
                             track, i_narrow))

    # Gap between consecutive cones of one line, as the crow flies. A gate's
    # cones count for the line on their side: there they are the track's edge.
    for side in ("left", "right"):
        line = sorted((s, x, y) for (t, x, y), s, sd in
                      zip(track.cones, track.cone_s, track.cone_side) if sd in (side, "gate_" + side))
        if closed and line:
            line.append((line[0][0] + track.length, line[0][1], line[0][2]))
        worst_i, worst_d = -1, 0.0
        for a, b in zip(line, line[1:]):
            d = math.hypot(b[1] - a[1], b[2] - a[2])
            if d > worst_d:
                worst_d, worst_i = d, int(round((a[0] % track.length) / (track.length / n))) % n
        if worst_d > MAX_SPACING_M + 0.25:
            colour = "blue" if side == "left" else "yellow"
            issues.append(_issue("spacing", rule,
                                 f"{worst_d:.1f} m between two {colour} cones, over the "
                                 f"{MAX_SPACING_M:.0f} m maximum", track, worst_i))

    # Longest run of straight samples. On a loop, start the scan at a curved
    # sample so a straight through the start line is counted in one piece.
    ds = track.length / (n if closed else n - 1)
    curved = [i for i in range(n) if abs(track.kappa[i]) >= STRAIGHT_KAPPA]
    start = curved[0] if (closed and curved) else 0
    run, best, best_end = 0.0, 0.0, 0
    for k in range(n):
        i = (start + k) % n
        if abs(track.kappa[i]) < STRAIGHT_KAPPA:
            run += ds
            if run > best:
                best, best_end = run, i
        else:
            run = 0.0
    if best > MAX_STRAIGHT_M:
        issues.append(_issue("straight", rule,
                             f"a {best:.0f} m straight, over the {MAX_STRAIGHT_M:.0f} m maximum",
                             track, best_end))

    i_tight = max(range(n), key=lambda i: abs(track.kappa[i]))
    if abs(track.kappa[i_tight]) > 1e-9:
        outside_d = 2 * (1 / abs(track.kappa[i_tight]) + track.width[i_tight] / 2)
        if outside_d < MIN_OUTSIDE_DIAMETER_M:
            issues.append(_issue("hairpin", rule,
                                 f"a turn of {outside_d:.1f} m outside diameter, under the "
                                 f"{MIN_OUTSIDE_DIAMETER_M:.0f} m minimum", track, i_tight))

    if closed and not (LAP_MIN_M <= track.length <= LAP_MAX_M):
        issues.append(_issue("lap_length", rule,
                             f"a {track.length:.0f} m lap, outside {LAP_MIN_M:.0f}-{LAP_MAX_M:.0f} m",
                             track))

    xs = [x for _, x, _ in track.cones]
    ys = [y for _, _, y in track.cones]
    if max(map(abs, xs + ys)) > FLOOR_HALF_M:
        issues.append(_issue("floor", "warn",
                             f"cones reach {max(map(abs, xs + ys)):.0f} m from the centre; "
                             f"customMap's floor is about 200 m x 250 m, so some may fall off it",
                             track))

    track.issues = issues
    return issues


def has_errors(issues: List[Dict]) -> bool:
    return any(i["severity"] == "error" for i in issues)
