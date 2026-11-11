#!/usr/bin/env python3
"""
Validate every track CSV, and every environment sidecar, under Content/tracks/
— runs as a CI gate.

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
    four track cones nearest it, whose mean must then be clearly ahead.
    tools/envgen/corridor.py's start zone satisfies both by construction:
    pairs at +2.0 and +3.5 m, nothing within 5 m behind.

A sidecar (`<track>.env.json`, docs/environment_sidecar.md) is the
environment around a track's cones. It is checked by the same rules the plugin
applies when it loads one (Environment/FSDSEnvironment.cpp), so a bad sidecar
fails here rather than at runtime, and it must sit next to its track's CSV.

Exit code:
  0 — every CSV and sidecar under the search root parses clean
  1 — at least one file failed validation; details on stderr
"""

from __future__ import annotations

import json
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
# Side test: |along| / |across| of the yellow->blue offset. The sim takes the
# gate axis more perpendicular to that offset, so it picks the wrong axis past
# 1.0 (45 deg). tan(35 deg) keeps 10 deg of margin; stress_circle_r5 sits at
# 0.61 (31 deg) because its nearest cones bend round the circle.
MAX_SIDE_SKEW = 0.70
# The K nearest cones' mean offset ahead of the gate picks forward. 0.5 m keeps
# the sign clear of cone-position noise; the stress circles sit at 0.68-0.81 m,
# the loops that once spawned backwards or by a few tenths sat at 0.33 m or less.
MIN_MEAN_AHEAD_M = 0.5
SET_MARGIN_M = 1.0       # a cone this close to joining the K nearest counts as able to swap in
SIDE_RADIUS_M = 8.0      # cones ahead of the gate within this radius decide which side blue is on
SIDE_NEAREST = 2         # nearest cones of each colour compared
MIN_SIDE_GAP_M = 1.0     # blue must sit at least this far left of yellow

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
        # No blue and yellow near the gate: the sim faces the car toward the mean of the K track cones nearest
        # the gate. Check that mean is clearly ahead, also for every set a
        # slightly farther cone could make by swapping in for one of them.
        by_dist = sorted(track, key=lambda c: c[3])
        nearest = by_dist[:K_DIRECTION]
        swaps = [c for c in by_dist[K_DIRECTION:] if c[3] - nearest[-1][3] < SET_MARGIN_M]
        sets = [nearest] + [nearest[:i] + nearest[i + 1:] + [c]
                            for c in swaps for i in range(len(nearest))]
        worst = min(sets, key=lambda s: sum(c[1] for c in s) / len(s))
        worst_mean = sum(c[1] for c in worst) / len(worst)
        if worst_mean < MIN_MEAN_AHEAD_M:
            offsets = ", ".join(f"{c[0]} {c[1]:+.2f}" for c in worst)
            errors.append(f"{where}: with no blue and yellow near the gate, the {K_DIRECTION} cones "
                          f"nearest it pick forward from "
                          f"their mean offset, which can fall to {worst_mean:+.2f} m "
                          f"(< {MIN_MEAN_AHEAD_M} m; along: {offsets}). Lay out the start zone "
                          f"as envgen/corridor.py does.")

    # Blue on the left: the nearest blue cones ahead of the gate sit left of
    # the nearest yellow ones. Compared with each other, not with the car's
    # heading, so a start on a tight curve (both lines bending one way) passes.
    ahead = sorted((c for c in track if c[1] >= 0.0 and c[3] <= SIDE_RADIUS_M), key=lambda c: c[3])
    blue = [c[2] for c in ahead if c[0] == "blue"][:SIDE_NEAREST]
    yellow = [c[2] for c in ahead if c[0] == "yellow"][:SIDE_NEAREST]
    if not blue or not yellow:
        errors.append(f"{where}: no blue and yellow cones within {SIDE_RADIUS_M:.0f} m ahead "
                      f"of the gate (blue {len(blue)}, yellow {len(yellow)})")
    elif sum(blue) / len(blue) - sum(yellow) / len(yellow) < MIN_SIDE_GAP_M:
        errors.append(f"{where}: blue is not on the car's left "
                      f"(lateral, + = left: nearest blue {[round(v, 2) for v in blue]}, "
                      f"nearest yellow {[round(v, 2) for v in yellow]}). FS has blue on the left.")
    return errors


ENV_FORMAT = "ifssim-env/1"
ENV_SUFFIX = ".env.json"
# Same bound as FSDSEnvironment::MaxCoordM: the ground extent reaches past the
# cones, so it is looser than MAX_COORD_M.
MAX_ENV_COORD_M = 1000.0


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


def _is_number(v) -> bool:
    # JSON true/false are not numbers (bool is an int subclass in Python).
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _unknown_keys(obj: dict, known: set, where: str) -> List[str]:
    return [f"{where}: unknown key {k!r}" for k in obj if k not in known]


def _coord(obj: dict, key: str, where: str) -> Tuple[float, List[str]]:
    v = obj.get(key)
    if not _is_number(v):
        return 0.0, [f"{where}: {key!r} must be a number"]
    if abs(v) > MAX_ENV_COORD_M:
        return 0.0, [f"{where}: {key!r} = {v} m is outside ±{MAX_ENV_COORD_M:.0f} m (cm instead of m?)"]
    return float(v), []


def validate_sidecar(path: Path) -> List[str]:
    """Errors for one <track>.env.json; empty list = clean. Mirrors
    FSDSEnvironment::Parse."""
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        return [f"{path}: not valid JSON: {e}"]
    if not isinstance(doc, dict):
        return [f"{path}: must be a JSON object"]

    errors = _unknown_keys(doc, {"format", "seed", "profile", "ground", "props", "world"}, f"{path}")
    if doc.get("format") != ENV_FORMAT:
        errors.append(f"{path}: 'format' must be {ENV_FORMAT!r} (got {doc.get('format')!r})")
    seed = doc.get("seed")
    if not _is_number(seed) or seed != int(seed) or seed < 0:
        errors.append(f"{path}: 'seed' must be a non-negative integer")
    if not isinstance(doc.get("profile"), str) or not doc.get("profile"):
        errors.append(f"{path}: 'profile' must be a non-empty string")

    if "ground" in doc:
        ground = doc["ground"]
        if not isinstance(ground, dict):
            errors.append(f"{path}: 'ground' must be an object")
        else:
            errors += _unknown_keys(ground, {"extent"}, f"{path}: ground")
            extent = ground.get("extent")
            if not isinstance(extent, dict):
                errors.append(f"{path}: ground: 'extent' must be an object")
            else:
                where = f"{path}: ground.extent"
                errors += _unknown_keys(extent, {"x_min", "y_min", "x_max", "y_max"}, where)
                vals = {}
                for k in ("x_min", "y_min", "x_max", "y_max"):
                    vals[k], errs = _coord(extent, k, where)
                    errors += errs
                if len(vals) == 4 and (vals["x_min"] >= vals["x_max"] or vals["y_min"] >= vals["y_max"]):
                    errors.append(f"{where}: min must be below max on both axes")

    if "props" in doc:
        props = doc["props"]
        if not isinstance(props, list):
            errors.append(f"{path}: 'props' must be an array")
        else:
            for i, prop in enumerate(props):
                where = f"{path}: props[{i}]"
                if not isinstance(prop, dict):
                    errors.append(f"{where}: must be an object")
                    continue
                errors += _unknown_keys(prop, {"class", "x", "y", "yaw_deg"}, where)
                if not isinstance(prop.get("class"), str) or not prop.get("class"):
                    errors.append(f"{where}: 'class' must be a non-empty string")
                for k in ("x", "y"):
                    errors += _coord(prop, k, where)[1]
                if "yaw_deg" in prop and not _is_number(prop["yaw_deg"]):
                    errors.append(f"{where}: 'yaw_deg' must be a number")

    if "world" in doc:
        errors += _validate_world_ref(path, doc["world"])

    track_csv = path.with_name(path.name[: -len(ENV_SUFFIX)] + ".csv")
    if not track_csv.is_file():
        errors.append(f"{path}: no track next to it ({track_csv.name}); the plugin loads a sidecar "
                      f"only with its track")
    return errors


def _validate_world_ref(path: Path, ref) -> List[str]:
    """The sidecar's "world": a name under worlds/ next to the track, and where
    its origin sits in the track frame. The world itself is checked too, by
    worldgen.world (the same rules the plugin applies)."""
    where = f"{path}: world"
    if not isinstance(ref, dict):
        return [f"{where}: must be an object"]
    errors = _unknown_keys(ref, {"name", "x", "y"}, where)
    name = ref.get("name")
    if not isinstance(name, str) or not name or not all(ch.isalnum() or ch == "_" for ch in name):
        return errors + [f"{where}: 'name' must be letters, digits and underscores"]
    for k in ("x", "y"):
        errors += _coord(ref, k, where)[1]
    world_dir = path.parent / "worlds" / name
    if not (world_dir / "world.json").is_file():
        return errors + [f"{where}: no world at {world_dir}"]
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    try:
        from worldgen import world as worldfmt
        worldfmt.load(world_dir)
    except Exception as e:  # WorldError, a bad terrain file, bad JSON
        errors.append(f"{where}: {world_dir}: {e}")
    finally:
        sys.path.pop(0)
    return errors


def find_csvs(root: Path) -> Iterable[Path]:
    return sorted(root.rglob("*.csv"))


def find_sidecars(root: Path) -> Iterable[Path]:
    return sorted(root.rglob("*" + ENV_SUFFIX))


def main(argv: List[str]) -> int:
    root = Path(argv[1]) if len(argv) > 1 else Path("Content/tracks")
    if not root.is_dir():
        print(f"track-validator: search root does not exist: {root}", file=sys.stderr)
        return 1

    paths = list(find_csvs(root))
    if not paths:
        print(f"track-validator: no CSV files under {root}", file=sys.stderr)
        return 1
    sidecars = list(find_sidecars(root))

    all_errors: List[Tuple[Path, List[str]]] = []
    for p in paths:
        errs = validate_one(p)
        if errs:
            all_errors.append((p, errs))
    for p in sidecars:
        errs = validate_sidecar(p)
        if errs:
            all_errors.append((p, errs))

    if all_errors:
        print(f"track-validator: {len(all_errors)} file(s) failed:", file=sys.stderr)
        for p, errs in all_errors:
            for e in errs:
                print(f"  {e}", file=sys.stderr)
        return 1

    print(f"track-validator: {len(paths)} CSV file(s) and {len(sidecars)} sidecar(s) clean")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
