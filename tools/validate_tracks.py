#!/usr/bin/env python3
"""
Validate every track CSV, and every environment sidecar, under Content/tracks/
— runs as a CI gate.

A track CSV is a simple text file: each line is `cone_type,x,y` (in
metres, ENU). cone_type ∈ {blue, yellow, big_orange, small_orange}.
Anything malformed (missing fields, non-numeric coordinates, unknown
cone type, blank file) is a regression that would silently break
loadTrack at runtime.

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
from typing import Iterable, List, Tuple


VALID_CONE_TYPES = frozenset({"blue", "yellow", "big_orange", "small_orange"})

# Sanity bounds on cone coordinates — tracks larger than 1 km square
# are almost certainly a unit-confusion bug (cm vs m). Tighten if real
# tracks ever push closer to this.
MAX_COORD_M = 500.0

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

    if cone_count == 0:
        errors.append(f"{path}: no valid cone rows parsed")
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

    errors = _unknown_keys(doc, {"format", "seed", "profile", "ground", "props"}, f"{path}")
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

    track_csv = path.with_name(path.name[: -len(ENV_SUFFIX)] + ".csv")
    if not track_csv.is_file():
        errors.append(f"{path}: no track next to it ({track_csv.name}); the plugin loads a sidecar "
                      f"only with its track")
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
