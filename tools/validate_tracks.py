#!/usr/bin/env python3
"""
Validate every track CSV under Content/tracks/ — runs as a CI gate.

A track CSV is a simple text file: each line is `cone_type,x,y` (in
metres, ENU). cone_type ∈ {blue, yellow, big_orange, small_orange}.
Anything malformed (missing fields, non-numeric coordinates, unknown
cone type, blank file) is a regression that would silently break
loadTrack at runtime.

Exit code:
  0 — every CSV under the search root parses clean
  1 — at least one file failed validation; details on stderr
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Iterable, List, Tuple


VALID_CONE_TYPES = frozenset({"blue", "yellow", "big_orange", "small_orange"})

# Sanity bounds on cone coordinates — tracks larger than 1 km square
# are almost certainly a unit-confusion bug (cm vs m). Tighten if real
# tracks ever push closer to this.
MAX_COORD_M = 500.0


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
