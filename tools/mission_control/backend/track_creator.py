"""Track creator endpoints' logic: a drawn spec in, a track CSV out.

The geometry lives in tools/envgen (shared with the CLI and, later, the
environment generator). This module is the backend's thin layer over it:
validate the request, build, refuse broken tracks, write the CSV and its spec
side by side. Pure functions, so tests/test_track_creator.py exercises them
without FastAPI or the sim -- the same split as track_validators.py.

Files, both in TRACKS_DIR:
    <name>.csv          the cones, what the sim loads
    <name>.track.json   the spec it was built from, so it can be re-edited
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any, Dict, Optional, Tuple

from benchmark_control_runner import _tools_dir
from track_validators import TrackNameError, protected_reason, resolve_track_path, validate_track_stem

SPEC_SUFFIX = ".track.json"


def _envgen():
    """Import tools/envgen (repo layout locally, /ifssim_tools in Docker)."""
    tools = str(_tools_dir())
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import envgen  # noqa: WPS433 -- resolved at call time, see above
    return envgen


def spec_path(csv_path: str) -> str:
    return csv_path[:-4] + SPEC_SUFFIX if csv_path.endswith(".csv") else csv_path + SPEC_SUFFIX


def _geometry(created) -> Dict[str, Any]:
    from envgen.startpose import compute_start_pose
    t = created.track
    r = lambda pts: [[round(x, 3), round(y, 3)] for x, y in pts]  # noqa: E731
    pose = compute_start_pose(t.cones)
    return {
        # where the sim will put the car: 3 m behind the gate, facing `forward`
        "start": None if pose is None else {
            "spawn": [round(v, 3) for v in pose.spawn],
            "forward": [round(v, 4) for v in pose.forward],
        },
        "cones": [{"type": c, "x": round(x, 3), "y": round(y, 3)} for c, x, y in t.cones],
        "centreline": r(t.centreline[::4]),   # 1 m resolution is plenty to draw
        "left": r(t.left[::4]),
        "right": r(t.right[::4]),
    }


def _venue(stem: str, tracks_dir: str, venue: Optional[str] = None):
    """The venue a track is drawn on: named, or from a `<venue>__` name. None if neither."""
    _envgen()
    from envgen import venue as ev   # shapely, which the env tools need too
    if venue:
        path = ev.venue_path(validate_track_stem(venue), tracks_dir)
        if not path.exists():
            raise TrackNameError(f"no venue {venue} (no {path.name} in venues/)")
        return ev.load_venue(path)
    return ev.venue_of(stem, tracks_dir) if stem else None


def _add_lot_issues(created, v) -> None:
    """A venue's lot is part of the rules: cones off it are errors, like a fold."""
    from envgen import venue as ev
    created.issues.extend(ev.lot_issues(v, created))


def preview(spec: Any, strict: bool = False, venue: Optional[str] = None,
            tracks_dir: str = "") -> Tuple[int, Dict[str, Any]]:
    """Build without writing: what the editor draws while you draw.

    With `venue`, the cones must also stay on that venue's lot.
    """
    eg = _envgen()
    try:
        created = eg.create_track(eg.parse_spec(spec), strict=strict)
        v = _venue("", tracks_dir, venue) if venue else None
    except (eg.SpecError, TrackNameError) as e:
        return 400, {"error": str(e)}
    if v is not None:
        _add_lot_issues(created, v)
    return 200, {**created.summary(), **_geometry(created)}


def create(name: str, spec: Any, tracks_dir: str, builtins, overwrite: bool = False,
           strict: bool = False) -> Tuple[int, Dict[str, Any]]:
    """Build and save `<name>.csv` + `<name>.track.json`. Returns (status, body)."""
    eg = _envgen()
    try:
        stem = validate_track_stem(name.strip())
    except TrackNameError as e:
        return 400, {"error": str(e)}
    csv_name = f"{stem}.csv"
    reason = protected_reason(csv_name, builtins)
    if reason:
        return 400, {"error": reason}
    try:
        csv_path = resolve_track_path(csv_name, tracks_dir)
    except TrackNameError as e:
        return 400, {"error": str(e)}
    if os.path.exists(csv_path) and not overwrite:
        return 409, {"error": f"{csv_name} already exists (overwrite to replace it)"}

    try:
        parsed = eg.parse_spec(spec)
    except eg.SpecError as e:
        return 400, {"error": str(e)}
    created = eg.create_track(parsed, strict=strict)
    # A `<venue>__<track>` is drawn on that venue: it must stay on the lot,
    # and gets the venue's props as its environment sidecar (docs/venues.md).
    try:
        v = _venue(stem, tracks_dir)
    except Exception as e:   # a broken venue file: say so rather than save without it
        return 400, {"error": f"the venue for {csv_name} can't be read: {e}"}
    if v is not None:
        _add_lot_issues(created, v)
    body = {"name": csv_name, **created.summary()}
    if v is not None:
        body["venue"] = v.name
    if not created.ok:
        return 422, {**body, "error": "the track has errors and was not saved"}

    os.makedirs(tracks_dir, exist_ok=True)
    for path, text in ((csv_path, created.csv),
                       (spec_path(csv_path), json.dumps(parsed.to_dict(), indent=2) + "\n")):
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            f.write(text)
        os.replace(tmp, path)
    if v is not None:
        from envgen import venue as ev
        ev.write_sidecar(v, csv_path, created.track.offset)
    return 200, body


def remove_venue_sidecar(csv_path: str) -> None:
    """A deleted venue track takes its generated sidecar with it."""
    stem = os.path.basename(csv_path)[:-4]
    _envgen()
    from envgen import venue as ev
    if ev.SEP in stem:
        side = ev.sidecar_path(csv_path)
        if side.exists():
            side.unlink()


def load_spec(name: str, tracks_dir: str) -> Optional[Dict[str, Any]]:
    """The spec a track was drawn from, or None for a track that wasn't drawn."""
    path = spec_path(resolve_track_path(name, tracks_dir))
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def has_spec(csv_path: str) -> bool:
    return os.path.exists(spec_path(csv_path))
