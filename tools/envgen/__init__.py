"""envgen: tracks and their environments, generated from data.

The track creator: a drawn path (a TrackSpec) becomes the cone CSV the sim
loads, checked against the FS rules and against where the sim will spawn the
car.

    from envgen import parse_spec, create_track
    result = create_track(parse_spec(json.load(open("my.track.json"))))
    result.csv, result.issues

The environment generator (docs/environment_sidecar.md,
docs/ENVIRONMENT_ROADMAP.md P2a): a track's trackside props, from a seed and a
profile, written as the <track>.env.json sidecar.

    cd tools && python3 -m envgen env ../Content/tracks/<track>.csv --seed 7 --profile stress

The track creator is standard library only, so it runs in CI, in the Mission
Control backend image and from a plain shell alike. The environment generator
(envgen.generate, envgen.track) needs shapely; importing envgen does not.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from .corridor import Track, build_track, to_csv
from .rules import check, has_errors
from .spec import FORMAT, SpecError, TrackSpec, parse_spec

__all__ = ["FORMAT", "SpecError", "TrackSpec", "Track", "Created",
           "parse_spec", "build_track", "create_track", "to_csv", "check", "has_errors"]


@dataclass
class Created:
    track: Track
    csv: str
    issues: List[Dict]

    @property
    def ok(self) -> bool:
        return not has_errors(self.issues)

    def summary(self) -> Dict:
        counts: Dict[str, int] = {}
        for t, _, _ in self.track.cones:
            counts[t] = counts.get(t, 0) + 1
        return {
            "length_m": round(self.track.length, 1),
            "closed": self.track.spec.closed,
            "cones": len(self.track.cones),
            "by_type": counts,
            "offset": [round(v, 3) for v in self.track.offset],
            "ok": self.ok,
            "issues": self.issues,
        }


def create_track(spec: TrackSpec, strict: bool = False) -> Created:
    """Build a spec into cones, check it, and render the CSV."""
    track = build_track(spec)
    issues = check(track, strict=strict)
    return Created(track=track, csv=to_csv(track), issues=issues)
