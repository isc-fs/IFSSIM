"""The clutter experiment's tracks (roadmap P2a reproduction).

One base track, copied once per (stressor, environment seed), each copy with its
own sidecar: clutter_<kind>_e<seed>.csv + .env.json. The cones are byte-identical
to the base track, so the only difference between two runs with the same
scenario seed is the props. clutter_control has the same layout, the same seeds
and the environment loaded, but no props: the experiment's control.

    cd tools
    python3 -m envgen clutter                      # check the committed files, write nothing
    python3 -m envgen clutter -o ../Content/tracks # regenerate them

Never edit those files by hand; tests/test_clutter.py fails if a committed one
no longer matches. The runs: tools/sim_benchmark/specs/clutter-*.yaml.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

from .generate import dumps, generate

BASE_TRACK = "track_20260404_013721.csv"   # autocross: one lap, a finish line the stop latch must find
KINDS = ("control", "narrow", "tripod", "tyrewall")
SEEDS = (1, 2, 3, 4, 5)


def name(kind: str, seed: int) -> str:
    return f"clutter_{kind}_e{seed}"


def build(tracks_dir: str | Path) -> Dict[str, bytes]:
    """{file name: bytes} of every clutter track and sidecar, from the base track in tracks_dir."""
    tracks_dir = Path(tracks_dir)
    base = tracks_dir / BASE_TRACK
    csv = base.read_bytes()
    out: Dict[str, bytes] = {}
    for kind in KINDS:
        for seed in SEEDS:
            n = name(kind, seed)
            out[f"{n}.csv"] = csv
            out[f"{n}.env.json"] = dumps(generate(base, seed, f"clutter_{kind}")).encode()
    return out


def check(tracks_dir: str | Path) -> List[Tuple[str, str]]:
    """(file, problem) for each committed file that is missing or differs."""
    tracks_dir = Path(tracks_dir)
    problems = []
    for fn, data in build(tracks_dir).items():
        p = tracks_dir / fn
        if not p.exists():
            problems.append((fn, "missing"))
        elif p.read_bytes().replace(b"\r\n", b"\n") != data.replace(b"\r\n", b"\n"):
            problems.append((fn, "differs from the generator"))
    return problems


def tracks(kind: str) -> str:
    """The comma-separated --tracks value for one kind (a spec's settings.tracks)."""
    return ",".join(f"{name(kind, s)}.csv" for s in SEEDS)
