"""The clutter experiment's committed tracks are the generator's, and differ only in props."""
import json
from pathlib import Path

from envgen import clutter
from envgen.catalogue import CLASSES

TRACKS = Path(__file__).resolve().parents[3] / "Content" / "tracks"


def test_the_committed_clutter_tracks_match_the_generator():
    assert clutter.check(TRACKS) == []


def test_every_copy_has_the_base_tracks_cones():
    base = (TRACKS / clutter.BASE_TRACK).read_bytes()
    for kind in clutter.KINDS:
        for seed in clutter.SEEDS:
            assert (TRACKS / f"{clutter.name(kind, seed)}.csv").read_bytes() == base


def test_each_kind_holds_only_its_stressor():
    want = {"control": set(), "narrow": {"bollard", "bin"}, "tripod": {"tripod"}, "tyrewall": {"tyre_wall"}}
    for kind in clutter.KINDS:
        for seed in clutter.SEEDS:
            env = json.loads((TRACKS / f"{clutter.name(kind, seed)}.env.json").read_text())
            assert env["profile"] == f"clutter_{kind}" and env["seed"] == seed
            classes = {p["class"] for p in env["props"]}
            assert classes == want[kind], (kind, seed, classes)
            assert all(c in CLASSES for c in classes)


def test_the_seeds_place_props_differently():
    sigs = {json.dumps(json.loads((TRACKS / f"{clutter.name('narrow', s)}.env.json").read_text())["props"])
            for s in clutter.SEEDS}
    assert len(sigs) == len(clutter.SEEDS)
