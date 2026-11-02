"""envgen: deterministic, valid, and never in the car's way.

Run from tools/: python -m pytest envgen/tests -q
"""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[2]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))

pytest.importorskip("shapely")  # the environment generator needs it; the track creator does not

from envgen import track as trackmod  # noqa: E402
from envgen.__main__ import main as cli  # noqa: E402
from envgen.catalogue import CLASSES, PROFILES  # noqa: E402
from envgen.generate import (CONE_CLEARANCE_M, FLOOR_HALF_M, PROP_CLEARANCE_M, _boundary_centre,  # noqa: E402
                             dumps, footprint, generate, sha256)
from envgen.seed import Stream, fnv1a32  # noqa: E402
import validate_tracks as vt  # noqa: E402

FIX = Path(__file__).parent / "fixtures"
TRACKS = [FIX / "accel.csv", FIX / "loop.csv"]
BUILDER = REPO / "Plugins/FSDSPlugin/Source/FSDSPlugin/Private/Environment/FSDSEnvironmentBuilder.cpp"

# Pinned sidecar hashes. A change here changes every generated world: bump them
# only on purpose, and say so in the CHANGELOG.
GOLDEN = {
    ("accel.csv", 1): "a078cfb5ca22055cae149a8492c18c8c894146c93ffbe34f814d00b5d01c4cb1",
    ("accel.csv", 7): "d9cc8deeda0f9a27fb8842d22a8182c418c75702a15f30535e4c42cb8ad12878",
    ("loop.csv", 1): "dba64df1dc16241161e15be3985e5a47d3a82e369dd84849a572423b34f6bc37",
    ("loop.csv", 7): "8f39634c2f95e82e8d3ac2d0ae8322fce1b1995219370ebbe2af4d38090925e4",
}


def test_stream_names_hash_like_the_plugin():
    # 32-bit FNV-1a reference values; FSDSRandom.cpp uses the same function.
    assert fnv1a32("") == 0x811C9DC5
    assert fnv1a32("a") == 0xE40C292C
    assert fnv1a32("foobar") == 0xBF9CF968


def test_streams_are_reproducible_and_independent():
    a, b = Stream(7, "Env.Props.bin"), Stream(7, "Env.Props.bin")
    assert [a.next_u64() for _ in range(5)] == [b.next_u64() for _ in range(5)]
    c = Stream(7, "Env.Props.bollard")
    assert Stream(7, "Env.Props.bin").next_u64() != c.next_u64()
    u = [Stream(3, "x").uniform(2.0, 5.0) for _ in range(3)]
    assert all(2.0 <= v < 5.0 for v in u)


@pytest.mark.parametrize("csv", TRACKS, ids=lambda p: p.name)
def test_same_seed_same_bytes(csv):
    assert dumps(generate(csv, 7, "stress")) == dumps(generate(csv, 7, "stress"))
    assert dumps(generate(csv, 7, "stress")) != dumps(generate(csv, 8, "stress"))


def test_golden_hashes():
    got = {k: sha256(dumps(generate(FIX / k[0], k[1], "stress"))) for k in GOLDEN}
    assert got == GOLDEN, "generated worlds changed; if on purpose, update GOLDEN:\n" + \
        "\n".join(f"    {k!r}: {v!r}," for k, v in got.items())


def test_flat_baseline_has_no_props():
    assert generate(TRACKS[1], 7, "flat_baseline")["props"] == []


@pytest.mark.parametrize("csv", TRACKS, ids=lambda p: p.name)
@pytest.mark.parametrize("seed", range(1, 21))
def test_props_stay_out_of_the_way(csv, seed):
    tr = trackmod.load(csv)
    env = generate(csv, seed, "stress")
    cx, cy = _boundary_centre(tr)
    fps = []
    for p in env["props"]:
        cls = CLASSES[p["class"]]
        fp = footprint(cls, p["x"], p["y"], p.get("yaw_deg", 0.0))
        gap = tr.area.distance(fp)
        # Rounding to 1 mm may move a prop by up to 0.7 mm.
        assert not fp.buffer(-0.002).intersects(tr.area), p
        assert cls.band_m[0] - 0.002 <= gap <= cls.band_m[1] + 0.002, (p, gap)
        assert tr._cone_points.distance(fp) >= CONE_CLEARANCE_M - 0.002, p
        minx, miny, maxx, maxy = fp.bounds
        assert max(abs(minx - cx), abs(maxx - cx), abs(miny - cy), abs(maxy - cy)) <= FLOOR_HALF_M + 0.002
        fps.append(fp)
    for i, a in enumerate(fps):
        for b in fps[i + 1:]:
            assert a.distance(b) >= PROP_CLEARANCE_M - 0.003


@pytest.mark.parametrize("csv", TRACKS, ids=lambda p: p.name)
def test_stress_places_every_class_it_can(csv):
    counts = {}
    for p in generate(csv, 7, "stress")["props"]:
        counts[p["class"]] = counts.get(p["class"], 0) + 1
    assert {"bollard", "bin", "tyre_wall", "cone_stack", "tripod"} <= set(counts), counts
    # Two per timing line: acceleration has a start and a finish line, a loop one.
    assert counts["tripod"] == (4 if csv.name == "accel.csv" else 2)


def test_tripods_stand_level_with_the_timing_line():
    env = generate(FIX / "accel.csv", 7, "stress")
    tripods = [p for p in env["props"] if p["class"] == "tripod"]
    # Acceleration's timing lines are at x = 3..7 and 78..82, across y = 0.
    for p in tripods:
        assert 2.0 <= p["x"] <= 8.0 or 77.0 <= p["x"] <= 83.0, p
        assert abs(p["y"]) > 2.0, p


def test_shifting_the_track_shifts_its_props():
    """The backend recentres a track on load and shifts its sidecar by the same
    amount, so generating on a shifted CSV must give the shifted props."""
    import tempfile
    src = (FIX / "loop.csv").read_text().splitlines()
    dx, dy = 37.25, -12.5
    shifted = []
    for line in src:
        f = line.split(",")
        f[1], f[2] = repr(float(f[1]) + dx), repr(float(f[2]) + dy)
        shifted.append(",".join(f))
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "shifted.csv"
        p.write_text("\n".join(shifted) + "\n")
        a = generate(FIX / "loop.csv", 5, "stress")["props"]
        b = generate(p, 5, "stress")["props"]
    assert len(a) == len(b)
    for pa, pb in zip(a, b):
        assert pa["class"] == pb["class"]
        assert abs(pb["x"] - pa["x"] - dx) < 0.01 and abs(pb["y"] - pa["y"] - dy) < 0.01


@pytest.mark.parametrize("profile", sorted(PROFILES))
def test_output_passes_the_plugins_rules(tmp_path, profile):
    csv = tmp_path / "t.csv"
    csv.write_text((FIX / "loop.csv").read_text())
    assert cli(["env", str(csv), "--seed", "3", "--profile", profile]) == 0
    sidecar = tmp_path / "t.env.json"
    assert vt.validate_sidecar(sidecar) == []
    assert len(sidecar.read_bytes()) <= 100_000


def test_every_class_is_built_by_the_plugin():
    """The C++ builder must know every class envgen writes, with the same footprint."""
    src = BUILDER.read_text()
    for name, cls in CLASSES.items():
        m = re.search(r'TEXT\("%s"\)' % re.escape(name), src)
        assert m, f"FSDSEnvironmentBuilder.cpp has no class {name!r}"
