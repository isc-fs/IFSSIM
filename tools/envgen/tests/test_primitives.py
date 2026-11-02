"""Tests for the stress-track library (tools/envgen/primitives.py). No sim.

    cd tools && python3 -m pytest envgen/tests -q
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOLS))

from envgen import build_track, create_track, parse_spec  # noqa: E402
from envgen.__main__ import main  # noqa: E402
from envgen.primitives import (  # noqa: E402
    FILE_PREFIX, LIBRARY, by_name, circle, hairpins, snail)

TRACKS = TOOLS.parent / "Content" / "tracks"


@pytest.mark.parametrize("stress", LIBRARY, ids=lambda s: s.name)
def test_every_stress_track_builds_clean(stress):
    """No errors: no fold, no crossing, and the sim spawns the car facing the
    way the shape is driven. FS-rule warnings are allowed, by design."""
    spec = stress.build()
    res = create_track(spec)
    assert res.ok, [i for i in res.issues if i["severity"] == "error"]


@pytest.mark.parametrize("stress", LIBRARY, ids=lambda s: s.name)
def test_committed_csv_matches_the_generator(stress):
    """Content/tracks/stress_*.csv is generated, never hand-edited: if the
    generator changes, regenerate with `python3 -m envgen stress -o ../Content/tracks`."""
    path = TRACKS / f"{FILE_PREFIX}{stress.name}.csv"
    assert path.exists(), f"{path.name} missing: regenerate the library"
    assert path.read_text() == create_track(stress.build()).csv


@pytest.mark.parametrize("stress", LIBRARY, ids=lambda s: s.name)
def test_sidecar_reopens_to_the_same_track(stress):
    path = TRACKS / f"{FILE_PREFIX}{stress.name}.track.json"
    spec = parse_spec(json.loads(path.read_text()))
    assert create_track(spec).csv == create_track(stress.build()).csv


def test_library_names_are_unique_and_match_their_specs():
    names = [s.name for s in LIBRARY]
    assert len(names) == len(set(names))
    for s in LIBRARY:
        assert s.build().to_dict()["closed"] in (True, False)
        assert by_name(s.name) is s
        assert by_name(FILE_PREFIX + s.name) is s


@pytest.mark.parametrize("radius", [5.0, 9.0, 15.0, 25.0])
@pytest.mark.parametrize("clockwise", [False, True])
def test_circle_holds_its_radius(radius, clockwise):
    t = build_track(circle(radius, clockwise=clockwise))
    for k in t.kappa:
        assert abs(1 / abs(k) - radius) / radius < 0.02
        assert (k < 0) == clockwise
    assert t.length == pytest.approx(2 * 3.141592653589793 * radius, rel=0.01)


def test_snail_tightens_through_its_spiral():
    """Sampled through the middle of the spiral, the radius only shrinks."""
    t = build_track(snail(20.0, 6.0, lead=20.0))
    # The spiral starts after the 20 m lead; sample its middle 60%.
    spiral = [1 / abs(k) for s, k in zip(t.s, t.kappa) if 30.0 <= s <= 55.0]
    assert spiral[0] > 14.0 and spiral[-1] < 10.0
    smooth = [sum(spiral[i:i + 8]) / 8 for i in range(0, len(spiral) - 8, 8)]
    assert all(a > b for a, b in zip(smooth, smooth[1:]))


def test_opening_snail_is_the_same_loop_driven_backwards():
    a = build_track(snail(20.0, 6.0))
    b = build_track(snail(20.0, 6.0, opening=True))
    assert a.length == pytest.approx(b.length, rel=0.01)
    assert a.spec.points[0] == b.spec.points[0]
    assert a.spec.points[1] == b.spec.points[-1]


def test_hairpins_needs_an_odd_count():
    with pytest.raises(ValueError):
        hairpins(4)


def test_cli_list_and_write(tmp_path, capsys):
    assert main(["stress", "--list"]) == 0
    assert "circle_r9" in capsys.readouterr().out
    assert main(["stress", "circle_r9", "-o", str(tmp_path)]) == 0
    assert (tmp_path / "stress_circle_r9.csv").exists()
    assert (tmp_path / "stress_circle_r9.track.json").exists()
    assert main(["stress", "no_such_track"]) == 2
