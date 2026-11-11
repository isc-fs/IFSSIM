"""Venues (envgen/venue.py): a fixed place several tracks share.

Run from tools/: python -m pytest envgen/tests -q
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[2]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))

pytest.importorskip("shapely")

from envgen import track as trackmod  # noqa: E402
from envgen.catalogue import CLASSES  # noqa: E402
from envgen.generate import dumps, footprint  # noqa: E402
from envgen.venue import (VenueError, build, check_track, check_venue, load_venue,  # noqa: E402
                          parse_venue, sidecar, track_offset)
import validate_tracks as vt  # noqa: E402

TRACKS_DIR = REPO / "Content/tracks"
PARKING_A = TRACKS_DIR / "venues/parking_a.venue.json"

LOT = [[-20, -15], [20, -15], [20, 15], [-20, 15]]
SMALL = {"format": "ifssim-venue/1", "name": "test_lot", "lot": LOT,
         "props": [{"class": "tree", "x": 0, "y": 20}]}
INSIDE = {"format": "ifssim-trackspec/1", "closed": True,
          "points": [[-12, -8], [12, -8], [14, 6], [-12, 8]]}


def test_parking_a_is_a_clean_venue():
    v = load_venue(PARKING_A)
    assert check_venue(v) == []
    classes = {p["class"] for p in v.props}
    assert {"tree", "lamp_post", "parked_car", "bench", "sidewalk", "berm"} <= classes


def test_its_tracks_sidecars_are_up_to_date():
    # The committed sidecars are what `envgen venue` writes: regenerate after
    # changing the venue or a track.
    v = load_venue(PARKING_A)
    tracks = sorted(TRACKS_DIR.glob("parking_a__*.csv"))
    assert tracks, "parking_a has no track"
    for csv in tracks:
        spec = json.loads(csv.with_suffix(".track.json").read_text())
        assert check_track(v, spec) == []
        offset, _ = track_offset(spec)
        expected = dumps(sidecar(v, offset, trackmod.load(csv), int(spec.get("seed", 1))))
        assert csv.with_name(csv.stem + ".env.json").read_text() == expected, \
            f"{csv.name}: run python -m envgen venue {PARKING_A.relative_to(REPO)}"


def test_rows_expand_and_face_along_themselves():
    v = parse_venue({**SMALL, "props": [], "rows": [
        {"class": "sidewalk", "from": [-20, 17], "to": [20, 17], "spacing": 2.0}]})
    assert len(v.props) == 21
    assert v.props[0]["x"] == -20 and v.props[-1]["x"] == 20
    assert all(p["yaw_deg"] == 0 for p in v.props)


def test_the_sidecar_moves_the_venue_with_the_track(tmp_path):
    v = parse_venue(SMALL)
    (ox, oy), created = track_offset(INSIDE)
    env = sidecar(v, (ox, oy))
    assert env["props"][0]["x"] == pytest.approx(0 + ox, abs=1e-3) and env["props"][0]["y"] == pytest.approx(20 + oy, abs=1e-3)   # stored to the mm
    (tmp_path / "t.csv").write_text(created.csv)   # the validator wants the track beside it
    path = tmp_path / "t.env.json"
    path.write_text(dumps(env))
    assert vt.validate_sidecar(path) == []


def test_a_track_gets_clutter_near_its_cones_and_on_the_lot(tmp_path):
    v = parse_venue(SMALL)
    offset, created = track_offset(INSIDE)
    csv = tmp_path / "t.csv"
    csv.write_text(created.csv)
    tr = trackmod.load(csv)
    env = sidecar(v, offset, tr, seed=4)
    own = env["props"][len(v.props):]
    assert env["props"][:len(v.props)] == sidecar(v, offset)["props"]   # the venue's come first
    assert own and {p["class"] for p in own} <= {"tyre_wall", "bollard", "bin", "cone_stack", "tripod"}
    for p in own:   # on the lot, in the venue's frame
        fp = footprint(CLASSES[p["class"]], p["x"] - offset[0], p["y"] - offset[1], p["yaw_deg"])
        assert v.lot.contains(fp), p
    assert sidecar(v, offset, tr, seed=4) == env                # deterministic
    assert sidecar(v, offset, tr, seed=5)["props"] != env["props"]
    (tmp_path / "t.env.json").write_text(dumps(env))
    assert vt.validate_sidecar(tmp_path / "t.env.json") == []


@pytest.mark.parametrize("bad, why", [
    ({"props": [{"class": "tree", "x": 0, "y": 0}]}, "stands on the lot"),
    ({"props": [{"class": "tree", "x": 0, "y": 20}, {"class": "bench", "x": 0.5, "y": 20}]}, "overlap"),
])
def test_venue_mistakes_are_caught(bad, why):
    assert any(why in e for e in check_venue(parse_venue({**SMALL, **bad})))


@pytest.mark.parametrize("bad", [
    {"props": [{"class": "rocket", "x": 0, "y": 30}]},          # unknown class
    {"props": [{"class": "tree", "x": 0, "y": 30, "yaw": 1}]},   # misspelt key
    {"lot": [[0, 0], [1, 1]]},                                     # not a polygon
    {"name": "has__separator"},
    {"props": [{"class": "tree", "x": 3000, "y": 0}]},             # cm for m
])
def test_malformed_venues_are_refused(bad):
    with pytest.raises(VenueError):
        parse_venue({**SMALL, **bad})


def test_a_track_off_the_lot_is_refused():
    off = copy.deepcopy(INSIDE)
    off["points"][1] = [30, -8]
    assert any("off the lot" in e for e in check_track(parse_venue(SMALL), off))


def test_a_venue_track_needs_its_drawing(tmp_path):
    venue_dir = tmp_path / "venues"
    venue_dir.mkdir()
    (venue_dir / "test_lot.venue.json").write_text(json.dumps(SMALL))
    (tmp_path / "test_lot__a.csv").write_text("blue,0,0,0,0,0,0\n")
    assert build(venue_dir / "test_lot.venue.json", log=lambda *_: None) == 1
    (tmp_path / "test_lot__a.track.json").write_text(json.dumps(INSIDE))
    (tmp_path / "test_lot__a.csv").write_text(track_offset(INSIDE)[1].csv)   # the drawing, built
    assert build(venue_dir / "test_lot.venue.json", log=lambda *_: None) == 0
    assert (tmp_path / "test_lot__a.env.json").exists()
