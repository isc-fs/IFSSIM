"""Track creator endpoint logic (track_creator.py), without FastAPI or the sim."""

from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import track_creator as tc  # noqa: E402
from track_centering import is_centered  # noqa: E402

BUILTINS = {"acceleration.csv": "acceleration", "skidpad.csv": "skidpad"}
LOOP = {"format": "ifssim-trackspec/1", "closed": True,
        "points": [[0, 0], [40, 0], [60, 20], [40, 40], [0, 40], [-20, 20]]}


def test_preview_returns_geometry_and_writes_nothing(tmp_path):
    code, body = tc.preview(LOOP)
    assert code == 200 and body["ok"]
    assert body["by_type"]["big_orange"] == 4
    assert {"cones", "centreline", "left", "right", "issues"} <= set(body)
    assert list(tmp_path.iterdir()) == []


def test_preview_reports_a_bad_spec():
    code, body = tc.preview({**LOOP, "widht": 3})
    assert code == 400 and "unknown key" in body["error"]


def test_create_writes_csv_and_spec(tmp_path):
    code, body = tc.create("my_loop", LOOP, str(tmp_path), BUILTINS)
    assert code == 200, body
    csv = tmp_path / "my_loop.csv"
    assert csv.exists() and (tmp_path / "my_loop.track.json").exists()
    assert is_centered(str(csv))        # loading won't rewrite it
    assert tc.load_spec("my_loop.csv", str(tmp_path))["points"] == LOOP["points"]
    assert tc.has_spec(str(csv))


def test_create_refuses_overwrite_builtins_and_bad_names(tmp_path):
    assert tc.create("t", LOOP, str(tmp_path), BUILTINS)[0] == 200
    assert tc.create("t", LOOP, str(tmp_path), BUILTINS)[0] == 409
    assert tc.create("t", LOOP, str(tmp_path), BUILTINS, overwrite=True)[0] == 200
    assert tc.create("acceleration", LOOP, str(tmp_path), BUILTINS)[0] == 400
    assert tc.create("../escape", LOOP, str(tmp_path), BUILTINS)[0] == 400


def test_create_refuses_a_broken_track(tmp_path):
    crossing = {**LOOP, "points": [[0, 0], [30, 30], [60, 0], [30, -30],
                                   [-30, 30], [-60, 0], [-30, -30]]}
    code, body = tc.create("x", crossing, str(tmp_path), BUILTINS)
    assert code == 422
    assert any(i["code"] == "crossing" for i in body["issues"])
    assert not (tmp_path / "x.csv").exists()


def test_strict_turns_rule_breaches_into_refusals(tmp_path):
    narrow = {**LOOP, "width": 2.5}
    assert tc.create("n", narrow, str(tmp_path), BUILTINS)[0] == 200
    assert tc.create("n2", narrow, str(tmp_path), BUILTINS, strict=True)[0] == 422


def test_spec_of_a_track_that_was_not_drawn(tmp_path):
    (tmp_path / "plain.csv").write_text("blue,0,0\n")
    assert tc.load_spec("plain.csv", str(tmp_path)) is None
    assert json.loads(json.dumps(LOOP))   # sanity: LOOP is plain JSON
