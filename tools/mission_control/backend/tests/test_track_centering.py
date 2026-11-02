"""Recentring a track moves its environment sidecar with it."""

import json

from track_centering import bbox_center, ensure_centered, shift_sidecar

CSV = "blue,10.0,0.0,0,0,0,0\nyellow,10.0,4.0,0,0,0,0\nblue,30.0,0.0,0,0,0,0\nyellow,30.0,4.0,0,0,0,0\n"


def _env():
    return {"format": "ifssim-env/1", "seed": 3, "profile": "stress",
            "ground": {"extent": {"x_min": 0.0, "y_min": -10.0, "x_max": 40.0, "y_max": 14.0}},
            "props": [{"class": "bollard", "x": 12.5, "y": -2.0, "yaw_deg": 30.0}]}


def test_recentring_moves_the_sidecar(tmp_path):
    csv = tmp_path / "t.csv"
    csv.write_text(CSV)
    (tmp_path / "t.env.json").write_text(json.dumps(_env()))
    assert ensure_centered(str(csv))
    assert bbox_center(str(csv)) == (0.0, 0.0)
    env = json.loads((tmp_path / "t.env.json").read_text())
    # The boundary centre was (20, 2).
    assert env["props"][0] == {"class": "bollard", "x": -7.5, "y": -4.0, "yaw_deg": 30.0}
    assert env["ground"]["extent"] == {"x_min": -20.0, "y_min": -12.0, "x_max": 20.0, "y_max": 12.0}
    assert (env["seed"], env["profile"]) == (3, "stress")


def test_a_centred_track_leaves_its_sidecar_alone(tmp_path):
    csv = tmp_path / "t.csv"
    csv.write_text("blue,-10.0,-2.0\nyellow,10.0,2.0\n")
    side = tmp_path / "t.env.json"
    side.write_text(json.dumps(_env()))
    before = side.read_text()
    assert not ensure_centered(str(csv))
    assert side.read_text() == before


def test_no_sidecar_or_a_broken_one(tmp_path):
    assert not shift_sidecar(str(tmp_path / "missing.env.json"), 1.0, 1.0)
    bad = tmp_path / "bad.env.json"
    bad.write_text("{not json")
    assert not shift_sidecar(str(bad), 1.0, 1.0)
    assert bad.read_text() == "{not json"
