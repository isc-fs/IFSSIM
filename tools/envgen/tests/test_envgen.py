"""Tests for the track creator (tools/envgen). Pure Python, no sim.

    cd tools && python3 -m pytest envgen/tests -q
"""

from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(TOOLS / "mission_control" / "backend"))

from envgen import SpecError, create_track, parse_spec  # noqa: E402
from envgen.startpose import compute_start_pose, heading_error_deg  # noqa: E402
from validate_tracks import validate_one  # noqa: E402
from track_centering import bbox_center  # noqa: E402

EXAMPLES = TOOLS / "envgen" / "examples"


def spec(points, closed=True, **kw):
    d = {"format": "ifssim-trackspec/1", "closed": closed, "points": points}
    d.update(kw)
    return parse_spec(d)


def oval(rx=40.0, ry=25.0, n=12, clockwise=False):
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n
        a = -a if clockwise else a
        # start at the bottom of the oval, driving +x (counter-clockwise)
        pts.append([round(rx * math.sin(a), 3), round(-ry * math.cos(a), 3)])
    return pts


def codes(res, severity=None):
    return {i["code"] for i in res.issues if severity is None or i["severity"] == severity}


def tangent_at(track, s):
    n = len(track.centreline)
    i = int(round(s / track.length * n)) % n
    j = (i + 1) % n
    return (track.centreline[j][0] - track.centreline[i][0],
            track.centreline[j][1] - track.centreline[i][1]), track.centreline[i]


# --- the examples ----------------------------------------------------------

@pytest.mark.parametrize("path", sorted(EXAMPLES.glob("*.track.json")), ids=lambda p: p.name)
def test_examples_build_without_errors(path, tmp_path):
    res = create_track(parse_spec(json.loads(path.read_text())))
    assert res.ok, res.issues
    out = tmp_path / "t.csv"
    out.write_text(res.csv)
    assert validate_one(out) == []          # the CI gate accepts it


def test_same_spec_same_cones():
    d = json.loads((EXAMPLES / "stress_loop.track.json").read_text())
    assert create_track(parse_spec(d)).csv == create_track(parse_spec(d)).csv


def test_spec_round_trips():
    d = json.loads((EXAMPLES / "stress_loop.track.json").read_text())
    s1 = parse_spec(d)
    s2 = parse_spec(s1.to_dict())
    assert create_track(s1).csv == create_track(s2).csv


# --- conventions -----------------------------------------------------------

@pytest.mark.parametrize("clockwise", [False, True])
def test_blue_left_yellow_right(clockwise):
    res = create_track(spec(oval(clockwise=clockwise)))
    t = res.track
    for (c, x, y), s, side in zip(t.cones, t.cone_s, t.cone_side):
        if side.startswith("gate"):
            continue
        (tx, ty), (cx, cy) = tangent_at(t, s)
        left = tx * (y - cy) - ty * (x - cx) > 0
        assert (c == "blue") == left, (c, x, y, s)


def test_centred_like_mission_control(tmp_path):
    pts = [[p[0] + 60, p[1] - 30] for p in oval()]
    res = create_track(spec(pts))
    f = tmp_path / "t.csv"
    f.write_text(res.csv)
    cx, cy = bbox_center(str(f))
    assert abs(cx) < 0.01 and abs(cy) < 0.01   # ensure_centered won't rewrite it


# --- the start pose --------------------------------------------------------

def random_loop(rng):
    n = rng.randint(6, 14)
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n + rng.uniform(-0.15, 0.15)
        r = rng.uniform(25, 45)
        pts.append([r * math.cos(a), r * math.sin(a)])
    if rng.random() < 0.5:
        pts.reverse()
    k = rng.randrange(n)
    return pts[k:] + pts[:k]


def test_start_pose_points_the_drawn_way_on_random_loops():
    rng = random.Random(1234)
    checked = 0
    for _ in range(150):
        res = create_track(spec(random_loop(rng), width=rng.choice([3.0, 3.5, 4.5])))
        if {"fold", "crossing"} & codes(res, "error"):
            continue
        t = res.track
        pose = compute_start_pose(t.cones)
        tx = t.centreline[1][0] - t.centreline[0][0]
        ty = t.centreline[1][1] - t.centreline[0][1]
        assert heading_error_deg(pose, (tx, ty)) < 10.0
        assert "start_pose" not in codes(res)
        checked += 1
    assert checked > 100


def test_open_track_has_two_gates_and_starts_at_the_first():
    res = create_track(spec([[0, 0], [30, 0], [60, 5], [90, 0]], closed=False))
    pose = compute_start_pose(res.track.cones)
    assert pose.method == "multi-gate" and pose.gates == 2
    assert pose.forward[0] > 0.9
    assert sum(1 for c in res.track.cones if c[0] == "big_orange") == 8


def test_startpose_port_matches_acceleration():
    rows = []
    for line in (TOOLS.parent / "Content" / "tracks" / "acceleration.csv").read_text().splitlines():
        p = line.split(",")
        rows.append((p[0], float(p[1]), float(p[2])))
    pose = compute_start_pose(rows)
    # gates at x=3..7 and 78..82; the start gate is the one nearest (5, -1.5)
    assert pose.method == "multi-gate"
    assert pose.anchor == pytest.approx((5.0, 0.0))
    assert pose.forward == pytest.approx((1.0, 0.0))


# --- rules -----------------------------------------------------------------

def test_clean_oval_has_no_issues():
    res = create_track(spec(oval(rx=45, ry=30, n=16)))
    assert res.issues == []


def test_narrow_track_warns_and_strict_errors():
    s = spec(oval(), width=2.5)
    assert "width" in codes(create_track(s), "warn")
    assert "width" in codes(create_track(s, strict=True), "error")


def test_corner_tighter_than_half_width_folds():
    res = create_track(spec([[0, 0], [40, 0], [41, 1.5], [0, 3]], width=4.0))
    assert "fold" in codes(res, "error")


def test_figure_eight_crosses():
    res = create_track(spec([[0, 0], [30, 30], [60, 0], [30, -30], [-30, 30], [-60, 0], [-30, -30]]))
    assert "crossing" in codes(res, "error")


def test_long_straight_and_short_lap():
    # A spline through far-apart points bows; a straight is drawn as points in a line.
    long = create_track(spec([[0, 0], [30, 0], [60, 0], [90, 0], [120, 0], [150, 0],
                              [165, 15], [150, 30], [0, 30], [-15, 15]]))
    assert "straight" in codes(long, "warn")
    small = create_track(spec(oval(rx=15, ry=10)))
    assert "lap_length" in codes(small, "warn")


# --- sections --------------------------------------------------------------

def test_drop_all_removes_one_side_only_inside_the_section():
    base = create_track(spec(oval(n=12)))
    cut = create_track(spec(oval(n=12), sections=[{"from": 4, "to": 7, "drop": 1.0, "side": "left"}]))
    nb = lambda r: sum(1 for c in r.track.cones if c[0] == "blue")
    ny = lambda r: sum(1 for c in r.track.cones if c[0] == "yellow")
    assert nb(cut) < nb(base) - 5
    assert ny(cut) == ny(base)
    assert "spacing" in codes(cut, "warn")


def test_swap_and_start_zone_protected():
    # A section over the whole loop: everything is swapped except the start zone.
    res = create_track(spec(oval(n=12), sections=[{"from": 0, "to": 12, "recolor": "swap"}]))
    t = res.track
    for (c, x, y), s, side in zip(t.cones, t.cone_s, t.cone_side):
        if side.startswith("gate"):
            continue
        protected = s <= 3.5 + 1e-6
        expected = ("blue" if side == "left" else "yellow") if protected else \
                   ("yellow" if side == "left" else "blue")
        assert c == expected
    assert "start_pose" not in codes(res)


# --- the spec is strict ------------------------------------------------------

@pytest.mark.parametrize("bad, msg", [
    ({"widht": 3}, "unknown key"),
    ({"width": True}, "must be a number"),
    ({"width": "3"}, "must be a number"),
    ({"closed": 1}, "true or false"),
    ({"sections": [{"from": 0, "to": 99}]}, "not a stretch"),
    ({"sections": [{"from": 1, "to": 2, "recolor": "big_orange"}]}, "start pose"),
    ({"points": [[0, 0], [10, 0]]}, "at least 3"),
    ({"points": [[0, 0], [0, 0], [10, 5]]}, "same point"),
    ({"points": [[0, 0], [9000, 0], [10, 5]]}, "centimetres"),
    ({"format": "ifssim-trackspec/2"}, "spec.format"),
])
def test_spec_rejects(bad, msg):
    d = {"format": "ifssim-trackspec/1", "points": [[0, 0], [30, 0], [15, 20]]}
    d.update(bad)
    with pytest.raises(SpecError, match=msg):
        parse_spec(d)
