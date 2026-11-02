"""Tests for tools/validate_tracks.py: start-pose checks and sidecar checks
(docs/environment_sidecar.md).

The cases mirror the plugin's FSDS.Environment.Reject automation test, so the CI
gate and the runtime loader reject the same sidecars.

    python -m unittest discover -s tools/tests -v
"""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import validate_tracks as vt  # noqa: E402

FIXTURES = REPO / "tools" / "smoke" / "fixtures"

VALID = {
    "format": "ifssim-env/1",
    "seed": 7,
    "profile": "stress",
    "ground": {"extent": {"x_min": -60, "y_min": -40.5, "x_max": 160, "y_max": 140}},
    "props": [
        {"class": "bollard", "x": 12.0, "y": -3.5, "yaw_deg": 30},
        {"class": "tripod", "x": 40.0, "y": 2.25},
    ],
}
# One start gate with blue on the left, laid out like envgen/corridor.py's
# start zone, so it also passes the start-pose check.
CSV = ("big_orange,0,2\nbig_orange,0,-2\nbig_orange,0.5,2\nbig_orange,0.5,-2\n"
       "blue,2,1.75\nyellow,2,-1.75\nblue,3.5,1.75\nyellow,3.5,-1.75\n")


class SidecarTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root / "t.csv").write_text(CSV)

    def tearDown(self):
        self.tmp.cleanup()

    def check(self, doc, name="t.env.json") -> list:
        path = self.root / name
        path.write_text(doc if isinstance(doc, str) else json.dumps(doc))
        return vt.validate_sidecar(path)

    def test_valid_sidecar_is_clean(self):
        self.assertEqual(self.check(VALID), [])

    def test_minimal_sidecar_is_clean(self):
        self.assertEqual(self.check({"format": "ifssim-env/1", "seed": 0, "profile": "flat_baseline"}), [])

    def test_rejections_match_the_plugin(self):
        base = {"format": "ifssim-env/1", "seed": 1, "profile": "p"}
        cases = [
            ("not JSON", "props: []", "not valid JSON"),
            ("wrong format", {**base, "format": "ifssim-env/2"}, "'format'"),
            ("missing seed", {k: v for k, v in base.items() if k != "seed"}, "'seed'"),
            ("fractional seed", {**base, "seed": 1.5}, "'seed'"),
            ("string seed", {**base, "seed": "1"}, "'seed'"),
            ("empty profile", {**base, "profile": ""}, "'profile'"),
            ("numeric profile", {**base, "profile": 5}, "'profile'"),
            ("misspelt top-level key", {**base, "prop": []}, "unknown key 'prop'"),
            ("inverted extent",
             {**base, "ground": {"extent": {"x_min": 5, "y_min": 0, "x_max": 1, "y_max": 9}}},
             "min must be below max"),
            ("extent in cm",
             {**base, "ground": {"extent": {"x_min": -6000, "y_min": 0, "x_max": 1, "y_max": 9}}},
             "cm instead of m"),
            ("prop without class", {**base, "props": [{"x": 1, "y": 2}]}, "props[0]: 'class'"),
            ("prop with a misspelt key", {**base, "props": [{"class": "c", "x": 1, "y": 2, "yaw": 3}]},
             "props[0]: unknown key 'yaw'"),
            ("prop with a string coordinate", {**base, "props": [{"class": "c", "x": "1", "y": 2}]},
             "props[0]: 'x' must be a number"),
            ("prop with a boolean coordinate", {**base, "props": [{"class": "c", "x": True, "y": 2}]},
             "props[0]: 'x' must be a number"),
        ]
        for what, doc, expected in cases:
            with self.subTest(what):
                errors = self.check(doc)
                self.assertTrue(any(expected in e for e in errors), f"{what}: {errors}")

    def test_sidecar_needs_its_track(self):
        errors = self.check(VALID, name="orphan.env.json")
        self.assertTrue(any("no track next to it (orphan.csv)" in e for e in errors), errors)

    def test_main_counts_sidecars(self):
        self.check(VALID)
        self.assertEqual(vt.main(["validate_tracks.py", str(self.root)]), 0)
        self.check({**VALID, "seed": -1})
        self.assertEqual(vt.main(["validate_tracks.py", str(self.root)]), 1)


def _loop(blue_left: bool = True, reverse: bool = False, radius: float = 0.0):
    """A one-gate start: 4 big orange across x=0..0.5, cone pairs ahead along +x.

    radius > 0 bends both lines left on a circle of that radius (a stress-circle
    start); reverse puts the pairs behind the gate instead of ahead.
    """
    import math
    rows = [("big_orange", 0.0, 2.0), ("big_orange", 0.0, -2.0),
            ("big_orange", 0.5, 2.0), ("big_orange", 0.5, -2.0)]
    side = 1.75 if blue_left else -1.75
    for s in (2.0, 3.5, 5.0, 6.5):
        for colour, lat in (("blue", side), ("yellow", -side)):
            along = -s if reverse else s
            if radius:
                # Arc of the centreline, offset by lat toward the centre (+y).
                th = along / radius
                r = radius - lat
                x, y = r * math.sin(th), radius - r * math.cos(th)
            else:
                x, y = along, lat
            rows.append((colour, x, y))
    return rows


class StartPoseTest(unittest.TestCase):
    """Where the sim spawns the car: facing the track, blue on its left."""

    def errors(self, cones):
        return vt.check_start_pose(Path("t.csv"), cones)

    def test_straight_start_with_blue_left_is_clean(self):
        self.assertEqual(self.errors(_loop()), [])

    def test_mirrored_start_fails(self):
        # The sim turns the car so blue is on its left, which points it away
        # from the cones: the track would be driven backwards.
        errors = self.errors(_loop(blue_left=False))
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("no blue and yellow cones within 8 m ahead", errors[0])

    def test_start_on_a_tight_circle_is_clean(self):
        # Both lines curve left, so cones ahead sit left of the car's heading
        # on both colours; only blue-vs-yellow decides the side.
        for radius in (5.0, 9.0, 15.0):
            with self.subTest(radius=radius):
                self.assertEqual(self.errors(_loop(radius=radius)), [])

    def test_track_drawn_the_other_way_fails(self):
        # Pairs only behind the gate, blue on +y: blue-on-the-left faces the
        # car +x, away from every cone.
        errors = self.errors(_loop(reverse=True))
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("no blue and yellow cones within 8 m ahead", errors[0])

    def test_no_big_orange_is_not_checked(self):
        self.assertEqual(self.errors([("blue", 1.0, 1.0), ("yellow", 1.0, -1.0)]), [])


class FixtureTest(unittest.TestCase):
    """The smoke-test fixtures must be what tools/smoke/test_environment.py expects."""

    def test_good_fixture_is_clean(self):
        self.assertEqual(vt.validate_sidecar(FIXTURES / "env_gate.env.json"), [])

    def test_bad_fixture_fails_for_its_misspelt_key(self):
        errors = vt.validate_sidecar(FIXTURES / "env_bad.env.json")
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("props[0]: unknown key 'yaw'", errors[0])


if __name__ == "__main__":
    unittest.main()
