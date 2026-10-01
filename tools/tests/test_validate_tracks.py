"""Tests for tools/validate_tracks.py's sidecar checks (docs/environment_sidecar.md).

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
CSV = "big_orange,0,2\nbig_orange,0,-2\nblue,5,1.75\nyellow,5,-1.75\n"


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
