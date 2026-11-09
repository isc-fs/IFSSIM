"""Tests for tools/worldgen (docs/worlds.md): the world format and the shipped
Montmeló world.

    python -m unittest discover -s tools/tests -v
"""
from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
from worldgen import world as W  # noqa: E402

SHIPPED = REPO / "Content" / "tracks" / "worlds" / "montmelo_straight"

MINI = {
    "format": "ifssim-world/1", "name": "mini", "title": "Mini",
    "attribution": ["made up for a test"],
    "frame": {"crs": "EPSG:25831", "origin_e": 1.0, "origin_n": 2.0, "origin_z": 3.0, "x_bearing_deg": 90.0},
    "terrain": {"file": "terrain.i16", "x0": -1.0, "y0": -1.0, "cell_m": 1.0, "nx": 3, "ny": 2, "z_unit_m": 0.01},
    "surfaces": [{"material": "asphalt", "x_min": -1, "x_max": 1, "y_min": -1, "y_max": 0}],
    "lines": [{"x0": 0, "y0": -1, "x1": 0, "y1": 0, "width_m": 0.5, "colour": "white"}],
    "walls": [{"material": "concrete", "x0": -1, "y0": 0, "x1": 1, "y1": 0, "height_m": 1.0,
               "thickness_m": 0.5, "fence_height_m": 2.0}],
    "buildings": [{"material": "concrete", "x_min": -1, "x_max": 1, "y_min": -1, "y_max": 0, "top_z": 5}],
    "grandstands": [{"x_min": -1, "x_max": 1, "front_y": 2, "back_y": 6, "front_z": 1, "back_z": 4,
                     "roof_front_y": 2, "roof_back_y": 7, "roof_front_z": 9, "roof_back_z": 8}],
}


class FormatTest(unittest.TestCase):
    def test_minimal_world_is_valid(self):
        W.validate(copy.deepcopy(MINI))

    def test_rejections(self):
        def bad(mutate):
            w = copy.deepcopy(MINI)
            mutate(w)
            return w
        cases = [
            ("format", bad(lambda w: w.update(format="ifssim-world/2")), "format"),
            ("unknown key", bad(lambda w: w.update(props=[])), "unknown key"),
            ("no attribution", bad(lambda w: w.update(attribution=[])), "attribution"),
            ("terrain file", bad(lambda w: w["terrain"].update(file="t.png")), "terrain.file"),
            ("too many cells", bad(lambda w: w["terrain"].update(nx=20000, ny=20000)), "cells"),
            ("surface material", bad(lambda w: w["surfaces"][0].update(material="lava")), "material"),
            ("inverted surface", bad(lambda w: w["surfaces"][0].update(x_min=5)), "x_min must be below"),
            ("line of zero length", bad(lambda w: w["lines"][0].update(x1=0, y1=-1)), "same point"),
            ("fence below wall", bad(lambda w: w["walls"][0].update(fence_height_m=0.5)), "fence_height_m"),
            ("coordinate in cm", bad(lambda w: w["buildings"][0].update(x_max=5000)), "outside"),
            ("seating falls", bad(lambda w: w["grandstands"][0].update(back_z=0.5)), "rise"),
            ("boolean number", bad(lambda w: w["walls"][0].update(height_m=True)), "number"),
        ]
        for what, w, expected in cases:
            with self.subTest(what):
                with self.assertRaises(W.WorldError) as cm:
                    W.validate(w)
                self.assertIn(expected, str(cm.exception))

    def test_terrain_round_trip_and_bilinear_height(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            (d / "world.json").write_text(json.dumps(MINI))
            # nx=3, ny=2: row j=0 is y=-1, row j=1 is y=0.
            W.write_terrain(d / "terrain.i16", [0, 100, 200, 0, 100, 200])
            w, h = W.load(d)
            self.assertAlmostEqual(W.height_at(w, h, -1.0, -1.0), 0.0)
            self.assertAlmostEqual(W.height_at(w, h, 0.5, -0.5), 1.5)
            self.assertAlmostEqual(W.height_at(w, h, 99.0, 99.0), 2.0)   # clamped at the edge
            (d / "terrain.i16").write_bytes(b"\0" * 10)
            with self.assertRaises(W.WorldError):
                W.load(d)


@unittest.skipUnless((SHIPPED / "world.json").is_file(), "montmelo_straight not generated")
class MontmeloTest(unittest.TestCase):
    """The shipped world matches what was measured (docs/worlds.md)."""

    @classmethod
    def setUpClass(cls):
        cls.w, cls.h = W.load(SHIPPED)

    def test_start_line_is_the_datum_and_the_acceleration_zone_is_flat(self):
        self.assertAlmostEqual(W.height_at(self.w, self.h, 0.0, 0.0), 0.0, delta=0.05)
        for x in range(-100, 160, 10):
            self.assertAlmostEqual(W.height_at(self.w, self.h, x, 0.0), 0.0, delta=0.15, msg=f"x={x}")

    def test_the_straight_falls_towards_turn_1(self):
        self.assertLess(W.height_at(self.w, self.h, 600.0, 0.0), -5.0)

    def test_structures_sit_where_they_were_measured(self):
        g = self.w["grandstands"][0]
        self.assertGreater(g["front_y"], 10.0)          # left of the race direction
        self.assertAlmostEqual(g["roof_front_z"], 28.5, delta=1.0)
        b = self.w["buildings"][0]
        self.assertLess(b["y_max"], -20.0)               # pits on the right
        self.assertAlmostEqual(b["top_z"], 8.3, delta=1.0)

    def test_credits_its_sources(self):
        text = " ".join(self.w["attribution"])
        self.assertIn("CC BY 4.0", text)
        self.assertIn("OpenStreetMap", text)
        self.assertTrue((SHIPPED / "ATTRIBUTION.md").is_file())


if __name__ == "__main__":
    unittest.main()
