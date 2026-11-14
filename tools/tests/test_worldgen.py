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
    "trees": [{"x": 0, "y": 5, "h": 8, "r": 3, "kind": "pine"}],
}


class FormatTest(unittest.TestCase):
    def test_minimal_world_is_valid(self):
        W.validate(copy.deepcopy(MINI))

    def test_optional_parts(self):
        w = copy.deepcopy(MINI)
        for k in W.ROOF_KEYS:
            w["grandstands"][0].pop(k)            # an open stand
        w["buildings"][0].update(yaw_deg=30, base_z=3, garages=True)
        W.validate(w)

    def test_realism_parts(self):
        w = copy.deepcopy(MINI)
        w["wear"] = {"file": "wear.u8", "x0": -1.0, "y0": -1.0, "dx_m": 1.0, "dy_m": 0.25, "nx": 3, "ny": 5}
        w["lines"].append({"x0": 0, "y0": -1, "x1": 1, "y1": -1, "width_m": 6.0, "end_width_m": 0.02,
                           "colour": "green"})
        w["lines"].append({"x0": 0, "y0": -1, "x1": 1, "y1": -1, "width_m": 1.1, "colour": "red"})
        w["buildings"][0].update(garages=True, canopy_m=2.0,
                                 roof_panels=[{"x_min": -0.5, "x_max": 0.5, "y_min": -0.8, "y_max": -0.2}])
        w["grandstands"][0]["roof_panels"] = [{"x_min": -1, "x_max": 1, "y_min": 3, "y_max": 6}]
        W.validate(w)

    def test_rejections(self):
        def bad(mutate):
            w = copy.deepcopy(MINI)
            mutate(w)
            return w
        cases = [
            ("format", bad(lambda w: w.update(format="ifssim-world/2")), "format"),
            ("unknown key", bad(lambda w: w.update(decals=[])), "unknown key"),
            ("prop mesh", bad(lambda w: w.update(props=[{"mesh": "Tree", "x": 0, "y": 0}])), "mesh"),
            ("prop key", bad(lambda w: w.update(props=[{"mesh": "SignBoard", "x": 0, "y": 0, "z": 1}])), "unknown key"),
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
            ("tree kind", bad(lambda w: w["trees"][0].update(kind="palm")), "kind"),
            ("tree too tall", bad(lambda w: w["trees"][0].update(h=120)), "trees[0].h"),
            ("tree key", bad(lambda w: w["trees"][0].update(z=1)), "unknown key"),
            ("half a roof", bad(lambda w: w["grandstands"][0].pop("roof_back_z")), "roof"),
            ("deck above roof", bad(lambda w: w["buildings"][0].update(base_z=6)), "base_z"),
            ("garages flag", bad(lambda w: w["buildings"][0].update(garages=1)), "garages"),
            ("wear file", bad(lambda w: w.update(wear={"file": "wear.png", "x0": 0, "y0": 0, "dx_m": 1,
                                                       "dy_m": 1, "nx": 2, "ny": 2})), "wear.file"),
            ("line colour", bad(lambda w: w["lines"][0].update(colour="blue")), "colour"),
            ("line too wide", bad(lambda w: w["lines"][0].update(width_m=12)), "width_m"),
            ("end width", bad(lambda w: w["lines"][0].update(end_width_m=0)), "end_width_m"),
            ("canopy without garages", bad(lambda w: w["buildings"][0].update(canopy_m=2.0)), "canopy"),
            ("panel off the roof", bad(lambda w: w["buildings"][0].update(
                roof_panels=[{"x_min": 0, "x_max": 3, "y_min": -0.8, "y_max": -0.2}])), "outside the roof"),
            ("panels on a turned building", bad(lambda w: w["buildings"][0].update(
                yaw_deg=10, roof_panels=[{"x_min": 0, "x_max": 0.5, "y_min": -0.8, "y_max": -0.2}])), "yaw_deg 0"),
            ("panels without a roof", bad(lambda w: [w["grandstands"][0].pop(k) for k in W.ROOF_KEYS]
                                          and w["grandstands"][0].update(
                roof_panels=[{"x_min": 0, "x_max": 0.5, "y_min": 3, "y_max": 4}])), "need a roof"),
            ("panel in front of the roof", bad(lambda w: w["grandstands"][0].update(
                roof_panels=[{"x_min": 0, "x_max": 0.5, "y_min": 1, "y_max": 4}])), "outside the roof"),
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

    def test_wear_map_size_is_checked(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            w = copy.deepcopy(MINI)
            w["wear"] = {"file": "wear.u8", "x0": -1.0, "y0": -1.0, "dx_m": 1.0, "dy_m": 1.0, "nx": 3, "ny": 2}
            (d / "world.json").write_text(json.dumps(w))
            W.write_terrain(d / "terrain.i16", [0] * 6)
            (d / "wear.u8").write_bytes(bytes([128] * 6))
            W.load(d)
            (d / "wear.u8").write_bytes(bytes([128] * 5))
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

    def test_track_markings_match_the_orthophoto(self):
        asphalt = self.w["surfaces"][0]
        self.assertAlmostEqual(asphalt["y_max"] - asphalt["y_min"], 13.35, delta=0.1)
        transverse = sorted(ln["x0"] for ln in self.w["lines"]
                            if ln["x0"] == ln["x1"] and ln["width_m"] >= 0.3 and ln["y0"] < -7 < 5 < ln["y1"])
        self.assertEqual(transverse, [-197.7, -127.0, 0.0, 32.4])     # start at 0, finish at -127
        colours = {ln["colour"] for ln in self.w["lines"]}
        self.assertEqual(colours, {"white", "yellow", "red", "green"})
        # 28 grid slots, a yellow mark each.
        self.assertEqual(sum(ln["colour"] == "yellow" for ln in self.w["lines"]), 28)

    def test_wear_darkens_the_racing_line(self):
        r = self.w["wear"]
        v = (SHIPPED / r["file"]).read_bytes()
        col = [v[j * r["nx"] + 400] for j in range(r["ny"])]
        at = lambda y: col[int(round((y - r["y0"]) / r["dy_m"]))]
        self.assertLess(at(2.75), 120)       # rubber, left of centre
        self.assertGreater(at(-5.5), 135)    # the unused right side
        self.assertTrue(all(100 <= c <= 160 for c in col))

    def test_roofs_have_their_panels(self):
        self.assertEqual(len(self.w["buildings"][0]["roof_panels"]), 36)
        self.assertEqual(self.w["buildings"][0]["canopy_m"], 2.0)
        self.assertEqual(len(self.w["grandstands"][0]["roof_panels"]), 24)

    def test_credits_its_sources(self):
        text = " ".join(self.w["attribution"])
        self.assertIn("CC BY 4.0", text)
        self.assertIn("OpenStreetMap", text)
        self.assertIn("orthophoto", text)
        self.assertTrue((SHIPPED / "ATTRIBUTION.md").is_file())


if __name__ == "__main__":
    unittest.main()
