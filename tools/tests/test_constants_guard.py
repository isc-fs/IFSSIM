"""Guard: constants hardcoded in tools/docs must match settings.json.

settings.json is the single source of truth for the sim car. Two long-lived
bugs came from copies drifting (wheelbase 1.18 vs 1.627, steer lock 22.4 vs
18.2). Each row below pins one literal in one file to one settings key.
Stdlib only; runs under unittest discover and pytest.
"""
import json
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VP = json.loads((ROOT / "settings.json").read_text())["Vehicles"]["FSCar"]["VehiclePhysics"]

# (file, regex with ONE capture group, settings key, transform, abs tolerance)
def ident(v): return v
def deg2rad(v): return math.radians(v)

CHECKS = [
    ("tools/track_driver.py", r'_VP\.get\("Wheelbase", ([\d.]+)\)', "Wheelbase", ident, 1e-9),
    ("tools/track_driver.py", r'_VP\.get\("MaxSteerAngle", ([\d.]+)\)', "MaxSteerAngle", ident, 1e-9),
    ("tools/dev/circle_test.py", r'_VP\.get\("Wheelbase", ([\d.]+)\)', "Wheelbase", ident, 1e-9),
    ("tools/dev/circle_test.py", r'_VP\.get\("MaxSteerAngle", ([\d.]+)\)', "MaxSteerAngle", ident, 1e-9),
    # Bridge normalized->rad scale (also in ifssim_bridge, default 0.317650).
    ("tools/sim_benchmark/odometry_filter_cpp.py", r"^MAX_STEERING_ANGLE_RAD = ([\d.]+)", "MaxSteerAngle", deg2rad, 1e-5),
    ("tools/sim_benchmark/slam_metrics.py", r"max_steer_rad: float = ([\d.]+)", "MaxSteerAngle", deg2rad, 1e-5),
    ("ros2/src/ifssim_bridge/include/ifssim_ros_wrapper.h", r"max_steering_angle_rad_ = ([\d.]+);", "MaxSteerAngle", deg2rad, 1e-5),
    ("tests/test_vehicle_physics.py", r'"mass": ([\d.]+)', "Mass", ident, 1e-9),
    ("tests/test_vehicle_physics.py", r'"tire_radius": ([\d.]+)', "WheelRadius", ident, 1e-9),
    ("tests/test_vehicle_physics.py", r'"tire_mu": ([\d.]+)', "TireMu", ident, 1e-9),
    ("tests/test_vehicle_physics.py", r'"CdA": ([\d.]+)', "CdA", ident, 1e-9),
    ("tests/test_vehicle_physics.py", r'"ClA": ([\d.]+)', "ClA", ident, 1e-9),
    ("docs/REFERENCE.md", r"\| Mass \| ([\d.]+) \|", "Mass", ident, 1e-9),
    ("docs/REFERENCE.md", r"\| Wheel radius \| ([\d.]+) \|", "WheelRadius", ident, 1e-9),
    ("docs/REFERENCE.md", r"\| Max steer angle \| ([\d.]+) \|", "MaxSteerAngle", ident, 1e-9),
    ("docs/REFERENCE.md", r"\| Wheelbase \| ([\d.]+) \|", "Wheelbase", ident, 1e-9),
    ("docs/REFERENCE.md", r"max steer ([\d.]+)°", "MaxSteerAngle", ident, 1e-9),
    ("docs/REFERENCE.md", r"`WheelRadius=([\d.]+) m`", "WheelRadius", ident, 1e-9),
]

# INTENTIONAL differences. Not checked against settings.json; each is pinned to
# its own expected value so it cannot drift silently either.
ALLOWLIST = [
    # Offline mirrors of the PIPELINE odometry filter (odometry_filter.hpp
    # kWheelbaseM = 1.570). Pipeline EKF value from car hardpoints; settings.json
    # says 1.627. Disputed pending a tape measurement. Mirror must follow the pipeline.
    ("tools/sim_benchmark/odometry_filter_cpp.py", r"^WHEELBASE_M = ([\d.]+)", 1.570),
    ("tools/sim_benchmark/slam_metrics.py", r"wheelbase_m: float = ([\d.]+)", 1.570),
]


def _grab(rel, pattern):
    text = (ROOT / rel).read_text(encoding="utf-8")
    m = re.search(pattern, text, re.M)
    return m, float(m.group(1)) if m else None


class ConstantsGuard(unittest.TestCase):
    def test_tool_and_doc_constants_match_settings(self):
        for rel, pattern, key, fn, tol in CHECKS:
            with self.subTest(file=rel, key=key):
                m, got = _grab(rel, pattern)
                self.assertIsNotNone(m, f"{rel}: pattern {pattern!r} not found (moved? update CHECKS)")
                want = fn(float(VP[key]))
                self.assertAlmostEqual(got, want, delta=tol,
                                       msg=f"{rel} has {got}, settings.json {key} implies {want}")

    def test_allowlisted_values_pinned(self):
        for rel, pattern, expected in ALLOWLIST:
            with self.subTest(file=rel):
                m, got = _grab(rel, pattern)
                self.assertIsNotNone(m, f"{rel}: pattern {pattern!r} not found")
                self.assertAlmostEqual(got, expected, delta=1e-9)

    def test_pipeline_wheelbase_if_present(self):
        params = ROOT / "pipeline/bringup/config/params.yaml"
        if not params.is_file():
            self.skipTest("pipeline submodule not checked out")
        m = re.search(r"wheelbase\w*:\s*([\d.]+)", params.read_text(), re.I)
        if not m:
            self.skipTest("no wheelbase key in pipeline params.yaml")
        # Pipeline may legitimately differ (see ALLOWLIST); just require sanity.
        self.assertTrue(1.4 < float(m.group(1)) < 1.8)


if __name__ == "__main__":
    unittest.main()
