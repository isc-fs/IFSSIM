"""Tests for the car characterisation (tools/characterise): the analysis on
synthetic traces, and the driver end to end against a fake stepped sim with a
kinematic-bicycle car (fake_car_sim.py), built right and built with the bugs
the check exists for.

    python3 -m unittest discover -s tools/tests -v
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import random
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from characterise import analysis  # noqa: E402
from characterise.__main__ import Layout, Rpc, StepClock, WallClock, characterise, main, pick_clock  # noqa: E402
import fake_car_sim  # noqa: E402

SETTINGS = json.loads((REPO / "settings.json").read_text())
PHYS = {"Mass": 275, "Wheelbase": 1.627, "WeightDistFront": 0.438, "MaxSteerAngle": 18.2,
        "TireMu": 1.4, "CoGHeight": 0.3, "RollingResistance": 0.02, "CdA": 0.9, "ClA": 1.8}


def sample(t, x, y, vx=0.0, vy=0.0, yaw=0.0, wz=0.0):
    return {"t": t, "x": x, "y": y, "z": 0.3, "vx": vx, "vy": vy, "yaw": yaw, "wz": wz}


def circle_trace(radius, v, seconds, dt=0.05, noise=0.0, left=True, cx=40.0, cy=-30.0, seed=0, u=None):
    """The body origin on a circle at speed v; heading tangent (no sideslip) unless u says otherwise."""
    rng = random.Random(seed)
    w = v / radius * (1 if left else -1)
    out = []
    for i in range(int(seconds / dt) + 1):
        t = i * dt
        a = w * t
        x = cx + radius * math.sin(a) * (1 if left else -1)
        y = cy - radius * math.cos(a) * (1 if left else -1)
        yaw = a
        vx, vy = v * math.cos(yaw), v * math.sin(yaw)
        out.append(sample(t, x + rng.gauss(0, noise), y + rng.gauss(0, noise), vx, vy, yaw, w))
    return out


class Expectations(unittest.TestCase):
    def test_from_the_real_settings(self):
        e = analysis.expectations(SETTINGS)
        p = SETTINGS["Vehicles"]["FSCar"]["VehiclePhysics"]
        L, wdf, d = p["Wheelbase"], p["WeightDistFront"], math.radians(p["MaxSteerAngle"])
        self.assertAlmostEqual(e["wheelbase_m"], L)
        self.assertAlmostEqual(e["full_lock_radius_rear_m"], L / math.tan(d))
        self.assertAlmostEqual(e["full_lock_radius_cog_m"], math.hypot(L / math.tan(d), L * wdf))
        self.assertAlmostEqual(e["yaw_gain_per_m"], 1 / L)

    def test_the_numbers_for_the_ifs08(self):
        e = analysis.expectations(PHYS)  # the VehiclePhysics block on its own works too
        self.assertAlmostEqual(e["full_lock_radius_rear_m"], 4.949, places=3)
        self.assertAlmostEqual(e["full_lock_radius_cog_m"], 5.000, places=3)
        self.assertAlmostEqual(e["cog_to_rear_m"], 0.7126, places=4)
        self.assertAlmostEqual(e["coast_a0"], 0.02 * 9.81)
        self.assertAlmostEqual(e["coast_b"], 1.225 / 550 * (0.9 + 0.02 * 1.8))
        # rear-only clamp, load off the rear: mu*g*lf / (L + mu*h)
        self.assertAlmostEqual(e["ebs_decel"], 1.4 * 9.81 * 1.627 * 0.562 / (1.627 + 0.42), places=6)

    def test_settings_drive_the_expectation(self):
        wide = dict(PHYS, MaxSteerAngle=22.4)
        self.assertLess(analysis.expectations(wide)["full_lock_radius_cog_m"],
                        analysis.expectations(PHYS)["full_lock_radius_cog_m"] * 0.85)

    def test_info_inputs_are_optional_hard_ones_are_not(self):
        e = analysis.expectations({k: PHYS[k] for k in ("Mass", "Wheelbase", "WeightDistFront", "MaxSteerAngle")})
        self.assertIsNone(e["ebs_decel"])
        self.assertIsNone(e["coast_a0"])
        with self.assertRaises(KeyError):
            analysis.expectations({"Vehicles": {"FSCar": {"VehiclePhysics": {"Mass": 275}}}})


class Fits(unittest.TestCase):
    def test_circle_fit_full_turn_with_noise(self):
        tr = circle_trace(5.0, 2.0, 16.0, noise=0.02)
        _, _, r, rms = analysis.fit_circle([s["x"] for s in tr], [s["y"] for s in tr])
        self.assertAlmostEqual(r, 5.0, delta=0.02)
        self.assertLess(rms, 0.05)

    def test_circle_fit_half_turn_far_from_the_origin(self):
        tr = circle_trace(20.0, 2.0, 31.4, noise=0.02, cx=500.0, cy=-800.0)
        _, _, r, _ = analysis.fit_circle([s["x"] for s in tr], [s["y"] for s in tr])
        self.assertAlmostEqual(r, 20.0, delta=0.1)

    def test_turn_summary(self):
        tr = circle_trace(5.0, 2.0, 10.0, noise=0.01, left=False)
        t = analysis.turn_summary(tr, 1.0, math.radians(18.2))
        self.assertEqual(t["turn"], "right")
        self.assertAlmostEqual(t["radius_m"], 5.0, delta=0.03)
        self.assertAlmostEqual(abs(t["kappa"]), 0.2, delta=0.002)
        self.assertAlmostEqual(t["gyro_gain"], 0.2 / math.tan(math.radians(18.2)), places=6)

    def test_short_arcs_get_no_radius(self):
        t = analysis.turn_summary(circle_trace(20.0, 2.0, 5.0), -0.25, math.radians(18.2))
        self.assertIsNone(t["radius_m"])
        self.assertAlmostEqual(t["kappa"], 0.05, delta=1e-3)

    def test_steer_exponent_linear_and_squared(self):
        d = math.radians(18.2)
        lin = [(s, math.tan(s * d) / 1.627) for s in (0.25, 0.5, 0.75, 1.0)]
        sq = [(s, math.tan(s * s * d) / 1.627) for s in (0.25, 0.5, 0.75, 1.0)]
        self.assertAlmostEqual(analysis.steer_exponent(lin, d), 1.0, places=6)
        self.assertGreater(analysis.steer_exponent(sq, d), 1.9)
        self.assertIsNone(analysis.steer_exponent([(1.0, 0.2)], d))

    def test_time_to_distance(self):
        a = 4.0
        tr = [sample(i * 0.05, 0.5 * a * (i * 0.05) ** 2, 0.0, a * i * 0.05) for i in range(200)]
        r = analysis.time_to_distance(tr, 30.0)
        self.assertAlmostEqual(r["time_s"], math.sqrt(2 * 30 / a), delta=0.01)
        self.assertAlmostEqual(r["speed_mps"], math.sqrt(2 * a * 30), delta=0.1)
        self.assertIsNone(analysis.time_to_distance(tr[:10], 30.0))

    def test_coast_fit_recovers_a0_and_b(self):
        a0, b, v, x, t, dt = 0.2, 0.003, 10.0, 0.0, 0.0, 0.01
        rng = random.Random(3)
        tr = []
        while t < 8.0:
            if round(t / dt) % 5 == 0:
                tr.append(sample(t, x, 0.0, v + rng.gauss(0, 0.002)))
            v -= (a0 + b * v * v) * dt
            x += v * dt
            t += dt
        f = analysis.coast_fit(tr)
        self.assertAlmostEqual(f["a0"], a0, delta=0.03)
        self.assertAlmostEqual(f["b"], b, delta=0.0005)
        self.assertAlmostEqual(f["decel_mean"], a0 + b * f["v2_mean"], delta=0.01)

    def test_stop_summary(self):
        a, v0 = 6.0, 7.0
        tr = []
        for i in range(60):
            t = i * 0.05
            v = max(v0 - a * t, 0.0)
            x = v0 * t - 0.5 * a * t * t if v > 0 else v0 * v0 / (2 * a)
            tr.append(sample(t, x, 0.0, v))
        s = analysis.stop_summary(tr)
        self.assertTrue(s["stopped"])
        self.assertAlmostEqual(s["distance_m"], v0 * v0 / (2 * a), delta=0.05)
        self.assertFalse(analysis.stop_summary(tr[:10])["stopped"])

    def test_yaw_from_quat(self):
        for deg in (-170, -30, 0, 45, 179):
            h = math.radians(deg)
            self.assertAlmostEqual(analysis.yaw_from_quat(math.cos(h / 2), 0, 0, math.sin(h / 2)), h)

    def test_unwrap(self):
        self.assertAlmostEqual(analysis.unwrap([3.0, -3.0, -2.5])[-1], 2 * math.pi - 2.5)


class WheelbaseLog(unittest.TestCase):
    OK = ("LogTemp: FSDS: physics wheelbase 162.7 cm, 43.8% of the weight on the front axle; "
          "w0=(91.4, -61.0, 0.0) w1=(91.4, 61.0, 0.0)")
    BAD = ("LogTemp: Error: FSDS: physics wheels are NOT where settings.json puts them "
           "(more than 1 cm off): wheelbase 118.0 cm; w0=(...)")

    def test_ok_line(self):
        r = analysis.parse_wheelbase_log("noise\n" + self.OK + "\n")
        self.assertAlmostEqual(r["wheelbase_m"], 1.627)
        self.assertAlmostEqual(r["front_weight"], 0.438)
        self.assertTrue(r["placed_ok"])

    def test_the_last_line_wins(self):
        r = analysis.parse_wheelbase_log(self.OK + "\n" + self.BAD + "\n")
        self.assertAlmostEqual(r["wheelbase_m"], 1.18)
        self.assertFalse(r["placed_ok"])
        self.assertTrue(analysis.parse_wheelbase_log(self.BAD + "\n" + self.OK)["placed_ok"])
        self.assertIsNone(analysis.parse_wheelbase_log("nothing here"))

    def test_a_wrong_wheelbase_fails(self):
        e = analysis.expectations(PHYS)
        bad = analysis.evaluate(e, {"wheelbase_log": analysis.parse_wheelbase_log(self.BAD)})
        c = next(c for c in bad["checks"] if c["id"] == "physics_wheelbase")
        self.assertEqual(c["status"], "fail")
        ok = analysis.evaluate(e, {"wheelbase_log": analysis.parse_wheelbase_log(self.OK)})
        self.assertEqual(next(c for c in ok["checks"] if c["id"] == "physics_wheelbase")["status"], "pass")


class Verdict(unittest.TestCase):
    def test_nothing_measured_is_an_error_not_a_pass(self):
        r = analysis.evaluate(analysis.expectations(PHYS), {})
        self.assertEqual(r["status"], "error")
        self.assertFalse(any(c["status"] == "fail" for c in r["checks"]))

    def test_markdown(self):
        r = analysis.evaluate(analysis.expectations(PHYS), {})
        md = analysis.markdown({**r, "clock": {"mode": "step", "sim_s": 86.0, "rtf": None}})
        self.assertIn("#### Car characterisation · incomplete", md)
        self.assertIn("| Check | Expected | Measured | Result |", md)
        self.assertIn("5.00 m ±3%", md)
        self.assertIn("Clock: step, 86 s of sim time.", md)
        err = analysis.markdown({"status": "error", "error": "RpcError: ping: refused"})
        self.assertIn("> RpcError: ping: refused", err)


class EndToEnd(unittest.TestCase):
    """The driver against a stepped fake sim: a right car passes, each bug fails its check."""

    def run_car(self, **plant):
        sim = fake_car_sim.FakeSim(fake_car_sim.Plant(**plant))
        srv, port = fake_car_sim.serve(sim)
        try:
            res, meas = characterise(Rpc("127.0.0.1", port), SETTINGS, Layout(), log=lambda m: None)
        finally:
            srv.shutdown()
        return res, meas, sim

    def status(self, res, cid):
        return [c["status"] for c in res["checks"] if c["id"].startswith(cid)]

    def test_a_car_built_to_its_settings_passes(self):
        res, meas, sim = self.run_car()
        self.assertEqual(res["status"], "pass", res.get("error") or res["checks"])
        self.assertEqual(self.status(res, "full_lock_radius"), ["pass", "pass"])
        self.assertEqual(res["clock"]["mode"], "step")
        self.assertLess(res["clock"]["sim_s"], 180)          # the budget: ~2-3 min of sim time
        self.assertEqual(len(meas["turns"]), 5)
        # The info checks are measured too, and near the fake's own physics.
        info = {c["id"]: c for c in res["checks"]}
        self.assertAlmostEqual(info["coast_decel"]["measured"], info["coast_decel"]["expected"], delta=0.03)
        self.assertIsNotNone(info["launch_time"]["measured"])
        self.assertIsNotNone(info["ebs_distance"]["measured"])
        # Left tidy: EBS latched, the sim running.
        self.assertTrue(sim.ebs)
        self.assertFalse(sim.paused)

    def test_the_old_wheelbase_fails(self):
        res, _, _ = self.run_car(wheelbase=1.18)
        self.assertEqual(res["status"], "fail")
        self.assertEqual(self.status(res, "full_lock_radius"), ["fail", "fail"])
        self.assertEqual(self.status(res, "yaw_gain"), ["fail"])

    def test_the_old_steering_lock_fails(self):
        res, _, _ = self.run_car(max_steer_deg=22.4)
        self.assertEqual(self.status(res, "full_lock_radius"), ["fail", "fail"])

    def test_a_squared_steering_input_fails_linearity(self):
        res, _, _ = self.run_car(squared_steer=True)
        self.assertEqual(self.status(res, "steer_linearity"), ["fail"])
        self.assertEqual(self.status(res, "full_lock_radius"), ["pass", "pass"])  # full lock is unaffected

    def test_falling_off_the_ground_is_an_error(self):
        sim = fake_car_sim.FakeSim()
        srv, port = fake_car_sim.serve(sim)
        orig = sim._kinematics
        n = {"k": 0}

        def falling():
            n["k"] += 1
            k = orig()
            if n["k"] > 200:
                k["pz"] = -5.0
            return k
        sim._kinematics = falling
        try:
            res, _ = characterise(Rpc("127.0.0.1", port), SETTINGS, Layout(), log=lambda m: None)
        finally:
            srv.shutdown()
        self.assertEqual(res["status"], "error")
        self.assertIn("fell", res["error"])
        self.assertTrue(sim.ebs)

    def test_no_sim_is_an_error_with_a_result(self):
        port = fake_car_sim.free_port()
        with tempfile.TemporaryDirectory() as d:
            settings = Path(d) / "settings.json"
            settings.write_text(json.dumps(SETTINGS))
            with contextlib.redirect_stdout(io.StringIO()):
                rc = main(["--sim", f"127.0.0.1:{port}", "--settings", str(settings), "--out", d])
            res = json.loads((Path(d) / "result.json").read_text())
            self.assertEqual(rc, 2)
            self.assertEqual(res["status"], "error")
            self.assertIn("Car characterisation", (Path(d) / "summary.md").read_text())

    def test_a_sim_that_cannot_step_falls_back_to_the_wall_clock(self):
        sim = fake_car_sim.FakeSim(fake_car_sim.Plant(step_works=False))
        srv, port = fake_car_sim.serve(sim)
        rpc = Rpc("127.0.0.1", port)
        try:
            orig = StepClock.__init__

            def quick(self, rpc, timeout_s=5.0):  # don't wait 5 s for the step that never ends
                orig(self, rpc, timeout_s=0.2)
            StepClock.__init__ = quick
            try:
                clock = pick_clock(rpc, "auto", log=lambda m: None)
            finally:
                StepClock.__init__ = orig
            self.assertIsInstance(clock, WallClock)
            self.assertFalse(sim.paused)
        finally:
            rpc.close()
            srv.shutdown()


if __name__ == "__main__":
    unittest.main()
