from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from odometry_filter_cpp import (  # noqa: E402
    EkfParams,
    MAX_STEERING_ANGLE_RAD,
    OdometryFilterCpp,
    kinematic_omega,
    steering_to_road_wheel_rad,
    wrap_pi,
)


class KinematicOmegaTest(unittest.TestCase):
    def test_steering_units(self) -> None:
        self.assertAlmostEqual(
            steering_to_road_wheel_rad(0.4, units="normalized"),
            0.4 * MAX_STEERING_ANGLE_RAD,
        )
        self.assertAlmostEqual(
            steering_to_road_wheel_rad(0.25, units="radians"),
            0.25,
        )

    def test_ue_steering_sign(self) -> None:
        # Road-wheel δ: positive = right (UE). REP-103 ω_z: positive = left.
        wz_right = kinematic_omega(5.0, 0.3, 1.57)
        wz_left = kinematic_omega(5.0, -0.3, 1.57)
        self.assertLess(wz_right, 0.0)
        self.assertGreater(wz_left, 0.0)
        self.assertAlmostEqual(
            wz_right,
            -5.0 / 1.57 * math.tan(0.3),
            places=5,
        )


class OdometryFilterCppTest(unittest.TestCase):
    def _calibrate(self, filt: OdometryFilterCpp, t_end: float = 3.1) -> None:
        dt = 0.01
        t = 0.0
        accel = np.array([0.0, 0.0, 9.81])
        gyro = np.zeros(3)
        while t < t_end:
            filt.push_imu(t, accel, gyro)
            t += dt
        self.assertTrue(filt.is_calibrated())

    def test_wheel_sensors_straight(self) -> None:
        filt = OdometryFilterCpp()
        self._calibrate(filt)
        filt.push_wheel_sensors(3.11, 1000.0, 0.0)
        filt.push_wheel_sensors(3.12, 1000.0, 0.0)
        self.assertGreater(filt.state.x, 0.0)
        self.assertAlmostEqual(filt.state.y, 0.0, places=2)
        self.assertAlmostEqual(filt.state.yaw, 0.0, places=2)

    def test_wheel_sensors_turn(self) -> None:
        filt = OdometryFilterCpp(EkfParams(wheelbase_m=1.0))
        self._calibrate(filt)
        # UE-negative road δ → left turn → positive yaw in REP-103
        filt.push_wheel_sensors(3.11, 500.0, -0.3, steering_units="radians")
        filt.push_wheel_sensors(3.12, 500.0, -0.3, steering_units="radians")
        filt.push_wheel_sensors(3.13, 500.0, -0.3, steering_units="radians")
        self.assertGreater(filt.state.yaw, 0.01)

    def test_wheel_sensors_normalized_steer(self) -> None:
        filt = OdometryFilterCpp(EkfParams(wheelbase_m=1.0))
        self._calibrate(filt)
        # 0.6 normalized → 0.3 rad road, left in UE is negative norm
        filt.push_wheel_sensors(3.11, 500.0, -0.6, steering_units="normalized")
        filt.push_wheel_sensors(3.12, 500.0, -0.6, steering_units="normalized")
        self.assertGreater(filt.state.yaw, 0.005)

    def test_wrap_pi(self) -> None:
        self.assertAlmostEqual(wrap_pi(4.0), 4.0 - 2 * math.pi, places=5)


if __name__ == "__main__":
    unittest.main()
