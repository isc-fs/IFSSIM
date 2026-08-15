"""Production C++ 9-state odometry EKF for the SLAM benchmark.

``OdometryFilterCpp`` runs the REAL filter from ``pipeline/odometry_filter``
via its pybind11 bindings (``odometry_filter_py``) whenever they're
importable, so offline replay exercises exactly what
``odometry_filter_node`` runs — no separately-maintained re-implementation
to drift out of sync (a stale port once made a filter change look like a
benchmark regression; that's why this now binds the real code).

If the compiled module isn't on the path (e.g. a host shell with no built
workspace), it transparently falls back to ``_PyOdometryFilter`` below — a
line-for-line Python mirror kept only as that fallback + a readable
reference. The open-loop wheel dead-reckoning (``push_wheel_sensors``) is a
deliberately-degraded *diagnostic*, not a port of any production code, so it
lives in the wrapper in Python regardless of backend.

Check ``OdometryFilterCpp(...).backend`` (``"cpp"`` / ``"python"``) to see
which is active.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# --- constants (odometry_filter.hpp) -----------------------------------------
G = 9.81
RPM_TO_MS = 0.00821
WHEELBASE_M = 1.570
CALIBRATION_SECONDS = 3.0
SLIP_YAW_RESIDUAL_THRESHOLD = 0.3
MIN_VX_FOR_STEERING_CORRECT = 3.0
# Bridge default: Chaos normalized [-1, 1] → road-wheel rad via this scale.
MAX_STEERING_ANGLE_RAD = 0.5
# Bridge ``/steering_angle`` uses UE/Chaos sign (positive = right). REP-103
# body ω_z and sim_supervisor integration use positive = left — negate δ.
STEERING_ANGLE_TO_YAW_RATE_SIGN = -1.0

X, Y, THETA, VX, VY, OMEGA, BA_X, BA_Y, BG_Z = range(9)
STATE_DIM = 9


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _set_initial_covariance(p: np.ndarray) -> None:
    p.fill(0.0)
    p[X, X] = 0.01**2
    p[Y, Y] = 0.01**2
    p[THETA, THETA] = 0.01**2
    p[VX, VX] = 0.5**2
    p[VY, VY] = 0.5**2
    p[OMEGA, OMEGA] = 0.05**2
    p[BA_X, BA_X] = 0.20**2
    p[BA_Y, BA_Y] = 0.20**2
    p[BG_Z, BG_Z] = 0.02**2


@dataclass
class OdometryState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0


@dataclass
class FilterDiagnostics:
    yaw_residual_rad_s: float = 0.0
    slip_flag: bool = False
    low_vx_gate_on: bool = False


@dataclass
class EkfParams:
    wheelbase_m: float = WHEELBASE_M
    rpm_to_ms: float = RPM_TO_MS
    calibration_seconds: float = CALIBRATION_SECONDS
    # Wheel speed below this (m/s) counts as a standstill for bias calibration.
    stationary_speed_ms: float = 0.1
    sigma_ax: float = 0.05
    sigma_ay: float = 0.05
    sigma_gz: float = 0.01
    sigma_ba_walk: float = 1.0e-4
    sigma_bg_walk: float = 1.0e-4
    sigma_rpm: float = 0.02
    sigma_steer: float = 0.30
    sigma_vy_nhc: float = 0.10
    sigma_vy_nhc_slip: float = 0.50
    slip_yaw_residual_threshold: float = SLIP_YAW_RESIDUAL_THRESHOLD
    min_vx_for_steering_correct: float = MIN_VX_FOR_STEERING_CORRECT
    dt_min: float = 1.0e-5
    dt_max: float = 0.1


@dataclass
class _Calibration:
    t_first: float | None = None
    accel_sum: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_sum: np.ndarray = field(default_factory=lambda: np.zeros(3))
    n_samples: int = 0
    completed: bool = False
    accel_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))


def steering_to_road_wheel_rad(
    steering: float,
    units: str = "radians",
    max_steer_rad: float = MAX_STEERING_ANGLE_RAD,
) -> float:
    """Map ``/steering_angle`` to road-wheel radians for the bicycle model.

    Bridge contract (since #462) is road-wheel rad in [-max, +max]. Some bags
    still store Chaos normalized [-1, 1] on the same topic; pass
    ``units='normalized'`` (auto-detected in ``slam_metrics``).
    """
    delta = float(steering)
    if units == "normalized":
        return delta * max_steer_rad
    return delta


def kinematic_omega(
    vx: float,
    road_wheel_rad: float,
    wheelbase_m: float,
) -> float:
    """Kinematic-bicycle yaw rate from road-wheel angle (rad).

    Same ``(vx / L) * tan(delta)`` as ``odometry_filter.cpp``, with
    ``STEERING_ANGLE_TO_YAW_RATE_SIGN`` so δ (UE: +right) matches REP-103 ω_z
    (positive = left).
    """
    if wheelbase_m <= 1.0e-3:
        return 0.0
    return (
        STEERING_ANGLE_TO_YAW_RATE_SIGN
        * (vx / wheelbase_m)
        * math.tan(road_wheel_rad)
    )


# --- production C++ bindings (preferred backend) -----------------------------
try:
    import odometry_filter_py as _native  # compiled pybind11 module

    _NATIVE_AVAILABLE = True
except ImportError:  # no built workspace on the path → Python fallback
    _native = None
    _NATIVE_AVAILABLE = False


_EKF_PARAM_FIELDS = (
    "wheelbase_m", "rpm_to_ms", "calibration_seconds", "stationary_speed_ms",
    "sigma_ax", "sigma_ay", "sigma_gz", "sigma_ba_walk", "sigma_bg_walk",
    "sigma_rpm", "sigma_steer", "sigma_vy_nhc", "sigma_vy_nhc_slip",
    "slip_yaw_residual_threshold", "min_vx_for_steering_correct",
    "dt_min", "dt_max",
)


def _to_native_params(params: EkfParams):
    """Copy a Python EkfParams dataclass into a native EkfParams struct."""
    np_params = _native.EkfParams()
    for field_name in _EKF_PARAM_FIELDS:
        setattr(np_params, field_name, getattr(params, field_name))
    return np_params


class _PyOdometryFilter:
    """Pure-Python mirror of ``odometry_filter::OdometryFilter`` — the
    fallback used only when the compiled ``odometry_filter_py`` bindings
    aren't importable. Kept line-for-line faithful to ``odometry_filter.cpp``
    as a readable reference; the wrapper below prefers the real C++."""

    def __init__(self, params: EkfParams | None = None) -> None:
        self.params = params or EkfParams()
        self._x = np.zeros(STATE_DIM)
        self._p = np.zeros((STATE_DIM, STATE_DIM))
        self.state = OdometryState()
        self.diagnostics = FilterDiagnostics()
        self._calib = _Calibration()
        self._t_imu_last: float | None = None
        self.latest_steering_rad: float = 0.0
        self.latest_rpm: float = 0.0
        self._have_steering = False
        self.reset()

    def reset(self) -> None:
        self._x.fill(0.0)
        _set_initial_covariance(self._p)
        self.state = OdometryState()
        self.diagnostics = FilterDiagnostics()
        self._calib = _Calibration()
        self._t_imu_last = None
        self.latest_steering_rad = 0.0
        self.latest_rpm = 0.0
        self._have_steering = False
        self._have_rpm = False

    def is_calibrated(self) -> bool:
        return self._calib.completed

    def seed_forward_velocity(self, vx: float) -> None:
        """Seed the post-calibration forward velocity for unaided dead-reckoning.

        The accel-bias calibration zeroes every state, which is only correct if
        the car is stationary at calibration. When a bag starts mid-motion (the
        recorder misses the pre-launch standstill — DDS discovery latency) the
        IMU-only filter can never recover absolute speed: at constant speed there
        is no forward accel to integrate, so vx stays ~0. The path then collapses
        onto the lateral (+y/left) leak, i.e. it comes out as the true path
        rotated +90 deg CCW and shrunk. Seeding vx once from a wheel/GT speed
        sample fixes only the initial condition — no continuous aiding, so this
        stays a genuine IMU dead-reckoning diagnostic.
        """
        if not self._calib.completed:
            return
        self._x[VX] = float(vx)
        self._publish_state_view()

    def push_imu(
        self,
        t: float,
        accel: np.ndarray,
        gyro: np.ndarray,
    ) -> None:
        if not self._calib.completed:
            self._accumulate_calibration(t, accel, gyro)
            return

        if self._t_imu_last is None:
            self._t_imu_last = t
            return

        dt = t - self._t_imu_last
        self._t_imu_last = t
        if dt < self.params.dt_min or dt > self.params.dt_max:
            return

        self._predict_step(dt, accel, gyro)
        if self._have_steering:
            self._correct_steering()
        self._correct_nhc()
        self._publish_state_view()

    def push_rpm(self, t: float, rpm: float) -> None:
        del t
        # Track wheel speed even before calibration completes so the bias
        # calibration can gate on a genuine standstill (rpm ~ 0).
        self.latest_rpm = float(rpm)
        self._have_rpm = True
        if not self._calib.completed:
            return
        z_vx = float(rpm) * self.params.rpm_to_ms
        self._correct_rpm(z_vx)
        self._publish_state_view()

    def push_steering(self, t: float, angle_rad: float) -> None:
        del t
        self.latest_steering_rad = float(angle_rad)
        self._have_steering = True

    def _predict_step(
        self,
        dt: float,
        accel: np.ndarray,
        gyro: np.ndarray,
    ) -> None:
        ax = float(accel[0]) - self._x[BA_X]
        ay = float(accel[1]) - self._x[BA_Y]
        wz = float(gyro[2]) - self._x[BG_Z]

        theta = self._x[THETA]
        vx = self._x[VX]
        vy = self._x[VY]

        c = math.cos(theta)
        s = math.sin(theta)

        self._x[X] += (vx * c - vy * s) * dt
        self._x[Y] += (vx * s + vy * c) * dt
        self._x[THETA] = wrap_pi(theta + wz * dt)
        self._x[VX] += (ax + wz * vy) * dt
        self._x[VY] += (ay - wz * vx) * dt
        self._x[OMEGA] = wz

        f = np.eye(STATE_DIM)
        f[X, THETA] = (-vx * s - vy * c) * dt
        f[X, VX] = c * dt
        f[X, VY] = -s * dt
        f[Y, THETA] = (vx * c - vy * s) * dt
        f[Y, VX] = s * dt
        f[Y, VY] = c * dt
        f[THETA, BG_Z] = -dt
        f[VX, VY] = wz * dt
        f[VX, BA_X] = -dt
        f[VX, BG_Z] = -vy * dt
        f[VY, VX] = -wz * dt
        f[VY, BA_Y] = -dt
        f[VY, BG_Z] = vx * dt
        f[OMEGA, OMEGA] = 0.0
        f[OMEGA, BG_Z] = -1.0

        q = np.zeros((STATE_DIM, STATE_DIM))
        q[THETA, THETA] = 0.005**2
        q[VX, VX] = self.params.sigma_ax**2
        q[VY, VY] = self.params.sigma_ay**2
        q[OMEGA, OMEGA] = self.params.sigma_gz**2
        q[BA_X, BA_X] = self.params.sigma_ba_walk**2
        q[BA_Y, BA_Y] = self.params.sigma_ba_walk**2
        q[BG_Z, BG_Z] = self.params.sigma_bg_walk**2

        self._p = f @ self._p @ f.T + q * dt

    def _correct_rpm(self, z_vx: float) -> None:
        h = np.zeros(STATE_DIM)
        h[VX] = 1.0
        y = z_vx - self._x[VX]
        r_rpm = self.params.sigma_rpm**2
        s = float(h @ self._p @ h) + r_rpm
        k = self._p @ h / s
        k[X] = 0.0
        k[Y] = 0.0
        k[THETA] = 0.0
        k[OMEGA] = 0.0
        k[BG_Z] = 0.0
        self._x += k * y
        self._x[THETA] = wrap_pi(self._x[THETA])
        ikh = np.eye(STATE_DIM) - np.outer(k, h)
        self._p = ikh @ self._p @ ikh.T + r_rpm * np.outer(k, k)

    def _correct_nhc(self) -> None:
        h = np.zeros(STATE_DIM)
        h[VY] = 1.0
        sigma_vy = (
            self.params.sigma_vy_nhc_slip
            if self.diagnostics.slip_flag
            else self.params.sigma_vy_nhc
        )
        y = 0.0 - self._x[VY]
        r_nhc = sigma_vy**2
        s = float(h @ self._p @ h) + r_nhc
        k = self._p @ h / s
        k[X] = 0.0
        k[Y] = 0.0
        k[THETA] = 0.0
        k[OMEGA] = 0.0
        k[VX] = 0.0
        # NHC observes VY only. The P[VY, BA_X] coupling (via the wz*vx Coriolis
        # term) is spurious: with RPM present BA_X is anchored, but IMU-only the
        # leak drives ba_x to ~-0.4 m/s^2, injecting a phantom +0.4 m/s^2 into
        # ax = accel_x - BA_X and running vx away. Zero it (same Schmidt-Kalman
        # partition reasoning as BG_Z in #555). Production C++ correct_nhc zeros
        # K[BA_X] too, for the same reason.
        k[BA_X] = 0.0
        k[BG_Z] = 0.0
        self._x += k * y
        self._x[THETA] = wrap_pi(self._x[THETA])
        ikh = np.eye(STATE_DIM) - np.outer(k, h)
        self._p = ikh @ self._p @ ikh.T + r_nhc * np.outer(k, k)

    def _correct_steering(self) -> None:
        vx = self._x[VX]
        l_ = self.params.wheelbase_m
        if l_ <= 1.0e-3:
            return

        self.diagnostics.low_vx_gate_on = False
        if vx < self.params.min_vx_for_steering_correct:
            self.diagnostics.low_vx_gate_on = True
            return

        tan_d = math.tan(self.latest_steering_rad)
        omega_pred = (vx / l_) * tan_d
        residual = omega_pred - self._x[OMEGA]
        self.diagnostics.yaw_residual_rad_s = residual
        self.diagnostics.slip_flag = (
            abs(residual) > self.params.slip_yaw_residual_threshold
        )
        if self.diagnostics.slip_flag:
            return

        h = np.zeros(STATE_DIM)
        h[OMEGA] = 1.0
        y = omega_pred - self._x[OMEGA]
        r_steer = self.params.sigma_steer**2
        s = float(h @ self._p @ h) + r_steer
        k = self._p @ h / s
        k[X] = 0.0
        k[Y] = 0.0
        k[THETA] = 0.0
        k[VX] = 0.0
        k[VY] = 0.0
        k[BA_X] = 0.0
        k[BA_Y] = 0.0
        k[BG_Z] = 0.0
        self._x += k * y
        self._x[THETA] = wrap_pi(self._x[THETA])
        ikh = np.eye(STATE_DIM) - np.outer(k, h)
        self._p = ikh @ self._p @ ikh.T + r_steer * np.outer(k, k)

    def _accumulate_calibration(
        self,
        t: float,
        accel: np.ndarray,
        gyro: np.ndarray,
    ) -> None:
        if self._calib.t_first is None:
            self._calib.t_first = t

        # Only fold in samples taken at a genuine standstill (wheel speed ~0).
        # A non-stationary window soaks real motion into the bias: a measured
        # +5.2 deg/s turn during the 3 s window became a +5.2 deg/s gyro-bias
        # error that drifted SLAM's heading until it lost lock (the true gyro
        # bias is ~0; the sensor slope vs GT yaw-rate is 0.992). Require at least
        # one /motor_rpm sample so a not-yet-seen rpm (latest_rpm still 0.0)
        # can't masquerade as standstill.
        speed = abs(self.latest_rpm) * self.params.rpm_to_ms
        if self._have_rpm and speed <= self.params.stationary_speed_ms:
            self._calib.accel_sum += accel
            self._calib.gyro_sum += gyro
            self._calib.n_samples += 1

        if (t - self._calib.t_first) < self.params.calibration_seconds:
            return

        if self._calib.n_samples > 0:
            n = float(self._calib.n_samples)
            accel_mean = self._calib.accel_sum / n
            gyro_mean = self._calib.gyro_sum / n
            self._calib.accel_bias = accel_mean - np.array([0.0, 0.0, G])
            self._calib.gyro_bias = gyro_mean
        else:
            # No standstill captured in the window (recorder opened late / bag
            # starts mid-motion). Don't fabricate a bias from moving data —
            # default to zero. The gyro is accurate (slope ~1), so 0 is far
            # closer to truth than a contaminated mean, and RPM aiding anchors
            # vx regardless of the accel bias.
            self._calib.accel_bias = np.zeros(3)
            self._calib.gyro_bias = np.zeros(3)
        self._calib.completed = True

        self._x.fill(0.0)
        self._x[BA_X] = self._calib.accel_bias[0]
        self._x[BA_Y] = self._calib.accel_bias[1]
        self._x[BG_Z] = self._calib.gyro_bias[2]
        _set_initial_covariance(self._p)
        self._publish_state_view()
        self._t_imu_last = t

    def _publish_state_view(self) -> None:
        self.state.x = self._x[X]
        self.state.y = self._x[Y]
        self.state.yaw = self._x[THETA]
        self.state.vx = self._x[VX]
        self.state.vy = self._x[VY]
        self.state.yaw_rate = self._x[OMEGA]


class OdometryFilterCpp:
    """The production 9-state EKF for benchmark replay.

    Delegates the EKF (predict + RPM/steering/NHC corrections + stationary
    calibration) to the compiled C++ ``odometry_filter_py`` bindings when
    available — exactly what ``odometry_filter_node`` runs — and to
    ``_PyOdometryFilter`` otherwise. ``backend`` reports which.

    ``push_wheel_sensors`` is an open-loop wheel/steering dead-reckoner used
    only as a comparison baseline. It is NOT the EKF and not a port of any
    production code, so it integrates a small Python "shadow" pose here
    regardless of backend; once it's used on an instance, ``state`` returns
    that shadow (the EKF and the wheel DR are never mixed on one instance —
    the benchmark dedicates a separate filter to each).
    """

    def __init__(self, params: EkfParams | None = None) -> None:
        self.params = params or EkfParams()
        if _NATIVE_AVAILABLE:
            self._ekf = _native.OdometryFilter(_to_native_params(self.params))
            self.backend = "cpp"
        else:
            self._ekf = _PyOdometryFilter(self.params)
            self.backend = "python"
        # Latest raw wheel inputs (read/written by the benchmark for the
        # wheel-DR path; mirror the legacy port's public attributes).
        self.latest_rpm: float = 0.0
        self.latest_steering_rad: float = 0.0
        # Open-loop wheel-DR shadow state (only active once
        # push_wheel_sensors is called on this instance).
        self._wheel_mode = False
        self._wx = np.zeros(STATE_DIM)
        self._wheel_t_last: float | None = None
        self._wheel_state = OdometryState()
        self._wheel_diag = FilterDiagnostics()

    # ----- EKF API: straight delegation to the active backend -------------
    def reset(self) -> None:
        self._ekf.reset()
        self.latest_rpm = 0.0
        self.latest_steering_rad = 0.0
        self._wheel_mode = False
        self._wx.fill(0.0)
        self._wheel_t_last = None
        self._wheel_state = OdometryState()
        self._wheel_diag = FilterDiagnostics()

    def is_calibrated(self) -> bool:
        return self._ekf.is_calibrated()

    @property
    def state(self) -> OdometryState:
        return self._wheel_state if self._wheel_mode else self._ekf.state

    @property
    def diagnostics(self) -> FilterDiagnostics:
        return self._wheel_diag if self._wheel_mode else self._ekf.diagnostics

    def push_imu(self, t: float, accel: np.ndarray, gyro: np.ndarray) -> None:
        self._ekf.push_imu(t, accel, gyro)

    def push_rpm(self, t: float, rpm: float) -> None:
        self.latest_rpm = float(rpm)
        self._ekf.push_rpm(t, rpm)

    def push_steering(self, t: float, angle_rad: float) -> None:
        self.latest_steering_rad = float(angle_rad)
        self._ekf.push_steering(t, angle_rad)

    def seed_forward_velocity(self, vx: float) -> None:
        self._ekf.seed_forward_velocity(vx)

    # ----- Open-loop wheel dead-reckoning (diagnostic baseline) -----------
    def push_wheel_sensors(
        self,
        t: float,
        rpm: float,
        steering_rad: float,
        *,
        steering_units: str = "radians",
    ) -> None:
        """Open-loop wheel DR: ``vx = rpm·rpm_to_ms``,
        ``omega = (vx/L)·tan(delta_road)`` (no IMU, no EKF), integrated with a
        midpoint heading. Anchors at the EKF's calibrated origin on first use.
        """
        if not self._ekf.is_calibrated():
            return
        road = steering_to_road_wheel_rad(steering_rad, units=steering_units)
        self.latest_rpm = float(rpm)
        self.latest_steering_rad = road

        if not self._wheel_mode:
            # Hand over from the EKF: seed the shadow at the current
            # (calibrated, origin) pose and establish the time baseline.
            st = self._ekf.state
            self._wx.fill(0.0)
            self._wx[X], self._wx[Y], self._wx[THETA] = st.x, st.y, st.yaw
            self._wheel_mode = True
            self._wheel_t_last = t
            self._publish_wheel_view()
            return

        dt = t - self._wheel_t_last
        self._wheel_t_last = t
        if dt < self.params.dt_min or dt > self.params.dt_max:
            return

        vx = self.latest_rpm * self.params.rpm_to_ms
        omega = kinematic_omega(vx, road, self.params.wheelbase_m)
        self._wheel_diag.yaw_residual_rad_s = omega
        self._wheel_diag.slip_flag = (
            abs(omega) > self.params.slip_yaw_residual_threshold
        )
        self._wheel_diag.low_vx_gate_on = (
            vx < self.params.min_vx_for_steering_correct
        )

        theta = self._wx[THETA]
        theta_mid = theta + 0.5 * omega * dt
        c = math.cos(theta_mid)
        s = math.sin(theta_mid)
        self._wx[X] += vx * c * dt
        self._wx[Y] += vx * s * dt
        self._wx[THETA] = wrap_pi(theta + omega * dt)
        self._wx[VX] = vx
        self._wx[VY] = 0.0
        self._wx[OMEGA] = omega
        self._publish_wheel_view()

    def _publish_wheel_view(self) -> None:
        self._wheel_state = OdometryState(
            x=self._wx[X], y=self._wx[Y], yaw=self._wx[THETA],
            vx=self._wx[VX], vy=self._wx[VY], yaw_rate=self._wx[OMEGA],
        )
