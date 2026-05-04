"""Decoupled IMU bias estimator.

Lives outside the cone-landmark factor graph. Maintains a current best
estimate of the IMU's accel and gyro biases as a 6-DoF state evolved
by random walk and corrected from RPM-vs-IMU velocity disagreement.

Why decoupled (vs. estimating B(k) inside the cone-graph SLAM):
A wrong cone bearing-range factor in iSAM2's joint optimisation
satisfies its residual partly by adjusting the bias node, which then
contaminates future IMU predictions and feeds back into more wrong
DA. By keeping bias estimation in a separate filter that never sees
cone factors, that feedback path is cut: a wrong cone match can
corrupt the SLAM pose locally, but it cannot reach the bias.

This is the AMZ / KIT / QUTMS pattern (FS-DV literature survey
2026-05-04). The factor graph downstream consumes IMU pose-delta
factors precomputed with the bias from this filter, not raw IMU
factors with an in-graph bias node.

Algorithm (deliberately simple — easy to reason about, easy to swap
out for something fancier later if needed):

  - State: x = [b_ax, b_ay, b_az, b_gx, b_gy, b_gz] ∈ ℝ⁶.
  - Initial value comes from the static calibration window
    (`init_from_calibration`), same as today's `ImuPreintegrator
    .estimate_bias`.
  - Time evolution: random walk with per-axis noise (BMI088 datasheet
    figures, same constants as the existing preintegrator).
  - Measurement update from RPM (`update_from_rpm_velocity`):
    when RPM has been flowing and the LiDAR scan integrated body-x
    velocity disagrees with `RPM × RPM_TO_MS`, the residual is
    attributed to accel-x bias. This is the common-case correction
    for the dominant-noise channel; gyro-bias drift is left to the
    random walk between calibration windows.

Limitations of this v1:
  - No yaw-axis correction beyond random-walk; if gyro-z bias drifts,
    we don't catch it without revisiting calibration.
  - Linear scalar update (innovation projected onto the bias-axis
    most likely to explain it). A fully-coupled 6-DoF measurement
    Jacobian would be more principled but is overkill for the regime
    we operate in.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass

import numpy as np


# Bias random-walk standard deviations (per-axis, per-√s). Match the
# values the preintegrator already uses internally; centralised here
# now that the EKF owns bias evolution.
ACCEL_BIAS_RW_STD = 1e-4   # m/s² per √s
GYRO_BIAS_RW_STD = 1e-5    # rad/s per √s

# RPM-vs-IMU velocity disagreement noise. The wheel-derived velocity
# has slip-related error; we don't trust it for sub-cm bias updates.
# 0.10 m/s 1σ is conservative for FS speeds.
RPM_VELOCITY_SIGMA = 0.10  # m/s

# Initial bias-state covariance — generous to express "we don't know
# better than calibration." Calibration mean has the static-bias
# estimate; this 1σ envelope gives the EKF room to refine via RPM.
INIT_ACCEL_BIAS_SIGMA = 0.05    # m/s²
INIT_GYRO_BIAS_SIGMA  = 0.001   # rad/s


@dataclass
class BiasEstimate:
    """Snapshot of the bias EKF state at a point in time."""
    accel_bias: np.ndarray   # (3,) m/s²
    gyro_bias: np.ndarray    # (3,) rad/s
    covariance: np.ndarray   # (6, 6), [accel; gyro] block-diagonal


class BiasEKF:
    """Decoupled IMU bias filter.

    Thread-safe: the IMU thread can call `predict_through` while the
    cone thread queries `current()` or pushes RPM updates. Internal
    state mutation is small and bounded.
    """

    def __init__(self) -> None:
        self._x = np.zeros(6, dtype=np.float64)         # bias mean
        self._P = np.diag([
            INIT_ACCEL_BIAS_SIGMA ** 2,
            INIT_ACCEL_BIAS_SIGMA ** 2,
            INIT_ACCEL_BIAS_SIGMA ** 2,
            INIT_GYRO_BIAS_SIGMA ** 2,
            INIT_GYRO_BIAS_SIGMA ** 2,
            INIT_GYRO_BIAS_SIGMA ** 2,
        ])
        self._last_predict_t: float = 0.0
        self._initialized: bool = False
        self._lock = threading.RLock()

    # ----- init ------------------------------------------------------------

    def init_from_calibration(
        self,
        accel_bias: np.ndarray,
        gyro_bias: np.ndarray,
        t: float,
    ) -> None:
        """Seed the filter from the static-window calibration."""
        with self._lock:
            self._x[0:3] = np.asarray(accel_bias, dtype=np.float64).flatten()
            self._x[3:6] = np.asarray(gyro_bias, dtype=np.float64).flatten()
            self._last_predict_t = t
            self._initialized = True

    # ----- predict ---------------------------------------------------------

    def predict_through(self, t: float) -> None:
        """Advance state to time `t` via random-walk noise.

        No-op if uninitialized or `t` ≤ last predict time. Idempotent
        in steady-state — the mean doesn't move (RW model), only the
        covariance grows.
        """
        with self._lock:
            if not self._initialized:
                return
            dt = t - self._last_predict_t
            if dt <= 0:
                return
            q_a = (ACCEL_BIAS_RW_STD ** 2) * dt
            q_g = (GYRO_BIAS_RW_STD ** 2) * dt
            self._P[0, 0] += q_a; self._P[1, 1] += q_a; self._P[2, 2] += q_a
            self._P[3, 3] += q_g; self._P[4, 4] += q_g; self._P[5, 5] += q_g
            self._last_predict_t = t

    # ----- update from RPM-vs-IMU velocity --------------------------------

    def update_from_rpm_velocity(
        self,
        v_imu_body_x: float,
        v_rpm_body_x: float,
    ) -> None:
        """Correct accel-x bias from RPM-vs-IMU body-x velocity disagreement.

        `v_imu_body_x` is the IMU-integrated body-x velocity over the
        most recent window (computed by the caller using the current
        bias estimate). `v_rpm_body_x` is the same quantity derived
        from wheel RPM. Innovation = v_rpm − v_imu; we attribute it
        to b_ax via a scalar Kalman update on that one component.

        Limitations: doesn't fight gyro bias or accel y/z bias. Most
        of the drift we see in practice is in accel-x (longitudinal),
        because that's what RPM directly observes and contradicts.
        """
        with self._lock:
            if not self._initialized:
                return
            innov = float(v_rpm_body_x - v_imu_body_x)
            # Scalar Kalman update on b_ax (state index 0). The
            # measurement model maps b_ax → integrated v error linearly;
            # we collapse the integration interval into the gain.
            sigma_v_sq = RPM_VELOCITY_SIGMA ** 2
            P_aa = float(self._P[0, 0])
            S = P_aa + sigma_v_sq
            K = P_aa / S
            self._x[0] += K * innov
            self._P[0, 0] = (1.0 - K) * P_aa

    # ----- accessors -------------------------------------------------------

    def current(self) -> BiasEstimate:
        with self._lock:
            return BiasEstimate(
                accel_bias=self._x[0:3].copy(),
                gyro_bias=self._x[3:6].copy(),
                covariance=self._P.copy(),
            )

    @property
    def initialized(self) -> bool:
        return self._initialized
