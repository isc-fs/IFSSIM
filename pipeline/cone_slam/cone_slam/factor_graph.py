"""GTSAM iSAM2 wrapper for the cone-graph SLAM node.

Position-only graph: pose nodes (X) and landmark nodes (L) are the
ONLY variables. Connected by:
  - BetweenFactorPose3: scan-to-scan pose delta, computed externally
    by the IMU preintegrator using a bias from the decoupled bias EKF
    (`cone_slam/bias_ekf.py`). No `B(k)` node, no `ImuFactor`.
  - BearingRangeFactor3D: cone observations from the LiDAR.

Symbols:
  X(k) = gtsam.symbol('x', k)   # Pose3 at scan k
  L(id) = gtsam.symbol('l', id) # Point3 cone landmark

Why drop V(k) and B(k):
A wrong cone bearing-range factor in the joint optimisation
satisfies its residual partly by adjusting the bias node, which then
contaminates future IMU predictions and feeds back into more wrong
DA. We saw this end-to-end on test_submodule: a single mis-DA at
~tick 460 of an otherwise-healthy lap-long run snapped pose, the
cascade detector skipped subsequent scans, IMU dead-reckoning
accumulated drift, and the run was unrecoverable. Decoupling bias
estimation cuts that feedback path.

This is the AMZ / KIT / QUTMS pattern (FS-DV literature survey
2026-05-04). The IMU+RPM filter (`bias_ekf.BiasEKF`) owns velocity
and bias; the SLAM consumes scan-to-scan pose deltas only.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

import gtsam
from gtsam.symbol_shorthand import X, L  # type: ignore[attr-defined]


# Anchor prior — the very first pose is locked at world origin with
# tight covariance.
PRIOR_POSE_SIGMAS = np.array([0.001, 0.001, 0.001,  # roll/pitch/yaw rad
                              0.001, 0.001, 0.001])  # x/y/z m

# Default per-scan BetweenFactorPose3 sigmas. Loose enough to absorb
# IMU integration noise over 100 ms; not so tight that the linearized
# Hessian becomes ill-conditioned when cone-factor residuals are large.
# Earlier values (0.005 rad on roll/pitch, 0.02 m on z) inherited the
# old in-graph IMU-factor regime and triggered IndeterminantLinearSystem
# under residual stress on the position-only graph (2026-05-04 run).
DEFAULT_BETWEEN_POSE_SIGMAS = np.array([
    0.02, 0.02, 0.02,     # roll/pitch/yaw rad — uniform; ground vehicle
    0.10, 0.10, 0.05,     # x/y/z m
])


@dataclass
class ScanResult:
    """What the node gets back after each iSAM2 update.

    Velocity and bias come from the decoupled bias EKF, NOT from
    iSAM2 — they're carried here for downstream consumers (TF /
    state publish, log lines, next-scan IMU prediction). The graph
    itself doesn't track them.
    """
    pose: gtsam.Pose3
    velocity: np.ndarray                     # (3,) m/s in nav frame
    bias: gtsam.imuBias.ConstantBias


class FactorGraph:
    """Thin owner of an iSAM2 instance + symbol counter + the rolling
    'new' pieces that get handed to update() each scan.
    """

    def __init__(self, isam2_relinearize_threshold: float = 0.1) -> None:
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(isam2_relinearize_threshold)
        # Re-linearize at most once every 10 update calls. Standard
        # GTSAM tuning that lets earlier graph state act as a soft
        # anchor on later updates — the "skip=1" alternative cascaded
        # earlier in past experiments on cone-only scenes.
        params.relinearizeSkip = 10
        self._isam = gtsam.ISAM2(params)

        # Accumulators between update() calls. Cleared after each update.
        self._new_factors = gtsam.NonlinearFactorGraph()
        self._new_values = gtsam.Values()

        self._k: int = 0  # scan index. 0 is the anchor.

    # ----- INIT (called once) ------------------------------------------------

    def initialize_anchor(self, initial_pose: gtsam.Pose3) -> None:
        """Add x_0 with a tight prior and run the first update."""
        if self._k != 0:
            raise RuntimeError("initialize_anchor() called more than once")

        self._new_values.insert(X(0), initial_pose)
        pose_noise = gtsam.noiseModel.Diagonal.Sigmas(PRIOR_POSE_SIGMAS)
        self._new_factors.add(
            gtsam.PriorFactorPose3(X(0), initial_pose, pose_noise))

        self._flush_update()

    # ----- pre-update accumulators (called by node before _flush_update) ---

    def stage_pose_delta(
        self,
        prev_pose: gtsam.Pose3,
        delta_pose: gtsam.Pose3,
        sigmas: Optional[np.ndarray] = None,
    ) -> None:
        """Append the new pose node + a BetweenFactorPose3 from the
        previous scan.

        Args:
            prev_pose: most recently committed pose (used to seed the
                initial estimate of the new node by composing with
                `delta_pose`).
            delta_pose: pose delta between the previous scan and this
                one in the previous-scan body frame, computed externally
                by the IMU preintegrator with the current bias EKF
                estimate.
            sigmas: optional per-axis BetweenFactor sigmas (rot xyz +
                trans xyz, 6 entries). Defaults to
                DEFAULT_BETWEEN_POSE_SIGMAS.
        """
        prev_k = self._k
        self._k += 1
        new_k = self._k

        predicted_pose = prev_pose.compose(delta_pose)
        self._new_values.insert(X(new_k), predicted_pose)

        sig = sigmas if sigmas is not None else DEFAULT_BETWEEN_POSE_SIGMAS
        noise = gtsam.noiseModel.Diagonal.Sigmas(sig)
        self._new_factors.add(gtsam.BetweenFactorPose3(
            X(prev_k), X(new_k), delta_pose, noise))

    def stage_new_landmark(
        self,
        landmark_id: int,
        initial_world_xyz: np.ndarray,
    ) -> None:
        """Insert a brand-new landmark variable plus a z-only anchor.

        The bearing-range factor we stage per observation uses
        `Unit3([body_x, body_y, 0.0])` and a horizontal range, so the
        Jacobian columns of every cone factor against the landmark's
        z are identically zero. With the position-only graph (no
        V(k)/B(k) pivots) this rank-1 deficiency on z compounds across
        landmarks and iSAM2 throws IndeterminantLinearSystem. The prior
        below anchors z near the initial estimate (5 cm 1σ) while
        leaving xy effectively free (10 m 1σ); it adds one constraint
        per landmark and costs nothing at runtime.
        """
        initial_point = gtsam.Point3(*initial_world_xyz)
        self._new_values.insert(L(landmark_id), initial_point)
        z_anchor_sigmas = np.array([10.0, 10.0, 0.05])
        z_anchor_noise = gtsam.noiseModel.Diagonal.Sigmas(z_anchor_sigmas)
        self._new_factors.add(gtsam.PriorFactorPoint3(
            L(landmark_id), initial_point, z_anchor_noise))

    def stage_cone_observation(
        self,
        landmark_id: int,
        body_x: float,
        body_y: float,
        sigma_xy: float = -1.0,
    ) -> None:
        """Add a BearingRange observation between the current pose
        (X(self._k)) and the landmark.

        Wrapped in a Huber robust loss: a single bad data association
        in a cone-only environment can drag the optimizer to a wrong
        global rotation. Huber caps the influence of residuals beyond
        k σ, so outliers stop contributing rather than dominating.

        Observation noise scales linearly with range: cluster point
        count ∝ 1/d² so the centroid's variance grows with distance
        (MUR's `num_expected_points(d)` rule, AMZ §3.2). When the
        detector reports a positive σ_xy, we use that; otherwise fall
        back to a linear-in-range formula.
        """
        bearing = gtsam.Unit3(np.array([body_x, body_y, 0.0]))
        range_m = float(np.hypot(body_x, body_y))

        if sigma_xy > 0.0:
            range_sigma = sigma_xy
            bearing_sigma = sigma_xy / range_m if range_m > 0.5 else 0.2
        else:
            range_sigma = 0.05 + 0.005 * range_m
            bearing_sigma = 0.02 + 0.001 * range_m

        sigmas = np.array([bearing_sigma, bearing_sigma, range_sigma])
        gaussian = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        huber = gtsam.noiseModel.mEstimator.Huber.Create(1.345)
        robust = gtsam.noiseModel.Robust.Create(huber, gaussian)

        self._new_factors.add(gtsam.BearingRangeFactor3D(
            X(self._k), L(landmark_id),
            bearing, range_m, robust,
        ))

    def commit(self) -> gtsam.Pose3:
        """Run iSAM2 with everything staged for this scan and return
        the latest pose estimate at X(self._k).

        Velocity and bias are NOT in the graph — the caller assembles
        a ScanResult separately using the bias-EKF state and a
        velocity derived from the pose delta over the scan period.
        """
        return self._flush_update()

    def landmark_position(self, landmark_id: int) -> Optional[np.ndarray]:
        """Return the latest world-frame Point3 estimate for a
        landmark, or None if iSAM2 hasn't merged it yet."""
        try:
            estimate = self._isam.calculateEstimate()
            return np.array(estimate.atPoint3(L(landmark_id)))
        except Exception:
            return None

    def pose_covariance(
        self, k: Optional[int] = None
    ) -> Optional[np.ndarray]:
        """Return the 6×6 marginal covariance of X(k) in Pose3 tangent
        space. Defaults to k = self._k − 1 (most recently committed
        pose, since the current step's marginal isn't available at DA
        time)."""
        target_k = (self._k - 1) if k is None else k
        if target_k < 0:
            return None
        try:
            return np.array(self._isam.marginalCovariance(X(target_k)))
        except Exception:
            return None

    def landmark_covariance(self, landmark_id: int) -> Optional[np.ndarray]:
        """Return the 3×3 marginal covariance of a landmark's world-
        frame position, or None if iSAM2 hasn't merged it yet."""
        try:
            return np.array(self._isam.marginalCovariance(L(landmark_id)))
        except Exception:
            return None

    # ----- internals ---------------------------------------------------------

    def _flush_update(self) -> gtsam.Pose3:
        """Hand new_factors + new_values to iSAM2, read back the latest
        pose at the current step.
        """
        self._isam.update(self._new_factors, self._new_values)
        # Optional second update for tighter convergence on big residuals.
        self._isam.update()

        # Reset accumulators.
        self._new_factors.resize(0)
        self._new_values.clear()

        estimate = self._isam.calculateEstimate()
        return estimate.atPose3(X(self._k))

    @property
    def step(self) -> int:
        return self._k

    def latest_pose(self) -> Optional[gtsam.Pose3]:
        if self._k < 0:
            return None
        try:
            estimate = self._isam.calculateEstimate()
            return estimate.atPose3(X(self._k))
        except Exception:
            return None
