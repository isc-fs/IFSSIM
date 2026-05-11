"""Vehicle state contract — shared by all controllers (lateral, longitudinal,
future LQR/MPC).

Single source of truth for "what the controller knows about the car this tick".
Populated once per tick by the ROS node from /cone_slam/state Odometry, then
passed by reference into every controller.compute() call.

Conventions (ISO 8855, matches cone_slam frame):
  - x forward, y left, z up
  - yaw measured CCW from +x
  - vx is longitudinal (along car heading); vy lateral (left positive)
  - all units SI (m, m/s, rad, rad/s)
"""
from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleState:
    # Pose in odom frame
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    # Body-frame velocity (twist in child_frame_id from Odometry)
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    # Optional: body-frame acceleration if/when we wire IMU through
    ax: float = 0.0
    ay: float = 0.0

    @property
    def speed(self) -> float:
        """Longitudinal speed magnitude (|vx|).

        Was previously √(vx² + vy²), the vector-magnitude reading.
        Looks right on paper but in practice `vy` from the
        supervisor's OdometryFilter (and the C++ port that
        preserves the same algorithm) accumulates centripetal-accel
        integration noise during cornering — measured peak
        |/odom.vy| = 4.79 m/s in the bag captured during the
        2026-05-11 trackdrive analysis (see bags/lap_attempt_*),
        with speed-inflation peaking at 2.59× of true |vx|.

        That contamination poisoned every consumer that read
        `state.speed`:
          * PIVelocity used `state.vx` directly and was fine.
          * PurePursuit's `Ld = lookahead_min + lookahead_k · speed`
            grew with the inflated speed, which actually *helped*
            it survive earlier corners by accident (longer lookahead
            → smoother tracking). A previous attempt to fix
            (fix/425-speed-isolation-from-vy-drift) was reverted
            because removing the contamination shortened Ld in
            corners and triggered bang-bang earlier on the lap.
          * Any future controller reading `state.speed` would have
            inherited the same lie.

        This fix lands together with a Pure Pursuit lookahead
        retune (lookahead_min/k bumped) so the effective Ld at the
        old corner speeds stays near where the controller was tuned.
        See pipeline/control/control/control_node.py params for the
        matching values.

        If a future controller genuinely needs vector speed (e.g.
        kinetic-energy budgeting), add a separate `vector_speed`
        property — leaving this one as the longitudinal contract.
        """
        return abs(self.vx)
