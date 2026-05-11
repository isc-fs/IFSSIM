// Copyright 2026 IFSSIM contributors.
//
// 9-state Extended Kalman Filter for IFS-08 dead-reckoning.
//
// Sibling implementation to `OdometryFilter` (the complementary
// filter). Same public surface — push_imu / push_rpm / push_steering
// / push_brake / state() / diagnostics() / is_calibrated() — so the
// rclcpp wrapper can choose between them via a parameter. Adds
// explicit accel and gyro bias states, the missing piece that the
// complementary filter and robot_localization both lacked
// (issue #447, Phase 2).
//
// State vector x (9 elements):
//
//     [0] x_w   — world-frame position X (m)
//     [1] y_w   — world-frame position Y (m)
//     [2] yaw   — world-frame heading (rad, wrapped to (-pi, pi])
//     [3] vx    — body-frame longitudinal velocity (m/s)
//     [4] vy    — body-frame lateral velocity (m/s)
//     [5] omega — yaw rate (rad/s)
//     [6] b_ax  — accel-X bias, body frame (m/s²)
//     [7] b_ay  — accel-Y bias, body frame (m/s²)
//     [8] b_gz  — gyro-Z bias (rad/s)
//
// IMU is the predict input — gyro_z drives state[5]; accel_x/y drive
// state[3]/state[4]; biases are subtracted, then random-walked in Q.
// RPM is a vx measurement update on state[3].
// NHC (non-holonomic constraint) during clean straights provides
// soft measurement updates of state[4] ≈ 0 and state[5] ≈ 0, which
// flow through the cross-covariance into b_ay and b_gz — the online
// bias-correction mechanism that bounds drift.
//
// All matrices are fixed-size Eigen (`Matrix<double, 9, 9>`),
// stack-allocated, no heap. The library compiles as part of
// libodometry_filter and can be linked into the rclcpp node, into
// a uDV firmware image, or into any harness — same constraints as
// the existing complementary filter.

#ifndef ODOMETRY_FILTER__ODOMETRY_EKF_HPP_
#define ODOMETRY_FILTER__ODOMETRY_EKF_HPP_

#include <Eigen/Core>
#include <optional>

#include "odometry_filter/odometry_filter.hpp"  // re-uses OdometryState + FilterDiagnostics

namespace odometry_filter {

// ---------------------------------------------------------------------
// EKF tuning constants — diagonal entries of the noise matrices.
// All in SI. All can be overridden per-instance via EkfParams below.
// ---------------------------------------------------------------------

// Process-noise (Q) — how fast we believe each state can drift
// between IMU samples, expressed as 1-sigma per second. Squared and
// scaled by dt inside the predict step. Values calibrated against
// BMI088 datasheet typicals + bag-replay sanity.
constexpr double kEkfQ_pos_m         = 0.05;   // m / sqrt(s)
constexpr double kEkfQ_yaw_rad       = 0.005;
constexpr double kEkfQ_vx_mps        = 0.20;
constexpr double kEkfQ_vy_mps        = 0.30;
constexpr double kEkfQ_omega_radps   = 0.05;
constexpr double kEkfQ_b_ax_mps2     = 0.001;  // bias random-walk
constexpr double kEkfQ_b_ay_mps2     = 0.001;
constexpr double kEkfQ_b_gz_radps    = 0.0005;

// Measurement-noise — 1-sigma per sample.
constexpr double kEkfR_rpm_vx_mps    = 0.05;
constexpr double kEkfR_nhc_vy_mps    = 0.05;
constexpr double kEkfR_nhc_omega_radps = 0.02;

// Initial covariance — how unsure we are at t=0 (after calibration).
// Pose tight (sim spawns at known origin); velocities loose; biases
// loose-but-bounded.
constexpr double kEkfP0_pos_m        = 1e-3;
constexpr double kEkfP0_yaw_rad      = 1e-3;
constexpr double kEkfP0_vx_mps       = 0.1;
constexpr double kEkfP0_vy_mps       = 0.1;
constexpr double kEkfP0_omega_radps  = 0.05;
constexpr double kEkfP0_b_ax_mps2    = 0.05;
constexpr double kEkfP0_b_ay_mps2    = 0.05;
constexpr double kEkfP0_b_gz_radps   = 0.02;

// "Clean straight" gate for NHC measurement updates. Mirrors the
// kinematic-bicycle cross-check thresholds already used for slip
// detection (FilterDiagnostics::slip_flag).
constexpr double kEkfCleanStraight_steering_rad = 0.05;
constexpr double kEkfCleanStraight_omega_radps  = 0.10;
constexpr double kEkfCleanStraight_brake        = 0.10;


struct EkfParams {
  // Inherit the complementary filter's mechanical constants. RPM_TO_MS,
  // wheelbase, calibration time — all same.
  double rpm_to_ms = kRpmToMs;
  double rpm_stale_s = kRpmStaleS;
  double calibration_seconds = kCalibrationSeconds;
  double wheelbase_m = kWheelbaseM;

  // Tuning — all the kEkf* values above, exposed for unit tests +
  // future per-vehicle tuning.
  double q_pos_m        = kEkfQ_pos_m;
  double q_yaw_rad      = kEkfQ_yaw_rad;
  double q_vx_mps       = kEkfQ_vx_mps;
  double q_vy_mps       = kEkfQ_vy_mps;
  double q_omega_radps  = kEkfQ_omega_radps;
  double q_b_ax_mps2    = kEkfQ_b_ax_mps2;
  double q_b_ay_mps2    = kEkfQ_b_ay_mps2;
  double q_b_gz_radps   = kEkfQ_b_gz_radps;

  double r_rpm_vx_mps         = kEkfR_rpm_vx_mps;
  double r_nhc_vy_mps         = kEkfR_nhc_vy_mps;
  double r_nhc_omega_radps    = kEkfR_nhc_omega_radps;

  double p0_pos_m       = kEkfP0_pos_m;
  double p0_yaw_rad     = kEkfP0_yaw_rad;
  double p0_vx_mps      = kEkfP0_vx_mps;
  double p0_vy_mps      = kEkfP0_vy_mps;
  double p0_omega_radps = kEkfP0_omega_radps;
  double p0_b_ax_mps2   = kEkfP0_b_ax_mps2;
  double p0_b_ay_mps2   = kEkfP0_b_ay_mps2;
  double p0_b_gz_radps  = kEkfP0_b_gz_radps;

  double clean_straight_steering_rad = kEkfCleanStraight_steering_rad;
  double clean_straight_omega_radps  = kEkfCleanStraight_omega_radps;
  double clean_straight_brake        = kEkfCleanStraight_brake;

  // Toggle NHC measurement updates. ON by default — without it,
  // IMU body-y accel during cornering (real centripetal, ~m/s²)
  // integrates into vy unbounded over a lap. NHC fires during
  // clean-straight windows and bounds vy ≈ 0 + bounds b_ay
  // through state covariance coupling. Disable only for unit tests
  // that need to verify the predict math in isolation.
  bool enable_nhc = true;
};


class OdometryEkf : public IOdometryFilter {
 public:
  // Fixed-size linear-algebra primitives — never heap.
  static constexpr int kN = 9;
  using State9  = Eigen::Matrix<double, kN, 1>;
  using Cov9    = Eigen::Matrix<double, kN, kN>;
  using Jac9    = Eigen::Matrix<double, kN, kN>;
  using Obs1    = Eigen::Matrix<double, 1, kN>;

  OdometryEkf() : OdometryEkf(EkfParams{}) {}
  explicit OdometryEkf(const EkfParams & params);

  // ----- Public read-only accessors (matches OdometryFilter) -----
  const OdometryState & state() const noexcept override { return state_; }
  const FilterDiagnostics & diagnostics() const noexcept override { return diag_; }
  bool is_calibrated() const noexcept override { return calib_completed_; }

  void reset() override;

  // ----- Ingestion API (matches OdometryFilter) -----
  void push_imu(
    double t,
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & gyro) override;
  void push_rpm(double t, double rpm) override;
  void push_steering(double t, double angle_rad) override;
  void push_brake(double t, double brake) override;

  // ----- Test-only state access -----
  Eigen::Vector3d accel_bias_for_tests() const {
    return {x_(6), x_(7), 0.0};   // EKF tracks only ax, ay biases
  }
  Eigen::Vector3d gyro_bias_for_tests() const {
    return {0.0, 0.0, x_(8)};     // EKF tracks only gz bias
  }
  int n_calibration_samples_for_tests() const noexcept {
    return n_calib_samples_;
  }
  const Cov9 & covariance_for_tests() const noexcept { return P_; }

 private:
  // Calibration: average the first kCalibrationSeconds of IMU
  // samples to seed the initial bias estimates. Same heuristic as
  // the complementary filter; the EKF will then update them online.
  void accumulate_calibration(
    double t,
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & gyro);

  // EKF predict — IMU as control input, advances state + covariance.
  void predict(
    double dt,
    const Eigen::Vector3d & accel_body,
    const Eigen::Vector3d & gyro_body);

  // EKF update — scalar measurement on row of x.
  // H is 1x9; z is the scalar measurement; r is its variance.
  void update_scalar(const Obs1 & H, double z, double r);

  // Sync the lightweight OdometryState struct from the internal
  // state vector. Called whenever the EKF state is mutated.
  void publish_state();

  // Wrap yaw to (-pi, pi].
  static double wrap_yaw(double yaw);

  EkfParams params_;
  OdometryState state_;
  FilterDiagnostics diag_;

  // Internal full state + covariance.
  State9 x_;
  Cov9   P_;

  // Calibration accumulators.
  bool calib_completed_{false};
  int  n_calib_samples_{0};
  Eigen::Vector3d calib_accel_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d calib_gyro_sum_{Eigen::Vector3d::Zero()};
  std::optional<double> t_calib_first_{};

  std::optional<double> t_imu_last_{};
  std::optional<double> t_rpm_last_{};
  std::optional<double> latest_rpm_vx_{};

  double latest_steering_rad_{0.0};
  double latest_brake_{0.0};
};

}  // namespace odometry_filter

#endif  // ODOMETRY_FILTER__ODOMETRY_EKF_HPP_
