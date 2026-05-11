// Copyright 2026 IFSSIM contributors.
//
// 9-state EKF — implementation. See odometry_ekf.hpp for the design.

#include "odometry_filter/odometry_ekf.hpp"

#include <algorithm>
#include <cmath>

namespace odometry_filter {

namespace {
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kG = 9.81;
}  // namespace


// State-vector indices — keep magic numbers out of the math.
namespace S {
constexpr int X = 0;
constexpr int Y = 1;
constexpr int YAW = 2;
constexpr int VX = 3;
constexpr int VY = 4;
constexpr int W  = 5;   // yaw rate
constexpr int B_AX = 6;
constexpr int B_AY = 7;
constexpr int B_GZ = 8;
}  // namespace S


OdometryEkf::OdometryEkf(const EkfParams & params) : params_(params) {
  reset();
}

void OdometryEkf::reset() {
  state_ = OdometryState{};
  diag_ = FilterDiagnostics{};
  x_.setZero();
  P_.setZero();
  // Diagonal initial covariance — 1σ values squared.
  P_(S::X,    S::X)    = params_.p0_pos_m       * params_.p0_pos_m;
  P_(S::Y,    S::Y)    = params_.p0_pos_m       * params_.p0_pos_m;
  P_(S::YAW,  S::YAW)  = params_.p0_yaw_rad     * params_.p0_yaw_rad;
  P_(S::VX,   S::VX)   = params_.p0_vx_mps      * params_.p0_vx_mps;
  P_(S::VY,   S::VY)   = params_.p0_vy_mps      * params_.p0_vy_mps;
  P_(S::W,    S::W)    = params_.p0_omega_radps * params_.p0_omega_radps;
  P_(S::B_AX, S::B_AX) = params_.p0_b_ax_mps2   * params_.p0_b_ax_mps2;
  P_(S::B_AY, S::B_AY) = params_.p0_b_ay_mps2   * params_.p0_b_ay_mps2;
  P_(S::B_GZ, S::B_GZ) = params_.p0_b_gz_radps  * params_.p0_b_gz_radps;

  calib_completed_ = false;
  n_calib_samples_ = 0;
  calib_accel_sum_.setZero();
  calib_gyro_sum_.setZero();
  t_calib_first_.reset();
  t_imu_last_.reset();
  t_rpm_last_.reset();
  latest_rpm_vx_.reset();
  latest_steering_rad_ = 0.0;
  latest_brake_ = 0.0;
}


double OdometryEkf::wrap_yaw(double yaw) {
  while (yaw > M_PI)  yaw -= kTwoPi;
  while (yaw < -M_PI) yaw += kTwoPi;
  return yaw;
}


void OdometryEkf::publish_state() {
  state_.x = x_(S::X);
  state_.y = x_(S::Y);
  state_.yaw = x_(S::YAW);
  state_.vx = x_(S::VX);
  state_.vy = x_(S::VY);
  state_.yaw_rate = x_(S::W);
}


void OdometryEkf::accumulate_calibration(
  double t,
  const Eigen::Vector3d & accel,
  const Eigen::Vector3d & gyro)
{
  if (!t_calib_first_.has_value()) {
    t_calib_first_ = t;
  }
  calib_accel_sum_ += accel;
  calib_gyro_sum_  += gyro;
  ++n_calib_samples_;

  if ((t - *t_calib_first_) < params_.calibration_seconds) {
    return;
  }
  if (n_calib_samples_ == 0) {
    return;
  }

  // Seed biases as mean of stationary readings minus gravity. Matches
  // the complementary filter's seeding policy exactly so unit tests
  // can swap between filters without re-tuning bias expectations.
  const Eigen::Vector3d accel_mean =
    calib_accel_sum_ / static_cast<double>(n_calib_samples_);
  const Eigen::Vector3d gyro_mean =
    calib_gyro_sum_  / static_cast<double>(n_calib_samples_);

  // Only b_ax, b_ay, b_gz are state — store the seeds.
  x_(S::B_AX) = accel_mean(0);              // body-X (forward) bias
  x_(S::B_AY) = accel_mean(1);              // body-Y (left)    bias
  x_(S::B_GZ) = gyro_mean(2);               // body-Z (up)      gyro bias
  // accel-Z is just gravity at rest, we trust the model; not a state.

  // Anchor pose at origin, velocity zero (just verified stationary).
  x_(S::X) = 0.0;
  x_(S::Y) = 0.0;
  x_(S::YAW) = 0.0;
  x_(S::VX) = 0.0;
  x_(S::VY) = 0.0;
  x_(S::W) = 0.0;

  // Tighten bias covariance — we just measured them.
  P_(S::B_AX, S::B_AX) *= 0.1;
  P_(S::B_AY, S::B_AY) *= 0.1;
  P_(S::B_GZ, S::B_GZ) *= 0.1;

  calib_completed_ = true;
  t_imu_last_ = t;
  publish_state();
}


void OdometryEkf::predict(
  double dt,
  const Eigen::Vector3d & accel_body,
  const Eigen::Vector3d & gyro_body)
{
  // Bias-corrected readings.
  const double ax = accel_body(0) - x_(S::B_AX);
  const double ay = accel_body(1) - x_(S::B_AY);
  const double wz = gyro_body(2)  - x_(S::B_GZ);

  // Cache pre-update yaw + velocities for the Jacobian.
  const double yaw0 = x_(S::YAW);
  const double vx0  = x_(S::VX);
  const double vy0  = x_(S::VY);
  const double c = std::cos(yaw0);
  const double s = std::sin(yaw0);

  // ----- State update (Euler integration) -----
  // Position (rotate body velocity into world).
  x_(S::X)   += (c * vx0 - s * vy0) * dt;
  x_(S::Y)   += (s * vx0 + c * vy0) * dt;
  // Yaw and yaw rate.
  x_(S::YAW)  = wrap_yaw(yaw0 + wz * dt);
  x_(S::W)    = wz;
  // Body-frame velocities (bias-corrected accel integrated). Vy gets
  // a slow leak — same kBetaVyLeak the complementary filter uses
  // (kinematic-bicycle assumption: vy ≈ 0 over a 2.5 s time constant).
  // Without this, IMU body-Y accel during sustained cornering
  // (centripetal, real, ~m/s² for FS speeds) integrates into vy
  // unbounded. With NHC additionally on, the EKF gets two
  // mutually reinforcing mechanisms to bound vy: the leak inside
  // predict + the NHC measurement during straights that also
  // updates b_ay through covariance coupling.
  x_(S::VX)  += ax * dt;
  x_(S::VY)   = (1.0 - kBetaVyLeak) * x_(S::VY) + ay * dt;
  // Biases — random walk; no state-vector change in predict, but Q
  // adds variance.

  // ----- Jacobian F = ∂f/∂x evaluated at (state, dt) -----
  Jac9 F = Jac9::Identity();
  // ∂x/∂yaw = (-sin(yaw)·vx - cos(yaw)·vy) · dt
  F(S::X, S::YAW) = (-s * vx0 - c * vy0) * dt;
  // ∂x/∂vx = cos(yaw)·dt; ∂x/∂vy = -sin(yaw)·dt
  F(S::X, S::VX)  =  c * dt;
  F(S::X, S::VY)  = -s * dt;
  // ∂y/∂yaw, ∂y/∂vx, ∂y/∂vy
  F(S::Y, S::YAW) = ( c * vx0 - s * vy0) * dt;
  F(S::Y, S::VX)  =  s * dt;
  F(S::Y, S::VY)  =  c * dt;
  // ∂yaw/∂omega = dt — yaw integrates omega. Wait, yaw_new = yaw + wz*dt
  // and wz = gyro_z - b_gz. So ∂yaw/∂b_gz = -dt.
  F(S::YAW, S::W)   = 0.0;       // omega is measured, not propagated through ∂yaw
  F(S::YAW, S::B_GZ) = -dt;       // yaw depends on b_gz via wz integration
  // omega state IS the bias-corrected gyro; ∂omega/∂b_gz = -1.
  F(S::W, S::W)     = 0.0;        // overwritten by measurement, not propagated
  F(S::W, S::B_GZ)  = -1.0;
  // vx, vy depend on bias.
  F(S::VX, S::B_AX) = -dt;
  F(S::VY, S::B_AY) = -dt;

  // ----- Process noise Q (continuous-time, scaled by dt) -----
  // Diagonal — assumes uncorrelated noise sources. 1σ values squared,
  // times dt to convert spectral density → variance per step.
  Cov9 Q = Cov9::Zero();
  Q(S::X,    S::X)    = params_.q_pos_m       * params_.q_pos_m       * dt;
  Q(S::Y,    S::Y)    = params_.q_pos_m       * params_.q_pos_m       * dt;
  Q(S::YAW,  S::YAW)  = params_.q_yaw_rad     * params_.q_yaw_rad     * dt;
  Q(S::VX,   S::VX)   = params_.q_vx_mps      * params_.q_vx_mps      * dt;
  Q(S::VY,   S::VY)   = params_.q_vy_mps      * params_.q_vy_mps      * dt;
  Q(S::W,    S::W)    = params_.q_omega_radps * params_.q_omega_radps * dt;
  Q(S::B_AX, S::B_AX) = params_.q_b_ax_mps2   * params_.q_b_ax_mps2   * dt;
  Q(S::B_AY, S::B_AY) = params_.q_b_ay_mps2   * params_.q_b_ay_mps2   * dt;
  Q(S::B_GZ, S::B_GZ) = params_.q_b_gz_radps  * params_.q_b_gz_radps  * dt;

  // P = F·P·Fᵀ + Q
  P_ = F * P_ * F.transpose() + Q;
}


void OdometryEkf::update_scalar(const Obs1 & H, double z, double r) {
  // 1-D EKF update — innovation y = z - H·x, S = H·P·Hᵀ + R, K = P·Hᵀ / S.
  const double y = z - (H * x_)(0);
  const double s = (H * P_ * H.transpose())(0, 0) + r;
  if (s <= 0.0) {
    return;  // numerical guard — should never happen with finite P/R
  }
  const Eigen::Matrix<double, kN, 1> K = (P_ * H.transpose()) / s;
  x_ += K * y;
  // Joseph form: (I - K·H)·P·(I - K·H)ᵀ + K·R·Kᵀ — symmetry-preserving.
  const Jac9 IKH = Jac9::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + (K * r) * K.transpose();
  // Re-wrap yaw if the update bumped it past pi.
  x_(S::YAW) = wrap_yaw(x_(S::YAW));
}


void OdometryEkf::push_imu(
  double t,
  const Eigen::Vector3d & accel,
  const Eigen::Vector3d & gyro)
{
  if (!calib_completed_) {
    accumulate_calibration(t, accel, gyro);
    return;
  }

  if (!t_imu_last_.has_value()) {
    t_imu_last_ = t;
    return;
  }
  const double dt = t - *t_imu_last_;
  t_imu_last_ = t;
  if (dt <= 0.0 || dt > 0.1) {
    return;  // clock glitch — skip
  }

  predict(dt, accel, gyro);

  // Kinematic-bicycle yaw-rate cross-check — same as the complementary
  // filter, preserves the slip-flag contract on /odom_diag/.
  if (params_.wheelbase_m > 1e-3) {
    const double omega_pred =
      x_(S::VX) / params_.wheelbase_m * std::tan(latest_steering_rad_);
    diag_.yaw_residual_rad_s = omega_pred - x_(S::W);
    diag_.slip_flag =
      std::abs(diag_.yaw_residual_rad_s) >
      kSlipYawResidualThreshold;  // re-use constant from complementary filter
  }

  // ----- Phase 2b (off by default): NHC measurement updates -----
  if (params_.enable_nhc) {
    const bool clean_straight =
      std::abs(latest_steering_rad_) < params_.clean_straight_steering_rad &&
      std::abs(x_(S::W)) < params_.clean_straight_omega_radps &&
      latest_brake_ < params_.clean_straight_brake;
    if (clean_straight) {
      // vy ≈ 0 update
      Obs1 H_vy = Obs1::Zero();
      H_vy(0, S::VY) = 1.0;
      update_scalar(H_vy, 0.0,
                    params_.r_nhc_vy_mps * params_.r_nhc_vy_mps);
      // omega ≈ 0 update — directly bounds gyro bias
      Obs1 H_w = Obs1::Zero();
      H_w(0, S::W) = 1.0;
      update_scalar(H_w, 0.0,
                    params_.r_nhc_omega_radps * params_.r_nhc_omega_radps);
    }
  }

  publish_state();
}


void OdometryEkf::push_rpm(double t, double rpm) {
  t_rpm_last_ = t;
  latest_rpm_vx_ = rpm * params_.rpm_to_ms;

  if (!calib_completed_) {
    return;
  }

  // Brake-lockup gating — same heuristic as the complementary filter.
  // When brake authority is high the rear wheels can lock; the RPM
  // measurement is no longer trustworthy. We widen R sharply so the
  // EKF effectively ignores the update.
  double r_eff = params_.r_rpm_vx_mps * params_.r_rpm_vx_mps;
  if (latest_brake_ > kBrakeLockupThreshold) {
    r_eff *= 100.0;  // 10× σ → 100× variance
  }
  diag_.effective_alpha_vx = r_eff < 1e6 ? params_.r_rpm_vx_mps : kAlphaVxBrake;

  Obs1 H = Obs1::Zero();
  H(0, S::VX) = 1.0;
  update_scalar(H, *latest_rpm_vx_, r_eff);
  publish_state();
}


void OdometryEkf::push_steering(double /*t*/, double angle_rad) {
  latest_steering_rad_ = angle_rad;
}


void OdometryEkf::push_brake(double /*t*/, double brake) {
  latest_brake_ = std::clamp(brake, 0.0, 1.0);
}

}  // namespace odometry_filter
