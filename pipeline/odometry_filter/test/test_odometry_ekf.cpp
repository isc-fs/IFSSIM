// Copyright 2026 IFSSIM contributors.
//
// Sanity tests for the 9-state EKF (issue #447 Phase 2). Verifies
// the predict / update math and bias-tracking behaviour against
// hand-built scenarios. Replay-bag-level validation lives outside
// the gtest suite (bags/compare_filters.py) — these tests just
// guarantee the math is wired correctly.

#include <gtest/gtest.h>

#include <cmath>

#include "odometry_filter/odometry_ekf.hpp"

using odometry_filter::OdometryEkf;
using odometry_filter::EkfParams;

namespace {

// Push N IMU samples spaced `dt` apart, simulating stationary
// upright car. Used to drive past calibration.
void DriveStationaryCalibration(OdometryEkf & ekf, double dt = 0.003) {
  // Stationary upright: gyro = 0, accel = (0, 0, 0) since the FSDS
  // sim removes gravity at the source (FSDSImuSensor pre-subtracts
  // body-frame gravity).
  const Eigen::Vector3d accel{0.0, 0.0, 0.0};
  const Eigen::Vector3d gyro{0.0, 0.0, 0.0};

  double t = 0.0;
  // Calibration is 3 seconds by default; drive 3.1 s of samples.
  while (t < 3.1) {
    ekf.push_imu(t, accel, gyro);
    t += dt;
  }
}

}  // namespace


TEST(OdometryEkf, CalibratesAfterThreeSeconds) {
  OdometryEkf ekf;
  EXPECT_FALSE(ekf.is_calibrated());
  DriveStationaryCalibration(ekf);
  EXPECT_TRUE(ekf.is_calibrated());
}


TEST(OdometryEkf, StationaryStateStaysAtOrigin) {
  // After calibration, more stationary samples shouldn't move the
  // car. The EKF predicts forward at every IMU tick but with zero
  // bias-corrected accel + zero gyro, integration should net to zero.
  OdometryEkf ekf;
  DriveStationaryCalibration(ekf);

  const auto initial = ekf.state();
  const Eigen::Vector3d accel{0.0, 0.0, 0.0};
  const Eigen::Vector3d gyro{0.0, 0.0, 0.0};

  for (int i = 0; i < 100; ++i) {
    ekf.push_imu(3.1 + i * 0.003, accel, gyro);
  }
  const auto after = ekf.state();
  EXPECT_NEAR(after.x, initial.x, 1e-6);
  EXPECT_NEAR(after.y, initial.y, 1e-6);
  EXPECT_NEAR(after.yaw, initial.yaw, 1e-6);
  EXPECT_NEAR(after.vx, initial.vx, 1e-6);
  EXPECT_NEAR(after.vy, initial.vy, 1e-6);
}


TEST(OdometryEkf, BiasIsLearnedFromStationarySamples) {
  // Calibration period sees a constant accel-X bias of +0.05 m/s².
  // After calibration, b_ax should reflect that — within a few %.
  OdometryEkf ekf;

  const double bias_x = 0.05;
  const Eigen::Vector3d accel{bias_x, 0.0, 0.0};
  const Eigen::Vector3d gyro{0.0, 0.0, 0.0};

  for (double t = 0.0; t < 3.1; t += 0.003) {
    ekf.push_imu(t, accel, gyro);
  }
  EXPECT_TRUE(ekf.is_calibrated());

  const auto biases = ekf.accel_bias_for_tests();
  EXPECT_NEAR(biases(0), bias_x, 1e-3);
}


TEST(OdometryEkf, RpmUpdatePullsVxTowardMeasurement) {
  // Post-calibration, push a steady RPM that maps to vx = 5 m/s.
  // The EKF's vx should approach 5 m/s over a handful of updates.
  OdometryEkf ekf;
  DriveStationaryCalibration(ekf);

  // 5 m/s / 0.00821 (m/s per rpm) ≈ 609 rpm.
  const double rpm = 5.0 / 0.00821;

  // Feed a few RPM samples (with intervening IMU ticks so the
  // predict math runs).
  double t = 3.1;
  const Eigen::Vector3d accel{0.0, 0.0, 0.0};
  const Eigen::Vector3d gyro{0.0, 0.0, 0.0};
  for (int i = 0; i < 200; ++i) {
    ekf.push_imu(t, accel, gyro);
    t += 0.003;
    if (i % 4 == 0) {        // RPM at ~80 Hz vs IMU at ~333 Hz
      ekf.push_rpm(t, rpm);
    }
  }

  const auto s = ekf.state();
  EXPECT_NEAR(s.vx, 5.0, 0.5);  // 10 % tolerance — depends on R/Q
}


TEST(OdometryEkf, NhcCanBeDisabledForPredictOnly) {
  // With enable_nhc=false, the EKF should not apply NHC updates even
  // when the clean-straight criteria are met. Useful for unit-testing
  // predict math in isolation. We assert by adding a deliberate vy
  // excursion and verifying it persists at the predict-driven value.
  EkfParams params;
  params.enable_nhc = false;
  OdometryEkf ekf(params);
  DriveStationaryCalibration(ekf);

  // Push accel-y that drives vy positive.
  const Eigen::Vector3d accel{0.0, 0.2, 0.0};
  const Eigen::Vector3d gyro{0.0, 0.0, 0.0};
  double t = 3.1;
  for (int i = 0; i < 50; ++i) {
    ekf.push_imu(t, accel, gyro);
    t += 0.003;
  }
  // vy should be non-zero (NHC is OFF — predict-only).
  EXPECT_GT(ekf.state().vy, 0.01);
}


TEST(OdometryEkf, NhcEnabledBoundsVyOnCleanStraight) {
  // With NHC on and clean-straight gates open, vy should be pulled
  // back toward zero by the update.
  EkfParams params;
  params.enable_nhc = true;
  OdometryEkf ekf(params);

  DriveStationaryCalibration(ekf);
  ekf.push_steering(0.0, 0.0);  // straight wheels — clean

  // Same vy excursion as the previous test, but with NHC active.
  const Eigen::Vector3d accel{0.0, 0.2, 0.0};
  const Eigen::Vector3d gyro{0.0, 0.0, 0.0};
  double t = 3.1;
  for (int i = 0; i < 200; ++i) {
    ekf.push_imu(t, accel, gyro);
    t += 0.003;
  }
  // NHC should drag vy hard toward zero — assert <= 0.05 m/s.
  EXPECT_LT(std::abs(ekf.state().vy), 0.05);
}


TEST(OdometryEkf, ResetClearsState) {
  OdometryEkf ekf;
  DriveStationaryCalibration(ekf);
  ekf.push_rpm(3.1, 500.0);
  EXPECT_GT(ekf.state().vx, 0.0);

  ekf.reset();
  EXPECT_FALSE(ekf.is_calibrated());
  EXPECT_EQ(ekf.state().x, 0.0);
  EXPECT_EQ(ekf.state().vx, 0.0);
}
