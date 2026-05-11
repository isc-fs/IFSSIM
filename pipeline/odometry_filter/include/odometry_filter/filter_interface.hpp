// Copyright 2026 IFSSIM contributors.
//
// Abstract base for any odometry-filter implementation in this package.
//
// Both the complementary `OdometryFilter` and the 9-state `OdometryEkf`
// derive from this so the rclcpp node can hold a single
// `std::unique_ptr<IOdometryFilter>` chosen at lifecycle-configure time
// via a parameter. Virtual-dispatch cost is negligible at IMU rate
// (~300 Hz × 1 vtable lookup per call) and the win in test surface
// area / config plumbing is substantial.
//
// The interface mirrors the public methods that the node calls — same
// names, same signatures. Concrete classes still own all their
// internal state.

#ifndef ODOMETRY_FILTER__FILTER_INTERFACE_HPP_
#define ODOMETRY_FILTER__FILTER_INTERFACE_HPP_

#include <Eigen/Core>

namespace odometry_filter {

// Forward declarations — defined in odometry_filter.hpp where both
// concrete classes already use them.
struct OdometryState;
struct FilterDiagnostics;


class IOdometryFilter {
 public:
  virtual ~IOdometryFilter() = default;

  virtual void reset() = 0;

  virtual void push_imu(
    double t,
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & gyro) = 0;
  virtual void push_rpm(double t, double rpm) = 0;
  virtual void push_steering(double t, double angle_rad) = 0;
  virtual void push_brake(double t, double brake) = 0;

  virtual const OdometryState & state() const noexcept = 0;
  virtual const FilterDiagnostics & diagnostics() const noexcept = 0;
  virtual bool is_calibrated() const noexcept = 0;
};

}  // namespace odometry_filter

#endif  // ODOMETRY_FILTER__FILTER_INTERFACE_HPP_
