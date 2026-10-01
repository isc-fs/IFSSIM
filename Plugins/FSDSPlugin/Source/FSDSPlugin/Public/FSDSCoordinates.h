#pragma once

#include "CoreMinimal.h"

/**
 * FSDS Coordinate Transform — converts between UE5 and ENU frames.
 *
 * UE5 (Left-Handed):    X=Forward, Y=Right,  Z=Up,   cm, degrees
 * ENU (Right-Handed):   X=East,    Y=North,  Z=Up,   meters, radians
 *
 * Mapping: ENU_X = UE_Y,  ENU_Y = UE_X,  ENU_Z = UE_Z
 * Scale:   ENU = UE / 100 (cm to meters)
 *
 * This follows ROS REP-103 (East-North-Up).
 */
namespace FSDSCoord
{
	/** Convert UE5 position (cm, X=fwd Y=right Z=up) to ENU (m, X=east Y=north Z=up) */
	inline FVector UEToENU(const FVector& UE)
	{
		return FVector(
			UE.Y / 100.0,   // East  = UE Right
			UE.X / 100.0,   // North = UE Forward
			UE.Z / 100.0    // Up    = UE Up
		);
	}

	/** Convert ENU position (m) to UE5 position (cm) */
	inline FVector ENUToUE(const FVector& ENU)
	{
		return FVector(
			ENU.Y * 100.0,  // UE Forward = ENU North
			ENU.X * 100.0,  // UE Right   = ENU East
			ENU.Z * 100.0   // UE Up      = ENU Up
		);
	}

	/** Convert UE5 velocity (cm/s) to ENU velocity (m/s) */
	inline FVector UEVelocityToENU(const FVector& UE)
	{
		return UEToENU(UE); // Same transform, just different interpretation
	}

	/** Convert a UE5 orientation to ENU, with the body in REP-103 FLU
	 *  (X forward, Y left, Z up).
	 *
	 *  UE5's world (X north, Y east, Z up) and body (X forward, Y right, Z up)
	 *  are both left-handed, so the rotation that takes a FLU body vector into
	 *  ENU is R_ENU = P_w * R_UE * P_b, with P_w = swap(x, y) and
	 *  P_b = diag(1, -1, 1). Both are reflections, so R_ENU is a proper
	 *  rotation. As a quaternion, with h = 1/sqrt(2):
	 *
	 *      q_ENU = h * (w + z,  -(x + y),  y - x,  w - z)    as (w, x, y, z)
	 *
	 *  For yaw only (x = y = 0) this is ENU_yaw = 90 deg - UE_yaw. The previous
	 *  q_90 * q_UE.Inverse() matched it only there: it swapped the ENU x and y
	 *  components, so a car pitched 8 deg nose-up read 8 deg nose-down (#638).
	 */
	inline FQuat UEQuatToENU(const FQuat& UE)
	{
		constexpr double H = 0.70710678118654752;  // 1/sqrt(2)
		return FQuat(
			-H * (UE.X + UE.Y),   // X
			 H * (UE.Y - UE.X),   // Y
			 H * (UE.W - UE.Z),   // Z
			 H * (UE.W + UE.Z));  // W
	}

	/** Convert an ENU orientation (FLU body) to UE5. P_w and P_b are their own
	 *  inverses, so this is the same map as UEQuatToENU.
	 */
	inline FQuat ENUQuatToUE(const FQuat& ENU)
	{
		return UEQuatToENU(ENU);
	}

	/** Convert UE5 rotator (degrees) to ENU yaw (radians) */
	inline double UEYawToENUHeading(double UEYawDegrees)
	{
		// UE yaw: 0=forward(X), 90=right(Y)
		// ENU heading: 0=east(X), 90=north(Y)
		// ENU heading = 90 - UE yaw (then convert to radians)
		return FMath::DegreesToRadians(90.0 - UEYawDegrees);
	}

	/** Convert a UE5 world-frame angular velocity (rad/s) to ENU (rad/s).
	 *
	 *  Angular velocity is an axial vector, so the axis swap between the two
	 *  worlds (a reflection) also flips its sign: (x, y, z) -> (-y, -x, -z).
	 *  A car turning left has ENU wz > 0, while its UE yaw decreases (the
	 *  heading turns from +X towards -Y), so UE wz < 0. The plain swap used
	 *  for positions and velocities gets the sign of every component wrong.
	 */
	inline FVector UEAngularVelocityToENU(const FVector& UE)
	{
		return FVector(-UE.Y, -UE.X, -UE.Z);
	}

	/** Convert a UE5 body-frame vector (X forward, Y right, Z up) to REP-103
	 *  FLU (X forward, Y left, Z up). For velocities and accelerations; no unit
	 *  change.
	 */
	inline FVector UEBodyToFLU(const FVector& UE)
	{
		return FVector(UE.X, -UE.Y, UE.Z);
	}

	/** Convert a UE5 body-frame angular velocity (rad/s) to REP-103 FLU.
	 *  Axial, so the Y reflection flips the other two components instead:
	 *  (x, y, z) -> (-x, y, -z). The same rule the bridge applies to /imu.
	 */
	inline FVector UEBodyAngularVelocityToFLU(const FVector& UE)
	{
		return FVector(-UE.X, UE.Y, -UE.Z);
	}

	/** Scale-only: cm to meters */
	inline double CmToM(double cm) { return cm / 100.0; }

	/** Scale-only: meters to cm */
	inline double MToCm(double m) { return m * 100.0; }
}
