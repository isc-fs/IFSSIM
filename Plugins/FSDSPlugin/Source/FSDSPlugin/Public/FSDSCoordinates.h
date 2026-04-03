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

	/** Convert UE5 quaternion to ENU quaternion */
	inline FQuat UEQuatToENU(const FQuat& UE)
	{
		// Swap X↔Y axes, keep Z
		return FQuat(UE.Y, UE.X, UE.Z, UE.W);
	}

	/** Convert ENU quaternion to UE5 quaternion */
	inline FQuat ENUQuatToUE(const FQuat& ENU)
	{
		return FQuat(ENU.Y, ENU.X, ENU.Z, ENU.W);
	}

	/** Convert UE5 rotator (degrees) to ENU yaw (radians) */
	inline double UEYawToENUHeading(double UEYawDegrees)
	{
		// UE yaw: 0=forward(X), 90=right(Y)
		// ENU heading: 0=east(X), 90=north(Y)
		// ENU heading = 90 - UE yaw (then convert to radians)
		return FMath::DegreesToRadians(90.0 - UEYawDegrees);
	}

	/** Convert UE5 angular velocity (rad/s in UE frame) to ENU (rad/s) */
	inline FVector UEAngularVelocityToENU(const FVector& UE)
	{
		return FVector(UE.Y, UE.X, UE.Z);
	}

	/** Scale-only: cm to meters */
	inline double CmToM(double cm) { return cm / 100.0; }

	/** Scale-only: meters to cm */
	inline double MToCm(double m) { return m * 100.0; }
}
