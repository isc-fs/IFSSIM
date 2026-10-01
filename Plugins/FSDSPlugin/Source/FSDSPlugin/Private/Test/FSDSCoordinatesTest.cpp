// Automation tests for FSDSCoord's quaternion conversions (#638).
//
// Run from an editor build:
//   UnrealEditor-Cmd.exe IFSSIM.uproject -ExecCmds="Automation RunTests FSDS.Coordinates; Quit"
//     -unattended -nullrhi -nosound -log

#include "FSDSCoordinates.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	constexpr double kTol = 1e-9;

	// What UEQuatToENU must produce, spelled out frame by frame: a FLU body
	// vector becomes a UE body vector (flip Y), the UE rotation takes it into
	// the UE world, and the UE world maps to ENU (swap X and Y).
	FVector ExpectedENU(const FQuat& UE, const FVector& BodyFLU)
	{
		const FVector WorldUE = UE.RotateVector(FVector(BodyFLU.X, -BodyFLU.Y, BodyFLU.Z));
		return FVector(WorldUE.Y, WorldUE.X, WorldUE.Z);
	}

	const FRotator kCases[] = {
		FRotator(0.0, 0.0, 0.0),
		FRotator(0.0, 37.0, 0.0),      // yaw only
		FRotator(0.0, -150.0, 0.0),
		FRotator(8.0, 0.0, 0.0),       // pitch only (the #638 ramp)
		FRotator(-8.0, 90.0, 0.0),
		FRotator(0.0, 0.0, 12.0),      // roll only
		FRotator(0.0, 0.0, -30.0),
		FRotator(8.0, 37.0, -5.0),     // composed
		FRotator(-20.0, 135.0, 30.0),
		FRotator(60.0, -75.0, 80.0),
	};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSCoordUEQuatToENUTest, "FSDS.Coordinates.UEQuatToENU",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSCoordUEQuatToENUTest::RunTest(const FString& Parameters)
{
	for (const FRotator& Rot : kCases)
	{
		const FQuat UE = Rot.Quaternion();
		const FQuat ENU = FSDSCoord::UEQuatToENU(UE);
		for (const FVector& Axis : { FVector::XAxisVector, FVector::YAxisVector, FVector::ZAxisVector })
		{
			const FVector Got = ENU.RotateVector(Axis);
			const FVector Want = ExpectedENU(UE, Axis);
			TestTrue(FString::Printf(TEXT("%s: FLU %s -> ENU %s, expected %s"),
				*Rot.ToString(), *Axis.ToString(), *Got.ToString(), *Want.ToString()),
				Got.Equals(Want, kTol));
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSCoordENUQuatToUETest, "FSDS.Coordinates.ENUQuatToUE",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSCoordENUQuatToUETest::RunTest(const FString& Parameters)
{
	for (const FRotator& Rot : kCases)
	{
		const FQuat UE = Rot.Quaternion();
		const FQuat RoundTrip = FSDSCoord::ENUQuatToUE(FSDSCoord::UEQuatToENU(UE));
		TestTrue(FString::Printf(TEXT("%s: round trip"), *Rot.ToString()),
			RoundTrip.Equals(UE, kTol));
	}
	return true;
}

// Readings a person would check by hand: heading, nose-up pitch and roll,
// and that flat-ground (yaw-only) output is what the old formula produced.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSCoordAttitudeTest, "FSDS.Coordinates.Attitude",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSCoordAttitudeTest::RunTest(const FString& Parameters)
{
	// UE yaw 0 faces north, which is ENU heading 90 deg.
	{
		const FVector Fwd = FSDSCoord::UEQuatToENU(FRotator(0.0, 0.0, 0.0).Quaternion())
			.RotateVector(FVector::XAxisVector);
		TestTrue(TEXT("UE yaw 0 faces ENU north"), Fwd.Equals(FVector(0.0, 1.0, 0.0), kTol));
	}
	// UE pitch +8 deg (nose up) is a body-x elevation of +8 deg in ENU.
	{
		const FVector Fwd = FSDSCoord::UEQuatToENU(FRotator(8.0, 0.0, 0.0).Quaternion())
			.RotateVector(FVector::XAxisVector);
		TestEqual(TEXT("UE pitch +8 deg -> ENU nose up 8 deg"),
			FMath::RadiansToDegrees(FMath::Asin(Fwd.Z)), 8.0, 1e-6);
	}
	// UE roll +12 deg (right side down) lifts the FLU left axis by 12 deg.
	{
		const FVector Left = FSDSCoord::UEQuatToENU(FRotator(0.0, 0.0, 12.0).Quaternion())
			.RotateVector(FVector::YAxisVector);
		TestEqual(TEXT("UE roll +12 deg -> FLU left side up 12 deg"),
			FMath::RadiansToDegrees(FMath::Asin(Left.Z)), 12.0, 1e-6);
	}
	// On flat ground nothing changes: for yaw only, the result equals the
	// previous q_90 * q_UE.Inverse().
	{
		const FQuat Q90(0.0, 0.0, 0.70710678118654752, 0.70710678118654752);
		for (double Yaw : { -170.0, -45.0, 0.0, 30.0, 90.0, 179.0 })
		{
			const FQuat UE = FRotator(0.0, Yaw, 0.0).Quaternion();
			TestTrue(FString::Printf(TEXT("yaw %.0f unchanged from the previous formula"), Yaw),
				FSDSCoord::UEQuatToENU(UE).Equals(Q90 * UE.Inverse(), kTol));
		}
	}
	return true;
}

#endif  // WITH_DEV_AUTOMATION_TESTS
