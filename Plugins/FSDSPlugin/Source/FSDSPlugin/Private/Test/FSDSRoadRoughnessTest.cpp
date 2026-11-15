// Automation tests for the level-floor road model (Vehicles/FSDSRoadRoughness.h).
//
// Run from an editor build:
//   UnrealEditor-Cmd.exe IFSSIM.uproject -ExecCmds="Automation RunTests FSDS.Road; Quit"
//     -unattended -nullrhi -nosound -log

#include "Vehicles/FSDSRoadRoughness.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSRoadProfileTest, "FSDS.Road.Profile",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSRoadProfileTest::RunTest(const FString& Parameters)
{
	// Unsmoothed profile: its spread is the analytic Sigma of the OU process.
	FFSDSRoadRoughness Road;
	Road.Configure(2.5e-5, 20.0, 0.0);
	TestTrue(TEXT("active"), Road.IsActive());
	Road.Reset(FRandomStream(42));

	double Sum = 0.0, SumSq = 0.0;
	int32 Count = 0;
	for (double S = 0.0; S < 4000.0; S += 0.5)
	{
		const double H = Road.Height(0, S);
		Sum += H; SumSq += H * H; ++Count;
		Road.TrimBefore(S);
	}
	const double Mean = Sum / Count;
	const double Std = FMath::Sqrt(SumSq / Count - Mean * Mean);
	TestTrue(FString::Printf(TEXT("profile std %.3f mm within 15%% of sigma %.3f mm"), Std * 1e3, Road.GetSigma() * 1e3),
		FMath::Abs(Std / Road.GetSigma() - 1.0) < 0.15);

	// Same stream, same road; the two tracks differ.
	FFSDSRoadRoughness A, B;
	A.Configure(2.5e-5, 20.0, 0.15);
	B.Configure(2.5e-5, 20.0, 0.15);
	A.Reset(FRandomStream(7));
	B.Reset(FRandomStream(7));
	TestEqual(TEXT("same seed, same height"), A.Height(1, 37.3), B.Height(1, 37.3));
	TestNotEqual(TEXT("left and right tracks differ"), A.Height(0, 37.3), A.Height(1, 37.3));

	// Off at zero level.
	FFSDSRoadRoughness Off;
	Off.Configure(0.0, 20.0, 0.15);
	TestFalse(TEXT("Gd0 0 is off"), Off.IsActive());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSRoadCornerTest, "FSDS.Road.Corner",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSRoadCornerTest::RunTest(const FString& Parameters)
{
	// A 5 mm step in the road: the wheel settles on it and the strut passes
	// on K x 5 mm, as if the ground under a rigid tyre had risen 5 mm.
	const double Ms = 69.0, Mu = 10.0, K = 20184.0, C = 3540.0, Kt = 100000.0;
	FFSDSRoadCorner Corner;
	Corner.Reset(0.0);
	for (int32 i = 0; i < 180; ++i)   // 3 s of 60 Hz frames
	{
		Corner.Advance(i == 0 ? 0.0 : 0.005, 0.005, 1.0 / 60.0, Ms, Mu, K, C, Kt);
	}
	TestTrue(FString::Printf(TEXT("wheel settles on the step (%.3f mm)"), Corner.Zu * 1e3),
		FMath::Abs(Corner.Zu - 0.005) < 1e-5);
	TestTrue(FString::Printf(TEXT("force K x step (%.1f N)"), Corner.Force(K, C)),
		FMath::Abs(Corner.Force(K, C) - K * 0.005) < 0.5);

	// Stable at a long frame (a hitch to 10 Hz).
	Corner.Reset(0.0);
	for (int32 i = 0; i < 30; ++i) { Corner.Advance(0.0, 0.002, 0.1, Ms, Mu, K, C, Kt); }
	TestTrue(TEXT("finite after long frames"), FMath::IsFinite(Corner.Zu) && FMath::Abs(Corner.Zu) < 0.01);
	return true;
}

#endif
