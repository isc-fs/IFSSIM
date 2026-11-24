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
	// Unsmoothed profile: its spread is the analytic Sigma of the two OU parts.
	FFSDSRoadRoughness Road;
	Road.Configure(2.5e-5, 20.0, 0.0, 3.0);
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
	A.Configure(2.5e-5, 20.0, 0.15, 3.0);
	B.Configure(2.5e-5, 20.0, 0.15, 3.0);
	A.Reset(FRandomStream(7));
	B.Reset(FRandomStream(7));
	TestEqual(TEXT("same seed, same height"), A.Height(1, 37.3), B.Height(1, 37.3));
	TestNotEqual(TEXT("left and right tracks differ"), A.Height(0, 37.3), A.Height(1, 37.3));

	// Long waves are common to both tracks, short ones are not: the 10 m means
	// of left and right move together, their 0.5 m steps barely do.
	{
		FFSDSRoadRoughness Two;
		Two.Configure(2.5e-5, 20.0, 0.0, 3.0);
		Two.Reset(FRandomStream(11));
		TArray<double> L10, R10, LStep, RStep;
		double SL = 0.0, SR = 0.0;
		int32 N = 0;
		double PrevL = Two.Height(0, 0.0), PrevR = Two.Height(1, 0.0);
		for (double S = 0.5; S < 4000.0; S += 0.5)
		{
			const double HL = Two.Height(0, S), HR = Two.Height(1, S);
			LStep.Add(HL - PrevL); RStep.Add(HR - PrevR);
			PrevL = HL; PrevR = HR;
			SL += HL; SR += HR;
			if (++N == 20) { L10.Add(SL / N); R10.Add(SR / N); SL = SR = 0.0; N = 0; }
			Two.TrimBefore(S);
		}
		auto Corr = [](const TArray<double>& X, const TArray<double>& Y)
		{
			double Mx = 0, My = 0;
			for (int32 i = 0; i < X.Num(); ++i) { Mx += X[i]; My += Y[i]; }
			Mx /= X.Num(); My /= Y.Num();
			double Sxy = 0, Sxx = 0, Syy = 0;
			for (int32 i = 0; i < X.Num(); ++i)
			{
				Sxy += (X[i] - Mx) * (Y[i] - My); Sxx += (X[i] - Mx) * (X[i] - Mx); Syy += (Y[i] - My) * (Y[i] - My);
			}
			return Sxy / FMath::Sqrt(Sxx * Syy);
		};
		const double RLong = Corr(L10, R10), RShort = Corr(LStep, RStep);
		TestTrue(FString::Printf(TEXT("10 m means move together (r %.2f)"), RLong), RLong > 0.85);
		TestTrue(FString::Printf(TEXT("0.5 m steps barely do (r %.2f)"), RShort), RShort < 0.4);
	}

	// Off at zero level.
	FFSDSRoadRoughness Off;
	Off.Configure(0.0, 20.0, 0.15, 3.0);
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
