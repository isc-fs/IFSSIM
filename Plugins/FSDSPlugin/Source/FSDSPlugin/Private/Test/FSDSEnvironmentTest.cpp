// Automation tests for the track environment sidecar (FSDSEnvironment.h,
// docs/environment_sidecar.md).
//
// Run from an editor build:
//   UnrealEditor-Cmd.exe IFSSIM.uproject -ExecCmds="Automation RunTests FSDS.Environment; Quit"
//     -unattended -nullrhi -nosound -log

#include "Environment/FSDSEnvironment.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	const TCHAR* kValid = TEXT(R"({
		"format": "ifssim-env/1",
		"seed": 7,
		"profile": "stress",
		"ground": { "extent": { "x_min": -60, "y_min": -40.5, "x_max": 160, "y_max": 140 } },
		"props": [
			{ "class": "bollard", "x": 12.0, "y": -3.5, "yaw_deg": 30 },
			{ "class": "tripod",  "x": 40.0, "y": 2.25 },
			{ "class": "bollard", "x": 14.0, "y": -3.5 }
		]
	})");

	bool ParseOk(const FString& Json, FFSDSEnvironment& Out)
	{
		FString Error;
		return FSDSEnvironment::Parse(Json, TEXT("test.env.json"), Out, Error);
	}

	FString ParseError(const FString& Json)
	{
		FFSDSEnvironment Out;
		FString Error;
		FSDSEnvironment::Parse(Json, TEXT("test.env.json"), Out, Error);
		return Error;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSEnvironmentParseTest, "FSDS.Environment.Parse",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSEnvironmentParseTest::RunTest(const FString& Parameters)
{
	FFSDSEnvironment Env;
	if (!TestTrue(TEXT("valid sidecar parses"), ParseOk(kValid, Env))) return false;
	TestEqual(TEXT("seed"), Env.Seed, (int64)7);
	TestEqual(TEXT("profile"), Env.Profile, FString(TEXT("stress")));
	TestTrue(TEXT("ground extent present"), Env.bHasGroundExtent);
	TestEqual(TEXT("ground y_min"), Env.GroundYMin, -40.5);
	TestEqual(TEXT("ground x_max"), Env.GroundXMax, 160.0);
	TestEqual(TEXT("props"), Env.Props.Num(), 3);
	TestEqual(TEXT("props[0].class"), Env.Props[0].Class, FString(TEXT("bollard")));
	TestEqual(TEXT("props[0].yaw_deg"), Env.Props[0].YawDeg, 30.0);
	TestEqual(TEXT("props[1].y"), Env.Props[1].Y, 2.25);
	TestEqual(TEXT("props[1].yaw_deg defaults to 0"), Env.Props[1].YawDeg, 0.0);
	TestEqual(TEXT("source path recorded"), Env.SidecarPath, FString(TEXT("test.env.json")));

	// The minimum: format, seed and profile. No ground, no props.
	FFSDSEnvironment Bare;
	TestTrue(TEXT("minimal sidecar parses"),
		ParseOk(TEXT(R"({"format":"ifssim-env/1","seed":0,"profile":"flat_baseline"})"), Bare));
	TestFalse(TEXT("minimal: no ground"), Bare.bHasGroundExtent);
	TestEqual(TEXT("minimal: no props"), Bare.Props.Num(), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSEnvironmentRejectTest, "FSDS.Environment.Reject",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSEnvironmentRejectTest::RunTest(const FString& Parameters)
{
	struct FCase { const TCHAR* What; const TCHAR* Json; const TCHAR* ErrorContains; };
	const FCase Cases[] = {
		{ TEXT("not JSON"), TEXT("props: []"), TEXT("not a JSON object") },
		{ TEXT("wrong format"), TEXT(R"({"format":"ifssim-env/2","seed":1,"profile":"p"})"), TEXT("'format'") },
		{ TEXT("missing seed"), TEXT(R"({"format":"ifssim-env/1","profile":"p"})"), TEXT("'seed'") },
		{ TEXT("fractional seed"), TEXT(R"({"format":"ifssim-env/1","seed":1.5,"profile":"p"})"), TEXT("'seed'") },
		{ TEXT("empty profile"), TEXT(R"({"format":"ifssim-env/1","seed":1,"profile":""})"), TEXT("'profile'") },
		{ TEXT("numeric profile"), TEXT(R"({"format":"ifssim-env/1","seed":1,"profile":5})"), TEXT("'profile'") },
		{ TEXT("string seed"), TEXT(R"({"format":"ifssim-env/1","seed":"1","profile":"p"})"), TEXT("'seed'") },
		{ TEXT("misspelt top-level key"),
		  TEXT(R"({"format":"ifssim-env/1","seed":1,"profile":"p","prop":[]})"), TEXT("unknown key 'prop'") },
		{ TEXT("inverted extent"),
		  TEXT(R"({"format":"ifssim-env/1","seed":1,"profile":"p","ground":{"extent":{"x_min":5,"y_min":0,"x_max":1,"y_max":9}}})"),
		  TEXT("min must be below max") },
		{ TEXT("extent in cm"),
		  TEXT(R"({"format":"ifssim-env/1","seed":1,"profile":"p","ground":{"extent":{"x_min":-6000,"y_min":0,"x_max":1,"y_max":9}}})"),
		  TEXT("cm instead of m") },
		{ TEXT("prop without class"),
		  TEXT(R"({"format":"ifssim-env/1","seed":1,"profile":"p","props":[{"x":1,"y":2}]})"), TEXT("props[0]: 'class'") },
		{ TEXT("prop with a misspelt key"),
		  TEXT(R"({"format":"ifssim-env/1","seed":1,"profile":"p","props":[{"class":"c","x":1,"y":2,"yaw":3}]})"),
		  TEXT("props[0]: unknown key 'yaw'") },
		{ TEXT("prop with a string coordinate"),
		  TEXT(R"({"format":"ifssim-env/1","seed":1,"profile":"p","props":[{"class":"c","x":"1","y":2}]})"),
		  TEXT("props[0]: 'x' must be a number") },
	};
	for (const FCase& C : Cases)
	{
		const FString Error = ParseError(C.Json);
		TestTrue(FString::Printf(TEXT("%s is rejected (error: \"%s\")"), C.What, *Error),
			Error.Contains(C.ErrorContains));
	}

	// A failed parse leaves nothing behind.
	FFSDSEnvironment Env;
	FString Error;
	TestTrue(TEXT("valid first"), FSDSEnvironment::Parse(kValid, TEXT("a"), Env, Error));
	TestFalse(TEXT("then invalid"),
		FSDSEnvironment::Parse(TEXT(R"({"format":"x"})"), TEXT("b"), Env, Error));
	TestEqual(TEXT("invalid parse clears the output"), Env.Props.Num(), 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSEnvironmentFrameTest, "FSDS.Environment.Frame",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSEnvironmentFrameTest::RunTest(const FString& Parameters)
{
	TestEqual(TEXT("sidecar path"),
		FSDSEnvironment::SidecarPathFor(TEXT("C:/x/tracks/acceleration.csv")),
		FString(TEXT("C:/x/tracks/acceleration.env.json")));

	// Same mapping as FSDSConeSpawner::SpawnFromCSV: UE X = x * 100, UE Y = -y * 100.
	TestTrue(TEXT("CSV (12, -3.5) m -> UE (1200, 350) cm"),
		FSDSEnvironment::CsvToUeCm(12.0, -3.5).Equals(FVector2D(1200.0, 350.0)));

	// A prop facing +y in the CSV frame (counter-clockwise 90) faces UE -Y.
	const FVector UeForward = FRotator(0.0, FSDSEnvironment::CsvYawToUeDeg(90.0), 0.0).Vector();
	const FVector2D CsvPlusY = FSDSEnvironment::CsvToUeCm(0.0, 1.0).GetSafeNormal();
	TestTrue(TEXT("yaw 90 in the CSV frame faces CSV +y in UE"),
		FVector2D(UeForward.X, UeForward.Y).Equals(CsvPlusY, 1e-9));
	return true;
}

#endif  // WITH_DEV_AUTOMATION_TESTS
