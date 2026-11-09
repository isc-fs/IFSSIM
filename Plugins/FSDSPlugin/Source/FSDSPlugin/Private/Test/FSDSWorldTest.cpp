// Automation tests for worlds (FSDSWorld.h, docs/worlds.md). The cases mirror
// tools/tests/test_worldgen.py: the plugin and tools/worldgen/world.py must
// accept and reject the same files.
//
// Run from an editor build:
//   UnrealEditor-Cmd IFSSIM.uproject -ExecCmds="Automation RunTests FSDS.World; Quit"
//     -unattended -nullrhi -nosound -log

#include "Environment/FSDSEnvironment.h"
#include "Environment/FSDSWorld.h"
#include "Environment/FSDSWorldActor.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "FSDSCollision.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
	const TCHAR* kMini = TEXT(R"({
		"format": "ifssim-world/1", "name": "mini", "title": "Mini",
		"attribution": ["made up for a test"],
		"frame": {"crs": "EPSG:25831", "origin_e": 1.0, "origin_n": 2.0, "origin_z": 3.0, "x_bearing_deg": 90.0},
		"terrain": {"file": "terrain.i16", "x0": -1.0, "y0": -1.0, "cell_m": 1.0, "nx": 3, "ny": 2, "z_unit_m": 0.01},
		"surfaces": [{"material": "asphalt", "x_min": -1, "x_max": 1, "y_min": -1, "y_max": 0}],
		"lines": [{"x0": 0, "y0": -1, "x1": 0, "y1": 0, "width_m": 0.5, "colour": "white"}],
		"walls": [{"material": "concrete", "x0": -1, "y0": 0, "x1": 1, "y1": 0, "height_m": 1.0,
		           "thickness_m": 0.5, "fence_height_m": 2.0}],
		"buildings": [{"material": "concrete", "x_min": -1, "x_max": 1, "y_min": -1, "y_max": 0, "top_z": 5}],
		"grandstands": [{"x_min": -1, "x_max": 1, "front_y": 2, "back_y": 6, "front_z": 1, "back_z": 4,
		                 "roof_front_y": 2, "roof_back_y": 7, "roof_front_z": 9, "roof_back_z": 8}]
	})");

	FString Mutate(const TCHAR* From, const TCHAR* To)
	{
		FString S(kMini);
		check(S.Contains(From));
		return S.Replace(From, To);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSWorldParseTest, "FSDS.World.Parse",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSWorldParseTest::RunTest(const FString& Parameters)
{
	FFSDSWorld W;
	FString Error;
	TestTrue(TEXT("the minimal world parses"), FSDSWorld::Parse(kMini, W, Error));
	TestEqual(TEXT("error is empty"), Error, FString());
	TestEqual(TEXT("name"), W.Name, FString(TEXT("mini")));
	TestEqual(TEXT("nx"), W.Nx, 3);
	TestEqual(TEXT("one of each"), W.Surfaces.Num() + W.Lines.Num() + W.Walls.Num() + W.Buildings.Num() + W.Grandstands.Num(), 5);
	TestEqual(TEXT("building top"), W.Buildings[0].TopZ, 5.0);
	TestEqual(TEXT("fence"), W.Walls[0].FenceHeightM, 2.0);
	TestFalse(TEXT("no detail zone"), W.bHasDetail);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSWorldRejectTest, "FSDS.World.Reject",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSWorldRejectTest::RunTest(const FString& Parameters)
{
	struct FCase { const TCHAR* What; FString Json; const TCHAR* Expected; };
	const TArray<FCase> Cases = {
		{TEXT("format"), Mutate(TEXT("ifssim-world/1"), TEXT("ifssim-world/2")), TEXT("format")},
		{TEXT("unknown key"), Mutate(TEXT(R"("name": "mini",)"), TEXT(R"("name": "mini", "decals": [],)")), TEXT("unknown key")},
		{TEXT("prop mesh"), Mutate(TEXT(R"("name": "mini",)"), TEXT(R"("name": "mini", "props": [{"mesh": "Tree", "x": 0, "y": 0}],)")), TEXT("not allowed")},
		{TEXT("no attribution"), Mutate(TEXT(R"(["made up for a test"])"), TEXT("[]")), TEXT("attribution")},
		{TEXT("terrain file"), Mutate(TEXT(R"("file": "terrain.i16")"), TEXT(R"("file": "t.png")")), TEXT("terrain.file")},
		{TEXT("too many cells"), Mutate(TEXT(R"("nx": 3, "ny": 2)"), TEXT(R"("nx": 20000, "ny": 20000)")), TEXT("cells")},
		{TEXT("surface material"), Mutate(TEXT(R"("material": "asphalt")"), TEXT(R"("material": "lava")")), TEXT("material")},
		{TEXT("inverted surface"), Mutate(TEXT(R"("asphalt", "x_min": -1)"), TEXT(R"("asphalt", "x_min": 5)")), TEXT("x_min must be below")},
		{TEXT("line of zero length"), Mutate(TEXT(R"("x1": 0, "y1": 0, "width_m")"), TEXT(R"("x1": 0, "y1": -1, "width_m")")), TEXT("same point")},
		{TEXT("fence below wall"), Mutate(TEXT(R"("fence_height_m": 2.0)"), TEXT(R"("fence_height_m": 0.5)")), TEXT("fence_height_m")},
		{TEXT("coordinate in cm"), Mutate(TEXT(R"("x_max": 1, "y_min": -1, "y_max": 0, "top_z")"), TEXT(R"("x_max": 5000, "y_min": -1, "y_max": 0, "top_z")")), TEXT("outside")},
		{TEXT("seating falls"), Mutate(TEXT(R"("back_z": 4)"), TEXT(R"("back_z": 0.5)")), TEXT("rise")},
		{TEXT("tree kind"), Mutate(TEXT(R"("name": "mini",)"), TEXT(R"("name": "mini", "trees": [{"x": 0, "y": 5, "h": 8, "r": 3, "kind": "palm"}],)")), TEXT("not allowed")},
		{TEXT("tree too tall"), Mutate(TEXT(R"("name": "mini",)"), TEXT(R"("name": "mini", "trees": [{"x": 0, "y": 5, "h": 120, "r": 3, "kind": "pine"}],)")), TEXT("trees[0].h")},
		{TEXT("half a roof"), Mutate(TEXT(R"(, "roof_back_z": 8)"), TEXT("")), TEXT("roof")},
		{TEXT("deck above roof"), Mutate(TEXT(R"("top_z": 5})"), TEXT(R"("top_z": 5, "base_z": 6})")), TEXT("base_z")},
		{TEXT("garages flag"), Mutate(TEXT(R"("top_z": 5})"), TEXT(R"("top_z": 5, "garages": 1})")), TEXT("garages")},
		{TEXT("boolean number"), Mutate(TEXT(R"("height_m": 1.0)"), TEXT(R"("height_m": true)")), TEXT("number")},
		{TEXT("not JSON"), FString(TEXT("{")), TEXT("JSON")},
	};
	for (const FCase& C : Cases)
	{
		FFSDSWorld W;
		FString Error;
		TestFalse(FString::Printf(TEXT("%s is rejected"), C.What), FSDSWorld::Parse(C.Json, W, Error));
		TestTrue(FString::Printf(TEXT("%s: '%s' mentions '%s'"), C.What, *Error, C.Expected), Error.Contains(C.Expected));
		TestEqual(FString::Printf(TEXT("%s leaves nothing behind"), C.What), W.Walls.Num(), 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSWorldTerrainTest, "FSDS.World.Terrain",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSWorldTerrainTest::RunTest(const FString& Parameters)
{
	FFSDSWorld W;
	FString Error;
	TestTrue(TEXT("parse"), FSDSWorld::Parse(kMini, W, Error));
	// nx=3, ny=2, x0=y0=-1, cm units; z rises 1 m per metre of x.
	W.Heights = {0, 100, 200, 0, 100, 200};
	TestEqual(TEXT("at a node"), W.HeightAt(0.0, -1.0), 1.0, 1e-9);
	TestEqual(TEXT("between nodes"), W.HeightAt(0.5, -0.5), 1.5, 1e-9);
	TestEqual(TEXT("clamped past the edge"), W.HeightAt(10.0, 10.0), 2.0, 1e-9);
	TestEqual(TEXT("x max"), W.TerrainXMax(), 1.0);

	TestTrue(TEXT("valid name"), FSDSWorld::IsValidName(TEXT("montmelo_straight")));
	TestFalse(TEXT("no paths"), FSDSWorld::IsValidName(TEXT("../etc")));
	TestFalse(TEXT("no empty"), FSDSWorld::IsValidName(TEXT("")));
	TestEqual(TEXT("dir next to the track"), FSDSWorld::DirFor(TEXT("/x/tracks/a.csv"), TEXT("w")),
		FString(TEXT("/x/tracks/worlds/w")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSWorldSidecarTest, "FSDS.World.Sidecar",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSWorldSidecarTest::RunTest(const FString& Parameters)
{
	FFSDSEnvironment Env;
	FString Error;
	TestTrue(TEXT("a sidecar naming a world parses"), FSDSEnvironment::Parse(TEXT(R"({
		"format": "ifssim-env/1", "seed": 0, "profile": "p", "world": {"name": "montmelo_straight", "x": -20.0, "y": 1.5}
	})"), TEXT("t.env.json"), Env, Error));
	TestTrue(TEXT("has a world"), Env.bHasWorld);
	TestEqual(TEXT("name"), Env.WorldName, FString(TEXT("montmelo_straight")));
	TestEqual(TEXT("x"), Env.WorldX, -20.0);
	TestEqual(TEXT("y"), Env.WorldY, 1.5);

	TestFalse(TEXT("a path is not a name"), FSDSEnvironment::Parse(TEXT(R"({
		"format": "ifssim-env/1", "seed": 0, "profile": "p", "world": {"name": "../x", "x": 0, "y": 0}
	})"), TEXT("t.env.json"), Env, Error));
	TestFalse(TEXT("unknown world key"), FSDSEnvironment::Parse(TEXT(R"({
		"format": "ifssim-env/1", "seed": 0, "profile": "p", "world": {"name": "w", "x": 0, "y": 0, "yaw": 3}
	})"), TEXT("t.env.json"), Env, Error));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSWorldShippedTest, "FSDS.World.Shipped",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSWorldShippedTest::RunTest(const FString& Parameters)
{
	// The Montmeló straight, as generated by tools/worldgen. The same checks
	// as MontmeloTest in tools/tests/test_worldgen.py, read through the plugin.
	const FString Dir = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("tracks/worlds/montmelo_straight"));
	FFSDSWorld W;
	FString Error;
	if (!TestTrue(FString::Printf(TEXT("loads (%s)"), *Error), FSDSWorld::Load(Dir, W, Error))) return false;
	TestEqual(TEXT("ground at the start line is the datum"), W.HeightAt(0.0, 0.0), 0.0, 0.15);
	TestTrue(TEXT("flat through the acceleration zone"), FMath::Abs(W.HeightAt(100.0, 0.0)) < 0.3);
	TestTrue(TEXT("falls towards turn 1"), W.HeightAt(600.0, 0.0) < -5.0);
	TestTrue(TEXT("pit building first, with garages"), W.Buildings.Num() > 0 && W.Buildings[0].bGarages);
	TestTrue(TEXT("main grandstand first, roofed"), W.Grandstands.Num() > 0 && W.Grandstands[0].bHasRoof);
	TestTrue(TEXT("open grandstands along the straight"), W.Grandstands.Num() >= 4);
	TestTrue(TEXT("buildings and footbridges from the LiDAR"), W.Buildings.Num() >= 20);
	TestTrue(TEXT("trees from the LiDAR"), W.Trees.Num() >= 500);
	TestTrue(TEXT("credits its data"), W.Attribution.Num() >= 2);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFSDSWorldBuildTest, "FSDS.World.Build",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFSDSWorldBuildTest::RunTest(const FString& Parameters)
{
	// Build the shipped world in a scratch UE world and probe it the way the
	// car, the cones and the LiDAR do.
	const FString Dir = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("tracks/worlds/montmelo_straight"));
	FFSDSWorld W;
	FString Error;
	if (!TestTrue(TEXT("loads"), FSDSWorld::Load(Dir, W, Error))) return false;

	UWorld* World = UWorld::CreateWorld(EWorldType::Game, /*bInformEngineOfWorld=*/false, TEXT("FSDSWorldBuildTest"));
	if (!TestNotNull(TEXT("scratch world"), World)) return false;
	FWorldContext& Ctx = GEngine->CreateNewWorldContext(EWorldType::Game);
	Ctx.SetCurrentWorld(World);

	// Origin 20 m along x and 2 m to the left, as after a recentre.
	const FVector2D Origin(20.0, 2.0);
	AFSDSWorldActor* A = World->SpawnActor<AFSDSWorldActor>();
	if (TestNotNull(TEXT("actor"), A))
	{
		A->Build(W, Origin);
		AddInfo(FString::Printf(TEXT("built in %.2f s, %d triangles, %d prop instances (%lld triangles at full detail)"),
			A->GetBuildSeconds(), A->GetNumTriangles(), A->GetNumInstances(), A->GetPropTriangles()));
		TestTrue(TEXT("built"), A->IsBuilt());
		TestTrue(TEXT("under 1.5 M triangles"), A->GetNumTriangles() < 1500000);
		// Seats, posts, boards, garage fronts, rails, columns and the listed props.
		TestTrue(TEXT("props placed from /Game/Worlds"), A->GetNumInstances() > 1000);
		TestEqual(TEXT("every tree placed"), A->GetNumTrees(), W.Trees.Num());
		// The listed StartGantry: its lights start out and follow SetStartLights.
		TestTrue(TEXT("start gantry has lights"), A->HasStartLights());
		TestFalse(TEXT("start lights out after a build"), A->AreStartLightsOn());
		A->SetStartLights(true);
		TestTrue(TEXT("start lights on"), A->AreStartLightsOn());
		A->SetStartLights(false);
		TestFalse(TEXT("start lights out at GO"), A->AreStartLightsOn());
		TestTrue(TEXT("builds in under 3 s"), A->GetBuildSeconds() < 3.0);

		auto Ue = [&](double X, double Y, double Z) { return FVector((X + Origin.X) * 100.0, -(Y + Origin.Y) * 100.0, Z * 100.0); };
		FCollisionQueryParams Params(SCENE_QUERY_STAT(FSDSWorldBuildTest), /*bTraceComplex=*/true);
		FCollisionObjectQueryParams Ground(ECC_WorldStatic);

		// Ground: as the road probe and the cone snap find it.
		struct FProbe { double X, Y, ExpectZ, Tol; const TCHAR* What; };
		for (const FProbe& P : {FProbe{0.0, 0.0, 0.03, 0.05, TEXT("start line, on the asphalt")},
		                        FProbe{75.0, 1.5, W.HeightAt(75.0, 1.5) + 0.03, 0.05, TEXT("end of the acceleration zone")},
		                        FProbe{600.0, 0.0, W.HeightAt(600.0, 0.0) + 0.03, 0.10, TEXT("towards turn 1")},
		                        FProbe{0.0, 80.0, W.HeightAt(0.0, 80.0), 0.10, TEXT("behind the grandstand")}})
		{
			FHitResult Hit;
			const bool bHit = World->LineTraceSingleByObjectType(Hit, Ue(P.X, P.Y, 50.0), Ue(P.X, P.Y, -50.0), Ground, Params);
			if (TestTrue(FString::Printf(TEXT("ground under %s"), P.What), bHit))
			{
				TestEqual(FString::Printf(TEXT("ground height at %s"), P.What), Hit.ImpactPoint.Z / 100.0, P.ExpectZ, P.Tol);
			}
		}

		// Structures: what the LiDAR (Visibility) hits, and that it is not ground.
		// Z is above the ground at the track's centre. ExpectY < 0 skips the
		// distance check. The rows: 28 of them rise from 3 m at 18.5 m to 20 m
		// at 42.5 m, so a ray at 10 m meets row 12's riser, 28.8 m out.
		struct FRay { double Y0, Y1, Z, ExpectY, Tol; const TCHAR* What; };
		for (const FRay& R : {FRay{0.0, -20.0, 0.6, 10.8, 0.05, TEXT("pit wall")},
		                      FRay{0.0, 20.0, 0.6, 10.25, 0.05, TEXT("grandstand-side barrier")},
		                      FRay{-20.0, -60.0, 4.0, 26.0, 0.05, TEXT("pit building")},
		                      FRay{12.0, 60.0, 10.0, 28.8, 0.1, TEXT("grandstand rows")},
		                      FRay{30.0, 30.0, 40.0, -1.0, 0.0, TEXT("grandstand roof")}})
		{
			const double X = 1.0;
			const double Z = W.HeightAt(X, 0.0) + R.Z;
			FHitResult Hit;
			// The roof ray looks straight down from above it.
			const FVector From = R.Y0 == R.Y1 ? Ue(X, R.Y0, R.Z) : Ue(X, R.Y0, Z);
			const FVector To = R.Y0 == R.Y1 ? Ue(X, R.Y0, 0.0) : Ue(X, R.Y1, Z);
			const bool bHit = World->LineTraceSingleByChannel(Hit, From, To, ECC_Visibility, Params);
			if (!TestTrue(FString::Printf(TEXT("LiDAR ray hits the %s"), R.What), bHit)) continue;
			const double HitY = -Hit.ImpactPoint.Y / 100.0 - Origin.Y;
			if (R.ExpectY > 0.0)
			{
				TestEqual(FString::Printf(TEXT("%s face"), R.What), FMath::Abs(HitY), R.ExpectY, R.Tol);
			}
			const UPrimitiveComponent* C = Hit.GetComponent();
			TestTrue(FString::Printf(TEXT("%s is a prop, never ground"), R.What),
				C && C->GetCollisionObjectType() == FSDSCollision::PropChannel);
		}

		A->Clear();
		TestFalse(TEXT("cleared"), A->IsBuilt());
		FHitResult Hit;
		TestFalse(TEXT("no ground left after Clear"),
			World->LineTraceSingleByObjectType(Hit, Ue(0, 0, 50.0), Ue(0, 0, -50.0), Ground, Params));
	}

	GEngine->DestroyWorldContext(World);
	World->DestroyWorld(false);
	return true;
}

#endif  // WITH_DEV_AUTOMATION_TESTS
