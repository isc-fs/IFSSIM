// Deliberately non-flat ground, with an analytic answer to compare against.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FSDSTestTerrain.generated.h"

/**
 * One flat patch of ground, in CONTRACT units (m, ENU, +y LEFT).
 *
 * Planar on purpose. A curved surface would need the test to agree with the
 * mesh's tessellation, so a failure could mean "the probe is wrong" or "the
 * cylinder has 32 sides" and the test could not tell you which. A plane has
 * one exact height and one exact normal everywhere on it.
 */
struct FSDSPLUGIN_API FFSDSTestPatch
{
	FString Name;
	double  CentreX = 0.0, CentreY = 0.0;   // m
	double  HalfLenX = 0.0, HalfLenY = 0.0; // m
	double  HeightAtCentre = 0.0;           // m
	/** Unit surface normal, contract frame. */
	double  Normal[3] = {0,0,1};

	bool ContainsXY(double X, double Y) const
	{
		return FMath::Abs(X - CentreX) <= HalfLenX && FMath::Abs(Y - CentreY) <= HalfLenY;
	}

	/** Exact surface height at (X,Y), from the plane equation. */
	double HeightAt(double X, double Y) const
	{
		if (FMath::Abs(Normal[2]) < KINDA_SMALL_NUMBER) return HeightAtCentre;
		return HeightAtCentre
			- ((X - CentreX) * Normal[0] + (Y - CentreY) * Normal[1]) / Normal[2];
	}
};

/**
 * A test level for the road probe: a RAMP and a CROWN.
 *
 * The migration doc is blunt about why this has to exist — "a stub returning
 * z=0, normal=+Z, mu=0.7 passes every test that exists today". Every lap so far
 * has run on dead-flat ground, where a probe that works and a probe that
 * returns nothing at all produce identical logs.
 *
 * The two shapes test different things.
 *
 * RAMP — inclined about Y, so the surface rises along the car's x. Catches a
 * probe that reports the wheel's own height instead of the ground's, which flat
 * ground cannot distinguish.
 *
 * CROWN — two faces inclined about X meeting at a ridge, i.e. ordinary road
 * camber. This is the sharp one: the two faces have normals with OPPOSITE y
 * components, so a probe that converted the normal with the axial rule (-x,y,-z)
 * instead of the polar rule (x,-y,z) reports the camber backwards. On flat
 * ground both rules agree, because the y component is zero.
 */
UCLASS()
class FSDSPLUGIN_API AFSDSTestTerrain : public AActor
{
	GENERATED_BODY()

public:
	AFSDSTestTerrain();

	/** Build the geometry and record the analytic patches. */
	void Build();

	/** The patch under (X,Y) in contract metres, or nullptr for a gap. */
	const FFSDSTestPatch* PatchAt(double X, double Y) const;

	const TArray<FFSDSTestPatch>& GetPatches() const { return Patches; }

private:
	void AddPatch(const FString& Name, double CentreX, double CentreY,
	              double HalfLenX, double HalfLenY,
	              double HeightAtCentre, double PitchDeg, double RollDeg);

	TArray<FFSDSTestPatch> Patches;
};
