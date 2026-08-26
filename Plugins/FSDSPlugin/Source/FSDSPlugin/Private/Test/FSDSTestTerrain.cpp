#include "Test/FSDSTestTerrain.h"

#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"

AFSDSTestTerrain::AFSDSTestTerrain()
{
	PrimaryActorTick.bCanEverTick = false;
}

void AFSDSTestTerrain::AddPatch(const FString& Name, double CentreX, double CentreY,
                                double HalfLenX, double HalfLenY,
                                double HeightAtCentre, double PitchDeg, double RollDeg)
{
	FFSDSTestPatch P;
	P.Name = Name;
	P.CentreX = CentreX; P.CentreY = CentreY;
	P.HalfLenX = HalfLenX; P.HalfLenY = HalfLenY;
	P.HeightAtCentre = HeightAtCentre;

	// Normal of a plane tilted by pitch about y and roll about x, in the
	// CONTRACT frame. Derived here rather than read back off the spawned mesh:
	// the point of the test is to have an answer that does not come from the
	// thing under test.
	const double Pitch = FMath::DegreesToRadians(PitchDeg);
	const double Roll  = FMath::DegreesToRadians(RollDeg);
	FVector N(-FMath::Sin(Pitch), FMath::Sin(Roll), FMath::Cos(Pitch) * FMath::Cos(Roll));
	N.Normalize();
	P.Normal[0] = N.X; P.Normal[1] = N.Y; P.Normal[2] = N.Z;
	Patches.Add(P);
}

const FFSDSTestPatch* AFSDSTestTerrain::PatchAt(double X, double Y) const
{
	// Last match wins, so a later patch laid over an earlier one behaves the
	// way the traces will — the probe sees the topmost surface.
	const FFSDSTestPatch* Found = nullptr;
	for (const FFSDSTestPatch& P : Patches)
	{
		if (P.ContainsXY(X, Y)) Found = &P;
	}
	return Found;
}

void AFSDSTestTerrain::Build()
{
	UWorld* W = GetWorld();
	if (!W) return;

	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (!Cube)
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS TestTerrain: no cube mesh — cannot build"));
		return;
	}

	Patches.Empty();

	// Laid out ahead of the start gate along +x, in the order a car meets them:
	// flat reference, ramp up, crown left, crown right. The flat patch is not
	// filler — it is the control. If the probe is broken in a way that also
	// breaks flat ground, the ramp results alone could not tell you.
	struct FSpec { const TCHAR* Name; double CX, CY, HX, HY, H, Pitch, Roll; };
	const FSpec Specs[] = {
		{ TEXT("flat"),        20.0,  0.0, 5.0, 4.0, 0.0,  0.0,  0.0 },
		{ TEXT("ramp_up"),     40.0,  0.0, 8.0, 4.0, 1.0,  8.0,  0.0 },
		{ TEXT("crown_left"),  70.0,  2.0, 8.0, 2.0, 0.2,  0.0,  6.0 },
		{ TEXT("crown_right"), 70.0, -2.0, 8.0, 2.0, 0.2,  0.0, -6.0 },
		// A STEP: two flat patches at different heights meeting at y=100.
		// Every surface above is planar, so a plane fit describes them
		// perfectly and the residual is zero whether or not the fit works.
		// This is the shape that makes the residual mean something — a wheel
		// straddling the seam sits on ground no plane describes, which is
		// precisely the case the contract wants the plant to be able to
		// DETECT rather than trust.
		{ TEXT("step_low"),   100.0,  1.5, 8.0, 1.5, 0.0,  0.0,  0.0 },
		{ TEXT("step_high"),  100.0, -1.5, 8.0, 1.5, 0.15, 0.0,  0.0 },
	};

	const double M2CM = 100.0;
	for (const FSpec& S : Specs)
	{
		AddPatch(S.Name, S.CX, S.CY, S.HX, S.HY, S.H, S.Pitch, S.Roll);

		// Contract (m, +y LEFT) -> UE (cm, +y RIGHT). Position is a POLAR
		// vector, so y negates. The rotation follows: a roll that lifts the
		// left edge in the contract frame lifts the same physical edge in UE,
		// which is the opposite sign there.
		const FRotator Rot(S.Pitch, 0.0, -S.Roll);   // pitch, yaw, roll

		// The engine cube is 100 cm on a side and centred on its origin, so a
		// slab's TOP face sits half a thickness above its centre. The patch
		// height is the top face, so the centre drops by half the thickness
		// ALONG THE SURFACE NORMAL — not along world Z, which would be wrong
		// by the cosine of the tilt.
		const double ThickM = 0.2;
		const FVector NormalUe(
			-FMath::Sin(FMath::DegreesToRadians(S.Pitch)),
			-FMath::Sin(FMath::DegreesToRadians(S.Roll)),
			 FMath::Cos(FMath::DegreesToRadians(S.Pitch))
			 * FMath::Cos(FMath::DegreesToRadians(S.Roll)));
		const FVector TopCentre(S.CX * M2CM, -S.CY * M2CM, S.H * M2CM);
		const FVector Loc = TopCentre - NormalUe.GetSafeNormal() * (ThickM * M2CM * 0.5);

		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		AStaticMeshActor* Slab = W->SpawnActor<AStaticMeshActor>(
			AStaticMeshActor::StaticClass(), Loc, Rot, Params);
		if (!Slab) continue;

		// MOVABLE, and this is not a style choice. AStaticMeshActor defaults to
		// Static mobility, and a Static component refuses SetStaticMesh,
		// SetWorldScale3D and any move at runtime — with a warning, not an
		// error. The first version of this spawned four empty actors with no
		// mesh and no collision, the probe fell through to the level floor,
		// and the test reported the PROBE as broken. Set it before touching
		// anything else.
		Slab->SetMobility(EComponentMobility::Movable);
		if (UStaticMeshComponent* MC = Slab->GetStaticMeshComponent())
		{
			MC->SetMobility(EComponentMobility::Movable);
			MC->SetStaticMesh(Cube);
			MC->SetWorldScale3D(FVector(S.HX * 2.0, S.HY * 2.0, ThickM));
			// WorldStatic, because that is the only channel the road probe
			// queries — a slab on any other channel would be invisible to it
			// and the test would fail for the wrong reason.
			MC->SetCollisionObjectType(ECC_WorldStatic);
			MC->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
			MC->SetCollisionResponseToAllChannels(ECR_Block);
		}
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS TestTerrain: built %d patches (flat, ramp 8 deg, crown +/-6 deg)"),
		Patches.Num());
}
