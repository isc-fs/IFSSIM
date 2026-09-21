// The existing Chaos vehicle, behind the plant interface.
#pragma once

#include "CoreMinimal.h"
#include "Plant/FSDSPlant.h"

class AFSDSVehiclePawn;

/**
 * Wraps the live Chaos vehicle so it satisfies IFSDSPlant.
 *
 * This is an OBSERVER, not a driver: Chaos is still integrating the car and
 * this reads it. That is the whole point of the phase — the seam goes in while
 * behaviour stays identical, so when an FMU replaces it later, any difference
 * is attributable to the plant rather than to the refactor.
 *
 * It is also the ONE PLACE the UE frame conversion lives. UE is left-handed,
 * centimetres, Y to the RIGHT; the contract is right-handed, metres, Y to the
 * LEFT. Scattering that conversion through sensors is how a sign error ends up
 * in one signal and not its neighbour.
 */
class FSDSPLUGIN_API FFSDSChaosPlant : public IFSDSPlant
{
public:
	explicit FFSDSChaosPlant(AFSDSVehiclePawn* InPawn);

	virtual FString GetName() const override { return TEXT("Chaos"); }
	virtual bool Initialise() override;
	virtual void PreStep(const FFSDSPlantInput& In) override;
	virtual void PostStep(FFSDSPlantOutput& Out) override;
	virtual void Reset(const double Position[3], const double Quat[4]) override;

	// --- the frame conversion, exposed so tests can check it directly ---

	/** UE world location (cm, left-handed) -> contract world (m, ENU). */
	static void UeToWorld(const FVector& Ue, double Out[3]);
	/** UE quaternion (left-handed) -> contract quaternion (w,x,y,z). */
	static void UeToQuat(const FQuat& Ue, double Out[4]);
	/** POLAR vector (position, velocity, force): (x, y, z) -> (x, -y, z). */
	static void UeToVec(const FVector& Ue, double Out[3], double Scale);

	/**
	 * AXIAL vector (angular velocity, angular acceleration, torque):
	 * (x, y, z) -> (-x, y, -z).
	 *
	 * NOT the same rule as a polar vector, and the difference is not cosmetic.
	 * The UE->contract map is a reflection through the XZ plane, which is
	 * IMPROPER (determinant -1). A polar vector transforms as M*v; an axial one
	 * — being a cross product of two polars — picks up the determinant as well,
	 * so it transforms as -M*v.
	 *
	 * Concretely, with yaw: UE is left-handed so +yaw is a RIGHT turn, while
	 * ISO 8855 is right-handed so +yaw is a LEFT turn. A UE yaw rate of +1 must
	 * therefore come out as -1. The polar rule gives +1 — a mirrored attitude,
	 * which is a bug this project has already shipped once.
	 */
	static void UeToAxial(const FVector& Ue, double Out[3], double Scale);

private:
	TWeakObjectPtr<AFSDSVehiclePawn> Pawn;
	FFSDSPlantInput LastInput;
	FVector PrevVelBodyMs = FVector::ZeroVector;
	bool bHavePrevVel = false;
};
