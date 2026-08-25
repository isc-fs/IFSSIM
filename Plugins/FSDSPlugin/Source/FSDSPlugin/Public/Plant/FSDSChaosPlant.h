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
	/** UE vector (cm/s or cm) -> contract vector (m). Handedness flip on Y. */
	static void UeToVec(const FVector& Ue, double Out[3], double Scale);

private:
	TWeakObjectPtr<AFSDSVehiclePawn> Pawn;
	FFSDSPlantInput LastInput;
	FVector PrevVelBodyMs = FVector::ZeroVector;
	bool bHavePrevVel = false;
};
