#include "Sensors/FSDSGssSensor.h"
#include "FSDSRandom.h"
#include "FSDSSensorNoise.h"
#include "FSDSVehiclePawn.h"

using FSDSNoise::RandStandardNormal;

UFSDSGssSensor::UFSDSGssSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSGssSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	if (!Owner) return;

	FGssOutput Output;
	Output.Timestamp = FPlatformTime::Cycles64();

	// World velocity in cm/s -> m/s
	// Body velocity from the plant. Neutral repoint: the same quantity, computed
	// once in the adapter instead of re-derived here.
	//
	// The guard matters more than the source. GetVelocity() on a non-simulating
	// actor returns an unwritten ComponentVelocity — zero — so a dead plant
	// published a perfectly-formed zero ground speed at full rate. That is the
	// worst possible failure for a sensor the EKF trusts.
	AFSDSVehiclePawn* Pawn = Cast<AFSDSVehiclePawn>(Owner);
	const FFSDSPlantOutput* Plant = Pawn ? &Pawn->GetPlantState() : nullptr;
	if (!Plant || !Plant->bPlantOk)
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS GSS: plant state unavailable — not publishing"));
		return;
	}

	// Transform to body frame
	// ISO 8855 (y LEFT) -> UE wire convention (y RIGHT).
	Output.LinearVelocity = FVector( Plant->VelBody[0],
	                                -Plant->VelBody[1],
	                                 Plant->VelBody[2]);

	// Apply velocity noise — Gaussian. The bridge publishes the GSS twist
	// covariance as VelocityNoiseStd², so the emitted noise must match the
	// declared stddev. The earlier FRandRange(-1, 1) was uniform, giving
	// 1/√3 ≈ 0.58× the declared stddev — silently overstating GSS noise
	// to any consumer EKF.
	if (VelocityNoiseStd > 0.f)
	{
		Output.LinearVelocity.X += VelocityNoiseStd * RandStandardNormal(Noise());
		Output.LinearVelocity.Y += VelocityNoiseStd * RandStandardNormal(Noise());
		Output.LinearVelocity.Z += VelocityNoiseStd * RandStandardNormal(Noise());
	}

	CachedOutput = Output;
}

FRandomStream& UFSDSGssSensor::Noise()
{
	// Re-seed on generation change, not just once: a scenario reset must
	// restart the sequence, otherwise run 2 continues run 1 from wherever it
	// happened to stop. Generation 0 means the seed has not been set yet.
	const uint32 Gen = FSDSRandom::GetGeneration();
	if (NoiseStreamGeneration != Gen)
	{
		NoiseStream = FSDSRandom::MakeStream(TEXT("Gss.noise"));
		NoiseStreamGeneration = Gen;
	}
	return NoiseStream;
}
