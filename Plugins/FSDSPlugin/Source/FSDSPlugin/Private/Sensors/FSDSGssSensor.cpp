#include "Sensors/FSDSGssSensor.h"
#include "FSDSSensorNoise.h"

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
	FVector WorldVelocity = Owner->GetVelocity() / 100.f;

	// Transform to body frame
	FQuat InvRotation = Owner->GetActorQuat().Inverse();
	Output.LinearVelocity = InvRotation.RotateVector(WorldVelocity);

	// Apply velocity noise — Gaussian. The bridge publishes the GSS twist
	// covariance as VelocityNoiseStd², so the emitted noise must match the
	// declared stddev. The earlier FRandRange(-1, 1) was uniform, giving
	// 1/√3 ≈ 0.58× the declared stddev — silently overstating GSS noise
	// to any consumer EKF.
	if (VelocityNoiseStd > 0.f)
	{
		Output.LinearVelocity.X += VelocityNoiseStd * RandStandardNormal();
		Output.LinearVelocity.Y += VelocityNoiseStd * RandStandardNormal();
		Output.LinearVelocity.Z += VelocityNoiseStd * RandStandardNormal();
	}

	CachedOutput = Output;
}
