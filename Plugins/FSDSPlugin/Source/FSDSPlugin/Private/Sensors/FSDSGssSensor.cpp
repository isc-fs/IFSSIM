#include "Sensors/FSDSGssSensor.h"

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

	CachedOutput = Output;
}
