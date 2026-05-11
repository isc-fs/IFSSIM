#include "Sensors/FSDSMagnetometerSensor.h"

UFSDSMagnetometerSensor::UFSDSMagnetometerSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSMagnetometerSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	if (!Owner) return;

	FMagnetometerOutput Output;
	Output.Timestamp = FPlatformTime::Cycles64();

	// Transform Earth's magnetic field from ENU world frame to body frame
	FQuat InvRotation = Owner->GetActorQuat().Inverse();
	Output.MagneticField = InvRotation.RotateVector(
		FVector(EarthFieldENU.Y * 100.f, EarthFieldENU.X * 100.f, EarthFieldENU.Z * 100.f) // ENU to UE
	) / 100.f; // Back to proper scale

	CachedOutput = Output;
}
