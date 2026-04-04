#include "Sensors/FSDSGpsSensor.h"

UFSDSGpsSensor::UFSDSGpsSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSGpsSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	if (!Owner) return;

	FVector WorldPos = Owner->GetActorLocation(); // in cm
	FVector WorldVel = Owner->GetVelocity(); // cm/s

	FGpsOutput Output;
	Output.Timestamp = FPlatformTime::Cycles64();

	// Convert UE position (cm) to geographic offset (meters then degrees)
	// UE: X = forward (North), Y = right (East), Z = up
	double NorthOffsetM = WorldPos.X / 100.0;
	double EastOffsetM = WorldPos.Y / 100.0;
	double UpOffsetM = WorldPos.Z / 100.0;

	double MetersPerDegreeLon = MetersPerDegreeLat * FMath::Cos(FMath::DegreesToRadians(HomeLatitude));

	Output.Latitude = HomeLatitude + (NorthOffsetM / MetersPerDegreeLat);
	Output.Longitude = HomeLongitude + (EastOffsetM / MetersPerDegreeLon);
	Output.Altitude = HomeAltitude + static_cast<float>(UpOffsetM);

	// Velocity in m/s
	Output.Velocity = WorldVel / 100.f;

	// Apply GPS noise
	if (GpsPositionNoiseStd > 0.f)
	{
		Output.Latitude += FMath::FRandRange(-1.f, 1.f) * GpsPositionNoiseStd / MetersPerDegreeLat;
		Output.Longitude += FMath::FRandRange(-1.f, 1.f) * GpsPositionNoiseStd / MetersPerDegreeLon;
		Output.Altitude += FMath::FRandRange(-1.f, 1.f) * GpsPositionNoiseStd;
	}

	if (GpsVelocityNoiseStd > 0.f)
	{
		Output.Velocity.X += FMath::FRandRange(-1.f, 1.f) * GpsVelocityNoiseStd;
		Output.Velocity.Y += FMath::FRandRange(-1.f, 1.f) * GpsVelocityNoiseStd;
		Output.Velocity.Z += FMath::FRandRange(-1.f, 1.f) * GpsVelocityNoiseStd;
	}

	CachedOutput = Output;
}
