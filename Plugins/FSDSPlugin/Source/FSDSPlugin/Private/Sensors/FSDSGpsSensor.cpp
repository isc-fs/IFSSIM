#include "Sensors/FSDSGpsSensor.h"
#include "FSDSSensorNoise.h"

using FSDSNoise::RandStandardNormal;

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

	// Apply GPS noise — Gaussian. The bridge publishes
	// position_covariance.diag = GpsPositionNoiseStd², so the actual
	// emitted noise must match the declared stddev. Earlier
	// `FRandRange(-1, 1)` gave 1/√3 ≈ 0.58× the declared stddev,
	// silently overstating noise to any consumer EKF.
	if (GpsPositionNoiseStd > 0.f)
	{
		Output.Latitude  += GpsPositionNoiseStd * RandStandardNormal() / MetersPerDegreeLat;
		Output.Longitude += GpsPositionNoiseStd * RandStandardNormal() / MetersPerDegreeLon;
		Output.Altitude  += GpsPositionNoiseStd * RandStandardNormal();
	}

	if (GpsVelocityNoiseStd > 0.f)
	{
		Output.Velocity.X += GpsVelocityNoiseStd * RandStandardNormal();
		Output.Velocity.Y += GpsVelocityNoiseStd * RandStandardNormal();
		Output.Velocity.Z += GpsVelocityNoiseStd * RandStandardNormal();
	}

	CachedOutput = Output;
}
