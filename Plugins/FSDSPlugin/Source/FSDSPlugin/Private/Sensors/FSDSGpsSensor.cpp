#include "Sensors/FSDSGpsSensor.h"
#include "FSDSRandom.h"
#include "FSDSSensorNoise.h"
#include "FSDSVehiclePawn.h"

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

	// Position is GEOMETRIC — it must match where the mesh actually is, so it
	// stays on the actor transform. Velocity is STATE and comes from the plant:
	// GetVelocity() returns an unwritten zero on a non-simulating actor, which
	// is how a dead plant published a well-formed zero GPS velocity.
	AFSDSVehiclePawn* Pawn = Cast<AFSDSVehiclePawn>(Owner);
	const FFSDSPlantOutput* Plant = Pawn ? &Pawn->GetPlantState() : nullptr;
	if (!Plant || !Plant->bPlantOk)
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS GPS: plant state unavailable — not publishing"));
		return;
	}
	FVector WorldPos = Owner->GetActorLocation(); // cm, geometric
	// ENU metres -> UE cm, y flips back to the left-handed wire convention.
	FVector WorldVel = FVector( Plant->VelWorld[0] * 100.0,
	                           -Plant->VelWorld[1] * 100.0,
	                            Plant->VelWorld[2] * 100.0);

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
		Output.Latitude  += GpsPositionNoiseStd * RandStandardNormal(Noise()) / MetersPerDegreeLat;
		Output.Longitude += GpsPositionNoiseStd * RandStandardNormal(Noise()) / MetersPerDegreeLon;
		Output.Altitude  += GpsPositionNoiseStd * RandStandardNormal(Noise());
	}

	if (GpsVelocityNoiseStd > 0.f)
	{
		Output.Velocity.X += GpsVelocityNoiseStd * RandStandardNormal(Noise());
		Output.Velocity.Y += GpsVelocityNoiseStd * RandStandardNormal(Noise());
		Output.Velocity.Z += GpsVelocityNoiseStd * RandStandardNormal(Noise());
	}

	CachedOutput = Output;
}

FRandomStream& UFSDSGpsSensor::Noise()
{
	// Re-seed on generation change, not just once: a scenario reset must
	// restart the sequence, otherwise run 2 continues run 1 from wherever it
	// happened to stop. Generation 0 means the seed has not been set yet.
	const uint32 Gen = FSDSRandom::GetGeneration();
	if (NoiseStreamGeneration != Gen)
	{
		NoiseStream = FSDSRandom::MakeStream(TEXT("Gps.noise"));
		NoiseStreamGeneration = Gen;
	}
	return NoiseStream;
}
