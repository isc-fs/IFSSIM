#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSGpsSensor.generated.h"

/**
 * GPS sensor — converts UE world position to geographic coordinates.
 * Supports configurable position and velocity noise (Gaussian).
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSGpsSensor : public UActorComponent
{
	GENERATED_BODY()

public:
	UFSDSGpsSensor();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	struct FGpsOutput
	{
		uint64 Timestamp = 0;
		double Latitude = 0.0;
		double Longitude = 0.0;
		float Altitude = 0.f;
		FVector Velocity = FVector::ZeroVector;
	};

	FGpsOutput GetOutput() const { return CachedOutput; }

	/** Home reference point (origin of the UE world in geographic coords) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS GPS")
	double HomeLatitude = 47.641468;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS GPS")
	double HomeLongitude = -122.140165;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS GPS")
	float HomeAltitude = 122.f;

	/** Position noise σ in metres, Gaussian (0 = no noise). Bridge publishes
	 *  position_covariance.diag = σ² so emitted noise must match this stddev. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS GPS Noise")
	float GpsPositionNoiseStd = 0.0f;

	/** Velocity noise σ in m/s per axis, Gaussian (0 = no noise). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS GPS Noise")
	float GpsVelocityNoiseStd = 0.0f;

private:
	FGpsOutput CachedOutput;

	// Approximate conversion: 1 degree latitude ~ 111320 meters
	static constexpr double MetersPerDegreeLat = 111320.0;
};
