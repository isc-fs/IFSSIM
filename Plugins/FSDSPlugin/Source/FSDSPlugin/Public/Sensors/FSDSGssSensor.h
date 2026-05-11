#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSGssSensor.generated.h"

/**
 * Ground Speed Sensor — reports linear velocity in the vehicle body frame.
 * Supports configurable velocity noise (Gaussian).
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSGssSensor : public UActorComponent
{
	GENERATED_BODY()

public:
	UFSDSGssSensor();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	struct FGssOutput
	{
		uint64 Timestamp = 0;
		FVector LinearVelocity = FVector::ZeroVector; // Body frame, m/s
	};

	FGssOutput GetOutput() const { return CachedOutput; }

	/** Velocity noise σ in m/s per axis, Gaussian (0 = no noise). Typical
	 *  wheel-speed sensor ~0.02. Bridge publishes twist covariance as σ². */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS GSS Noise")
	float VelocityNoiseStd = 0.0f;

private:
	FGssOutput CachedOutput;
};
