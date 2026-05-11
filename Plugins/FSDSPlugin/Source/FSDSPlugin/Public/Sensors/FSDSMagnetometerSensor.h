#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSMagnetometerSensor.generated.h"

/**
 * Magnetometer sensor — reports magnetic field vector in body frame.
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSMagnetometerSensor : public UActorComponent
{
	GENERATED_BODY()

public:
	UFSDSMagnetometerSensor();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	struct FMagnetometerOutput
	{
		uint64 Timestamp = 0;
		FVector MagneticField = FVector::ZeroVector; // Gauss, body frame
	};

	FMagnetometerOutput GetOutput() const { return CachedOutput; }

	/** Earth's magnetic field at location (Gauss) — default ~0.5 Gauss pointing North */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Magnetometer")
	FVector EarthFieldENU = FVector(0.0f, 0.22f, -0.42f); // ~0.5 Gauss, Northern hemisphere

private:
	FMagnetometerOutput CachedOutput;
};
