#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSBarometerSensor.generated.h"

/**
 * Barometer sensor — derives altitude-based pressure from vehicle position.
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSBarometerSensor : public UActorComponent
{
	GENERATED_BODY()

public:
	UFSDSBarometerSensor();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	struct FBarometerOutput
	{
		uint64 Timestamp = 0;
		float Altitude = 0.f; // meters above sea level
		float Pressure = 101325.f; // Pascals (sea level default)
		float Temperature = 288.15f; // Kelvin (~15°C)
	};

	FBarometerOutput GetOutput() const { return CachedOutput; }

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Barometer")
	float SeaLevelPressure = 101325.f; // Pa

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Barometer")
	float SeaLevelTemperature = 288.15f; // K

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Barometer")
	float HomeAltitude = 122.f; // meters

private:
	FBarometerOutput CachedOutput;
};
