#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSDistanceSensor.generated.h"

/**
 * Distance sensor — single raycast measuring distance to nearest obstacle.
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSDistanceSensor : public UActorComponent
{
	GENERATED_BODY()

public:
	UFSDSDistanceSensor();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	struct FDistanceOutput
	{
		uint64 Timestamp = 0;
		float Distance = -1.f; // meters, -1 = no hit
		float MinDistance = 0.2f;
		float MaxDistance = 40.f;
	};

	FDistanceOutput GetOutput() const { return CachedOutput; }

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Distance")
	float MaxRange = 4000.f; // cm (40m)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Distance")
	float MinRange = 20.f; // cm (0.2m)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Distance")
	FVector SensorOffset = FVector(200.f, 0.f, 0.f); // Forward-facing

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Distance")
	FRotator SensorRotation = FRotator::ZeroRotator;

private:
	FDistanceOutput CachedOutput;
};
