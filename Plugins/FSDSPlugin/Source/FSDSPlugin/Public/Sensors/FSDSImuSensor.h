#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSImuSensor.generated.h"

/**
 * IMU sensor — derives angular velocity and linear acceleration from vehicle kinematics.
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSImuSensor : public UActorComponent
{
	GENERATED_BODY()

public:
	UFSDSImuSensor();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	struct FImuOutput
	{
		uint64 Timestamp = 0;
		FQuat Orientation = FQuat::Identity;
		FVector AngularVelocity = FVector::ZeroVector;
		FVector LinearAcceleration = FVector::ZeroVector;
	};

	FImuOutput GetOutput() const { return CachedOutput; }

private:
	FImuOutput CachedOutput;
	FVector PreviousVelocity = FVector::ZeroVector;
};
