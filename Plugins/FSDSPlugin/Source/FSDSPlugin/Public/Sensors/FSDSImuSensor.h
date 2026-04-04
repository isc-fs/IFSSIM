#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSImuSensor.generated.h"

/**
 * IMU sensor — derives angular velocity and linear acceleration from vehicle kinematics.
 * Supports configurable white noise and bias random walk (BMI088-like model).
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

	/** Accelerometer white noise σ in cm/s² (0 = no noise). BMI088 ~0.18 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float AccelNoiseStd = 0.0f;

	/** Gyroscope white noise σ in rad/s (0 = no noise). BMI088 ~0.004 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float GyroNoiseStd = 0.0f;

	/** Accelerometer bias random walk σ in cm/s² (drift per sqrt(s)). 0 = no bias drift */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float AccelBiasStd = 0.0f;

	/** Gyroscope bias random walk σ in rad/s (drift per sqrt(s)). 0 = no bias drift */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float GyroBiasStd = 0.0f;

private:
	FImuOutput CachedOutput;
	FVector PreviousVelocity = FVector::ZeroVector;

	// Persistent bias state (random walk)
	FVector AccelBias = FVector::ZeroVector;
	FVector GyroBias = FVector::ZeroVector;
};
