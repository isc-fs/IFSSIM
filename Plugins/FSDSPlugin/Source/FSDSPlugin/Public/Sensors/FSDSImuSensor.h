#pragma once

#include "CoreMinimal.h"
#include "Math/RandomStream.h"
#include "Components/ActorComponent.h"
#include "FSDSImuSensor.generated.h"

/**
 * IMU sensor — derives angular velocity and linear acceleration from vehicle kinematics.
 *
 * Noise model:
 *   measurement = truth + bias(t) + N(0, σ_white²)
 *   bias(t) = Ornstein–Uhlenbeck process:
 *     bias[k+1] = bias[k]·exp(-Δt/τ) + σ_bias·sqrt(1 - exp(-2Δt/τ))·randn()
 *
 * The O-U formulation gives bounded long-run drift (steady-state stddev = σ_bias)
 * with correlation time τ. This matches real IMUs (Allan-variance flicker-noise
 * floor) and prevents the unbounded drift of a pure random walk over long
 * sessions.
 *
 * Both white-noise and bias-step samples are drawn from a Gaussian via Box–Muller.
 * Earlier versions used `FMath::FRandRange(-1, 1)` which is uniform, so the actual
 * stddev was 1/√3 ≈ 58% of the declared value.
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

	/** Accelerometer white noise σ in cm/s² (Gaussian). BMI088 ~0.18 m/s² → 18 cm/s². 0 = off. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float AccelNoiseStd = 0.0f;

	/** Gyroscope white noise σ in rad/s (Gaussian). BMI088 ~0.004. 0 = off. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float GyroNoiseStd = 0.0f;

	/** Accelerometer bias *steady-state* σ in cm/s² (long-run stddev of the O-U process). 0 = no bias. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float AccelBiasStd = 0.0f;

	/** Gyroscope bias steady-state σ in rad/s (long-run stddev). 0 = no bias. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float GyroBiasStd = 0.0f;

	/** Accelerometer bias correlation time τ in seconds. Larger τ = slower drift. Default 100 s ~ BMI088 class. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float AccelBiasTau = 100.0f;

	/** Gyroscope bias correlation time τ in seconds. Default 100 s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float GyroBiasTau = 100.0f;

private:
	FImuOutput CachedOutput;
	FVector PreviousVelocity = FVector::ZeroVector;

	// Persistent bias state (O-U process)
	FVector AccelBias = FVector::ZeroVector;
	FVector GyroBias = FVector::ZeroVector;

	// Deterministic noise source. Lazily seeded from the scenario seed on first
	// use (the seed is set after component construction). Each sensor has its
	// own stream so sensors cannot perturb each other's sequences.
	// See FSDSRandom.h.
	FRandomStream NoiseStream;
	bool bNoiseStreamReady = false;
	FRandomStream& Noise();
};
