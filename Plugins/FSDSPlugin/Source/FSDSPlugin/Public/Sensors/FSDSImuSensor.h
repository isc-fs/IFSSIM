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
 *
 * Sample stream (uDV parity). The car's uDV reads the BMI088 at 400 Hz on a
 * hardware timer; the sim ticks at ~60 Hz. Each tick therefore emits every
 * SampleRateHz slot of sim time that fell inside it, with the plant truth
 * interpolated between the previous tick and this one and independent noise
 * per sample. Before this the sensor produced one sample per tick and the
 * stream resent it ~7 times, so /imu carried 60 Hz of information at 400 Hz.
 * The accelerometer runs at AccelOdrHz (uDV configures ACC_CONF 0xA8 = 100 Hz)
 * and holds its value between updates, as the car's /imu does. Readings are
 * quantised to the configured LSB and clamped to the configured range.
 *
 * The truth itself is still the plant's per-tick state: nothing above the
 * tick's Nyquist frequency (30 Hz at 60 fps) is in it, so vibration is not
 * modelled here.
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
		uint64 Timestamp = 0;      // wall clock (Cycles64) at generation; diagnostics only
		double SimTimeSec = 0.0;   // sim time this sample stands for (the stream's stamp)
		FQuat Orientation = FQuat::Identity;
		FVector AngularVelocity = FVector::ZeroVector;
		FVector LinearAcceleration = FVector::ZeroVector;
	};

	FImuOutput GetOutput() const { return CachedOutput; }

	/** Move the samples generated since the last call into Out (appends). Game thread only. */
	void ConsumeSamples(TArray<FImuOutput>& Out);

	/** Output rate in Hz of sim time (uDV: 400, TIM2). 0 = one sample per tick (legacy). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU")
	float SampleRateHz = 400.0f;

	/** Accelerometer output data rate in Hz; the value is held between updates. 0 = every sample. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU")
	float AccelOdrHz = 0.0f;

	/** Per-axis accel white noise σ in m/s² per accel update (REP-103 axes). Overrides AccelNoiseStd when non-zero. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	FVector AccelNoiseStdXYZ = FVector::ZeroVector;

	/** Per-axis gyro white noise σ in rad/s per sample (REP-103 axes). Overrides GyroNoiseStd when non-zero. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	FVector GyroNoiseStdXYZ = FVector::ZeroVector;

	/** σ of the constant accel offset drawn once per session, in cm/s² (set from m/s² by the pawn). 0 = none. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float AccelTurnOnBiasStd = 0.0f;

	/** σ of the constant gyro offset drawn once per session, in rad/s. 0 = none. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU Noise")
	float GyroTurnOnBiasStd = 0.0f;

	/** Accel resolution in cm/s² (set from m/s² by the pawn). 0 = no quantisation. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU")
	float AccelLsb = 0.0f;

	/** Gyro resolution in rad/s. 0 = no quantisation. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU")
	float GyroLsb = 0.0f;

	/** Accel full scale in cm/s² (set from m/s² by the pawn); readings clamp to ±this. 0 = unlimited. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU")
	float AccelRange = 0.0f;

	/** Gyro full scale in rad/s; readings clamp to ±this. 0 = unlimited. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS IMU")
	float GyroRange = 0.0f;

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
	/** Noise, bias, quantisation and the accel hold for one sample; queues it. */
	void EmitSample(double SampleTimeSec, const FVector& TrueAccelCm, const FVector& TrueGyro,
		const FQuat& SampleOrientation, float SampleDt, bool bAccelUpdate);
	/** Forget the sample clock and interpolation state (new session, clock went back). */
	void ResetSampleClock();

	FImuOutput CachedOutput;
	FVector PreviousVelocity = FVector::ZeroVector;

	// Persistent bias state (O-U process)
	FVector AccelBias = FVector::ZeroVector;
	FVector GyroBias = FVector::ZeroVector;
	// Constant per-session offsets, drawn when the noise stream is (re)seeded.
	FVector AccelTurnOnBias = FVector::ZeroVector;
	FVector GyroTurnOnBias = FVector::ZeroVector;

	// Sample stream state. Slot k stands for sim time k / SampleRateHz.
	TArray<FImuOutput> PendingSamples;
	int64 LastSampleSlot = -1;
	bool bHavePrevTruth = false;
	double PrevTruthTimeSec = 0.0;
	FVector PrevTruthAccelCm = FVector::ZeroVector;
	FVector PrevTruthGyro = FVector::ZeroVector;
	FVector HeldAccelCm = FVector::ZeroVector;
	bool bHaveHeldAccel = false;

	// Deterministic noise source. Lazily seeded from the scenario seed on first
	// use (the seed is set after component construction). Each sensor has its
	// own stream so sensors cannot perturb each other's sequences.
	// See FSDSRandom.h.
	FRandomStream NoiseStream;
	// Generation this stream was seeded for. A scenario reset bumps the
	// generation, which forces a re-seed so a repeat run genuinely restarts
	// the sequence instead of continuing the previous one.
	uint32 NoiseStreamGeneration = 0;
	// Determinism probe counter — see Noise() in the .cpp.
	int32 ProbeDrawsLogged = 0;
	FRandomStream& Noise();
};
