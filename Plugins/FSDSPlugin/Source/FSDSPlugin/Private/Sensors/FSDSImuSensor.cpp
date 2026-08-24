#include "Sensors/FSDSImuSensor.h"
#include "FSDSRandom.h"
#include "Misc/Parse.h"
#include "Misc/CommandLine.h"
#include "FSDSSensorNoise.h"

using FSDSNoise::RandStandardNormal;
using FSDSNoise::StepOrnsteinUhlenbeck;

UFSDSImuSensor::UFSDSImuSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSImuSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	if (!Owner) return;

	FImuOutput Output;
	Output.Timestamp = FPlatformTime::Cycles64();
	Output.Orientation = Owner->GetActorQuat();

	// Pre-compute the world→body rotation once; angular velocity and
	// linear acceleration both need it.
	const FQuat InvRotation = Owner->GetActorQuat().Inverse();

	// Angular velocity. GetPhysicsAngularVelocityInRadians() returns
	// world-space ω; a real IMU gyro outputs body-frame ω, so rotate it
	// into the body frame here. Without this, downstream SLAM nodes
	// integrate a mirrored attitude during turns (validated empirically
	// against fast_LIMO — yaw direction was inverted vs. ground-truth
	// odom, see 2026-04-27 Phase 2 drive test).
	UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
	if (RootPrim && RootPrim->IsSimulatingPhysics())
	{
		const FVector WorldAngVel = RootPrim->GetPhysicsAngularVelocityInRadians();
		Output.AngularVelocity = InvRotation.RotateVector(WorldAngVel);
	}

	// Linear acceleration from velocity delta
	FVector CurrentVelocity = Owner->GetVelocity();
	if (DeltaTime > 0.f)
	{
		FVector WorldAccel = (CurrentVelocity - PreviousVelocity) / DeltaTime;
		// Add gravity
		WorldAccel.Z += 980.f; // cm/s^2

		// Transform to body frame (re-uses the InvRotation above)
		Output.LinearAcceleration = InvRotation.RotateVector(WorldAccel);
	}
	PreviousVelocity = CurrentVelocity;

	// --- Apply noise ---

	// UNIT CONVERSION — the accelerometer only.
	//
	// This sensor works in UE units: Owner->GetVelocity() is cm/s, so
	// LinearAcceleration above is cm/s^2 (note the explicit `WorldAccel.Z +=
	// 980.f; // cm/s^2`). The cm->m conversion happens LATER, at frame packing
	// (FSDSUdpBroadcaster.cpp:211, FSDSRpcServer.cpp:1556, both `/ 100.f`).
	//
	// AccelNoiseStd / AccelBiasStd are configured in m/s^2 — settings.json
	// derives them from the BMI088 datasheet: "~0.024 m/s^2 ... per-sample
	// standard deviations". Applying an m/s^2 sigma to a cm/s^2 quantity and
	// then dividing by 100 made the published noise 100x too small: measured
	// std on a stationary car was 0.00024 m/s^2 against the configured 0.024.
	//
	// The EKF was therefore tuned against a simulated accelerometer roughly
	// 100x quieter than the real sensor, so scale into the sensor's working
	// units here. The gyro needs no equivalent: angular velocity comes from
	// GetPhysicsAngularVelocityInRadians() and is packed unscaled, so rad/s
	// is already the published unit.
	constexpr float AccelMToCm = 100.f;

	if (DeltaTime > 0.f)
	{
		// Bias drift — Ornstein–Uhlenbeck process. Bounded long-run stddev.
		// SteadyStd is scaled so the bias state lives in cm/s^2, matching the
		// LinearAcceleration it is added to below.
		if (AccelBiasStd > 0.f)
		{
			const float AccelBiasStdCm = AccelBiasStd * AccelMToCm;
			AccelBias.X = StepOrnsteinUhlenbeck(Noise(), AccelBias.X, DeltaTime, AccelBiasTau, AccelBiasStdCm);
			AccelBias.Y = StepOrnsteinUhlenbeck(Noise(), AccelBias.Y, DeltaTime, AccelBiasTau, AccelBiasStdCm);
			AccelBias.Z = StepOrnsteinUhlenbeck(Noise(), AccelBias.Z, DeltaTime, AccelBiasTau, AccelBiasStdCm);
		}

		if (GyroBiasStd > 0.f)
		{
			GyroBias.X = StepOrnsteinUhlenbeck(Noise(), GyroBias.X, DeltaTime, GyroBiasTau, GyroBiasStd);
			GyroBias.Y = StepOrnsteinUhlenbeck(Noise(), GyroBias.Y, DeltaTime, GyroBiasTau, GyroBiasStd);
			GyroBias.Z = StepOrnsteinUhlenbeck(Noise(), GyroBias.Z, DeltaTime, GyroBiasTau, GyroBiasStd);
		}

		// Apply bias + Gaussian white noise to accelerometer
		if (AccelNoiseStd > 0.f || AccelBiasStd > 0.f)
		{
			const float AccelNoiseStdCm = AccelNoiseStd * AccelMToCm;
			Output.LinearAcceleration.X += AccelBias.X + AccelNoiseStdCm * RandStandardNormal(Noise());
			Output.LinearAcceleration.Y += AccelBias.Y + AccelNoiseStdCm * RandStandardNormal(Noise());
			Output.LinearAcceleration.Z += AccelBias.Z + AccelNoiseStdCm * RandStandardNormal(Noise());
		}

		// Apply bias + Gaussian white noise to gyroscope
		if (GyroNoiseStd > 0.f || GyroBiasStd > 0.f)
		{
			Output.AngularVelocity.X += GyroBias.X + GyroNoiseStd * RandStandardNormal(Noise());
			Output.AngularVelocity.Y += GyroBias.Y + GyroNoiseStd * RandStandardNormal(Noise());
			Output.AngularVelocity.Z += GyroBias.Z + GyroNoiseStd * RandStandardNormal(Noise());
		}
	}

	CachedOutput = Output;
}

FRandomStream& UFSDSImuSensor::Noise()
{
	// DETERMINISM PROBE (fsds.probeNoise). Logs the first N raw draws straight
	// from this sensor's stream, BEFORE they touch physics, the bridge, or the
	// republishing that duplicates samples and flattens timestamps.
	//
	// This exists because /imu turned out to be unusable for answering "does the
	// same seed reproduce the same noise": it mixes signal with noise, arrives
	// resampled with duplicates, and its header stamps neither advance per
	// sample nor reset across a PIE restart. Three attempts to measure
	// determinism downstream all failed on the observable, not on the sim.
	//
	// Compare two fresh sessions: identical lines => seeding reproduces.
	// Enable with -fsds.probeNoise on the command line, or by flipping the
	// default below for a quick local check.
	static bool bProbe = FParse::Param(FCommandLine::Get(), TEXT("fsds.probeNoise"));
	if (bProbe && ProbeDrawsLogged < 8)
	{
		// Peek WITHOUT consuming: copy the stream, draw from the copy. Drawing
		// from the live stream here would change the sequence the sensor then
		// uses, i.e. the probe would alter what it is measuring.
		FRandomStream Peek = (NoiseStreamGeneration == FSDSRandom::GetGeneration())
			? NoiseStream
			: FSDSRandom::MakeStream(TEXT("Imu.noise"));
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS PROBE imu.noise[%d] seed=%d gen=%u draw=%.9f"),
			ProbeDrawsLogged, FSDSRandom::GetScenarioSeed(),
			FSDSRandom::GetGeneration(), Peek.GetFraction());
		++ProbeDrawsLogged;
	}

	// Re-seed on generation change, not just once: a scenario reset must
	// restart the sequence, otherwise run 2 continues run 1 from wherever it
	// happened to stop. Generation 0 means the seed has not been set yet.
	const uint32 Gen = FSDSRandom::GetGeneration();
	if (NoiseStreamGeneration != Gen)
	{
		NoiseStream = FSDSRandom::MakeStream(TEXT("Imu.noise"));
		NoiseStreamGeneration = Gen;
		// Bias is persistent O-U state, not just a draw. Leaving it across a
		// reset would start the repeat mid-drift with the previous run's
		// accumulated bias — the run would not be a repeat.
		AccelBias = FVector::ZeroVector;
		GyroBias  = FVector::ZeroVector;
	}
	return NoiseStream;
}
