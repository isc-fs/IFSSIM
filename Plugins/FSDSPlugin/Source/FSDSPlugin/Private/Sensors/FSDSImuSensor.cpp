#include "Sensors/FSDSImuSensor.h"
#include "FSDSRandom.h"
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

	if (DeltaTime > 0.f)
	{
		// Bias drift — Ornstein–Uhlenbeck process. Bounded long-run stddev.
		if (AccelBiasStd > 0.f)
		{
			AccelBias.X = StepOrnsteinUhlenbeck(Noise(), AccelBias.X, DeltaTime, AccelBiasTau, AccelBiasStd);
			AccelBias.Y = StepOrnsteinUhlenbeck(Noise(), AccelBias.Y, DeltaTime, AccelBiasTau, AccelBiasStd);
			AccelBias.Z = StepOrnsteinUhlenbeck(Noise(), AccelBias.Z, DeltaTime, AccelBiasTau, AccelBiasStd);
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
			Output.LinearAcceleration.X += AccelBias.X + AccelNoiseStd * RandStandardNormal(Noise());
			Output.LinearAcceleration.Y += AccelBias.Y + AccelNoiseStd * RandStandardNormal(Noise());
			Output.LinearAcceleration.Z += AccelBias.Z + AccelNoiseStd * RandStandardNormal(Noise());
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
	if (!bNoiseStreamReady)
	{
		NoiseStream = FSDSRandom::MakeStream(TEXT("Imu.noise"));
		bNoiseStreamReady = true;
	}
	return NoiseStream;
}
