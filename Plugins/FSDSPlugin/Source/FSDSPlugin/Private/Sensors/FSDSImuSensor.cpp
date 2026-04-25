#include "Sensors/FSDSImuSensor.h"
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

	// Angular velocity from physics body
	UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
	if (RootPrim && RootPrim->IsSimulatingPhysics())
	{
		Output.AngularVelocity = RootPrim->GetPhysicsAngularVelocityInRadians();
	}

	// Linear acceleration from velocity delta
	FVector CurrentVelocity = Owner->GetVelocity();
	if (DeltaTime > 0.f)
	{
		FVector WorldAccel = (CurrentVelocity - PreviousVelocity) / DeltaTime;
		// Add gravity
		WorldAccel.Z += 980.f; // cm/s^2

		// Transform to body frame
		FQuat InvRotation = Owner->GetActorQuat().Inverse();
		Output.LinearAcceleration = InvRotation.RotateVector(WorldAccel);
	}
	PreviousVelocity = CurrentVelocity;

	// --- Apply noise ---

	if (DeltaTime > 0.f)
	{
		// Bias drift — Ornstein–Uhlenbeck process. Bounded long-run stddev.
		if (AccelBiasStd > 0.f)
		{
			AccelBias.X = StepOrnsteinUhlenbeck(AccelBias.X, DeltaTime, AccelBiasTau, AccelBiasStd);
			AccelBias.Y = StepOrnsteinUhlenbeck(AccelBias.Y, DeltaTime, AccelBiasTau, AccelBiasStd);
			AccelBias.Z = StepOrnsteinUhlenbeck(AccelBias.Z, DeltaTime, AccelBiasTau, AccelBiasStd);
		}

		if (GyroBiasStd > 0.f)
		{
			GyroBias.X = StepOrnsteinUhlenbeck(GyroBias.X, DeltaTime, GyroBiasTau, GyroBiasStd);
			GyroBias.Y = StepOrnsteinUhlenbeck(GyroBias.Y, DeltaTime, GyroBiasTau, GyroBiasStd);
			GyroBias.Z = StepOrnsteinUhlenbeck(GyroBias.Z, DeltaTime, GyroBiasTau, GyroBiasStd);
		}

		// Apply bias + Gaussian white noise to accelerometer
		if (AccelNoiseStd > 0.f || AccelBiasStd > 0.f)
		{
			Output.LinearAcceleration.X += AccelBias.X + AccelNoiseStd * RandStandardNormal();
			Output.LinearAcceleration.Y += AccelBias.Y + AccelNoiseStd * RandStandardNormal();
			Output.LinearAcceleration.Z += AccelBias.Z + AccelNoiseStd * RandStandardNormal();
		}

		// Apply bias + Gaussian white noise to gyroscope
		if (GyroNoiseStd > 0.f || GyroBiasStd > 0.f)
		{
			Output.AngularVelocity.X += GyroBias.X + GyroNoiseStd * RandStandardNormal();
			Output.AngularVelocity.Y += GyroBias.Y + GyroNoiseStd * RandStandardNormal();
			Output.AngularVelocity.Z += GyroBias.Z + GyroNoiseStd * RandStandardNormal();
		}
	}

	CachedOutput = Output;
}
