#include "Sensors/FSDSImuSensor.h"

namespace
{
	// One sample from N(0, 1) via Box–Muller. UE has no built-in Gaussian
	// helper. Cheap (one log + one cos), and we only call it 2× per axis
	// per IMU tick. Cap u1 away from zero so the log() is finite.
	static float RandStandardNormal()
	{
		const float u1 = FMath::Max(FMath::FRand(), 1e-7f);
		const float u2 = FMath::FRand();
		return FMath::Sqrt(-2.f * FMath::Loge(u1)) * FMath::Cos(2.f * PI * u2);
	}

	// Step the O-U process one tick. Closed-form discrete update for
	//   db/dt = -(b - 0)/τ + diffusion · dW
	// gives:
	//   b[k+1] = b[k]·exp(-Δt/τ) + SteadyStd · sqrt(1 - exp(-2Δt/τ)) · N(0,1)
	// where SteadyStd is the long-run stddev (the user-facing knob).
	static float StepOrnsteinUhlenbeck(float Bias, float Dt, float Tau, float SteadyStd)
	{
		if (Tau <= 0.f || SteadyStd <= 0.f) return Bias;
		const float Decay = FMath::Exp(-Dt / Tau);
		const float Sigma = SteadyStd * FMath::Sqrt(1.f - Decay * Decay);
		return Bias * Decay + Sigma * RandStandardNormal();
	}
}

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
