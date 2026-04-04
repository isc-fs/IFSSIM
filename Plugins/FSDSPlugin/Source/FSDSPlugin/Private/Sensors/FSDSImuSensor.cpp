#include "Sensors/FSDSImuSensor.h"

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
		float SqrtDt = FMath::Sqrt(DeltaTime);

		// Bias random walk (persistent drift)
		if (AccelBiasStd > 0.f)
		{
			AccelBias.X += FMath::FRandRange(-1.f, 1.f) * AccelBiasStd * SqrtDt;
			AccelBias.Y += FMath::FRandRange(-1.f, 1.f) * AccelBiasStd * SqrtDt;
			AccelBias.Z += FMath::FRandRange(-1.f, 1.f) * AccelBiasStd * SqrtDt;
		}

		if (GyroBiasStd > 0.f)
		{
			GyroBias.X += FMath::FRandRange(-1.f, 1.f) * GyroBiasStd * SqrtDt;
			GyroBias.Y += FMath::FRandRange(-1.f, 1.f) * GyroBiasStd * SqrtDt;
			GyroBias.Z += FMath::FRandRange(-1.f, 1.f) * GyroBiasStd * SqrtDt;
		}

		// Apply bias + white noise to accelerometer
		if (AccelNoiseStd > 0.f || AccelBiasStd > 0.f)
		{
			Output.LinearAcceleration.X += AccelBias.X + FMath::FRandRange(-1.f, 1.f) * AccelNoiseStd;
			Output.LinearAcceleration.Y += AccelBias.Y + FMath::FRandRange(-1.f, 1.f) * AccelNoiseStd;
			Output.LinearAcceleration.Z += AccelBias.Z + FMath::FRandRange(-1.f, 1.f) * AccelNoiseStd;
		}

		// Apply bias + white noise to gyroscope
		if (GyroNoiseStd > 0.f || GyroBiasStd > 0.f)
		{
			Output.AngularVelocity.X += GyroBias.X + FMath::FRandRange(-1.f, 1.f) * GyroNoiseStd;
			Output.AngularVelocity.Y += GyroBias.Y + FMath::FRandRange(-1.f, 1.f) * GyroNoiseStd;
			Output.AngularVelocity.Z += GyroBias.Z + FMath::FRandRange(-1.f, 1.f) * GyroNoiseStd;
		}
	}

	CachedOutput = Output;
}
