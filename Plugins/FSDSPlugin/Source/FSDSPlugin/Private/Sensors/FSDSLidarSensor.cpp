#include "Sensors/FSDSLidarSensor.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"

UFSDSLidarSensor::UFSDSLidarSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSLidarSensor::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(LogTemp, Log, TEXT("FSDS LiDAR: Initialized (%d channels, %d pts/sec, range %.0fcm)"),
		NumberOfChannels, PointsPerSecond, MaxRange);
}

void UFSDSLidarSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	PerformScan();
}

void UFSDSLidarSensor::PerformScan()
{
	UWorld* World = GetWorld();
	AActor* Owner = GetOwner();
	if (!World || !Owner) return;

	float DeltaTime = World->GetDeltaSeconds();
	if (DeltaTime <= 0.f) return;

	// Calculate how many rays to cast this frame
	int32 PointsThisFrame = FMath::CeilToInt(PointsPerSecond * DeltaTime);
	float HorizontalStep = (HorizontalFOVEnd - HorizontalFOVStart) / (PointsPerSecond / (NumberOfChannels * RotationsPerSecond));
	float VerticalStep = (NumberOfChannels > 1) ?
		(VerticalFOVUpper - VerticalFOVLower) / (NumberOfChannels - 1) : 0.f;

	// Sensor world transform
	FTransform OwnerTransform = Owner->GetActorTransform();
	FVector SensorWorldPos = OwnerTransform.TransformPosition(SensorOffset);
	FQuat OwnerRotation = OwnerTransform.GetRotation();

	TArray<float> NewPoints;
	NewPoints.Reserve(PointsThisFrame * 3);

	FCollisionQueryParams TraceParams;
	TraceParams.AddIgnoredActor(Owner);
	TraceParams.bTraceComplex = true;

	int32 RaysCast = 0;
	for (int32 i = 0; i < PointsThisFrame && RaysCast < PointsThisFrame; i++)
	{
		for (int32 Channel = 0; Channel < NumberOfChannels && RaysCast < PointsThisFrame; Channel++)
		{
			float VerticalAngle = VerticalFOVLower + Channel * VerticalStep;
			float HorizontalAngle = CurrentHorizontalAngle;

			// Ray direction in local space
			FRotator RayRotation(VerticalAngle, HorizontalAngle, 0.f);
			FVector RayDirection = OwnerRotation.RotateVector(RayRotation.Vector());

			FVector RayEnd = SensorWorldPos + RayDirection * MaxRange;

			FHitResult Hit;
			bool bHit = World->LineTraceSingleByChannel(
				Hit, SensorWorldPos, RayEnd,
				ECC_Visibility, TraceParams);

			if (bHit)
			{
				// Convert hit point to sensor-local coordinates
				FVector LocalHit = OwnerTransform.InverseTransformPosition(Hit.ImpactPoint);

				// Store as meters (UE uses cm)
				NewPoints.Add(LocalHit.X / 100.f);
				NewPoints.Add(LocalHit.Y / 100.f);
				NewPoints.Add(LocalHit.Z / 100.f);
			}

			RaysCast++;
		}

		CurrentHorizontalAngle += HorizontalStep;
		if (CurrentHorizontalAngle >= HorizontalFOVEnd)
		{
			CurrentHorizontalAngle = HorizontalFOVStart;
		}
	}

	// Thread-safe update of the point cloud buffer
	FScopeLock Lock(&PointCloudLock);
	PointCloudBuffer = MoveTemp(NewPoints);
	LastTimestamp = FPlatformTime::Cycles64();
}

TArray<float> UFSDSLidarSensor::GetPointCloud() const
{
	FScopeLock Lock(&const_cast<UFSDSLidarSensor*>(this)->PointCloudLock);
	return PointCloudBuffer;
}
