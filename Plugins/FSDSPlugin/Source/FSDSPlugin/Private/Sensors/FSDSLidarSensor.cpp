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

	float HFov = HorizontalFOVEnd - HorizontalFOVStart;
	float VFov = VerticalFOVUpper - VerticalFOVLower;
	int32 PointsPerScan = PointsPerSecond / FMath::Max(1.f, RotationsPerSecond);

	UE_LOG(LogTemp, Log, TEXT("FSDS LiDAR: %d channels, %d pts/sec, %d pts/scan, H-FOV=%.0f° V-FOV=%.0f°, range=%.0fm"),
		NumberOfChannels, PointsPerSecond, PointsPerScan,
		HFov, VFov, MaxRange / 100.f);
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

	// How many rays to cast this frame
	int32 PointsThisFrame = FMath::CeilToInt(PointsPerSecond * DeltaTime);
	PointsThisFrame = FMath::Min(PointsThisFrame, 100000); // Cap per-frame for performance

	float HFov = HorizontalFOVEnd - HorizontalFOVStart;
	float VFov = VerticalFOVUpper - VerticalFOVLower;

	// Horizontal step per ray (degrees)
	int32 HorizontalSteps = PointsThisFrame / FMath::Max(1, NumberOfChannels);
	float HStep = (HorizontalSteps > 1) ? HFov / (float)HorizontalSteps : 0.f;

	// Vertical step per channel
	float VStep = (NumberOfChannels > 1) ? VFov / (float)(NumberOfChannels - 1) : 0.f;

	// Sensor world transform
	FTransform OwnerTransform = Owner->GetActorTransform();
	FVector SensorWorldPos = OwnerTransform.TransformPosition(SensorOffset);
	FQuat OwnerRotation = OwnerTransform.GetRotation();

	TArray<float> NewPoints;
	NewPoints.Reserve(PointsThisFrame * 3);

	FCollisionQueryParams TraceParams;
	TraceParams.AddIgnoredActor(Owner);
	TraceParams.bTraceComplex = false; // Faster with simple collision
	TraceParams.bReturnPhysicalMaterial = false;

	int32 HitCount = 0;

	for (int32 h = 0; h < HorizontalSteps; h++)
	{
		float HAngle = HorizontalFOVStart + CurrentHorizontalAngle;

		for (int32 v = 0; v < NumberOfChannels; v++)
		{
			float VAngle = VerticalFOVLower + v * VStep;

			// Ray direction in local space, then transform to world
			FRotator RayRotation(VAngle, HAngle, 0.f);
			FVector RayDir = OwnerRotation.RotateVector(RayRotation.Vector());
			FVector RayEnd = SensorWorldPos + RayDir * MaxRange;

			FHitResult Hit;
			if (World->LineTraceSingleByChannel(Hit, SensorWorldPos, RayEnd, ECC_Visibility, TraceParams))
			{
				float Dist = (Hit.ImpactPoint - SensorWorldPos).Size();
				if (Dist >= MinRange)
				{
					// Convert to sensor-local coordinates (meters)
					FVector LocalHit = OwnerTransform.InverseTransformPosition(Hit.ImpactPoint);
					NewPoints.Add(LocalHit.X / 100.f);
					NewPoints.Add(LocalHit.Y / 100.f);
					NewPoints.Add(LocalHit.Z / 100.f);
					HitCount++;

					if (bDrawDebugPoints)
					{
						DrawDebugPoint(World, Hit.ImpactPoint, 3.f, FColor::Green, false, 0.1f);
					}
				}
			}
		}

		// Advance horizontal angle for next column
		CurrentHorizontalAngle += HStep;
		if (CurrentHorizontalAngle >= HFov)
		{
			CurrentHorizontalAngle = 0.f;
		}
	}

	// Thread-safe update
	FScopeLock Lock(&PointCloudLock);
	PointCloudBuffer = MoveTemp(NewPoints);
	CachedPointCount = HitCount;
	LastTimestamp = FPlatformTime::Cycles64();
}

TArray<float> UFSDSLidarSensor::GetPointCloud() const
{
	FScopeLock Lock(&const_cast<UFSDSLidarSensor*>(this)->PointCloudLock);
	return PointCloudBuffer;
}

int32 UFSDSLidarSensor::GetPointCount() const
{
	FScopeLock Lock(&const_cast<UFSDSLidarSensor*>(this)->PointCloudLock);
	return CachedPointCount;
}
