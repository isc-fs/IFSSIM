#include "Sensors/FSDSLidarSensor.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Async/Async.h"
#include "Async/ParallelFor.h"
#include "FSDSSensorNoise.h"

using FSDSNoise::RandStandardNormal;

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

	// Rate-limit: only scan at RotationsPerSecond Hz (default 10 Hz), not every frame.
	// This keeps the game thread free — the actual raycasts run on a background thread.
	float ScanInterval = 1.f / FMath::Max(1.f, RotationsPerSecond);
	ScanAccumulator += DeltaTime;

	if (ScanAccumulator < ScanInterval)
		return;

	ScanAccumulator -= ScanInterval;

	// Skip if a previous scan is still in flight (can happen at very low FPS)
	if (bScanInProgress.exchange(true))
		return;

	// Snapshot the transform on the game thread before handing off
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		bScanInProgress = false;
		return;
	}
	FTransform OwnerTransform = Owner->GetActorTransform();
	UWorld* World = GetWorld();
	if (!World)
	{
		bScanInProgress = false;
		return;
	}

	// Dispatch scan to a background thread — line traces with bTraceComplex=false
	// are read-only and safe to call from non-game threads in UE5.
	//
	// Owner/World are captured as TWeakObjectPtr so a Chaos vehicle pawn
	// destroyed between this dispatch and the scan execution (e.g. level
	// reload, pawn despawn) is noticed and skipped rather than
	// dereferenced as a dangling raw pointer. The OwnerTransform snapshot
	// is already by value.
	TWeakObjectPtr<UWorld> WeakWorld(World);
	TWeakObjectPtr<AActor> WeakOwner(Owner);
	AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [this, WeakWorld, WeakOwner, OwnerTransform]()
	{
		UWorld* W = WeakWorld.Get();
		AActor* O = WeakOwner.Get();
		if (W && O)
		{
			PerformScan(W, O, OwnerTransform);
		}
		bScanInProgress = false;
	});
}

void UFSDSLidarSensor::PerformScan(UWorld* InWorld, AActor* InOwner, FTransform OwnerTransform)
{
	if (!InWorld) return;

	// Full rotation: all points for one 360° (or partial-FOV) sweep
	int32 PointsPerRotation = FMath::Max(1, FMath::RoundToInt((float)PointsPerSecond / FMath::Max(1.f, RotationsPerSecond)));

	float HFov = HorizontalFOVEnd - HorizontalFOVStart;
	float VFov = VerticalFOVUpper - VerticalFOVLower;

	int32 HorizontalSteps = PointsPerRotation / FMath::Max(1, NumberOfChannels);
	float HStep = (HorizontalSteps > 1) ? HFov / (float)HorizontalSteps : 0.f;
	float VStep = (NumberOfChannels > 1) ? VFov / (float)(NumberOfChannels - 1) : 0.f;

	FVector SensorWorldPos = OwnerTransform.TransformPosition(SensorOffset);
	FQuat OwnerRotation = OwnerTransform.GetRotation();

	TArray<float> NewPoints;
	NewPoints.Reserve(PointsPerRotation * 3);

	FCollisionQueryParams TraceParams;
	TraceParams.AddIgnoredActor(InOwner);
	// Chaos vehicles have wheels/suspension as attached actors — recursively ignore
	// them so horizontal rays don't graze wheel collision at ground level.
	TArray<AActor*> AttachedActors;
	InOwner->GetAttachedActors(AttachedActors, /*bResetArray*/ true, /*bRecursivelyIncludeAttachedActors*/ true);
	for (AActor* Attached : AttachedActors) TraceParams.AddIgnoredActor(Attached);
	TraceParams.bTraceComplex = false;
	TraceParams.bReturnPhysicalMaterial = false;

	// Parallelise the line traces across all horizontal steps. Previously
	// this was a single-threaded loop of HorizontalSteps × NumberOfChannels
	// LineTraceSingleByChannel calls — at 1 M pts/s ≈ 100 k traces/scan
	// the GameThread budget capped /lidar at ~3.5 Hz instead of the 10 Hz
	// the sensor is configured for. Line traces against UWorld are
	// thread-safe (the physics scene takes its own lock per call), so
	// each horizontal step can run on its own worker thread.
	//
	// CurrentHorizontalAngle was a per-iteration accumulator; replaced
	// with direct h-index → HAngle math so iterations are independent.
	// The resulting state at end-of-scan is identical (one HFov sweep).
	struct FRayResult { float X, Y, Z; bool bHit; };
	TArray<FRayResult> SlotResults;
	const int32 TotalSlots = HorizontalSteps * NumberOfChannels;
	SlotResults.SetNumUninitialized(TotalSlots);

	ParallelFor(HorizontalSteps, [&](int32 h)
	{
		const float HAngle = HorizontalFOVStart + h * HStep;

		for (int32 v = 0; v < NumberOfChannels; v++)
		{
			const int32 SlotIdx = h * NumberOfChannels + v;
			SlotResults[SlotIdx].bHit = false;

			const float VAngle = VerticalFOVLower + v * VStep;
			FRotator RayRotation(VAngle, HAngle, 0.f);
			FVector RayDir = OwnerRotation.RotateVector(RayRotation.Vector());
			FVector RayEnd = SensorWorldPos + RayDir * MaxRange;

			FHitResult Hit;
			if (!InWorld->LineTraceSingleByChannel(Hit, SensorWorldPos, RayEnd, ECC_Visibility, TraceParams))
				continue;

			// Dropout/noise use FMath::FRand and RandStandardNormal which
			// share global state across threads — for sim noise the racy
			// reads are acceptable (every consumer just sees jitter on
			// jitter), and UE5's FMath PRNG won't crash.
			if (DropoutRate > 0.f && FMath::FRand() < DropoutRate)
				continue;

			float Dist = (Hit.ImpactPoint - SensorWorldPos).Size();
			if (RangeNoiseStd > 0.f)
				Dist += RangeNoiseStd * RandStandardNormal();

			if (Dist < MinRange)
				continue;

			const FVector NoisyHitPoint = SensorWorldPos + RayDir * Dist;
			const FVector LocalHit = OwnerTransform.InverseTransformPosition(NoisyHitPoint);
			// UE5 vehicle local frame is left-handed (X-fwd, Y-RIGHT, Z-up).
			// ROS REP-103 vehicle frame is right-handed (X-fwd, Y-LEFT, Z-up).
			// Without the Y flip, every cone shows up mirrored across the
			// vehicle's longitudinal axis in /lidar/Lidar1, which then
			// mirrors the cones detected, the SLAM map, and the planned
			// path. On a straight track the mirror is self-symmetric so
			// the car drives fine; at the first curve the mirrored path
			// diverges from physical geometry and the controller turns
			// the wrong way.
			SlotResults[SlotIdx] = {
				LocalHit.X / 100.f,
				-LocalHit.Y / 100.f,
				LocalHit.Z / 100.f,
				true
			};
		}
	});

	// Pack hits into NewPoints. Single-threaded — DrawDebug calls are
	// game-thread-only, so debug visualisation has been dropped from the
	// parallel path; if we ever need it back we can store the world-space
	// hit points in SlotResults and draw them here.
	int32 HitCount = 0;
	for (int32 i = 0; i < TotalSlots; i++)
	{
		const FRayResult& R = SlotResults[i];
		if (!R.bHit) continue;
		NewPoints.Add(R.X);
		NewPoints.Add(R.Y);
		NewPoints.Add(R.Z);
		HitCount++;
	}

	// Match the previous behaviour: one HFov sweep per scan, state wraps.
	CurrentHorizontalAngle = 0.f;

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
