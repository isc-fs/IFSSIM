#include "Sensors/FSDSLidarSensor.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"
#include "Async/Async.h"
#include "Async/ParallelFor.h"
#include "FSDSSensorNoise.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/KismetRenderingLibrary.h"
#include "GameFramework/Actor.h"
#include "Misc/Paths.h"

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

	UE_LOG(LogTemp, Log,
		TEXT("FSDS LiDAR: backend=%s  %d channels, %d pts/sec, %d pts/scan, H-FOV=%.0f° V-FOV=%.0f°, range=%.0fm"),
		LidarPath == EFSDSLidarPath::GPU ? TEXT("GPU") : TEXT("CPU"),
		NumberOfChannels, PointsPerSecond, PointsPerScan,
		HFov, VFov, MaxRange / 100.f);

	if (LidarPath == EFSDSLidarPath::GPU)
	{
		InitializeGPUPath();
	}
}

void UFSDSLidarSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// GPU backend (#223): rate-limited depth capture; no point-cloud
	// production yet (Phase 1 milestone is "depth render set up at the
	// LiDAR's geometry"). Phase 2 adds GPU→CPU readback, Phase 3 the
	// decode shader. Under GPU mode the CPU path is fully short-
	// circuited; broadcaster sees an empty cloud until Phase 3 lands.
	if (LidarPath == EFSDSLidarPath::GPU)
	{
		TickGPUPath(DeltaTime);
		return;
	}

	// --- CPU path below. Unchanged from dev. ---

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

	// Pre-compute per-channel base direction (pre-yaw, in sensor-local frame).
	// At 174 k rays/scan the per-ray FRotator(VAngle, HAngle, 0).Vector() was
	// 174 k sin/cos pairs per scan; pre-computing the V-channel components
	// once and reducing the per-ray work to a single Z-yaw rotation cuts the
	// trig cost to NumberOfChannels + HorizontalSteps sin/cos calls per scan
	// (=~1616 instead of ~348 000). The line trace itself is the dominant
	// cost so this is a few-percent saving, but it's free correctness-wise.
	struct FChannelDir { float CosV; float SinV; };
	TArray<FChannelDir> ChannelDir;
	ChannelDir.SetNumUninitialized(NumberOfChannels);
	for (int32 v = 0; v < NumberOfChannels; v++)
	{
		const float VAngleRad = FMath::DegreesToRadians(VerticalFOVLower + v * VStep);
		ChannelDir[v] = { FMath::Cos(VAngleRad), FMath::Sin(VAngleRad) };
	}

	ParallelFor(HorizontalSteps, [&](int32 h)
	{
		const float HAngleRad = FMath::DegreesToRadians(HorizontalFOVStart + h * HStep);
		const float CosH = FMath::Cos(HAngleRad);
		const float SinH = FMath::Sin(HAngleRad);

		for (int32 v = 0; v < NumberOfChannels; v++)
		{
			const int32 SlotIdx = h * NumberOfChannels + v;
			SlotResults[SlotIdx].bHit = false;

			// Sensor-local ray dir from the (V, H) angle pair: a unit vector
			// pitched up by VAngle then yawed by HAngle (matches the original
			// FRotator(VAngle, HAngle, 0).Vector() Tait-Bryan order).
			const FChannelDir& C = ChannelDir[v];
			const FVector RayDirLocal(C.CosV * CosH, C.CosV * SinH, C.SinV);
			FVector RayDir = OwnerRotation.RotateVector(RayDirLocal);
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

// ===================================================================
// GPU path (#223) — Phase 1
// ===================================================================
//
// What's here in Phase 1 (this file):
//   - SceneCapture set up at the LiDAR mount pose with the LiDAR's
//     scan grid as the RT size (PointsPerScan / NumberOfChannels
//     horizontal × NumberOfChannels vertical).
//   - Captures fire on the rate limiter at RotationsPerSecond Hz.
//   - Diagnostic logging mirrors the Phase-0 spike (avg
//     CaptureScene() GT cost reported every 5 s; one-shot RT EXR
//     dump after 5 captures for visual sanity).
//
// What's NOT here yet (deliberately):
//   - Asymmetric V-FOV via custom projection matrix. Stage-1.3 work
//     in #223; current symmetric setup over-renders ~26 % of texels
//     for the Hesai's asymmetric -12.4°..+5.9° V-FOV but that's
//     functionally fine for the rendering wiring — the decode
//     shader (Phase 3) will read only the rows matching real
//     channels.
//   - Async readback. Phase 2.
//   - Decode shader → 3D points. Phase 3. Until then,
//     PointCloudBuffer is left empty when LidarPath==GPU.

void UFSDSLidarSensor::InitializeGPUPath()
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	// Match the LiDAR scan grid one-to-one. PointsPerScan = the
	// CPU path's per-scan ray count; we render exactly that many
	// texels so the Phase-3 decode shader can sample at integer
	// texel coords without interpolation.
	const int32 PointsPerScan = FMath::Max(1, FMath::RoundToInt(
		(float)PointsPerSecond / FMath::Max(1.f, RotationsPerSecond)));
	const int32 RTH = FMath::Max(1, NumberOfChannels);
	const int32 RTW = FMath::Max(64, PointsPerScan / RTH);

	const float HFov = HorizontalFOVEnd - HorizontalFOVStart;

	GPUDepthRT = NewObject<UTextureRenderTarget2D>(this);
	GPUDepthRT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_R32f;
	GPUDepthRT->ClearColor          = FLinearColor::Black;
	GPUDepthRT->bAutoGenerateMips   = false;
	GPUDepthRT->InitAutoFormat(RTW, RTH);
	GPUDepthRT->UpdateResourceImmediate(true);

	GPUDepthCapture = NewObject<USceneCaptureComponent2D>(Owner);
	GPUDepthCapture->SetupAttachment(Owner->GetRootComponent());
	GPUDepthCapture->RegisterComponent();
	GPUDepthCapture->SetRelativeLocation(SensorOffset);
	GPUDepthCapture->SetRelativeRotation(FRotator::ZeroRotator);
	GPUDepthCapture->TextureTarget         = GPUDepthRT;
	GPUDepthCapture->CaptureSource         = ESceneCaptureSource::SCS_SceneDepth;
	GPUDepthCapture->bCaptureEveryFrame    = false;
	GPUDepthCapture->bCaptureOnMovement    = false;
	GPUDepthCapture->bAlwaysPersistRenderingState = true;

	// Symmetric H-FOV; V-FOV implicit via aspect. Asymmetric V via
	// tilt+custom-matrix lands in stage 1.3.
	GPUDepthCapture->FOVAngle = HFov;

	// Same show-flag stripping as the Phase-0 spike: depth-only,
	// no AA / post / SSR / AO. AA-off in particular avoids fake
	// intermediate-depth hits at cone silhouettes (#223 risk #3).
	GPUDepthCapture->ShowFlags.SetAntiAliasing(false);
	GPUDepthCapture->ShowFlags.SetTemporalAA(false);
	GPUDepthCapture->ShowFlags.SetMotionBlur(false);
	GPUDepthCapture->ShowFlags.SetBloom(false);
	GPUDepthCapture->ShowFlags.SetTonemapper(false);
	GPUDepthCapture->ShowFlags.SetEyeAdaptation(false);
	GPUDepthCapture->ShowFlags.SetVignette(false);
	GPUDepthCapture->ShowFlags.SetGrain(false);
	GPUDepthCapture->ShowFlags.SetLensFlares(false);
	GPUDepthCapture->ShowFlags.SetScreenSpaceReflections(false);
	GPUDepthCapture->ShowFlags.SetReflectionEnvironment(false);
	GPUDepthCapture->ShowFlags.SetAmbientOcclusion(false);

	UE_LOG(LogTemp, Log,
		TEXT("FSDS LiDAR GPU: depth RT %dx%d (R32f), H-FOV=%.1f°, scan=%.1f Hz"),
		RTW, RTH, HFov, RotationsPerSecond);
}

bool UFSDSLidarSensor::TickGPUPath(float DeltaTime)
{
	if (!GPUDepthCapture) return false;

	const float ScanInterval = 1.f / FMath::Max(1.f, RotationsPerSecond);
	GPUScanAccumulator += DeltaTime;
	if (GPUScanAccumulator < ScanInterval) return false;
	GPUScanAccumulator -= ScanInterval;

	const double T0 = FPlatformTime::Seconds();
	GPUDepthCapture->CaptureScene();
	const double T1 = FPlatformTime::Seconds();

	GPUCapAccumulatorMs += (T1 - T0) * 1000.0;
	GPUCapSampleCount++;
	GPUCaptureCount++;

	if (T1 - GPULastReportTime > 5.0)
	{
		const double AvgMs = GPUCapAccumulatorMs / FMath::Max(1, GPUCapSampleCount);
		UE_LOG(LogTemp, Log,
			TEXT("FSDS LiDAR GPU: avg CaptureScene() game-thread = %.3f ms over %d samples"),
			AvgMs, GPUCapSampleCount);
		GPUCapAccumulatorMs = 0.0;
		GPUCapSampleCount   = 0;
		GPULastReportTime   = T1;
	}

	if (!bGPUDumpedRT && GPUCaptureCount >= 5 && GPUDepthRT)
	{
		const FString OutDir  = FPaths::ProjectSavedDir() / TEXT("LidarGPU");
		UKismetRenderingLibrary::ExportRenderTarget(GetWorld(), GPUDepthRT, OutDir, TEXT("DepthRT.exr"));
		UE_LOG(LogTemp, Log,
			TEXT("FSDS LiDAR GPU: dumped depth RT after capture #%d → %s/DepthRT.exr"),
			GPUCaptureCount, *OutDir);
		bGPUDumpedRT = true;
	}

	return true;
}
