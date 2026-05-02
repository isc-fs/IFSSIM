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

	// --- Geometry derivation: spherical LiDAR scan → planar render ---
	//
	// The LiDAR scans in spherical coordinates (azimuth h, elevation v)
	// but a perspective camera samples a planar grid. Two facts to
	// reconcile:
	//
	//   1. The LiDAR's V-FOV is *asymmetric* (Hesai ATX: -12.4°..+5.9°).
	//      We center the camera on the V-FOV midpoint and let the camera
	//      see a symmetric ±half-span around its tilted forward axis.
	//      Pitch tilt = (Upper + Lower) / 2; symmetric half-span =
	//      (Upper - Lower) / 2. No CustomProjectionMatrix needed.
	//
	//   2. A spherical ray at (h, v) projects to image-plane y = tan(v)
	//      * sec(h). At the wide-H corners of a 120° H-FOV sweep, sec(h)
	//      = sec(±60°) = 2, so the planar V extent the camera must
	//      cover is 2× the on-axis V extent. Sizing the RT to the
	//      *corner-worst* planar V keeps every spherical ray inside the
	//      frustum; the decode shader (Phase 3) samples at the right
	//      texel for each (h, v) pair using the inverse mapping the
	//      round-trip test below validates.
	//
	// Convention: image-plane coords (x, y) = (tan(h_cam), tan(v_cam)
	// * sec(h_cam)) where (h_cam, v_cam) are angles relative to the
	// camera's tilted forward axis. y is *up*-positive on the image
	// plane (so positive v_cam = above forward).

	const float HFovDeg          = HorizontalFOVEnd - HorizontalFOVStart;
	const float VFovCenterDeg    = (VerticalFOVUpper + VerticalFOVLower) * 0.5f;
	const float VFovHalfSpanDeg  = (VerticalFOVUpper - VerticalFOVLower) * 0.5f;

	// Worst-case sec(h) over the horizontal sweep — symmetric or not.
	const float WorstHRad = FMath::DegreesToRadians(
		FMath::Max(FMath::Abs(HorizontalFOVStart), FMath::Abs(HorizontalFOVEnd)));
	const float SecMax = 1.f / FMath::Max(KINDA_SMALL_NUMBER, FMath::Cos(WorstHRad));

	const float TanHalfHRad      = FMath::Tan(FMath::DegreesToRadians(HFovDeg * 0.5f));
	const float TanVHalfSpanRad  = FMath::Tan(FMath::DegreesToRadians(VFovHalfSpanDeg));
	const float PlanarVHalfSpan  = TanVHalfSpanRad * SecMax;            // image-plane Y half-span
	const float PlanarHHalfSpan  = TanHalfHRad;                         // image-plane X half-span (symmetric)
	const float PlanarVFovDeg    = 2.f * FMath::RadiansToDegrees(FMath::Atan(PlanarVHalfSpan));

	// RT sizing — keep horizontal at the LiDAR's spec resolution
	// (PointsPerScan / channels), let vertical follow the planar
	// aspect so center-resolution ≈ LiDAR's nominal V-step. At wide-H
	// corners the per-row spherical-V resolution is finer (cos(h)
	// factor), which is fine — the decode picks the right texel per
	// LiDAR ray and the surplus is just unused samples.
	const int32 PointsPerScan = FMath::Max(1, FMath::RoundToInt(
		(float)PointsPerSecond / FMath::Max(1.f, RotationsPerSecond)));
	const int32 RTW = FMath::Max(64, PointsPerScan / FMath::Max(1, NumberOfChannels));
	const float Aspect = PlanarHHalfSpan / FMath::Max(KINDA_SMALL_NUMBER, PlanarVHalfSpan);
	const int32 RTH = FMath::Max(1, FMath::RoundToInt((float)RTW / Aspect));

	// Cache the geometry — used by the round-trip test now and the
	// decode shader's uniform buffer in Phase 3.
	GPUVerticalFOVCenterDeg = VFovCenterDeg;
	GPUPlanarHalfWidth      = PlanarHHalfSpan;
	GPUPlanarBottom         = -PlanarVHalfSpan;
	GPUPlanarTop            = +PlanarVHalfSpan;
	GPURTWidth              = RTW;
	GPURTHeight             = RTH;

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
	// Pitch by the V-FOV center so the camera's forward axis bisects
	// the (asymmetric) LiDAR V-range. UE's FRotator pitch: positive =
	// nose up; LiDAR V positive = up; signs match. Yaw/roll zero.
	GPUDepthCapture->SetRelativeRotation(FRotator(VFovCenterDeg, 0.f, 0.f));
	GPUDepthCapture->TextureTarget         = GPUDepthRT;
	GPUDepthCapture->CaptureSource         = ESceneCaptureSource::SCS_SceneDepth;
	GPUDepthCapture->bCaptureEveryFrame    = false;
	GPUDepthCapture->bCaptureOnMovement    = false;
	GPUDepthCapture->bAlwaysPersistRenderingState = true;

	// FOVAngle is *horizontal*; vertical FOV is implicit via aspect
	// (V-FOV = 2·atan(tan(H/2)/aspect)). Sizing RT to Aspect above
	// gives V-FOV = PlanarVFovDeg, which covers the worst-corner
	// spherical V exactly.
	GPUDepthCapture->FOVAngle = HFovDeg;

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
		TEXT("FSDS LiDAR GPU: RT %dx%d (R32f) | H-FOV=%.1f° | V-FOV (planar)=%.1f° | tilt=%.2f° | range=%.0f m"),
		RTW, RTH, HFovDeg, PlanarVFovDeg, VFovCenterDeg, MaxRange / 100.f);

	if (!ValidateProjectionRoundTrip())
	{
		UE_LOG(LogTemp, Error,
			TEXT("FSDS LiDAR GPU: projection round-trip FAILED — Phase-3 decode will produce incorrect points. Check geometry derivation in InitializeGPUPath."));
	}
}

// Verify the spherical-ray ↔ planar-texel mapping derived in
// InitializeGPUPath. For each sample LiDAR ray (h, v):
//   forward: (h, v) → camera-tilted (h_cam, v_cam) →
//            image-plane (image_x, image_y) →
//            texel (col, row)
//   inverse: (col, row) → (image_x, image_y) →
//            (h_cam, v_cam) → (h, v)
// PASS when |Δh| and |Δv| are below 1/RT_pixel-equivalent. The Phase-3
// decode shader will use the same forward formulas to pick its texel
// per LiDAR ray; if this test passes, the decode's per-pixel inverse
// (depth → 3D point) is the only remaining variable.
bool UFSDSLidarSensor::ValidateProjectionRoundTrip() const
{
	if (GPURTWidth <= 0 || GPURTHeight <= 0) return false;

	const float HHalf       = GPUPlanarHalfWidth;       // = tan(HFOV/2)
	const float VBottom     = GPUPlanarBottom;          // negative
	const float VTop        = GPUPlanarTop;             // positive
	const float VRange      = VTop - VBottom;
	const float TiltRad     = FMath::DegreesToRadians(GPUVerticalFOVCenterDeg);

	// Worst-case angular precision per texel: 1 texel ≈ image-plane
	// extent / RT side. Convert to worst-case sphericalΔ at the
	// centre (where conversions are tightest) to set the pass tolerance.
	const float TexelWorstAng = FMath::RadiansToDegrees(
		FMath::Atan2(VRange / (float)GPURTHeight, 1.f));

	// Sample ray grid: 5 H positions × 5 V positions over the full
	// LiDAR scan. Includes corners (worst-sec(h)) and edges of V.
	const float HSampleDeg[] = {
		HorizontalFOVStart, HorizontalFOVStart * 0.5f, 0.f,
		HorizontalFOVEnd * 0.5f, HorizontalFOVEnd
	};
	const float VSampleDeg[] = {
		VerticalFOVLower, VerticalFOVLower * 0.5f, GPUVerticalFOVCenterDeg,
		VerticalFOVUpper * 0.5f, VerticalFOVUpper
	};

	float MaxErrDeg = 0.f;
	int32 Tested = 0;

	for (float HDeg : HSampleDeg)
	{
		for (float VDeg : VSampleDeg)
		{
			const float HRad = FMath::DegreesToRadians(HDeg);
			const float VRad = FMath::DegreesToRadians(VDeg);
			const float VCamRad = VRad - TiltRad;

			// Forward: spherical → image-plane → texel
			const float ImageX = FMath::Tan(HRad);
			const float ImageY = FMath::Tan(VCamRad) / FMath::Cos(HRad); // = tan(v_cam) * sec(h)

			// Skip rays that fall outside the planar frustum — the
			// frustum is sized to cover the scan but at very oblique
			// corner-of-corner pairs the corners can fall slightly
			// outside numerically.
			if (FMath::Abs(ImageX) > HHalf * 1.0001f) continue;
			if (ImageY < VBottom * 1.0001f || ImageY > VTop * 1.0001f) continue;

			const float Col = (ImageX + HHalf) / (2.f * HHalf) * (float)GPURTWidth;
			const float Row = (ImageY - VBottom) / VRange * (float)GPURTHeight;

			// Inverse: texel → image-plane → camera-tilted spherical → vehicle-frame spherical
			const float ImageX2 = Col / (float)GPURTWidth * (2.f * HHalf) - HHalf;
			const float ImageY2 = Row / (float)GPURTHeight * VRange + VBottom;
			const float HRad2 = FMath::Atan(ImageX2);
			const float VCamRad2 = FMath::Atan(ImageY2 * FMath::Cos(HRad2));
			const float VRad2 = VCamRad2 + TiltRad;

			const float ErrHDeg = FMath::Abs(FMath::RadiansToDegrees(HRad - HRad2));
			const float ErrVDeg = FMath::Abs(FMath::RadiansToDegrees(VRad - VRad2));
			MaxErrDeg = FMath::Max3(MaxErrDeg, ErrHDeg, ErrVDeg);
			Tested++;
		}
	}

	const bool bPass = MaxErrDeg < TexelWorstAng;
	UE_LOG(LogTemp, Log,
		TEXT("FSDS LiDAR GPU: projection round-trip %s — max err %.6f° on %d samples (1-texel tol %.4f°)"),
		bPass ? TEXT("PASS") : TEXT("FAIL"), MaxErrDeg, Tested, TexelWorstAng);
	return bPass;
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
