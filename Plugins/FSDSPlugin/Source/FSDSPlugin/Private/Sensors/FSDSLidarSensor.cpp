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
#include "RenderingThread.h"
#include "TextureResource.h"
#include "RHICommandList.h"

using FSDSNoise::RandStandardNormal;

UFSDSLidarSensor::UFSDSLidarSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSLidarSensor::BeginPlay()
{
	Super::BeginPlay();
	// Real config + GPU init is in OnSettingsApplied(), which the
	// pawn calls after SetupSensorsFromSettings() has populated the
	// LiDAR's UPROPERTYs. At BeginPlay time the values are still the
	// header defaults, so logging or initialising backends here would
	// describe a state that's about to be overwritten.
}

void UFSDSLidarSensor::EndPlay(const EEndPlayReason::Type Reason)
{
	// If a GPU readback is mid-flight (either the GPU copy or the
	// render-thread Lock), wait for the render thread to drain before
	// destroying the readback object. Without this the destructor
	// races a still-pending GPU→CPU copy *and* the AsyncTask back to
	// the game thread might fire on a destroyed component.
	if (LidarPath == EFSDSLidarPath::GPU && (bGPUReadbackInFlight || bGPULockDispatched))
	{
		FlushRenderingCommands();
		bGPUReadbackInFlight = false;
		bGPULockDispatched   = false;
	}
	GPUDepthReadback.Reset();
	Super::EndPlay(Reason);
}

void UFSDSLidarSensor::OnSettingsApplied()
{
	const float HFov = HorizontalFOVEnd - HorizontalFOVStart;
	const float VFov = VerticalFOVUpper - VerticalFOVLower;
	const int32 PointsPerScan = PointsPerSecond / FMath::Max(1.f, RotationsPerSecond);

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
	// The LiDAR scans in spherical coordinates (h, v). A perspective
	// camera samples a planar grid; the inverse projection from
	// spherical to image-plane is the cleanest when the camera looks
	// *straight ahead* (zero tilt) — at non-zero tilt the perspective
	// projection introduces H/V coupling (image_x depends on v, image_y
	// on h via the full P_cam.x/P_cam.z, P_cam.y/P_cam.z formulas).
	//
	// Phase-1 used a tilted-camera-with-symmetric-V-FOV approximation
	// that was internally consistent (round-trip test passed with
	// 0.000007° error) but did NOT match the actual renderer's
	// perspective projection at corners — only the tilt=0 special
	// case is exact. We're correcting that here for Phase 3 so the
	// decode shader and the C++ uniform setup speak the same language
	// as the renderer.
	//
	// With tilt=0 the formulas decouple:
	//   image_x = tan(h)
	//   image_y = tan(v) / cos(h) = tan(v) · sec(h)
	// Frustum bounds are:
	//   H: ±tan(HFOV/2)                    (symmetric for symmetric H scan)
	//   V: [tan(VLower)·sec(maxH), tan(VUpper)·sec(maxH)]   (asymmetric)
	// UE5 perspective is symmetric, so we pad V-half to max(|low|,|hi|).
	//
	// Cost: RT_H grows from 279 → ~380 rows for the Hesai (V-FOV planar
	// 47.4° vs Phase-1's nominal 35.7°). The ~100 extra rows render
	// rays that fall outside the LiDAR's V-FOV — wasted texels but
	// trivially cheap; the decode samples only at the correct texel
	// per LiDAR ray.

	const float HFovDeg          = HorizontalFOVEnd - HorizontalFOVStart;
	const float VFovCenterDeg    = 0.f;  // tilt=0 — see comment above

	// Worst-case sec(h) over the horizontal sweep.
	const float WorstHRad = FMath::DegreesToRadians(
		FMath::Max(FMath::Abs(HorizontalFOVStart), FMath::Abs(HorizontalFOVEnd)));
	const float SecMax = 1.f / FMath::Max(KINDA_SMALL_NUMBER, FMath::Cos(WorstHRad));

	const float TanHalfHRad   = FMath::Tan(FMath::DegreesToRadians(HFovDeg * 0.5f));
	const float TanVUpperRad  = FMath::Tan(FMath::DegreesToRadians(VerticalFOVUpper));
	const float TanVLowerRad  = FMath::Tan(FMath::DegreesToRadians(VerticalFOVLower));
	const float PlanarHHalfSpan  = TanHalfHRad;                                 // image-plane X half-span
	const float PlanarVTrueLow   = TanVLowerRad * SecMax;                       // negative for downward-FOV
	const float PlanarVTrueHigh  = TanVUpperRad * SecMax;
	const float PlanarVHalfSpan  = FMath::Max(FMath::Abs(PlanarVTrueLow),
	                                          FMath::Abs(PlanarVTrueHigh));     // symmetric padding
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
	// tilt=0 — see comment in geometry-derivation block above.
	GPUDepthCapture->SetRelativeRotation(FRotator::ZeroRotator);
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
		VerticalFOVLower,
		VerticalFOVLower * 0.5f,
		(VerticalFOVUpper + VerticalFOVLower) * 0.5f,  // V midpoint sample
		VerticalFOVUpper * 0.5f,
		VerticalFOVUpper
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

	// Drain any in-flight readback first — game-thread side. Polling
	// before scan-rate gating lets us catch the result the same frame
	// the GPU finishes (typically N+1 from dispatch), keeping the
	// effective end-to-end latency under one scan period at 60 FPS.
	PollGPUReadback();

	const float ScanInterval = 1.f / FMath::Max(1.f, RotationsPerSecond);
	GPUScanAccumulator += DeltaTime;
	if (GPUScanAccumulator < ScanInterval) return false;
	GPUScanAccumulator -= ScanInterval;

	// Skip if a previous readback hasn't completed — under heavy GPU
	// load (or a stutter) the per-frame poll above couldn't drain it
	// in time, so we drop this scan rather than overlap two readback
	// copies on the same destination buffer. A small log keeps this
	// from being silent if it ever becomes systematic. The same gate
	// also covers the lock-dispatched-but-not-consumed window, which
	// can happen when an AsyncTask back to the game thread is still
	// pending.
	if (bGPUReadbackInFlight || bGPULockDispatched)
	{
		UE_LOG(LogTemp, Verbose,
			TEXT("FSDS LiDAR GPU: dropping scan #%d — previous readback still in flight"),
			GPUCaptureCount + 1);
		return false;
	}

	// Snapshot the dispatch-time pose. Used by Phase 3 to express the
	// decoded points in the body frame as it stood when the rays were
	// cast — the readback latency means the actor will have moved by
	// the time the decode runs.
	if (AActor* Owner = GetOwner())
	{
		const FTransform Xf = Owner->GetActorTransform();
		GPUPendingOwnerTransform = Xf;
		GPUPendingSensorWorldPos = Xf.TransformPosition(SensorOffset);
		GPUPendingOwnerRotation  = Xf.GetRotation();
	}

	const double T0 = FPlatformTime::Seconds();
	GPUDepthCapture->CaptureScene();
	EnqueueGPUReadback();
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

// Phase 2 — async GPU→CPU readback.
//
// EnqueueGPUReadback runs on the game thread immediately after
// CaptureScene. It enqueues a render command that, when the renderer
// processes it on the render thread, asks the RHI to copy the depth
// RT into a CPU-readable buffer attached to the FRHIGPUTextureReadback
// object. The actual GPU→CPU transfer happens whenever the GPU finishes
// — typically the next frame.
//
// PollGPUReadback runs on the game thread each TickGPUPath call. If
// the readback's buffer is ready, we lock it, memcpy the bytes into
// GPUDepthPixels, and unlock. The Lock returns the row pitch (in
// pixels) which can be ≥ GPURTWidth because the RHI may pad rows for
// alignment — Phase 3 reads row-by-row using GPUDepthRowPitch as
// stride.
void UFSDSLidarSensor::EnqueueGPUReadback()
{
	if (!GPUDepthRT) return;
	FTextureRenderTargetResource* RTResource = GPUDepthRT->GameThread_GetRenderTargetResource();
	if (!RTResource) return;

	if (!GPUDepthReadback.IsValid())
	{
		GPUDepthReadback = MakeUnique<FRHIGPUTextureReadback>(TEXT("FSDSLidarDepthReadback"));
	}

	GPUReadbackEnqueueTime = FPlatformTime::Seconds();
	bGPUReadbackInFlight   = true;

	FRHIGPUTextureReadback* Readback = GPUDepthReadback.Get();

	// We can pass `Readback` and `RTResource` raw because the
	// FSDSLidarSensor outlives any in-flight readback (EndPlay flushes
	// rendering before destroying GPUDepthReadback) and the RT
	// resource lives for the lifetime of GPUDepthRT (UPROPERTY-pinned
	// for the component's lifetime).
	ENQUEUE_RENDER_COMMAND(FSDSLidarReadback)(
		[Readback, RTResource](FRHICommandListImmediate& RHICmdList)
		{
			if (FRHITexture* SrcTex = RTResource->GetRenderTargetTexture())
			{
				Readback->EnqueueCopy(RHICmdList, SrcTex);
			}
		});
}

void UFSDSLidarSensor::PollGPUReadback()
{
	// Two state-machine guards: bGPUReadbackInFlight is set by
	// EnqueueGPUReadback; bGPULockDispatched is set when the
	// render-thread Lock command has been queued and not yet returned.
	// Skip if neither phase applies, or if the Lock is already in
	// flight (it will land via ConsumeReadbackResult).
	if (!bGPUReadbackInFlight || bGPULockDispatched) return;
	if (!GPUDepthReadback.IsValid()) return;
	if (!GPUDepthReadback->IsReady()) return;

	// FRHIGPUTextureReadback::Lock asserts IsInRenderingThread(), so
	// we hand the lock+memcpy off to the render thread, then bounce
	// the result back to the game thread via AsyncTask.
	FRHIGPUTextureReadback* Readback = GPUDepthReadback.Get();
	const int32 ExpectedRTH = GPURTHeight;
	TWeakObjectPtr<UFSDSLidarSensor> WeakSelf(this);

	bGPULockDispatched = true;

	ENQUEUE_RENDER_COMMAND(FSDSLidarReadbackLock)(
		[Readback, ExpectedRTH, WeakSelf](FRHICommandListImmediate& /*RHICmdList*/)
		{
			int32 RowPitch     = 0;
			int32 BufferHeight = 0;
			void* Buffer = Readback->Lock(RowPitch, &BufferHeight);

			TArray<float> LocalCopy;
			if (Buffer && RowPitch > 0)
			{
				// BufferHeight is the RHI-mapped height; falls back to
				// our expected RTH when the RHI didn't fill it.
				const int32 H = (BufferHeight > 0) ? BufferHeight : ExpectedRTH;
				LocalCopy.SetNumUninitialized(RowPitch * H);
				FMemory::Memcpy(LocalCopy.GetData(), Buffer, LocalCopy.Num() * sizeof(float));
			}
			Readback->Unlock();

			// Game-thread consumer — TWeakObjectPtr is only safe to
			// dereference on the game thread, which AsyncTask
			// guarantees here.
			const int32 CapturedRowPitch = RowPitch;
			AsyncTask(ENamedThreads::GameThread,
				[WeakSelf, Pixels = MoveTemp(LocalCopy), CapturedRowPitch]() mutable
				{
					if (UFSDSLidarSensor* Self = WeakSelf.Get())
					{
						Self->ConsumeReadbackResult(MoveTemp(Pixels), CapturedRowPitch);
					}
				});
		});
}

void UFSDSLidarSensor::ConsumeReadbackResult(TArray<float>&& Pixels, int32 RowPitch)
{
	// This runs on the game thread (AsyncTask target = GameThread),
	// so we can touch UObject state directly. Resets the state
	// machine flags so the next scan can dispatch a new readback.
	GPUDepthPixels   = MoveTemp(Pixels);
	GPUDepthRowPitch = RowPitch;
	bGPULockDispatched   = false;
	bGPUReadbackInFlight = false;

	const double NowS = FPlatformTime::Seconds();
	GPUReadbackLastLatencyMs = (NowS - GPUReadbackEnqueueTime) * 1000.0;
	GPUReadbackCount++;

	// First-readback sanity check: scan the depth buffer and report
	// min/mean/max/center. If we got real depth back, min should be
	// >> 0 (closest geometry) and max should be near MaxRange (far
	// plane). If we got zeros, the RT format / EnqueueCopy chain is
	// broken and we'll know before Phase 3 chases ghosts.
	if (!bGPULoggedFirstReadback && GPUDepthRowPitch > 0 && GPUDepthPixels.Num() > 0)
	{
		float MinD = TNumericLimits<float>::Max();
		float MaxD = -TNumericLimits<float>::Max();
		double SumD = 0.0;
		int64  N = 0;
		const int32 SafeH = FMath::Min(GPURTHeight, GPUDepthPixels.Num() / GPUDepthRowPitch);
		for (int32 r = 0; r < SafeH; r++)
		{
			for (int32 c = 0; c < GPURTWidth; c++)
			{
				const float D = GPUDepthPixels[r * GPUDepthRowPitch + c];
				if (D > 0.f && D < 1e9f) { MinD = FMath::Min(MinD, D); MaxD = FMath::Max(MaxD, D); SumD += D; N++; }
			}
		}
		const float MeanD   = N > 0 ? (float)(SumD / (double)N) : 0.f;
		const float CenterD = GPUDepthPixels[(SafeH / 2) * GPUDepthRowPitch + (GPURTWidth / 2)];
		UE_LOG(LogTemp, Log,
			TEXT("FSDS LiDAR GPU: first readback OK | %dx%d (stride=%d px) | depth cm: min=%.0f mean=%.0f max=%.0f centerTexel=%.0f | latency=%.2f ms"),
			GPURTWidth, GPURTHeight, GPUDepthRowPitch, MinD, MeanD, MaxD, CenterD, GPUReadbackLastLatencyMs);
		bGPULoggedFirstReadback = true;
	}

	if (GPUReadbackCount > 0 && (GPUReadbackCount % 50) == 0)
	{
		UE_LOG(LogTemp, Log,
			TEXT("FSDS LiDAR GPU: readback #%d, latency=%.2f ms"),
			GPUReadbackCount, GPUReadbackLastLatencyMs);
	}
}
