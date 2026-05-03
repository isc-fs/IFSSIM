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
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"  // FComputeShaderUtils, RegisterExternalTexture, AddEnqueueCopyPass
#include "GlobalShader.h"
#include "ShaderParameterMacros.h"
#include "Sensors/FSDSLidarDecodeShader.h"

using FSDSNoise::RandStandardNormal;

// CVar for sweeping H oversample factor without rebuilds. The
// "ExtraOversample" multiplier inside InitializeGPUPath sits on top
// of the analytic 1.65× minimum that handles the perspective non-
// linearity at h=0; this CVar replaces the hardcoded 2× for sweep
// experiments. Phase-4 baseline = 2×; we want to know if 3× or 4×
// drops cone-body 95th-pct meaningfully, given the rasterization-vs-
// trace floor at silhouette edges. Read at OnSettingsApplied time
// (so a runtime change requires PIE stop/start to re-init the RT).
static TAutoConsoleVariable<float> CVarLidarGPUExtraOversample(
	TEXT("fsds.LidarGPU.ExtraOversample"),
	4.0f,
	TEXT("Multiplier on top of the analytic-min H oversample (#223). Default 4.0 — measured ")
	TEXT("on the tune branch (commit a4907be sweep): cone-body 95th-pct drops 13.5→7.0 cm, ")
	TEXT("<5cm rate climbs 90→97 %. 2.0 (Phase-4 baseline) is a fallback for thermal-constrained ")
	TEXT("Mac runs; 3.0 is a flat plateau (no measurable gain over 2.0)."),
	ECVF_Default);

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
	// If any readback slot is mid-flight (GPU copy or render-thread
	// Lock), wait for the render thread to drain before destroying
	// the readback objects. Without this the destructor races a
	// still-pending GPU→CPU copy *and* the AsyncTask back to the
	// game thread might fire on a destroyed component.
	if (LidarPath == EFSDSLidarPath::GPU)
	{
		bool bAnyInFlight = false;
		for (const FReadbackSlot& Slot : ReadbackSlots)
		{
			if (Slot.bInFlight || Slot.bLockDispatched) { bAnyInFlight = true; break; }
		}
		if (bAnyInFlight)
		{
			FlushRenderingCommands();
			for (FReadbackSlot& Slot : ReadbackSlots)
			{
				Slot.bInFlight       = false;
				Slot.bLockDispatched = false;
			}
		}
		for (FReadbackSlot& Slot : ReadbackSlots)
		{
			Slot.Readback.Reset();
		}
	}
	Super::EndPlay(Reason);
}

void UFSDSLidarSensor::OnSettingsApplied()
{
	const float HFov = HorizontalFOVEnd - HorizontalFOVStart;
	const float VFov = VerticalFOVUpper - VerticalFOVLower;
	const int32 PointsPerScan = PointsPerSecond / FMath::Max(1.f, RotationsPerSecond);

	// Normalise PerChannelMaxRangeCm to length == NumberOfChannels.
	// Empty (default) → fill with global MaxRange. Length mismatch →
	// warn + fall back to global MaxRange. Both paths (GPU shader,
	// CPU PerformScan) read from this normalised array directly.
	if (PerChannelMaxRangeCm.Num() == 0)
	{
		PerChannelMaxRangeCm.Init(MaxRange, NumberOfChannels);
	}
	else if (PerChannelMaxRangeCm.Num() != NumberOfChannels)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS LiDAR: PerChannelMaxRangeM has %d entries but NumberOfChannels=%d — falling back to global MaxRange for all channels."),
			PerChannelMaxRangeCm.Num(), NumberOfChannels);
		PerChannelMaxRangeCm.Reset();
		PerChannelMaxRangeCm.Init(MaxRange, NumberOfChannels);
	}

	// Per-channel range stats for the log line — useful when validating
	// that the override actually got applied (e.g. populated from the
	// Hesai datasheet) vs. silently falling back to the global value.
	float MinPerCh = MaxRange, MaxPerCh = MaxRange;
	if (PerChannelMaxRangeCm.Num() > 0)
	{
		MinPerCh = PerChannelMaxRangeCm[0];
		MaxPerCh = PerChannelMaxRangeCm[0];
		for (float V : PerChannelMaxRangeCm) { MinPerCh = FMath::Min(MinPerCh, V); MaxPerCh = FMath::Max(MaxPerCh, V); }
	}
	const bool bUniformRange = (MinPerCh == MaxPerCh);

	UE_LOG(LogTemp, Log,
		TEXT("FSDS LiDAR: backend=%s  %d channels, %d pts/sec, %d pts/scan, H-FOV=%.0f° V-FOV=%.0f°, "
			"range=%s"),
		LidarPath == EFSDSLidarPath::GPU ? TEXT("GPU") : TEXT("CPU"),
		NumberOfChannels, PointsPerSecond, PointsPerScan,
		HFov, VFov,
		bUniformRange
			? *FString::Printf(TEXT("%.0fm (uniform)"), MaxRange / 100.f)
			: *FString::Printf(TEXT("%.1f-%.1fm (per-channel from settings.json)"), MinPerCh / 100.f, MaxPerCh / 100.f));

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
			// Per-channel max range (Hesai datasheet App. A.1.1 style).
			// PerChannelMaxRangeCm is normalised to NumberOfChannels in
			// OnSettingsApplied; index by V like the GPU shader does.
			const float ChannelMaxR = PerChannelMaxRangeCm.IsValidIndex(v)
				? PerChannelMaxRangeCm[v]
				: MaxRange;
			FVector RayEnd = SensorWorldPos + RayDir * ChannelMaxR;

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

			// Re-cull after noise — a +noise sample can push Dist past
			// the channel's max range. Match the GPU path which checks
			// ChannelMaxR after Box-Muller.
			if (Dist < MinRange || Dist > ChannelMaxR)
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

	// RT sizing.
	//
	// Naively setting RTW = PointsPerScan/channels gives a column count
	// equal to the LiDAR's H sample count, but image_x = tan(h) is
	// *non-linear*: at h=0 each RT column covers ~0.13° while LiDAR's
	// H step is 0.08°, so 1.66 adjacent LiDAR rays share a column and
	// read identical depth — producing the texel-quantization artifacts
	// that Phase-4 NN-distance diff caught (95th pct 26 cm at 10-20 m).
	// To get ≤1 LiDAR ray per RT column at the worst case (h=0),
	// oversample H by sec²(0)/sec²(±60°) ratio = 1/(1/4) = the H-FOV's
	// peak vs avg sec². Empirically a 1.66× oversample is enough; we
	// use the analytic per-step-at-h=0 ratio so it tracks any FOV
	// reconfiguration. RT_H is then derived from the planar aspect.
	const int32 PointsPerScan = FMath::Max(1, FMath::RoundToInt(
		(float)PointsPerSecond / FMath::Max(1.f, RotationsPerSecond)));
	const int32 NominalH = FMath::Max(64, PointsPerScan / FMath::Max(1, NumberOfChannels));
	// At h=0 (centre of the FOV, where image_x = tan(h) is most
	// compressed), cols-per-LiDAR-H-step =
	//     NominalH · HStepRad / (2·tan(HFOV/2))
	//   = HFovRad / (2·tan(HFOV/2))                         (NominalH cancels)
	// For a 120° H-FOV that's 0.605 — so the analytic minimum
	// oversample to push to ≥1 col/step at h=0 is 1.65×. ExtraOversample
	// pushes further to:
	//   (1) cleanly absorb int-truncation rounding in TexelColF
	//       (otherwise neighbour LiDAR rays at TexelColF=N.4 and N.6
	//        both round to N and read identical depth);
	//   (2) halve the V-row-offset depth error — at far distance the
	//       depth function is steep in image_y, so half-row-y offset
	//       turns into ~28 cm radial error at v=-3° on flat ground.
	//       Phase-4 95th-pct grew linearly with distance because of
	//       this. Doubling RT_H halves it.
	// Cost: RT pixel count grows as ExtraOversample². At 2× this is
	// ~12.5 M R32f = 50 MB on Apple Silicon — well within budget at 10 Hz.
	const float HFovRad             = FMath::DegreesToRadians(HFovDeg);
	const float HOversampleAnalytic = (2.f * TanHalfHRad) / FMath::Max(KINDA_SMALL_NUMBER, HFovRad);
	const float ExtraOversample     = FMath::Max(1.f, CVarLidarGPUExtraOversample.GetValueOnGameThread());
	const float HOversample         = FMath::Max(1.f, HOversampleAnalytic) * ExtraOversample;
	const int32 RTW                 = FMath::Max(64, FMath::CeilToInt((float)NominalH * HOversample));
	const float Aspect              = PlanarHHalfSpan / FMath::Max(KINDA_SMALL_NUMBER, PlanarVHalfSpan);
	const int32 RTH                 = FMath::Max(1, FMath::RoundToInt((float)RTW / Aspect));

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
		TEXT("FSDS LiDAR GPU: RT %dx%d (R32f) | H-FOV=%.1f° | V-FOV (planar)=%.1f° | tilt=%.2f° | range=%.0f m | ExtraOversample=%.2f×"),
		RTW, RTH, HFovDeg, PlanarVFovDeg, VFovCenterDeg, MaxRange / 100.f, ExtraOversample);

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

	// Skip if the next-dispatch slot is still occupied — at queue-depth
	// 2 this only happens under sustained slow readback (Apple Metal
	// occasional 2-3-frame readback hiccup on contention). Dropping
	// the scan is safer than overlapping two dispatches on the same
	// readback object. The Verbose log keeps this from being silent
	// if it becomes systematic.
	{
		const FReadbackSlot& Slot = ReadbackSlots[NextDispatchSlot];
		if (Slot.bInFlight || Slot.bLockDispatched)
		{
			UE_LOG(LogTemp, Verbose,
				TEXT("FSDS LiDAR GPU: dropping scan #%d — slot %d still in flight (queue-depth=%d saturated)"),
				GPUCaptureCount + 1, NextDispatchSlot, ReadbackQueueDepth);
			return false;
		}
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
	EnqueueDecodePass();
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

// Phase 3 — RDG decode pass + async buffer readback.
//
// EnqueueDecodePass runs on the game thread immediately after
// CaptureScene. It dispatches a render command which:
//   1. Builds an FRDGBuilder.
//   2. Registers the depth RT as an external texture (the camera has
//      already filled it via the SceneCapture run before this command
//      was queued — UE's render-thread queue is FIFO).
//   3. Allocates a transient structured buffer of float4 sized to the
//      LiDAR's ray count.
//   4. Adds a compute pass running FFSDSLidarDecodeCS — one thread per
//      ray, sampling depth at the spherical-to-planar texel and
//      producing a 3D point in vehicle frame (cm→m, ROS REP-103).
//   5. Adds an enqueue-copy pass from the structured buffer into the
//      persistent FRHIGPUBufferReadback so we can pull it back to CPU
//      next frame.
//
// PollGPUReadback runs on the game thread; when the readback IsReady,
// it dispatches a second render command to Lock(NumBytes)/memcpy/Unlock
// the buffer, then AsyncTask-bounces the points back to the game
// thread for ConsumeReadbackResult to unpack.
void UFSDSLidarSensor::EnqueueDecodePass()
{
	if (!GPUDepthRT) return;
	FTextureRenderTargetResource* RTResource = GPUDepthRT->GameThread_GetRenderTargetResource();
	if (!RTResource) return;

	const int32 PointsPerScan = FMath::Max(1, FMath::RoundToInt(
		(float)PointsPerSecond / FMath::Max(1.f, RotationsPerSecond)));
	const int32 NumChannels_LCL    = NumberOfChannels;
	const int32 NumHorizontalSteps = FMath::Max(1, PointsPerScan / FMath::Max(1, NumChannels_LCL));
	const int32 NumPoints          = NumChannels_LCL * NumHorizontalSteps;
	const uint32 PointsBytes       = NumPoints * sizeof(FVector4f);

	// Snapshot uniform values on the game thread for capture by the
	// render-thread lambda. Avoids touching `this` on the render
	// thread (UObject access from there is unsafe).
	struct FUniforms
	{
		uint32 NumChannels;
		uint32 NumHorizontalSteps;
		uint32 RTWidth;
		uint32 RTHeight;
		float HStartRad, HEndRad, VUpperRad, VLowerRad, TiltRad;
		float HHalfPlanar, VBottomPlanar, VTopPlanar;
		float MinRangeCm, MaxRangeCm, RangeNoiseStdCm, DropoutRate;
		uint32 RNGSeed;
		float SensorOffsetXm, SensorOffsetYm, SensorOffsetZm;
	} U;
	U.NumChannels        = (uint32)NumChannels_LCL;
	U.NumHorizontalSteps = (uint32)NumHorizontalSteps;
	U.RTWidth            = (uint32)GPURTWidth;
	U.RTHeight           = (uint32)GPURTHeight;
	U.HStartRad          = FMath::DegreesToRadians(HorizontalFOVStart);
	U.HEndRad            = FMath::DegreesToRadians(HorizontalFOVEnd);
	U.VUpperRad          = FMath::DegreesToRadians(VerticalFOVUpper);
	U.VLowerRad          = FMath::DegreesToRadians(VerticalFOVLower);
	U.TiltRad            = FMath::DegreesToRadians(GPUVerticalFOVCenterDeg);
	U.HHalfPlanar        = GPUPlanarHalfWidth;
	U.VBottomPlanar      = GPUPlanarBottom;
	U.VTopPlanar         = GPUPlanarTop;
	U.MinRangeCm         = MinRange;
	U.MaxRangeCm         = MaxRange;
	U.RangeNoiseStdCm    = RangeNoiseStd;
	U.DropoutRate        = DropoutRate;
	// Re-seed each scan from the current cycle counter so dropouts and
	// range-noise patterns aren't deterministic across the lifetime of
	// the simulation (matches the CPU path's FMath::FRand-driven
	// randomness in spirit).
	U.RNGSeed            = (uint32)FPlatformTime::Cycles();
	// SensorOffset is stored in cm; shader takes metres.
	U.SensorOffsetXm     = SensorOffset.X / 100.f;
	U.SensorOffsetYm     = SensorOffset.Y / 100.f;
	U.SensorOffsetZm     = SensorOffset.Z / 100.f;

	const int32 SlotIdx = NextDispatchSlot;
	FReadbackSlot& Slot = ReadbackSlots[SlotIdx];
	if (!Slot.Readback.IsValid())
	{
		// Each slot owns its own FRHIGPUBufferReadback. Naming them per
		// slot makes RDG event traces / debug captures distinguish the
		// in-flight scans.
		Slot.Readback = MakeUnique<FRHIGPUBufferReadback>(
			*FString::Printf(TEXT("FSDSLidarPointsReadback#%d"), SlotIdx));
	}

	Slot.EnqueueTimeSec  = FPlatformTime::Seconds();
	Slot.bInFlight       = true;
	Slot.bLockDispatched = false;
	NextDispatchSlot     = (NextDispatchSlot + 1) % ReadbackQueueDepth;

	FRHIGPUBufferReadback* Readback = Slot.Readback.Get();

	// Snapshot per-channel max-range so the render thread doesn't read
	// from the game-thread-owned UPROPERTY array. The render command
	// uploads this into a structured buffer SRV bound as the shader's
	// ChannelMaxRangeCm parameter.
	const TArray<float> ChannelMaxRangeCmCopy = PerChannelMaxRangeCm;

	ENQUEUE_RENDER_COMMAND(FSDSLidarDecode)(
		[Readback, RTResource, U, NumPoints, PointsBytes, ChannelMaxRangeCmCopy](FRHICommandListImmediate& RHICmdList)
		{
			FRHITexture* DepthRHI = RTResource->GetRenderTargetTexture();
			if (!DepthRHI) return;

			FRDGBuilder GraphBuilder(RHICmdList);

			FRDGTextureRef DepthRDG = RegisterExternalTexture(
				GraphBuilder, DepthRHI, TEXT("FSDSLidarDepthRT"));

			const FRDGBufferDesc BufDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector4f), NumPoints);
			FRDGBufferRef OutBuf = GraphBuilder.CreateBuffer(BufDesc, TEXT("FSDSLidarPoints"));

			// Per-channel max-range structured buffer. ~464 B for a 116-ch
			// LiDAR, allocated transient so it dies at end of graph.
			// CreateUploadBuffer's helper return doesn't carry the
			// "structured" type tag through to CreateSRV (it tries to
			// build a typed-buffer SRV → "Format cannot be unknown for
			// typed buffers" assert). Build the structured buffer
			// explicitly instead.
			const int32 ChannelCount = FMath::Max(1, ChannelMaxRangeCmCopy.Num());
			FRDGBufferRef ChannelRangeBuf = GraphBuilder.CreateBuffer(
				FRDGBufferDesc::CreateStructuredDesc(sizeof(float), ChannelCount),
				TEXT("FSDSLidarChannelMaxRange"));
			GraphBuilder.QueueBufferUpload(
				ChannelRangeBuf,
				ChannelMaxRangeCmCopy.GetData(),
				ChannelMaxRangeCmCopy.Num() * sizeof(float));

			auto* Params = GraphBuilder.AllocParameters<FFSDSLidarDecodeCS::FParameters>();
			Params->DepthTexture          = DepthRDG;
			Params->OutPoints             = GraphBuilder.CreateUAV(OutBuf);
			Params->ChannelMaxRangeCm     = GraphBuilder.CreateSRV(ChannelRangeBuf);
			Params->NumChannels           = U.NumChannels;
			Params->NumHorizontalSteps    = U.NumHorizontalSteps;
			Params->RTWidth               = U.RTWidth;
			Params->RTHeight              = U.RTHeight;
			Params->HorizontalFOVStartRad = U.HStartRad;
			Params->HorizontalFOVEndRad   = U.HEndRad;
			Params->VerticalFOVUpperRad   = U.VUpperRad;
			Params->VerticalFOVLowerRad   = U.VLowerRad;
			Params->TiltRad               = U.TiltRad;
			Params->HHalfPlanar           = U.HHalfPlanar;
			Params->VBottomPlanar         = U.VBottomPlanar;
			Params->VTopPlanar            = U.VTopPlanar;
			Params->MinRangeCm            = U.MinRangeCm;
			Params->MaxRangeCm            = U.MaxRangeCm;
			Params->RangeNoiseStdCm       = U.RangeNoiseStdCm;
			Params->DropoutRate           = U.DropoutRate;
			Params->RNGSeed               = U.RNGSeed;
			Params->SensorOffsetXm        = U.SensorOffsetXm;
			Params->SensorOffsetYm        = U.SensorOffsetYm;
			Params->SensorOffsetZm        = U.SensorOffsetZm;

			TShaderMapRef<FFSDSLidarDecodeCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
			const int32 ThreadGroups = FMath::DivideAndRoundUp(NumPoints, FFSDSLidarDecodeCS::ThreadGroupSize);
			FComputeShaderUtils::AddPass(
				GraphBuilder,
				RDG_EVENT_NAME("FSDSLidarDecode"),
				Shader, Params,
				FIntVector(ThreadGroups, 1, 1));

			AddEnqueueCopyPass(GraphBuilder, Readback, OutBuf, PointsBytes);

			GraphBuilder.Execute();
		});
}

void UFSDSLidarSensor::PollGPUReadback()
{
	// Iterate every readback slot so multiple in-flight scans can
	// drain in a single tick (queue-depth=2 — see header). For each
	// slot: if EnqueueCopy has been issued (bInFlight) and the lock
	// hasn't been queued yet (!bLockDispatched) and the GPU copy is
	// ready, dispatch a render-thread Lock + AsyncTask back to game
	// thread. ConsumeReadbackResult clears the slot.
	const int32 PointsPerScan = FMath::Max(1, FMath::RoundToInt(
		(float)PointsPerSecond / FMath::Max(1.f, RotationsPerSecond)));
	const int32 NumPoints = (NumberOfChannels > 0)
		? NumberOfChannels * (PointsPerScan / NumberOfChannels)
		: 0;
	if (NumPoints <= 0) return;

	TWeakObjectPtr<UFSDSLidarSensor> WeakSelf(this);

	for (int32 SlotIdx = 0; SlotIdx < ReadbackQueueDepth; ++SlotIdx)
	{
		FReadbackSlot& Slot = ReadbackSlots[SlotIdx];
		if (!Slot.bInFlight || Slot.bLockDispatched) continue;
		if (!Slot.Readback.IsValid()) continue;
		if (!Slot.Readback->IsReady()) continue;

		// FRHIGPUBufferReadback::Lock asserts IsInRenderingThread(), so
		// the lock+memcpy lives in a render command and the result
		// bounces back via AsyncTask(GameThread) to
		// ConsumeReadbackResult, tagged with SlotIdx so we know which
		// slot to clear.
		FRHIGPUBufferReadback* Readback = Slot.Readback.Get();
		Slot.bLockDispatched = true;

		ENQUEUE_RENDER_COMMAND(FSDSLidarReadbackLock)(
			[Readback, NumPoints, WeakSelf, SlotIdx](FRHICommandListImmediate& /*RHICmdList*/)
			{
				const uint32 NumBytes = NumPoints * sizeof(FVector4f);
				const FVector4f* Src = (const FVector4f*)Readback->Lock(NumBytes);

				TArray<FVector4f> LocalCopy;
				if (Src)
				{
					LocalCopy.SetNumUninitialized(NumPoints);
					FMemory::Memcpy(LocalCopy.GetData(), Src, NumBytes);
				}
				Readback->Unlock();

				AsyncTask(ENamedThreads::GameThread,
					[WeakSelf, Points = MoveTemp(LocalCopy), SlotIdx]() mutable
					{
						if (UFSDSLidarSensor* Self = WeakSelf.Get())
						{
							Self->ConsumeReadbackResult(SlotIdx, MoveTemp(Points));
						}
					});
			});
	}
}

void UFSDSLidarSensor::ConsumeReadbackResult(int32 SlotIdx, TArray<FVector4f>&& Points)
{
	// Game-thread consumer of the GPU decode pass. Walks the float4[]
	// output of FSDSLidarDecode.usf and packs valid hits (w == 1) into
	// PointCloudBuffer in the flat-float [x,y,z, x,y,z, ...] format
	// FSDSUdpBroadcaster + GetPointCloud() consumers expect. Skips
	// invalid entries (w == 0: dropped, far-plane, out-of-range, etc.).
	if (SlotIdx < 0 || SlotIdx >= ReadbackQueueDepth) return;
	FReadbackSlot& Slot = ReadbackSlots[SlotIdx];
	Slot.bLockDispatched = false;
	Slot.bInFlight       = false;

	const double NowS = FPlatformTime::Seconds();
	GPUReadbackLastLatencyMs = (NowS - Slot.EnqueueTimeSec) * 1000.0;
	GPUReadbackCount++;

	TArray<float> NewBuffer;
	NewBuffer.Reserve(Points.Num() * 3);
	int32 HitCount = 0;
	for (const FVector4f& P : Points)
	{
		if (P.W < 0.5f) continue;  // not a valid hit
		NewBuffer.Add(P.X);
		NewBuffer.Add(P.Y);
		NewBuffer.Add(P.Z);
		HitCount++;
	}

	{
		FScopeLock Lock(&PointCloudLock);
		PointCloudBuffer = MoveTemp(NewBuffer);
		CachedPointCount = HitCount;
		LastTimestamp    = FPlatformTime::Cycles64();
	}

	if (!bGPULoggedFirstReadback)
	{
		UE_LOG(LogTemp, Log,
			TEXT("FSDS LiDAR GPU: first readback OK | %d valid hits / %d rays | latency=%.2f ms"),
			HitCount, Points.Num(), GPUReadbackLastLatencyMs);
		bGPULoggedFirstReadback = true;
	}

	if (GPUReadbackCount > 0 && (GPUReadbackCount % 50) == 0)
	{
		UE_LOG(LogTemp, Log,
			TEXT("FSDS LiDAR GPU: readback #%d, %d hits, latency=%.2f ms"),
			GPUReadbackCount, HitCount, GPUReadbackLastLatencyMs);
	}
}
