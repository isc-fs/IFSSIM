#pragma once

#include "CoreMinimal.h"
#include "Math/RandomStream.h"
#include "Components/ActorComponent.h"
#include "RHIGPUReadback.h"
#include "Math/Vector4.h"
#include "FSDSLidarSensor.generated.h"

class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

UENUM()
enum class EFSDSLidarPath : uint8
{
	// Legacy CPU path: ParallelFor across horizontal steps issuing
	// LineTraceSingleByChannel against the Chaos physics scene.
	// ~250 % CPU at 1.74 M pts/s (audit on dev, see #223).
	CPU,

	// GPU path (#223): depth-only render at the LiDAR's exact ray
	// grid, decoded into 3D points by a compute shader, async-readback
	// to game thread. Phase 1 only sets up the depth render; full
	// point production lands in Phases 2-3.
	GPU,
};

/**
 * LiDAR sensor — performs batch raycasts to generate 3D point clouds.
 *
 * Default profile: Hesai ATX (scaled for real-time sim performance)
 *   Real ATX: 256ch, 3.84M pts/s, 120°x20° FOV, 0.08°x0.05° res, 230m range
 *   Sim default: 128ch, 153,600 pts/s, 120°x20° FOV, 0.24°x0.16° res, 200m range
 *
 * Output: flat float array [x,y,z, x,y,z, ...] in sensor-local coordinates (meters)
 * Each point includes intensity as 4th float if bReturnIntensity is true.
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSLidarSensor : public UActorComponent
{
	GENERATED_BODY()

public:
	UFSDSLidarSensor();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Get the latest point cloud as flat [x,y,z,...] array (meters, sensor-local) */
	TArray<float> GetPointCloud() const;

	/** Get point count (number of 3D points, not floats) */
	int32 GetPointCount() const;

	/** Get wall-clock (Cycles64) timestamp of last scan */
	uint64 GetTimestamp() const { return LastTimestamp; }

	/** Get UE game/sim time of last scan capture, in ns. This is the
	 *  absolute sim instant the rays were cast; broadcasters put it on
	 *  the wire so the bridge can stamp header.stamp from it directly,
	 *  immune to GPU-readback + transport + DDS-backlog latency (which
	 *  the capture-to-send LagNs scheme could not fully cover — it drifted
	 *  upward over a long run). 0 until the first scan completes. */
	uint64 GetTimestampSimNs() const { return LastTimestampSimNs; }

	// --- Configuration (Hesai ATX-like defaults) ---

	/** Number of vertical laser channels */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	int32 NumberOfChannels = 128;

	/** Total points generated per second (across all channels) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	int32 PointsPerSecond = 153600;

	/** Scan rotations per second (Hz) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float RotationsPerSecond = 10.f;

	/** Vertical FOV upper bound (degrees, positive = up) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float VerticalFOVUpper = 7.f;

	/** Vertical FOV lower bound (degrees, negative = down) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float VerticalFOVLower = -13.f;

	/** Horizontal FOV start (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float HorizontalFOVStart = -60.f;

	/** Horizontal FOV end (degrees) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float HorizontalFOVEnd = 60.f;

	/** Maximum detection range in cm */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float MaxRange = 20000.f; // 200m

	/** Minimum detection range in cm */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float MinRange = 50.f; // 0.5m

	/** Sensor offset from vehicle origin (cm, local space).
	 *  Z must place the sensor ABOVE the vehicle origin — if it ends up below the
	 *  ground collider, downward rays start inside world geometry and register no
	 *  hits, producing a flat 2D scan. +60cm ≈ roof-mounted lidar for a formula car. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	FVector SensorOffset = FVector(140.f, 0.f, 60.f);

	/** Mount rotation relative to the vehicle (settings.json Lidar Roll/Pitch/Yaw,
	 *  degrees; UE convention, negative Pitch = tilted down). The scan pattern
	 *  (V-FOV, channels) is in the sensor's own frame; points stay in the vehicle
	 *  frame, so consumers see the same frame with the beams where the car has them. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	FRotator MountRotation = FRotator::ZeroRotator;

	/** Draw debug points in editor */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	bool bDrawDebugPoints = false;

	/** Range measurement noise σ in cm, Gaussian (0 = no noise). Hesai ATX ~2 cm.
	 *  settings.json declares this in metres; FSDSVehiclePawn does the m→cm
	 *  conversion (× 100) when wiring it through. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float RangeNoiseStd = 0.0f;

	/** Random point dropout probability [0,1] (0 = no dropout). Typical ~0.01 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float DropoutRate = 0.0f;

	/** GPU path: read the depth capture at each beam's sub-texel position
	 *  (1/Z stepped toward the neighbour texel on the same surface) instead
	 *  of the nearest texel's centre. Removes the fixed range ripple along
	 *  each ring on far, grazing ground. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	bool bDepthInterpolation = false;

	/** Weak-return dropout: P(no return) = DropoutWeakRate · I^-DropoutWeakExp,
	 *  capped at DropoutWeakMax, with I the hit's noiseless intensity in output
	 *  units (0-255 on the car's scale). The car's Hesai loses ~20 % of returns
	 *  at I 0-2 and ~1 % at I 32-64. A drop lasts several scans of the same
	 *  sweep direction: each beam gets its own run length, log-uniform from 2 to
	 *  DropoutBurstScans (0 = a fresh draw every scan), and DropoutBurstIidShare
	 *  of the probability stays a one-scan draw. GPU path only. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float DropoutWeakRate = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float DropoutWeakExp = 0.77f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float DropoutWeakMax = 0.3f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float DropoutBurstScans = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float DropoutBurstIidShare = 0.15f;

	/** Range-dependent noise: σ grows by this · range_m², in cm per m² (0 = flat σ). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float RangeNoiseRangeCoeffCm = 0.0f;

	/** Reflectance-dependent noise: σ ∝ (RefReflectance / ρ)^Exp, clamped to ×0.3..×4 (Exp 0 = off). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float RangeNoiseRefReflectance = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float RangeNoiseReflectanceExp = 0.0f;

	/** Depth edges: where a neighbouring beam (left or right, same channel)
	 *  ranges more than 30 cm away, the return comes back from that other
	 *  surface with probability EdgeSwapRate, or not at all with probability
	 *  EdgeDropRate. The car's Hesai does both at cone silhouettes: objects
	 *  grow into the beams beside them and edge beams flicker. GPU path only. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float EdgeSwapRate = 0.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float EdgeDropRate = 0.0f;

	/** Relative σ of scan-to-scan intensity jitter (0 = none). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR Noise")
	float IntensityNoise = 0.0f;

	/** Intensity law: I = IntensityScale · ρ · cos(θ_inc)^IntensityIncidenceExp
	 *  · range_m^-IntensityRangeExp · jitter, clamped to [0, IntensityMax] and
	 *  rounded to whole numbers when bIntensityRound. The car's Hesai reports
	 *  0-255 whole numbers that fall as r^-1.29 and don't depend on incidence.
	 *  IntensityScale < 0 keeps the old law: ρ · cos · (R_ref / r)², 0..1, with
	 *  R_ref 5 m on the GPU path and 1 m on the CPU path. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float IntensityScale = -1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float IntensityRangeExp = 2.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float IntensityIncidenceExp = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float IntensityMax = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	bool bIntensityRound = false;

	/** GPU path: ρ for hits with no reflectance-LUT entry (ground, paint,
	 *  scenery) is clamp(gain · scene luminance, 0, 1). Visible luminance
	 *  is darker than 905 nm reflectance (asphalt renders at ~0.04); the
	 *  shipped gain is fitted so the ground matches the car. Also feeds the
	 *  reflectance term of the range noise. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float LuminanceReflectanceGain = 1.0f;

	/** LuminanceReflectanceGain while a track's world (Montmeló) is built.
	 *  A world brings photo-matched colours and hides the level's floor,
	 *  which renders darker than asphalt. < 0 = use LuminanceReflectanceGain. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float WorldLuminanceReflectanceGain = -1.0f;

	/** Mirror sweep. The car's Hesai stamps a scan at its first point and
	 *  measures the rest over the next ~90 ms, linearly in azimuth, without
	 *  motion correction; the sweep direction alternates every scan. The sim
	 *  captures a scan in one instant, so each point is moved by the car's
	 *  motion over its delay: p' = p - tau (v + w x p), tau = SweepDurationMs
	 *  times the azimuth's fraction of the FOV (1 - fraction on reversed
	 *  scans). 0 = no sweep (one instant). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float SweepDurationMs = 0.0f;
	/** Reverse the sweep direction every scan, like the car's mirror. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	bool bSweepAlternate = true;

private:
	/** -1 unknown, 0 level floor, 1 a world is built: logs the gain on change. */
	int8 LastWorldGainState = -1;
public:

	/** Per-channel max-range overrides in CENTIMETRES (vehicle units),
	 *  one entry per channel indexed by VIdx (0 = lowest V angle, last
	 *  = highest). When this array's length matches NumberOfChannels,
	 *  the LiDAR uses ChannelMaxRange[VIdx] instead of the global
	 *  MaxRange — models the per-beam laser-power variance real LiDARs
	 *  exhibit (Hesai ATX_S01 datasheet App. A.1.1 style). When empty,
	 *  all channels fall back to MaxRange (preserves existing
	 *  settings.json behaviour). settings.json declares this in METRES;
	 *  FSDSVehiclePawn does the m→cm conversion when wiring through. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	TArray<float> PerChannelMaxRangeCm;

	/** Per-channel elevation in DEGREES (sensor frame), one per channel,
	 *  from settings.json ChannelElevationsDeg. Empty = the uniform
	 *  VerticalFOVLower..Upper grid. With NumberOfChannels entries,
	 *  OnSettingsApplied sorts a copy lowest first (channel 0 = lowest)
	 *  and sets VerticalFOVLower/Upper to its ends so the GPU capture
	 *  covers every beam. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	TArray<float> ChannelElevationsDeg;

	/** Ray-cast backend selection. Driven by settings.json LidarPath
	 *  ("cpu" | "gpu"); see #223. Switching at runtime requires a PIE
	 *  stop/start because BeginPlay sets up backend-specific resources. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	EFSDSLidarPath LidarPath = EFSDSLidarPath::CPU;

	// Pawn callback after settings.json values have been written to
	// the UPROPERTYs above. Must be called from AFSDSVehiclePawn after
	// SetupSensorsFromSettings — component BeginPlay runs *before*
	// the pawn's BeginPlay finishes its config pass, so the GPU path
	// setup has to be deferred until the real config is in place.
	void OnSettingsApplied();

private:
	/** The elevations both back-ends use, degrees, sorted lowest first:
	 *  ChannelElevationsDeg when it has NumberOfChannels entries, else the
	 *  uniform grid. Rebuilt by OnSettingsApplied; the input array is left
	 *  as given so a later re-apply sees the same input. */
	TArray<float> EffectiveElevationsDeg;

	// --- GPU path (#223) state. All null/zero when LidarPath==CPU. ---

	UPROPERTY() USceneCaptureComponent2D* GPUDepthCapture = nullptr;
	UPROPERTY() UTextureRenderTarget2D*   GPUDepthRT      = nullptr;

	// #255 — intensity capture. Renders SCS_FinalColorLDR (the post-
	// shaded scene as the camera would see it under world lighting)
	// alongside the depth capture; the decode shader samples it at the
	// same texel as the depth to compute Hesai-class intensity:
	//   intensity = ρ × cos(θ_inc) × (R_ref / range)²
	// — with ρ proxied by Rec.709 luminance(FinalColor) and cos(θ_inc)
	// computed from a depth-gradient surface normal inside the shader.
	//
	// FinalColorLDR was chosen over SCS_BaseColor / SCS_Normal because
	// the GBuffer-sourced captures return blank-white on Mac UE5 5.7's
	// Metal renderer (forward-shading path doesn't populate the GBuffer
	// the same way deferred does). FinalColorLDR is renderer-mode-
	// agnostic; the trade-off is that its luminance includes scene
	// lighting (a shadowed cone reads dimmer than a sunlit one), which
	// is a tolerable artefact for a sim-side LiDAR proxy.
	UPROPERTY() USceneCaptureComponent2D* GPUColorCapture  = nullptr;
	UPROPERTY() UTextureRenderTarget2D*   GPUColorRT       = nullptr;

	// True when the M_LiDARStencilEncoder post-process material was
	// loaded successfully and added as a blendable to GPUColorCapture
	// (#321 D-Phase-2). When true, the FinalColorLDR capture's alpha
	// channel carries `stencil_id / 255` per-pixel, and the decode
	// shader's `UseReflectanceLUT` uniform is set to 1 so the
	// per-cone-material reflectance LUT lookup actually fires.
	// When false, the LUT is dormant and the shader falls back to
	// Rec.709 luminance — D-Phase-1 behaviour, no regression. Set
	// once in `InitializeGPUPath` and read each scan in
	// `EnqueueDecodePass`.
	bool bReflectanceLUTActive = false;

	// Stand up the depth-only SceneCapture for the GPU path. Called
	// from BeginPlay when LidarPath==GPU. Phase-1 wiring; the depth
	// data is rendered but not yet consumed (Phase 2 adds readback,
	// Phase 3 the decode shader).
	void InitializeGPUPath();

	// Drive the GPU capture from the rate-limited tick. Returns true
	// if a capture was issued this tick.
	bool TickGPUPath(float DeltaTime);

	// Phase-3 (#223) — RDG decode pass + async buffer readback.
	//   1. EnqueueDecodePass: render command that builds an FRDGBuilder,
	//      runs the FSDSLidarDecode compute shader against the depth RT
	//      to produce a structured buffer of float4 points, then issues
	//      AddEnqueueCopyPass into a persistent FRHIGPUBufferReadback.
	//   2. PollGPUReadback: dispatches a render command that checks
	//      IsReady there and, if ready, does Lock(NumBytes)/memcpy/Unlock and
	//      AsyncTask the result back to the game thread.
	//   3. ConsumeReadbackResult: unpacks the float4 array into the
	//      flat-float [x,y,z, x,y,z, ...] PointCloudBuffer that
	//      FSDSUdpBroadcaster + GetPointCloud() consumers expect.
	void EnqueueDecodePass();
	void PollGPUReadback();
	void ConsumeReadbackResult(int32 SlotIdx, TArray<FVector4f>&& Points);

	// Queue-depth-2 readback ring. At 10 Hz scan + ~70-100 ms readback
	// latency on Apple Metal, a single in-flight readback meant the
	// scan-N result landed right as scan-N+1 fired (no headroom under
	// any render stutter). Pipelining 2 keeps the effective end-to-end
	// latency close to one frame instead of one full scan period —
	// /lidar/Lidar1 always carries a result that's at most ~50 ms old
	// instead of ~100 ms. Cost: 2 × (NumPoints × float4) of staging
	// memory ≈ 5.6 MB at 174 k pts/scan; trivial.
	struct FReadbackSlot
	{
		TUniquePtr<FRHIGPUBufferReadback> Readback;
		bool   bInFlight       = false;  // decode + EnqueueCopy dispatched; not yet consumed
		bool   bLockDispatched = false;  // render-thread Lock queued; awaiting ConsumeReadbackResult
		double EnqueueTimeSec  = 0.0;    // wallclock for latency telemetry
		// Cycles64 stamp captured at dispatch (when the depth render fires);
		// becomes LastTimestamp when this slot's readback is consumed, so
		// downstream consumers see the *physical capture* time of the cloud
		// rather than the consume time (which lands ≥1 frame later via the
		// readback ring). Mirrors the CPU path where LastTimestamp is set
		// at scan time. See issue #232.
		uint64 CaptureCycles64 = 0;
		// UE game/sim time at dispatch, in ns — the sim-clock twin of
		// CaptureCycles64, becomes LastTimestampSimNs at consume. This is
		// the value the bridge stamps header.stamp from under use_sim_time.
		uint64 CaptureSimNs = 0;
	};
	static constexpr int32 ReadbackQueueDepth = 2;
	FReadbackSlot ReadbackSlots[ReadbackQueueDepth];
	int32 NextDispatchSlot = 0;

	// Pose snapshot at the moment the capture was issued. The readback
	// arrives ≥1 frame later when the actor has moved on; Phase 3
	// expresses decoded points relative to this captured pose so the
	// cloud matches the geometry of when the rays were cast.
	FTransform GPUPendingOwnerTransform;
	FVector    GPUPendingSensorWorldPos = FVector::ZeroVector;
	FQuat      GPUPendingOwnerRotation  = FQuat::Identity;
	// Vehicle velocity (m/s) and angular velocity (rad/s) in the vehicle's UE
	// frame at capture time, for the sweep smear.
	FVector    GPUPendingVelLocalMps    = FVector::ZeroVector;
	FVector    GPUPendingOmegaLocal     = FVector::ZeroVector;
	// The same for the CPU path, snapshot on the game thread before the async scan.
	FVector    CpuSweepVelLocalMps      = FVector::ZeroVector;
	FVector    CpuSweepOmegaLocal       = FVector::ZeroVector;
	uint32     CpuSweepCounter          = 0;

	// Diagnostic latency tracking — emitted on first successful
	// readback and every 50th thereafter. Per-slot enqueue time lives
	// in FReadbackSlot::EnqueueTimeSec; LastLatencyMs is the most
	// recent slot's measured GPU→CPU round-trip.
	double GPUReadbackLastLatencyMs  = 0.0;
	int32  GPUReadbackCount          = 0;
	// Polls that found the copy still running on the render thread. Before
	// the readiness check moved there, these were read early and returned
	// the previous scan in this slot.
	int32  GPUReadbackNotReadyPolls  = 0;
	bool   bGPULoggedFirstReadback   = false;

	// Round-trip test for the spherical-ray ↔ planar-texel mapping.
	// Run at the end of InitializeGPUPath. Logs PASS/FAIL and the
	// max observed reprojection error in radians. Phase-3 decode
	// shader uses the same formulas this test covers; if the test
	// fails here we know the decode will produce wrong points before
	// we ever GPU-debug a shader.
	bool ValidateProjectionRoundTrip() const;

	// Cached projection geometry derived from the LiDAR FOV at GPU-
	// path init time. Used by the round-trip test now and by the
	// Phase-3 decode shader's uniform buffer later.
	float GPUVerticalFOVCenterDeg = 0.f;   // camera tilt pitch (deg)
	float GPUPlanarHalfWidth      = 0.f;   // tan(HFOV/2)
	float GPUPlanarBottom         = 0.f;   // image-plane Y at frustum bottom
	float GPUPlanarTop            = 0.f;   // image-plane Y at frustum top
	int32 GPURTWidth              = 0;
	int32 GPURTHeight             = 0;

	float GPUScanAccumulator = 0.f;

	// Diagnostic counters mirrored from the Phase-0 spike: rolling avg
	// of CaptureScene() game-thread cost, reported every 5 s. Will be
	// kept through Phase 4 cross-validation, removed at Phase 5.
	double GPUCapAccumulatorMs = 0.0;
	int32  GPUCapSampleCount   = 0;
	double GPULastReportTime   = 0.0;
	int32  GPUCaptureCount     = 0;
	bool   bGPUDumpedRT        = false;
	void PerformScan(UWorld* InWorld, AActor* InOwner, FTransform OwnerTransform);

	// Rate limiter — scan fires at RotationsPerSecond Hz, not every frame
	float ScanAccumulator = 0.f;

	// Async guard — prevents overlapping scans if a frame runs long
	std::atomic<bool> bScanInProgress{false};

	TArray<float> PointCloudBuffer;
	int32 CachedPointCount = 0;
	uint64 LastTimestamp = 0;       // wall-clock (Cycles64) at capture
	uint64 LastTimestampSimNs = 0;  // UE game/sim time at capture, ns
	float CurrentHorizontalAngle = 0.f;

	FCriticalSection PointCloudLock;

	// Deterministic noise source. Lazily seeded from the scenario seed on first
	// use (the seed is set after component construction). Each sensor has its
	// own stream so sensors cannot perturb each other's sequences.
	// See FSDSRandom.h.
	// Monotonic scan index, salts the per-scan GPU RNG seed so each scan
	// differs while the whole sequence stays reproducible for a given seed.
	uint32 GpuScanCounter = 0;

	// Generation the scan counter was last restarted for. Without this the
	// counter runs monotonically from editor start and a repeat scenario draws
	// different GPU noise than the run it is meant to reproduce.
	uint32 GpuScanCounterGeneration = 0;
	/** Per-beam dropout-burst seed, drawn on the first scan of each seed generation. */
	uint32 GpuDropoutCellSeed = 0;
	FRandomStream NoiseStream;
	// Generation this stream was seeded for. A scenario reset bumps the
	// generation, which forces a re-seed so a repeat run genuinely restarts
	// the sequence instead of continuing the previous one.
	uint32 NoiseStreamGeneration = 0;
	FRandomStream& Noise();
};
