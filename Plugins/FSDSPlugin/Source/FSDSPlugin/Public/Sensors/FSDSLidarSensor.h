#pragma once

#include "CoreMinimal.h"
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

	/** Get timestamp of last scan */
	uint64 GetTimestamp() const { return LastTimestamp; }

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
	// --- GPU path (#223) state. All null/zero when LidarPath==CPU. ---

	UPROPERTY() USceneCaptureComponent2D* GPUDepthCapture = nullptr;
	UPROPERTY() UTextureRenderTarget2D*   GPUDepthRT      = nullptr;

	// #255 — intensity captures. ColorCapture renders SCS_BaseColor,
	// NormalCapture renders SCS_Normal. Both share view geometry with
	// GPUDepthCapture (same FOV / RT size / position / rotation), so
	// the decode shader samples them at the same texel as the depth
	// to compute Hesai-class intensity:
	//   intensity = ρ_905 × cos(θ_inc) × (R_ref / range)²
	UPROPERTY() USceneCaptureComponent2D* GPUColorCapture  = nullptr;
	UPROPERTY() UTextureRenderTarget2D*   GPUColorRT       = nullptr;
	UPROPERTY() USceneCaptureComponent2D* GPUNormalCapture = nullptr;
	UPROPERTY() UTextureRenderTarget2D*   GPUNormalRT      = nullptr;

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
	//   2. PollGPUReadback: game-thread checks IsReady, dispatches a
	//      second render command to do Lock(NumBytes)/memcpy/Unlock and
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
		bool   bInFlight       = false;  // EnqueueCopy issued; IsReady not yet observed
		bool   bLockDispatched = false;  // render-thread Lock queued; awaiting ConsumeReadbackResult
		double EnqueueTimeSec  = 0.0;    // wallclock for latency telemetry
		// Cycles64 stamp captured at dispatch (when the depth render fires);
		// becomes LastTimestamp when this slot's readback is consumed, so
		// downstream consumers see the *physical capture* time of the cloud
		// rather than the consume time (which lands ≥1 frame later via the
		// readback ring). Mirrors the CPU path where LastTimestamp is set
		// at scan time. See issue #232.
		uint64 CaptureCycles64 = 0;
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

	// Diagnostic latency tracking — emitted on first successful
	// readback and every 50th thereafter. Per-slot enqueue time lives
	// in FReadbackSlot::EnqueueTimeSec; LastLatencyMs is the most
	// recent slot's measured GPU→CPU round-trip.
	double GPUReadbackLastLatencyMs  = 0.0;
	int32  GPUReadbackCount          = 0;
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
	uint64 LastTimestamp = 0;
	float CurrentHorizontalAngle = 0.f;

	FCriticalSection PointCloudLock;
};
