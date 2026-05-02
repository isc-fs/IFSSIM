#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
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

	/** Ray-cast backend selection. Driven by settings.json LidarPath
	 *  ("cpu" | "gpu"); see #223. Switching at runtime requires a PIE
	 *  stop/start because BeginPlay sets up backend-specific resources. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	EFSDSLidarPath LidarPath = EFSDSLidarPath::CPU;

private:
	// --- GPU path (#223) state. All null/zero when LidarPath==CPU. ---

	UPROPERTY() USceneCaptureComponent2D* GPUDepthCapture = nullptr;
	UPROPERTY() UTextureRenderTarget2D*   GPUDepthRT      = nullptr;

	// Stand up the depth-only SceneCapture for the GPU path. Called
	// from BeginPlay when LidarPath==GPU. Phase-1 wiring; the depth
	// data is rendered but not yet consumed (Phase 2 adds readback,
	// Phase 3 the decode shader).
	void InitializeGPUPath();

	// Drive the GPU capture from the rate-limited tick. Returns true
	// if a capture was issued this tick.
	bool TickGPUPath(float DeltaTime);

	// Phase-1 stage 2 still uses the spike-style symmetric setup. The
	// asymmetric V-FOV via custom projection lands in stage 3 of phase
	// 1 alongside the projection round-trip test.
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
