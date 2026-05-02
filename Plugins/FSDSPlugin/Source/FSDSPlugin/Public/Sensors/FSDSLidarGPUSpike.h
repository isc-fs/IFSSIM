#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSLidarGPUSpike.generated.h"

class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UFSDSLidarSensor;

/**
 * Phase-0 viability spike for issue #223 (GPU compute LiDAR).
 *
 * Adds a depth-only USceneCaptureComponent2D pinned to the LiDAR
 * sensor's pose and triggers it at the LiDAR scan rate. Not a sensor —
 * produces no point cloud, doesn't feed the broadcaster. Sole purpose:
 * measure the incremental GPU and game-thread cost of the depth
 * capture before committing to the rest of the GPU LiDAR plan (Phases
 * 1–5 in #223).
 *
 * Toggle via CVar fsds.LidarGPUSpike.Enable (default 0). When 0, no
 * resources are created, BeginPlay is a no-op, Tick is a no-op.
 *
 * Throwaway. Deletes itself at Phase 1 once the real GPU LiDAR class
 * lands. If you're reading this in the future and Phases 1+ have
 * shipped, this whole file should be removed.
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSLidarGPUSpike : public UActorComponent
{
	GENERATED_BODY()

public:
	UFSDSLidarGPUSpike();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// Stand the spike up after the pawn's LidarSensor has been
	// configured from settings.json. Reads HFOV / SensorOffset /
	// RotationsPerSecond off SourceLidar so the depth capture has the
	// same geometry envelope as the CPU path it's a stand-in for.
	void Initialize(const UFSDSLidarSensor* SourceLidar);

private:
	UPROPERTY() USceneCaptureComponent2D* CaptureComponent = nullptr;
	UPROPERTY() UTextureRenderTarget2D*   DepthRT          = nullptr;

	float ScanIntervalSeconds = 0.1f;  // 10 Hz default; overridden in Initialize.
	float ScanAccumulator = 0.f;

	// Rolling stats for CaptureScene() game-thread cost. Reported
	// every 5 s so we don't spam the log under sustained driving.
	double CapAccumulatorMs = 0.0;
	int32  CapSampleCount   = 0;
	double LastReportTime   = 0.0;

	// Auto-dump the depth RT to disk on the 5th capture so we get the
	// Phase-0 visual sanity check (cones nearer than the road plane)
	// without needing console-command plumbing. One-shot per session.
	int32 DumpAfterNCaptures = 5;
	int32 CaptureCount       = 0;
	bool  bDumpedRT          = false;
};
