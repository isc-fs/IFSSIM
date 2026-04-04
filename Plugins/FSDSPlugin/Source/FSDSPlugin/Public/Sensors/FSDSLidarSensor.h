#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSLidarSensor.generated.h"

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

	/** Sensor offset from vehicle origin (cm, local space) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	FVector SensorOffset = FVector(140.f, 0.f, -20.f);

	/** Draw debug points in editor */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	bool bDrawDebugPoints = false;

private:
	void PerformScan();

	TArray<float> PointCloudBuffer;
	int32 CachedPointCount = 0;
	uint64 LastTimestamp = 0;
	float CurrentHorizontalAngle = 0.f;

	FCriticalSection PointCloudLock;
};
