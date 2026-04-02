#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "FSDSLidarSensor.generated.h"

/**
 * LiDAR sensor — performs batch raycasts to generate 3D point clouds.
 * Configurable channels, FOV, points per second, and range.
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSLidarSensor : public UActorComponent
{
	GENERATED_BODY()

public:
	UFSDSLidarSensor();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Get the latest point cloud as flat [x,y,z,x,y,z,...] array */
	TArray<float> GetPointCloud() const;

	/** Get timestamp of last scan */
	uint64 GetTimestamp() const { return LastTimestamp; }

	// --- Configuration ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	int32 NumberOfChannels = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	int32 PointsPerSecond = 40960;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float RotationsPerSecond = 10.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float VerticalFOVUpper = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float VerticalFOVLower = -25.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float HorizontalFOVStart = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float HorizontalFOVEnd = 359.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	float MaxRange = 10000.f; // in cm (100m)

	/** Sensor offset from vehicle origin (in local space) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS LiDAR")
	FVector SensorOffset = FVector(140.f, 0.f, -20.f);

private:
	void PerformScan();

	TArray<float> PointCloudBuffer;
	uint64 LastTimestamp = 0;
	float AccumulatedTime = 0.f;
	float CurrentHorizontalAngle = 0.f;

	FCriticalSection PointCloudLock;
};
