#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FSDSReferee.h"
#include "FSDSConeSpawner.generated.h"

/**
 * Spawns traffic cones on the track at BeginPlay.
 * Place this actor in your map and it will spawn cones
 * from the configured arrays or from a CSV file.
 * When a Referee is set, registers all cones for hit tracking.
 */
UCLASS(BlueprintType, Blueprintable)
class FSDSPLUGIN_API AFSDSConeSpawner : public AActor
{
	GENERATED_BODY()

public:
	AFSDSConeSpawner();

	virtual void BeginPlay() override;

	/** Set the referee actor for cone registration */
	void SetReferee(AFSDSReferee* InReferee) { Referee = InReferee; }

	/** Reload cones from a new CSV path without calling BeginPlay again (safe to call at runtime) */
	void ReloadTrack(const FString& NewCSVPath)
	{
		CSVFilePath = NewCSVPath;
		TotalSpawned = 0;
		SpawnFromCSV();
	}

	/** Cone mesh paths */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	FString BlueConeAssetPath = TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/blue_trafficone.blue_trafficone");

	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	FString YellowConeAssetPath = TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/yellow_trafficone.yellow_trafficone");

	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	FString OrangeBigConeAssetPath = TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/orange_trafficone.orange_trafficone");

	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	FString OrangeSmallConeAssetPath = TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/orange_mini_trafficone.orange_mini_trafficone");

	/** Cone scale */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	float ConeScale = 1.0f;

	/** Height offset above ground */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	float HeightOffset = 5.0f;

	/** If set, load cone positions from this CSV file */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	FString CSVFilePath;

	/** Spawn a simple oval test track */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	bool bSpawnTestTrack = true;

	/** Center of the test track */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	FVector TrackCenter = FVector(4500.f, 8500.f, 100.f);

	/** Radius of the test track oval */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	float TrackRadius = 3000.f;

	/** Track width (distance between blue and yellow cones) */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	float TrackWidth = 300.f;

	/** Number of cones per side */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	int32 NumConesPerSide = 40;

	/** All spawned cone actors (for cleanup on track reload) */
	UPROPERTY()
	TArray<AActor*> SpawnedCones;

private:
	void SpawnTestTrack();
	void SpawnFromCSV();
	void SpawnCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation = FRotator::ZeroRotator);
	void SpawnConeBP(UClass* BPClass, FVector Location, FRotator Rotation = FRotator::ZeroRotator);

	/** Spawn a static mesh cone and register it with the referee */
	AActor* SpawnStaticMeshCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation, EFSDSConeColor Color);

	int32 TotalSpawned = 0;

	UPROPERTY()
	AFSDSReferee* Referee = nullptr;
};
