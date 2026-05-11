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

	/**
	 * Derive the canonical "behind the start gate, facing the track"
	 * vehicle pose from the cones spawned by the most recent
	 * SpawnFromCSV call. Used by the loadTrack RPC to teleport the car
	 * into a known-good starting pose so the autonomy stack doesn't
	 * have to fight a 90° map/track misalignment on first ticks.
	 *
	 * Algorithm — robust to any gate geometry:
	 *   OrangeCentroid = mean(big_orange positions)
	 *   TrackCentroid  = mean(blue + yellow positions)
	 *   Forward        = normalize(TrackCentroid − OrangeCentroid)
	 *   OutLocation    = OrangeCentroid − BackupCm × Forward, lifted by HeightOffset
	 *   OutRotation    = yaw = atan2(Forward.Y, Forward.X)
	 *
	 * Returns false (and leaves out-params untouched) when there isn't
	 * enough cone data to compute a sensible answer (≥1 big_orange and
	 * ≥2 track cones required). Caller falls back to the level's
	 * PlayerStart in that case.
	 *
	 * BackupCm is the gap behind the start gate in centimetres
	 * (default 300 cm = 3 m, matches FS Driverless start-area spec).
	 */
	bool ComputeStartGatePose(FVector& OutLocation, FQuat& OutRotation, float BackupCm = 300.f) const;

private:
	void SpawnTestTrack();
	void SpawnFromCSV();
	void SpawnCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation = FRotator::ZeroRotator);
	void SpawnConeBP(UClass* BPClass, FVector Location, FRotator Rotation = FRotator::ZeroRotator);

	/** Spawn a static mesh cone and register it with the referee */
	AActor* SpawnStaticMeshCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation, EFSDSConeColor Color);

	int32 TotalSpawned = 0;

	// Per-spawn-pass ground-snap counters. SpawnStaticMeshCone increments
	// one of these every call; SpawnFromCSV/SpawnTestTrack reset both on
	// entry and log the totals on exit so the per-track-load summary
	// makes it visible whether the floor was found under every cone.
	int32 GroundSnapHits = 0;
	int32 GroundSnapMisses = 0;

	// Cone-position bookkeeping populated by SpawnFromCSV, consumed by
	// ComputeStartGatePose. UE world-space cm. Cleared at the start of
	// each spawn pass so a track reload always sees only the current
	// track's cones.
	TArray<FVector> BigOrangePositions;
	TArray<FVector> BlueYellowPositions;

	UPROPERTY()
	AFSDSReferee* Referee = nullptr;
};
