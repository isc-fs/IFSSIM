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
	/**
	 * A cone reported a collision. Routes the impulse to the vehicle so the
	 * plant feels the hit.
	 *
	 * Bound on the CONE, not the car, and that is the whole trick. When the
	 * FMU drives, the car's mesh is kinematic and its own hit reports a
	 * NormalImpulse of zero — the solver does not integrate it, so there is no
	 * reaction to report. The cone simulates, so its hit carries the real
	 * impulse, and Newton's third law gives the car's.
	 */
	UFUNCTION()
	void OnConeHit(UPrimitiveComponent* HitComp, AActor* OtherActor,
	               UPrimitiveComponent* OtherComp, FVector NormalImpulse,
	               const FHitResult& Hit);

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

	/** Half-height (cm) of the vertical window searched for ground under a
	 *  cone or the start pose, centred on z = 0. 200 m covers any FS venue's
	 *  relief; ground outside it falls back to the flat-world placement. */
	UPROPERTY(EditAnywhere, Category = "FSDS Cones")
	float GroundSearchHalfHeightCm = 20000.f;

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
	 * SpawnFromCSV call. Used by the loadTrack and reset RPCs to teleport
	 * the car into a known-good starting pose.
	 *
	 * Forward axis:
	 *   - two or more big-orange gates (acceleration, skidpad): from the
	 *     start gate (the one nearest the first track cone) toward the
	 *     others;
	 *   - one gate (closed loops): the smaller-variance PCA axis of the
	 *     big-orange cones, signed toward the 4 nearest blue/yellow cones.
	 * The car sits BackupCm behind the start gate along -Forward.
	 *
	 * Height and attitude come from the ground under that point: Z is the
	 * ground plus HeightOffset + 50 cm, and pitch/roll follow the plane fitted
	 * to a 4-ray cross ±0.8 m around it, with heading kept along Forward. On
	 * a flat floor at z = 0 this is the old pose exactly. Where no ground is
	 * found it falls back to that flat pose, with a warning.
	 *
	 * Returns false (out-params untouched) without at least 4 big-orange and
	 * 1 blue/yellow cone; the caller then falls back to the level's
	 * PlayerStart. BackupCm defaults to 300 cm, the FS Driverless start area.
	 */
	bool ComputeStartGatePose(FVector& OutLocation, FQuat& OutRotation, float BackupCm = 300.f) const;

private:
	void SpawnTestTrack();
	void SpawnFromCSV();
	void SpawnCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation = FRotator::ZeroRotator);
	void SpawnConeBP(UClass* BPClass, FVector Location, FRotator Rotation = FRotator::ZeroRotator);

	/** Spawn a static mesh cone and register it with the referee */
	AActor* SpawnStaticMeshCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation, EFSDSConeColor Color);

	/** Ground under (X, Y): a downward trace for WorldStatic *objects* within
	 *  ±GroundSearchHalfHeightCm, ignoring this spawner, its cones and pawns. */
	bool TraceGroundAt(const FVector2D& XY, FHitResult& OutHit) const;

	/** Start pose at FlatLocation's XY facing Forward, set on the ground there. */
	void PlaceStartPoseOnGround(const FVector& FlatLocation, const FVector& Forward,
	                            FVector& OutLocation, FQuat& OutRotation) const;

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
