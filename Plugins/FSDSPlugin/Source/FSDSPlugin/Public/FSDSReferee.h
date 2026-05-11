#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FSDSReferee.generated.h"

/**
 * FSDS Referee — tracks competition state: cone hits, lap times, track layout.
 * Automatically detects cone hits (overlap events) and lap completions
 * (finish line trigger from big orange cones).
 */

UENUM(BlueprintType)
enum class EFSDSEventType : uint8
{
	Trackdrive,    // Multiple laps (default 10)
	Acceleration,  // Single straight-line run
	Skidpad,       // Figure-8, 4 crossings (2 right + 2 left)
	Autocross,     // Single lap
	Unknown
};

UENUM(BlueprintType)
enum class EFSDSConeColor : uint8
{
	Yellow,
	Blue,
	OrangeLarge,
	OrangeSmall,
	Unknown
};

/**
 * Stencil IDs the cone spawner writes into each cone's
 * CustomDepthStencilValue (#321 D-Phase-1). A future LiDAR post-process
 * material (D-Phase-2 follow-up) samples SceneTexture:CustomStencil and
 * encodes this ID into the alpha channel of the FinalColorLDR capture
 * the LiDAR shader already reads. The decode shader then indexes
 * FSDSLidarDecode.usf's ReflectanceLUT[] by this ID to use per-material
 * 905 nm reflectance instead of the Rec.709-luminance placeholder.
 *
 * ID 0 is reserved for "not a tagged cone" — anything not in the LUT
 * falls back to the luminance placeholder. Order matches the
 * EFSDSConeColor enum slots that exist; Unknown gets no tag.
 *
 * Adding a new ID also means updating:
 *   - Plugins/FSDSPlugin/Shaders/Private/FSDSLidarDecode.usf
 *   - Plugins/FSDSPlugin/Source/FSDSPlugin/Private/Sensors/FSDSLidarSensor.cpp
 *     (LUT initialisation in InitializeGPUPath)
 *   - The Phase-2 post-process material's stencil-to-alpha encoding
 */
namespace FSDSConeStencil
{
	static constexpr uint8 None        = 0;  // not a tagged cone
	static constexpr uint8 Blue        = 1;
	static constexpr uint8 Yellow      = 2;
	static constexpr uint8 OrangeLarge = 3;
	static constexpr uint8 OrangeSmall = 4;
	// IDs 5-15 reserved for future cone types (white-stripe sub-material,
	// custom DV cones). Cap at 15 because the LUT is uint8 stencil →
	// 8-bit alpha encoding; anything > 15 drops dynamic range we want
	// for the lookup-vs-fallback discrimination.
	static constexpr uint8 MaxID       = 15;
}

USTRUCT(BlueprintType)
struct FFSDSCone
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	FVector2D Location = FVector2D::ZeroVector;

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	EFSDSConeColor Color = EFSDSConeColor::Unknown;
};

USTRUCT(BlueprintType)
struct FFSDSRefereeState
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	int32 DooCounter = 0;

	/** Off-track / out-of-bounds counter */
	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	int32 OffTrackCounter = 0;

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	TArray<float> Laps;

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	TArray<FFSDSCone> Cones;

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	FVector2D CarStartLocation = FVector2D::ZeroVector;

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	EFSDSEventType EventType = EFSDSEventType::Trackdrive;

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	bool bFinished = false;

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	int32 RequiredLaps = 10;
};

UCLASS(BlueprintType, Blueprintable)
class FSDSPLUGIN_API AFSDSReferee : public AActor
{
	GENERATED_BODY()

public:
	AFSDSReferee();

	virtual void Tick(float DeltaTime) override;

	/** Get the current referee state */
	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	FFSDSRefereeState GetState() const { return State; }

	/** Record a cone being hit/knocked down */
	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	int32 ConeHit(FString ConeName);

	/** Record a lap completion */
	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	int32 LapCompleted(float LapTime);

	/** Register cones on the track */
	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	void AppendYellowCone(FTransform ConeTransform);

	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	void AppendBlueCone(FTransform ConeTransform);

	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	void AppendBigOrangeCone(FTransform ConeTransform);

	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	void AppendSmallOrangeCone(FTransform ConeTransform);

	/** Set the car start position */
	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	void LoadStartPos(FVector Pos);

	/** Set event type and configure rules accordingly */
	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	void SetEventType(EFSDSEventType Type, int32 NumLaps = 10);

	/** Register a spawned cone actor for collision tracking */
	void RegisterConeActor(AActor* ConeActor, EFSDSConeColor Color);

	/** Reset referee state (for new session / track reload) */
	UFUNCTION(BlueprintCallable, Category = "FSDS Referee")
	void ResetState();

protected:
	virtual void BeginPlay() override;

private:
	FFSDSRefereeState State;

	void AppendCone(FTransform Transform, EFSDSConeColor Color);

	/** Callback when vehicle overlaps a cone */
	UFUNCTION()
	void OnConeOverlap(UPrimitiveComponent* OverlappedComp, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex,
		bool bFromSweep, const FHitResult& SweepResult);

	/** Finish line detection */
	UPROPERTY()
	class UBoxComponent* FinishLineTrigger = nullptr;

	UFUNCTION()
	void OnFinishLineOverlap(UPrimitiveComponent* OverlappedComp, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex,
		bool bFromSweep, const FHitResult& SweepResult);

	/** Track cone original positions for displacement detection */
	TMap<AActor*, FVector> ConeOriginalPositions;
	TSet<AActor*> HitCones; // Already counted cones (avoid double-counting)

	/** Lap timing */
	double LapStartTime = 0.0;
	bool bLapTimerRunning = false;
	bool bVehicleInsideFinishZone = false; // Debounce

	/** Finish line geometry (computed from big orange cones) */
	FVector FinishLineCenter = FVector::ZeroVector;
	FVector FinishLineDirection = FVector::ForwardVector;
	bool bFinishLineValid = false;

	/** Displacement threshold for cone hit (cm) */
	float ConeHitThreshold = 15.0f;

	/** Delay before recording cone positions (let physics settle) */
	float PositionSnapshotDelay = 2.0f;
	float PositionSnapshotTimer = 0.f;
	bool bPositionsRecorded = false;

	/** Out-of-bounds detection */
	bool bWasOffTrack = false; // Debounce: only count transitions on→off
	float OffTrackCheckInterval = 0.2f; // Check every 200ms (not every frame)
	float OffTrackTimer = 0.f;

	/** Check if a 2D point is inside the track boundaries defined by cones */
	bool IsInsideTrack(FVector2D Point) const;

	/** Find the nearest cone of a given color to a point */
	float DistToNearestCone(FVector2D Point, EFSDSConeColor Color) const;
};
