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
enum class EFSDSConeColor : uint8
{
	Yellow,
	Blue,
	OrangeLarge,
	OrangeSmall,
	Unknown
};

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

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	TArray<float> Laps;

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	TArray<FFSDSCone> Cones;

	UPROPERTY(BlueprintReadWrite, Category = "FSDS")
	FVector2D CarStartLocation = FVector2D::ZeroVector;
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
};
