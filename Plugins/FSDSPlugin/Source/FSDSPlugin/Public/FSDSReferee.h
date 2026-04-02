#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FSDSReferee.generated.h"

/**
 * FSDS Referee — tracks competition state: cone hits, lap times, track layout.
 * Called from Blueprint actors (FinishLine, cone triggers) or from C++.
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

protected:
	virtual void BeginPlay() override;

private:
	FFSDSRefereeState State;

	void AppendCone(FTransform Transform, EFSDSConeColor Color);
};
