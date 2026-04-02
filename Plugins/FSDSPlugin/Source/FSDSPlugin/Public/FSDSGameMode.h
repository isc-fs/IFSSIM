#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "FSDSGameMode.generated.h"

class AFSDSVehiclePawn;

/**
 * FSDS Game Mode — Spawns the vehicle and manages the simulator lifecycle.
 */
UCLASS()
class FSDSPLUGIN_API AFSDSGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AFSDSGameMode();

	virtual void StartPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** Get the active vehicle pawn */
	AFSDSVehiclePawn* GetVehiclePawn() const { return VehiclePawn; }

private:
	void SpawnVehicle();
	void LogStartup();

	UPROPERTY()
	AFSDSVehiclePawn* VehiclePawn = nullptr;
};
