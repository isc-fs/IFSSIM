#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "FSDSGameMode.generated.h"

/**
 * FSDS Game Mode - Initializes the simulator when Play is pressed.
 * Spawns the vehicle and will start the RPC server (in later phases).
 */
UCLASS()
class FSDSPLUGIN_API AFSDSGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AFSDSGameMode();

	virtual void StartPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	void LogStartup();
};
