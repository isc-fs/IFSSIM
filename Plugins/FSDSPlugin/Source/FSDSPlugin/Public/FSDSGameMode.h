#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "RPC/FSDSRpcServer.h"
#include "FSDSGameMode.generated.h"

class AFSDSVehiclePawn;

/**
 * FSDS Game Mode — Spawns the vehicle and manages the RPC server.
 */
UCLASS()
class FSDSPLUGIN_API AFSDSGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AFSDSGameMode();

	virtual void StartPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual APawn* SpawnDefaultPawnAtTransform_Implementation(AController* NewPlayer, const FTransform& SpawnTransform) override;

	AFSDSVehiclePawn* GetVehiclePawn() const { return VehiclePawn; }
	FFSDSRpcServer* GetRpcServer() { return &RpcServer; }

private:
	void SpawnVehicle();
	void LogStartup();

	UPROPERTY()
	AFSDSVehiclePawn* VehiclePawn = nullptr;

	FFSDSRpcServer RpcServer;
};
