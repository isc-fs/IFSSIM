#include "FSDSGameMode.h"
#include "FSDSVehiclePawn.h"
#include "FSDSSettings.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"

AFSDSGameMode::AFSDSGameMode()
{
	DefaultPawnClass = AFSDSVehiclePawn::StaticClass();

	// Allow spawning even if there's collision at the spawn point
	bUseSeamlessTravel = false;
}

void AFSDSGameMode::StartPlay()
{
	Super::StartPlay();
	LogStartup();
	SpawnVehicle();

	// Start RPC server with settings string
	RpcServer.SetVehiclePawn(VehiclePawn);
	RpcServer.SetSettingsString(FFSDSSettings::Get().GetSettingsString());
	RpcServer.Start(41451);
}

void AFSDSGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	RpcServer.Stop();
	UE_LOG(LogTemp, Log, TEXT("FSDS: Simulator shutting down"));
	Super::EndPlay(EndPlayReason);
}

APawn* AFSDSGameMode::SpawnDefaultPawnAtTransform_Implementation(AController* NewPlayer, const FTransform& SpawnTransform)
{
	// Spawn with collision override to avoid "collision at spawn location" failure
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

	APawn* Pawn = GetWorld()->SpawnActor<AFSDSVehiclePawn>(
		AFSDSVehiclePawn::StaticClass(), SpawnTransform, SpawnParams);

	if (Pawn)
	{
		UE_LOG(LogTemp, Log, TEXT("FSDS: Vehicle spawned via custom spawn (collision override)"));
	}
	return Pawn;
}

void AFSDSGameMode::SpawnVehicle()
{
	APlayerController* PC = GetWorld()->GetFirstPlayerController();
	if (PC && PC->GetPawn())
	{
		VehiclePawn = Cast<AFSDSVehiclePawn>(PC->GetPawn());
		if (VehiclePawn)
		{
			UE_LOG(LogTemp, Log, TEXT("FSDS: Vehicle pawn possessed at %s"), *VehiclePawn->GetActorLocation().ToString());
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("FSDS: No player controller found. Add a PlayerStart to your map!"));
	}
}

void AFSDSGameMode::LogStartup()
{
	UE_LOG(LogTemp, Log, TEXT("========================================"));
	UE_LOG(LogTemp, Log, TEXT("  IFSSIM - Formula Student Simulator"));
	UE_LOG(LogTemp, Log, TEXT("  UE5.7 Native Build"));
	UE_LOG(LogTemp, Log, TEXT("========================================"));

	if (GetWorld())
	{
		UE_LOG(LogTemp, Log, TEXT("FSDS: Map loaded: %s"), *GetWorld()->GetMapName());
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS: GameMode initialized - Vehicle spawning..."));
}
