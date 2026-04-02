#include "FSDSGameMode.h"
#include "FSDSVehiclePawn.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"

AFSDSGameMode::AFSDSGameMode()
{
	DefaultPawnClass = AFSDSVehiclePawn::StaticClass();
}

void AFSDSGameMode::StartPlay()
{
	Super::StartPlay();
	LogStartup();
	SpawnVehicle();

	// Start RPC server
	RpcServer.SetVehiclePawn(VehiclePawn);
	RpcServer.SetSettingsString(TEXT("{\"SimMode\": \"Car\"}"));
	RpcServer.Start(41451);
}

void AFSDSGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	RpcServer.Stop();
	UE_LOG(LogTemp, Log, TEXT("FSDS: Simulator shutting down"));
	Super::EndPlay(EndPlayReason);
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
