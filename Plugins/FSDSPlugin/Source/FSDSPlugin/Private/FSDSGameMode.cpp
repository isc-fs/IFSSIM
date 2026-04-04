#include "FSDSGameMode.h"
#include "FSDSVehiclePawn.h"
#include "FSDSSettings.h"
#include "FSDSConeSpawner.h"
#include "FSDSReferee.h"
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

	// Spawn referee actor (tracks cone hits, laps, timing)
	FActorSpawnParameters RefereeSpawnParams;
	RefereeSpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	RefereeActor = GetWorld()->SpawnActor<AFSDSReferee>(
		AFSDSReferee::StaticClass(), FTransform::Identity, RefereeSpawnParams);

	if (RefereeActor && VehiclePawn)
	{
		RefereeActor->LoadStartPos(VehiclePawn->GetActorLocation());
		UE_LOG(LogTemp, Log, TEXT("FSDS: Referee actor spawned"));
	}

	// Spawn cone spawner with deferred construction so we can set the referee
	// before BeginPlay runs (cones need referee reference during spawning)
	ConeSpawnerActor = GetWorld()->SpawnActorDeferred<AFSDSConeSpawner>(
		AFSDSConeSpawner::StaticClass(), FTransform::Identity);

	if (ConeSpawnerActor)
	{
		ConeSpawnerActor->SetReferee(RefereeActor);
		ConeSpawnerActor->FinishSpawning(FTransform::Identity); // Triggers BeginPlay
		UE_LOG(LogTemp, Log, TEXT("FSDS: ConeSpawner spawned with Referee wired (%d cones)"),
			ConeSpawnerActor->SpawnedCones.Num());
	}

	// Start RPC server
	RpcServer.SetVehiclePawn(VehiclePawn);
	RpcServer.SetReferee(RefereeActor);
	RpcServer.SetWorld(GetWorld());
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
