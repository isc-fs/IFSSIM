#include "FSDSGameMode.h"
#include "FSDSVehiclePawn.h"
#include "FSDSSettings.h"
#include "FSDSConeSpawner.h"
#include "FSDSReferee.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformFileManager.h"
#include "GenericPlatform/GenericPlatformFile.h"

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

	if (RefereeActor)
	{
		if (VehiclePawn)
		{
			RefereeActor->LoadStartPos(VehiclePawn->GetActorLocation());
		}

		// Auto-detect event type from map name
		FString MapName = GetWorld()->GetMapName();
		MapName.RemoveFromStart(TEXT("UEDPIE_0_"));

		if (MapName.Contains(TEXT("Acceleration")))
		{
			RefereeActor->SetEventType(EFSDSEventType::Acceleration);
		}
		else if (MapName.Contains(TEXT("Skidpad")))
		{
			RefereeActor->SetEventType(EFSDSEventType::Skidpad);
		}
		else if (MapName.Contains(TEXT("Autocross")))
		{
			RefereeActor->SetEventType(EFSDSEventType::Autocross);
		}
		else
		{
			RefereeActor->SetEventType(EFSDSEventType::Trackdrive, 10);
		}

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

	// Start RPC server (commands + camera)
	RpcServer.SetVehiclePawn(VehiclePawn);
	RpcServer.SetReferee(RefereeActor);
	RpcServer.SetWorld(GetWorld());
	RpcServer.SetSettingsString(FFSDSSettings::Get().GetSettingsString());
	RpcServer.SetUdpBroadcaster(&UdpBroadcaster);
	RpcServer.Start(41451);

	// AF_UNIX (UDS) listener removed in this PR. It only ever served
	// the LiDAR-over-UDS experiment that #322 retired; the
	// StreamSensorsUds stub kept after that was a no-op. Sensor-over-
	// UDS can return as a small scoped feature when bandwidth pressure
	// on the TCP sensor stream actually shows up. The
	// /tmp/ifssim_streams directory + docker-compose bind-mount that
	// supported it are also gone.

	// UDP broadcaster — pushes sensor + LiDAR frames over UDP to the
	// bridge. Started here unconditionally. The bridge consumes the
	// LiDAR UDP stream on port 41500 (LiDAR is UDP-only since #322).
	// Sensor UDP is currently benign-but-unused — sensors still ride
	// the TCP push.
	//
	// Targeting 127.0.0.1: on macOS Docker Desktop the bridge sits inside
	// the Linux VM; the docker-compose.yml UDP port forwards (41452/41500)
	// route host-loopback datagrams into the container's listener. On
	// Linux hosts the same port forward works natively. Broadcast/multicast
	// would force the host to multicast across interfaces unnecessarily.
	//
	// Why 41500 (not 41453 nor 51453):
	//   - Adjacent ports (41452/41453) hit macOS Docker Desktop's UDP-
	//     range-proxying bug where only ONE port of a contiguous range
	//     gets forwarded. So we want a non-adjacent port.
	//   - High ports (≥49152) collide with Windows' dynamic/ephemeral
	//     port range (`netsh int ipv4 show dynamicportrange udp`), where
	//     Docker Desktop UDP forwards silently fail (the earlier choice
	//     of 51453 broke Windows live LiDAR for exactly this reason —
	//     SendTo returned ok=1 but packets never reached the container).
	//   - 41500 is non-adjacent to 41452 AND below 49152: works on both.
	UdpBroadcaster.SetVehiclePawn(VehiclePawn);
	UdpBroadcaster.SetReferee(RefereeActor);
	UdpBroadcaster.Start(TEXT("127.0.0.1"), 41452, 41500);
}

void AFSDSGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Null pawn pointers before stopping threads to prevent dangling access
	RpcServer.SetVehiclePawn(nullptr);
	UdpBroadcaster.SetVehiclePawn(nullptr);
	UdpBroadcaster.Stop();
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
