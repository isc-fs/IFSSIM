#include "FSDSGameMode.h"
#include "Engine/World.h"

AFSDSGameMode::AFSDSGameMode()
{
	// No default pawn — vehicles will be spawned explicitly
	DefaultPawnClass = nullptr;
}

void AFSDSGameMode::StartPlay()
{
	Super::StartPlay();
	LogStartup();
}

void AFSDSGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	UE_LOG(LogTemp, Log, TEXT("FSDS: Simulator shutting down"));
	Super::EndPlay(EndPlayReason);
}

void AFSDSGameMode::LogStartup()
{
	UE_LOG(LogTemp, Log, TEXT("========================================"));
	UE_LOG(LogTemp, Log, TEXT("  IFSSIM - Formula Student Simulator"));
	UE_LOG(LogTemp, Log, TEXT("  UE5.7 Native Build"));
	UE_LOG(LogTemp, Log, TEXT("========================================"));

	if (GetWorld())
	{
		FString MapName = GetWorld()->GetMapName();
		UE_LOG(LogTemp, Log, TEXT("FSDS: Map loaded: %s"), *MapName);
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS: GameMode initialized successfully"));
	UE_LOG(LogTemp, Log, TEXT("FSDS: Phase 1 - Plugin skeleton ready"));
}
