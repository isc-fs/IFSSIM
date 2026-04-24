#include "FSDSConeSpawner.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/Blueprint.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformProcess.h"

AFSDSConeSpawner::AFSDSConeSpawner()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AFSDSConeSpawner::BeginPlay()
{
	Super::BeginPlay();

	// Auto-detect track CSV from map name
	if (CSVFilePath.IsEmpty() && GetWorld())
	{
		FString MapName = GetWorld()->GetMapName();
		MapName.RemoveFromStart(TEXT("UEDPIE_0_")); // Strip PIE prefix

		// Build candidate search directories.
		// 1) Editor / PIE: ProjectDir()/Content/tracks  (source tree)
		// 2) Packaged Mac: AdditionalNonUFSFiles are staged into LaunchDir, so the
		//    CSV files land at <App>/Contents/UE/IFSSIM/Binaries/Mac/Content/tracks
		TArray<FString> CandidateDirs;
		CandidateDirs.Add(FPaths::Combine(FPaths::ProjectDir(),  TEXT("Content"), TEXT("tracks")));  // editor / PIE source tree
		CandidateDirs.Add(FPaths::Combine(FPaths::LaunchDir(),   TEXT("Content"), TEXT("tracks")));  // LaunchDir fallback
		CandidateDirs.Add(FPaths::Combine(FPaths::LaunchDir(),   TEXT("tracks")));                   // safety net
		// Packaged: tracks/ lives NEXT TO the .app in the distribution folder.
		// ProjectDir = {App}/Contents/UE/IFSSIM/  →  ../../../../ = parent of {App}
		{
			FString SiblingTracksDir = FPaths::ConvertRelativePathToFull(
				FPaths::Combine(FPaths::ProjectDir(), TEXT("../../../../tracks")));
			CandidateDirs.Add(SiblingTracksDir);
		}
		// User-writable fallback for runtime-generated tracks
		CandidateDirs.Add(FPaths::Combine(FPlatformProcess::UserSettingsDir(), TEXT("IFSSIM"), TEXT("tracks")));

		// Helper: resolve a leaf CSV name across all candidate dirs, preferring the
		// first directory that actually contains the file.
		auto ResolveCSV = [&](const FString& Leaf) -> FString
		{
			for (const FString& Dir : CandidateDirs)
			{
				FString Full = FPaths::Combine(Dir, Leaf);
				if (FPaths::FileExists(Full))
				{
					UE_LOG(LogTemp, Log, TEXT("FSDS ConeSpawner: Resolved '%s' → %s"), *Leaf, *Full);
					return Full;
				}
			}
			// Return the primary candidate (may not exist; SpawnFromCSV will log the error)
			FString Primary = FPaths::Combine(CandidateDirs[0], Leaf);
			UE_LOG(LogTemp, Warning, TEXT("FSDS ConeSpawner: CSV not found in any search dir, will try primary path: %s"), *Primary);
			return Primary;
		};

		if (MapName.Contains(TEXT("Acceleration")))
		{
			CSVFilePath = ResolveCSV(TEXT("acceleration.csv"));
		}
		else if (MapName.Contains(TEXT("Skidpad")))
		{
			CSVFilePath = ResolveCSV(TEXT("skidpad.csv"));
		}
		else if (MapName.Contains(TEXT("customMap")) || MapName.Contains(TEXT("Custom")))
		{
			CSVFilePath = ResolveCSV(TEXT("random_track.csv"));
		}
		else
		{
			// Try a CSV matching the map name, then fall back to random_track.csv
			FString MapCSV = ResolveCSV(MapName.ToLower() + TEXT(".csv"));
			if (FPaths::FileExists(MapCSV))
			{
				CSVFilePath = MapCSV;
			}
			else
			{
				FString DefaultCSV = ResolveCSV(TEXT("random_track.csv"));
				if (FPaths::FileExists(DefaultCSV))
				{
					CSVFilePath = DefaultCSV;
					UE_LOG(LogTemp, Log, TEXT("FSDS ConeSpawner: No track CSV for '%s', using random_track.csv"), *MapName);
				}
			}
		}

		if (!CSVFilePath.IsEmpty())
		{
			UE_LOG(LogTemp, Log, TEXT("FSDS ConeSpawner: Auto-detected track CSV: %s"), *CSVFilePath);
		}
	}

	if (!CSVFilePath.IsEmpty())
	{
		SpawnFromCSV();
	}
	else if (bSpawnTestTrack)
	{
		SpawnTestTrack();
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS ConeSpawner: Spawned %d cones on %s"),
		TotalSpawned, *GetWorld()->GetMapName());
}

AActor* AFSDSConeSpawner::SpawnStaticMeshCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation, EFSDSConeColor Color)
{
	if (!Mesh || !GetWorld()) return nullptr;

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	AStaticMeshActor* ConeActor = GetWorld()->SpawnActor<AStaticMeshActor>(
		AStaticMeshActor::StaticClass(),
		FTransform(Rotation, Location),
		Params);

	if (ConeActor)
	{
		UStaticMeshComponent* MeshComp = ConeActor->GetStaticMeshComponent();
		MeshComp->SetMobility(EComponentMobility::Movable);
		MeshComp->SetStaticMesh(Mesh);
		MeshComp->SetRelativeScale3D(FVector(ConeScale));
		TotalSpawned++;

		SpawnedCones.Add(ConeActor);

		// Register with referee for hit tracking and cone position publishing
		if (Referee)
		{
			FTransform ConeTransform = ConeActor->GetActorTransform();
			switch (Color)
			{
			case EFSDSConeColor::Yellow:
				Referee->AppendYellowCone(ConeTransform);
				break;
			case EFSDSConeColor::Blue:
				Referee->AppendBlueCone(ConeTransform);
				break;
			case EFSDSConeColor::OrangeLarge:
				Referee->AppendBigOrangeCone(ConeTransform);
				break;
			case EFSDSConeColor::OrangeSmall:
				Referee->AppendSmallOrangeCone(ConeTransform);
				break;
			default:
				break;
			}
			Referee->RegisterConeActor(ConeActor, Color);
		}
	}

	return ConeActor;
}

void AFSDSConeSpawner::SpawnCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation)
{
	// Delegate to new method — color unknown from test track context
	SpawnStaticMeshCone(Mesh, Location, Rotation, EFSDSConeColor::Unknown);
}

void AFSDSConeSpawner::SpawnConeBP(UClass* BPClass, FVector Location, FRotator Rotation)
{
	if (!BPClass || !GetWorld()) return;

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	AActor* ConeActor = GetWorld()->SpawnActor<AActor>(BPClass, FTransform(Rotation, Location), Params);
	if (ConeActor)
	{
		ConeActor->SetActorScale3D(FVector(ConeScale));
		TotalSpawned++;
		SpawnedCones.Add(ConeActor);
	}
}

void AFSDSConeSpawner::SpawnTestTrack()
{
	// Load cone meshes — fallback to engine Cone shape
	UStaticMesh* BlueMesh = LoadObject<UStaticMesh>(nullptr, *BlueConeAssetPath);
	UStaticMesh* YellowMesh = LoadObject<UStaticMesh>(nullptr, *YellowConeAssetPath);
	UStaticMesh* OrangeBigMesh = LoadObject<UStaticMesh>(nullptr, *OrangeBigConeAssetPath);

	UStaticMesh* FallbackCone = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cone.Cone"));
	if (!BlueMesh) { BlueMesh = FallbackCone; UE_LOG(LogTemp, Log, TEXT("FSDS ConeSpawner: Using fallback cone for blue")); }
	if (!YellowMesh) { YellowMesh = FallbackCone; UE_LOG(LogTemp, Log, TEXT("FSDS ConeSpawner: Using fallback cone for yellow")); }
	if (!OrangeBigMesh) { OrangeBigMesh = FallbackCone; }

	float HalfWidth = TrackWidth / 2.f;

	// Spawn cones in an oval pattern
	for (int32 i = 0; i < NumConesPerSide; i++)
	{
		float Angle = (float)i / (float)NumConesPerSide * 2.f * PI;

		// Oval shape (elongated in X)
		float OvalX = TrackRadius * 1.5f * FMath::Cos(Angle);
		float OvalY = TrackRadius * FMath::Sin(Angle);

		// Direction perpendicular to track
		FVector TrackDir(-TrackRadius * 1.5f * FMath::Sin(Angle), TrackRadius * FMath::Cos(Angle), 0.f);
		TrackDir.Normalize();

		// Blue cones on the left (inside)
		FVector BluePos = TrackCenter + FVector(OvalX, OvalY, HeightOffset) - TrackDir * HalfWidth;
		SpawnStaticMeshCone(BlueMesh, BluePos, FRotator(0.f, FMath::RandRange(0.f, 360.f), 0.f), EFSDSConeColor::Blue);

		// Yellow cones on the right (outside)
		FVector YellowPos = TrackCenter + FVector(OvalX, OvalY, HeightOffset) + TrackDir * HalfWidth;
		SpawnStaticMeshCone(YellowMesh, YellowPos, FRotator(0.f, FMath::RandRange(0.f, 360.f), 0.f), EFSDSConeColor::Yellow);
	}

	// Orange big cones at start/finish
	if (OrangeBigMesh)
	{
		FVector StartPos = TrackCenter + FVector(TrackRadius * 1.5f, 0.f, HeightOffset);
		SpawnStaticMeshCone(OrangeBigMesh, StartPos + FVector(0.f, -HalfWidth, 0.f), FRotator::ZeroRotator, EFSDSConeColor::OrangeLarge);
		SpawnStaticMeshCone(OrangeBigMesh, StartPos + FVector(0.f, HalfWidth, 0.f), FRotator::ZeroRotator, EFSDSConeColor::OrangeLarge);
		SpawnStaticMeshCone(OrangeBigMesh, StartPos + FVector(0.f, -HalfWidth - 100.f, 0.f), FRotator::ZeroRotator, EFSDSConeColor::OrangeLarge);
		SpawnStaticMeshCone(OrangeBigMesh, StartPos + FVector(0.f, HalfWidth + 100.f, 0.f), FRotator::ZeroRotator, EFSDSConeColor::OrangeLarge);
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS ConeSpawner: Test track spawned at (%.0f, %.0f) r=%.0f"),
		TrackCenter.X, TrackCenter.Y, TrackRadius);
}

void AFSDSConeSpawner::SpawnFromCSV()
{
	FString FileContent;
	if (!FFileHelper::LoadFileToString(FileContent, *CSVFilePath))
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS ConeSpawner: Failed to load CSV: %s"), *CSVFilePath);
		return;
	}

	// Load per-color cone StaticMeshes.
	// Note: blue_trafficone / yellow_trafficone / orange_trafficone / orange_mini_trafficone
	// are Texture2D assets (not Blueprints) — use the trafficone_mini_* StaticMesh variants.
	// These are cooked via DirectoriesToAlwaysCook = /Game/RaceCourse/Model/Environment/trafficones_scaled.
	UStaticMesh* BlueConeMesh = LoadObject<UStaticMesh>(nullptr,
		TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_blue.trafficone_mini_blue"));
	UStaticMesh* YellowConeMesh = LoadObject<UStaticMesh>(nullptr,
		TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_yellow.trafficone_mini_yellow"));
	UStaticMesh* OrangeConeMesh = LoadObject<UStaticMesh>(nullptr,
		TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_orange.trafficone_mini_orange"));
	UStaticMesh* OrangeBigConeMesh = LoadObject<UStaticMesh>(nullptr,
		TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_big_orange.trafficone_big_orange"));

	// Use the first available cooked mesh as fallback — do NOT rely on
	// /Engine/BasicShapes/Cone which is not cooked in Shipping builds.
	UStaticMesh* FallbackMesh = BlueConeMesh ? BlueConeMesh
	                          : YellowConeMesh ? YellowConeMesh
	                          : OrangeConeMesh ? OrangeConeMesh
	                          : OrangeBigConeMesh;

	UE_LOG(LogTemp, Log, TEXT("FSDS Cones: blue=%s yellow=%s orange=%s big_orange=%s"),
		BlueConeMesh    ? TEXT("OK") : TEXT("FAIL"),
		YellowConeMesh  ? TEXT("OK") : TEXT("FAIL"),
		OrangeConeMesh  ? TEXT("OK") : TEXT("FAIL"),
		OrangeBigConeMesh ? TEXT("OK") : TEXT("FAIL"));

	if (!FallbackMesh)
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS ConeSpawner: No cone mesh available. "
			"Check DirectoriesToAlwaysCook includes /Game/RaceCourse/Model/Environment/trafficones_scaled"));
		return;
	}

	if (!BlueConeMesh)      BlueConeMesh      = FallbackMesh;
	if (!YellowConeMesh)    YellowConeMesh    = FallbackMesh;
	if (!OrangeConeMesh)    OrangeConeMesh    = FallbackMesh;
	if (!OrangeBigConeMesh) OrangeBigConeMesh = FallbackMesh;

	TArray<FString> Lines;
	FileContent.ParseIntoArrayLines(Lines);

	for (const FString& Line : Lines)
	{
		TArray<FString> Parts;
		Line.ParseIntoArray(Parts, TEXT(","));
		if (Parts.Num() < 3) continue;

		FString Type = Parts[0].TrimStartAndEnd();
		float X = FCString::Atof(*Parts[1]) * 100.f; // meters to cm
		float Y = FCString::Atof(*Parts[2]) * -100.f; // flip Y, meters to cm

		EFSDSConeColor Color = EFSDSConeColor::Unknown;
		if      (Type == TEXT("blue"))                                   Color = EFSDSConeColor::Blue;
		else if (Type == TEXT("yellow"))                                 Color = EFSDSConeColor::Yellow;
		else if (Type == TEXT("big_orange"))                             Color = EFSDSConeColor::OrangeLarge;
		else if (Type == TEXT("small_orange") || Type == TEXT("orange")) Color = EFSDSConeColor::OrangeSmall;

		UStaticMesh* ConeMesh = FallbackMesh;
		if      (Type == TEXT("blue"))                                   ConeMesh = BlueConeMesh;
		else if (Type == TEXT("yellow"))                                 ConeMesh = YellowConeMesh;
		else if (Type == TEXT("big_orange"))                             ConeMesh = OrangeBigConeMesh;
		else if (Type == TEXT("small_orange") || Type == TEXT("orange")) ConeMesh = OrangeConeMesh;

		FVector Location(X, Y, HeightOffset);
		FRotator Rotation(0.f, FMath::RandRange(0.f, 360.f), 0.f);
		SpawnStaticMeshCone(ConeMesh, Location, Rotation, Color);
	}
}
