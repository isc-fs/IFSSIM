#include "FSDSConeSpawner.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/Blueprint.h"
#include "Misc/FileHelper.h"

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

		FString TracksDir = FPaths::Combine(FPaths::ProjectDir(), TEXT("Content"), TEXT("tracks"));

		if (MapName.Contains(TEXT("Acceleration")))
		{
			CSVFilePath = FPaths::Combine(TracksDir, TEXT("acceleration.csv"));
		}
		else if (MapName.Contains(TEXT("Skidpad")))
		{
			CSVFilePath = FPaths::Combine(TracksDir, TEXT("skidpad.csv"));
		}
		else if (MapName.Contains(TEXT("customMap")) || MapName.Contains(TEXT("Custom")))
		{
			CSVFilePath = FPaths::Combine(TracksDir, TEXT("random_track.csv"));
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

void AFSDSConeSpawner::SpawnCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation)
{
	// Not used anymore — we spawn Blueprint actors instead
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
		SpawnCone(BlueMesh, BluePos, FRotator(0.f, FMath::RandRange(0.f, 360.f), 0.f));

		// Yellow cones on the right (outside)
		FVector YellowPos = TrackCenter + FVector(OvalX, OvalY, HeightOffset) + TrackDir * HalfWidth;
		SpawnCone(YellowMesh, YellowPos, FRotator(0.f, FMath::RandRange(0.f, 360.f), 0.f));
	}

	// Orange big cones at start/finish
	if (OrangeBigMesh)
	{
		FVector StartPos = TrackCenter + FVector(TrackRadius * 1.5f, 0.f, HeightOffset);
		SpawnCone(OrangeBigMesh, StartPos + FVector(0.f, -HalfWidth, 0.f));
		SpawnCone(OrangeBigMesh, StartPos + FVector(0.f, HalfWidth, 0.f));
		SpawnCone(OrangeBigMesh, StartPos + FVector(0.f, -HalfWidth - 100.f, 0.f));
		SpawnCone(OrangeBigMesh, StartPos + FVector(0.f, HalfWidth + 100.f, 0.f));
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

	// Load cone Blueprint classes — try multiple loading strategies
	auto LoadConeBP = [](const TCHAR* Path) -> UClass* {
		FString FullPath(Path);

		// Strategy 1: StaticLoadClass with _C suffix
		FString ClassPath = FullPath + TEXT("_C");
		UClass* Class = StaticLoadClass(AActor::StaticClass(), nullptr, *ClassPath);
		if (Class) { UE_LOG(LogTemp, Log, TEXT("FSDS Cones: Loaded via StaticLoadClass: %s"), *ClassPath); return Class; }

		// Strategy 2: Load UBlueprint then get GeneratedClass
		UBlueprint* BP = LoadObject<UBlueprint>(nullptr, Path);
		if (BP && BP->GeneratedClass) { UE_LOG(LogTemp, Log, TEXT("FSDS Cones: Loaded via UBlueprint: %s"), Path); return BP->GeneratedClass; }

		// Strategy 3: FSoftClassPath (async-safe)
		FSoftClassPath SoftPath(FullPath + TEXT("_C"));
		UClass* SoftClass = SoftPath.TryLoadClass<AActor>();
		if (SoftClass) { UE_LOG(LogTemp, Log, TEXT("FSDS Cones: Loaded via FSoftClassPath: %s"), Path); return SoftClass; }

		// Strategy 4: Try with .asset_name suffix pattern
		FString AssetName = FPaths::GetBaseFilename(FullPath);
		FString PackagePath = FPaths::GetPath(FullPath);
		FString AltPath = PackagePath + TEXT(".") + AssetName + TEXT("_C");
		Class = StaticLoadClass(AActor::StaticClass(), nullptr, *AltPath);
		if (Class) { UE_LOG(LogTemp, Log, TEXT("FSDS Cones: Loaded via alt path: %s"), *AltPath); return Class; }

		UE_LOG(LogTemp, Warning, TEXT("FSDS Cones: ALL strategies failed for %s"), Path);
		return nullptr;
	};

	UClass* BlueBP = LoadConeBP(TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/blue_trafficone"));
	UClass* YellowBP = LoadConeBP(TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/yellow_trafficone"));
	UClass* OrangeBigBP = LoadConeBP(TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/orange_trafficone"));
	UClass* OrangeSmallBP = LoadConeBP(TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/orange_mini_trafficone"));

	// Fallback: load per-color cone StaticMeshes (these have baked materials)
	UStaticMesh* BlueConeMesh = nullptr;
	UStaticMesh* YellowConeMesh = nullptr;
	UStaticMesh* OrangeConeMesh = nullptr;
	UStaticMesh* OrangeBigConeMesh = nullptr;
	UStaticMesh* DefaultConeMesh = nullptr;

	if (!BlueBP || !YellowBP)
	{
		// Try per-color meshes first (these are complete colored cone models)
		BlueConeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_blue.trafficone_mini_blue"));
		YellowConeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_yellow.trafficone_mini_yellow"));
		OrangeConeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_orange.trafficone_mini_orange"));
		OrangeBigConeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_big_orange.trafficone_big_orange"));

		// Fallback to engine cone
		DefaultConeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cone.Cone"));

		if (!BlueConeMesh) BlueConeMesh = DefaultConeMesh;
		if (!YellowConeMesh) YellowConeMesh = DefaultConeMesh;
		if (!OrangeConeMesh) OrangeConeMesh = DefaultConeMesh;
		if (!OrangeBigConeMesh) OrangeBigConeMesh = OrangeConeMesh;

		UE_LOG(LogTemp, Log, TEXT("FSDS Cones: StaticMesh mode (blue=%s, yellow=%s, orange=%s)"),
			BlueConeMesh != DefaultConeMesh ? TEXT("REAL") : TEXT("fallback"),
			YellowConeMesh != DefaultConeMesh ? TEXT("REAL") : TEXT("fallback"),
			OrangeConeMesh != DefaultConeMesh ? TEXT("REAL") : TEXT("fallback"));
	}

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

		UClass* BPClass = nullptr;
		if (Type == TEXT("blue")) BPClass = BlueBP;
		else if (Type == TEXT("yellow")) BPClass = YellowBP;
		else if (Type == TEXT("big_orange")) BPClass = OrangeBigBP;
		else if (Type == TEXT("small_orange")) BPClass = OrangeSmallBP;

		if (BPClass)
		{
			SpawnConeBP(BPClass, FVector(X, Y, HeightOffset), FRotator(0.f, FMath::RandRange(0.f, 360.f), 0.f));
		}
		else if (BlueConeMesh || DefaultConeMesh)
		{
			// Select correct mesh per cone type
			UStaticMesh* ConeMesh = DefaultConeMesh;
			if (Type == TEXT("blue")) ConeMesh = BlueConeMesh;
			else if (Type == TEXT("yellow")) ConeMesh = YellowConeMesh;
			else if (Type == TEXT("big_orange")) ConeMesh = OrangeBigConeMesh;
			else if (Type == TEXT("small_orange") || Type == TEXT("orange")) ConeMesh = OrangeConeMesh;

			if (!ConeMesh) ConeMesh = DefaultConeMesh;

			// Spawn AStaticMeshActor
			FActorSpawnParameters Params;
			Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

			AStaticMeshActor* ConeActor = GetWorld()->SpawnActor<AStaticMeshActor>(
				AStaticMeshActor::StaticClass(),
				FTransform(FRotator(0.f, FMath::RandRange(0.f, 360.f), 0.f), FVector(X, Y, HeightOffset)),
				Params);

			if (ConeActor)
			{
				UStaticMeshComponent* MeshComp = ConeActor->GetStaticMeshComponent();
				MeshComp->SetMobility(EComponentMobility::Movable);
				MeshComp->SetStaticMesh(ConeMesh);
				MeshComp->SetRelativeScale3D(FVector(ConeScale));
				TotalSpawned++;
			}
		}
	}
}
