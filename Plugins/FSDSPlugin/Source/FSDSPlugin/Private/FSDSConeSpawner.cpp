#include "FSDSConeSpawner.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Misc/FileHelper.h"

AFSDSConeSpawner::AFSDSConeSpawner()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AFSDSConeSpawner::BeginPlay()
{
	Super::BeginPlay();

	if (!CSVFilePath.IsEmpty())
	{
		SpawnFromCSV();
	}
	else if (bSpawnTestTrack)
	{
		SpawnTestTrack();
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS ConeSpawner: Spawned %d cones"), TotalSpawned);
}

void AFSDSConeSpawner::SpawnCone(UStaticMesh* Mesh, FVector Location, FRotator Rotation)
{
	if (!Mesh || !GetWorld()) return;

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	AActor* ConeActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform(Rotation, Location), Params);
	if (ConeActor)
	{
		UStaticMeshComponent* MeshComp = NewObject<UStaticMeshComponent>(ConeActor, TEXT("ConeMesh"));
		MeshComp->SetStaticMesh(Mesh);
		MeshComp->SetRelativeScale3D(FVector(ConeScale * 0.5f)); // Scale down cone shape
		ConeActor->SetRootComponent(MeshComp);
		MeshComp->RegisterComponent();
		ConeActor->AddInstanceComponent(MeshComp);
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

	UStaticMesh* BlueMesh = LoadObject<UStaticMesh>(nullptr, *BlueConeAssetPath);
	UStaticMesh* YellowMesh = LoadObject<UStaticMesh>(nullptr, *YellowConeAssetPath);
	UStaticMesh* OrangeBigMesh = LoadObject<UStaticMesh>(nullptr, *OrangeBigConeAssetPath);
	UStaticMesh* OrangeSmallMesh = LoadObject<UStaticMesh>(nullptr, *OrangeSmallConeAssetPath);

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

		UStaticMesh* Mesh = nullptr;
		if (Type == TEXT("blue")) Mesh = BlueMesh;
		else if (Type == TEXT("yellow")) Mesh = YellowMesh;
		else if (Type == TEXT("big_orange")) Mesh = OrangeBigMesh;
		else if (Type == TEXT("small_orange")) Mesh = OrangeSmallMesh;

		if (Mesh)
		{
			SpawnCone(Mesh, FVector(X, Y, HeightOffset), FRotator(0.f, FMath::RandRange(0.f, 360.f), 0.f));
		}
	}
}
