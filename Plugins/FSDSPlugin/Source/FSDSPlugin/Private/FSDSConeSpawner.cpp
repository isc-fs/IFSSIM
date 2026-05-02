#include "FSDSConeSpawner.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/Blueprint.h"
#include "Engine/World.h"
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

	// Ground-snap the spawn point. Cones are spawned with physics enabled
	// downstream by the Referee, so any mesh that pokes into the floor at
	// spawn time gets ejected by the Chaos solver — and on a floor whose
	// collision is QueryOnly (vs QueryAndPhysics) the eject has nothing to
	// resist it and the cone phases through. We can't fix the floor's
	// collision profile from C++ (editor-side asset fix), but we can stop
	// spawning cones below it.
	//
	// Line-trace down from a generous height to find the floor under (X, Y),
	// then place the cone's mesh-base on the surface plus a small clearance.
	// Falls back to the configured HeightOffset if no floor is detected
	// (level edge / hole) — same behaviour as before for that case.
	FVector AdjustedLocation = Location;
	{
		const FVector TraceStart(Location.X, Location.Y, Location.Z + 1000.f);
		const FVector TraceEnd  (Location.X, Location.Y, Location.Z - 1000.f);
		FCollisionQueryParams QueryParams;
		QueryParams.bTraceComplex = true;
		QueryParams.AddIgnoredActor(this);
		FHitResult Hit;
		if (GetWorld()->LineTraceSingleByChannel(Hit, TraceStart, TraceEnd, ECC_WorldStatic, QueryParams))
		{
			// Mesh local-space min-Z, scaled, gives the offset from the
			// cone's pivot to the lowest point of its mesh. We want the
			// lowest point at Hit.ImpactPoint.Z + HeightOffset (small
			// clearance to absorb sub-cm penetration jitter when physics
			// turns on).
			const FBoxSphereBounds MeshBounds = Mesh->GetBounds();
			const float ScaledLocalMinZ = (MeshBounds.Origin.Z - MeshBounds.BoxExtent.Z) * ConeScale;
			AdjustedLocation.Z = Hit.ImpactPoint.Z - ScaledLocalMinZ + HeightOffset;
			GroundSnapHits++;
		}
		else
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FSDS ConeSpawner: ground line-trace missed at (%.1f, %.1f) — "
				     "spawning at HeightOffset=%.1f. Cone may fall through if floor "
				     "is below the trace start; check level layout."),
				Location.X, Location.Y, HeightOffset);
			GroundSnapMisses++;
		}
	}

	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	AStaticMeshActor* ConeActor = GetWorld()->SpawnActor<AStaticMeshActor>(
		AStaticMeshActor::StaticClass(),
		FTransform(Rotation, AdjustedLocation),
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
	GroundSnapHits = 0;
	GroundSnapMisses = 0;

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
	UE_LOG(LogTemp, Log,
		TEXT("FSDS ConeSpawner: ground-snap %d hit / %d missed across %d cones"),
		GroundSnapHits, GroundSnapMisses, GroundSnapHits + GroundSnapMisses);
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

	// Reset the position caches that ComputeStartGatePose() reads from.
	// Doing it here (not in ReloadTrack) means a fresh BeginPlay-time
	// spawn also gets a clean slate.
	BigOrangePositions.Reset();
	BlueYellowPositions.Reset();
	GroundSnapHits = 0;
	GroundSnapMisses = 0;

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

		// Record positions for the start-gate-pose derivation. We capture
		// the *post-flip* UE world-space coords so ComputeStartGatePose
		// returns values directly usable by SetActorLocationAndRotation.
		if (Color == EFSDSConeColor::OrangeLarge)
		{
			BigOrangePositions.Add(Location);
		}
		else if (Color == EFSDSConeColor::Blue || Color == EFSDSConeColor::Yellow)
		{
			BlueYellowPositions.Add(Location);
		}
	}

	UE_LOG(LogTemp, Log,
		TEXT("FSDS ConeSpawner: ground-snap %d hit / %d missed across %d cones"),
		GroundSnapHits, GroundSnapMisses, GroundSnapHits + GroundSnapMisses);
}

bool AFSDSConeSpawner::ComputeStartGatePose(FVector& OutLocation, FQuat& OutRotation, float BackupCm) const
{
	// Need a full 4-cone gate for PCA. Track cones used only for sign
	// disambiguation (which way "forward" points along the gate axis).
	if (BigOrangePositions.Num() < 4 || BlueYellowPositions.Num() < 1)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS ConeSpawner: ComputeStartGatePose — not enough cones "
				 "(big_orange=%d, need 4; blue+yellow=%d, need ≥1)"),
			BigOrangePositions.Num(), BlueYellowPositions.Num());
		return false;
	}

	// Orange centroid = the gate anchor (vehicle spawns BackupCm
	// behind this point, along the inferred forward axis).
	FVector OrangeCentroid = FVector::ZeroVector;
	for (const FVector& P : BigOrangePositions) OrangeCentroid += P;
	OrangeCentroid /= BigOrangePositions.Num();

	// PCA on the 4 orange cones to recover the gate axes.
	//
	// FSG start gates are wider than deep (4.4 m × 2.6 m for the standard
	// layout): the gate spans the *cross-track* axis (cones across the
	// track width) more than the *along-track* axis (the small offset
	// between front and back gate pairs). So the eigenvector with the
	// LARGER variance is cross-track, and the eigenvector with the
	// SMALLER variance is along-track — that's our "forward" axis.
	//
	// Why PCA over centroid-of-track-cones (the v1/v2 approaches): on
	// closed-loop tracks (autocross/trackdrive) the loop centroid sits
	// off to one side of the gate, and even k-nearest-cones picks up
	// both the entry-side and the loop-return-side cones. The orange
	// gate's own geometry is the only stable axis reference.
	float Cxx = 0.f, Cyy = 0.f, Cxy = 0.f;
	for (const FVector& P : BigOrangePositions)
	{
		const float dx = P.X - OrangeCentroid.X;
		const float dy = P.Y - OrangeCentroid.Y;
		Cxx += dx * dx;
		Cyy += dy * dy;
		Cxy += dx * dy;
	}

	// Eigendecomposition of the 2×2 symmetric covariance matrix:
	//   [[Cxx, Cxy], [Cxy, Cyy]]
	// Eigenvalues: λ = (Cxx + Cyy)/2 ± √((Cxx + Cyy)²/4 − (Cxx·Cyy − Cxy²))
	const float HalfTrace = 0.5f * (Cxx + Cyy);
	const float Det = Cxx * Cyy - Cxy * Cxy;
	const float Disc = FMath::Max(0.f, HalfTrace * HalfTrace - Det);
	const float SqrtDisc = FMath::Sqrt(Disc);
	const float LambdaSmall = HalfTrace - SqrtDisc;

	// Eigenvector for the smaller eigenvalue. For a symmetric 2×2
	// matrix M with eigenvalue λ, an eigenvector is (M_01, λ − M_00) =
	// (Cxy, λ − Cxx). Falls back to a coordinate axis when Cxy ≈ 0
	// (perfectly axis-aligned gate, like a generated track in canonical
	// pose). In the axis-aligned case the smaller variance axis is
	// trivially X if Cxx < Cyy, else Y.
	FVector Forward = FVector::ZeroVector;
	if (FMath::Abs(Cxy) > 1e-3f)
	{
		Forward.X = Cxy;
		Forward.Y = LambdaSmall - Cxx;
	}
	else
	{
		Forward = (Cxx <= Cyy) ? FVector(1.f, 0.f, 0.f) : FVector(0.f, 1.f, 0.f);
	}
	Forward.Z = 0.f;
	if (Forward.IsNearlyZero()) return false;
	Forward.Normalize();

	// Sign disambiguation: PCA gives the axis but not the direction.
	// Pick the half-line that points TOWARD the bulk of nearby track
	// cones. We use the K=4 nearest blue/yellow cones to the orange
	// centroid because:
	//   - K small enough that only the immediate gate-entry cones
	//     dominate (loop-return cones are ≥ a lap-length away)
	//   - K large enough to be robust to a single misclassified cone
	constexpr int32 K_DIRECTION = 4;
	struct FConeDist { double DSq; FVector Pos; };
	TArray<FConeDist> Sorted;
	Sorted.Reserve(BlueYellowPositions.Num());
	for (const FVector& P : BlueYellowPositions)
	{
		Sorted.Add({FVector::DistSquaredXY(P, OrangeCentroid), P});
	}
	Sorted.Sort([](const FConeDist& A, const FConeDist& B) { return A.DSq < B.DSq; });

	const int32 N = FMath::Min(K_DIRECTION, Sorted.Num());
	FVector NearbyCentroid = FVector::ZeroVector;
	for (int32 i = 0; i < N; i++) NearbyCentroid += Sorted[i].Pos;
	NearbyCentroid /= N;

	const FVector ToNearby = NearbyCentroid - OrangeCentroid;
	if (FVector::DotProduct(ToNearby, Forward) < 0.f) Forward = -Forward;

	// Back up from the gate along -Forward, lifted slightly above
	// ground so the wheels settle without clipping into terrain.
	OutLocation = OrangeCentroid - Forward * BackupCm;
	OutLocation.Z = HeightOffset + 50.f; // 50 cm above cone base height

	const float YawDeg = FMath::RadiansToDegrees(FMath::Atan2(Forward.Y, Forward.X));
	OutRotation = FRotator(0.f, YawDeg, 0.f).Quaternion();

	UE_LOG(LogTemp, Log,
		TEXT("FSDS ConeSpawner: PCA start-gate pose — "
			 "orange centroid (%.1f, %.1f), Cxx=%.1f Cyy=%.1f Cxy=%.1f, "
			 "forward (%.2f, %.2f) [sign-checked vs %d nearest], "
			 "spawn (%.1f, %.1f), yaw %.1f°"),
		OrangeCentroid.X, OrangeCentroid.Y,
		Cxx, Cyy, Cxy,
		Forward.X, Forward.Y, N,
		OutLocation.X, OutLocation.Y, YawDeg);

	return true;
}
