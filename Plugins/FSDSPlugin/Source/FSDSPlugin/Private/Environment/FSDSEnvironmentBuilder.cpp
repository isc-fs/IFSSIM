#include "Environment/FSDSEnvironmentBuilder.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "FSDSCollision.h"
#include "Misc/SecureHash.h"

namespace
{
	// Engine basic shapes are 100 cm across with their pivot at the centre;
	// /Engine/BasicShapes is cooked (Config/DefaultGame.ini).
	const TCHAR* CylinderPath = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
	const TCHAR* CubePath = TEXT("/Engine/BasicShapes/Cube.Cube");
	// The small cone the spawner uses for yellow cones: 228 x 228 x 325 mm.
	const TCHAR* SmallConePath =
		TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_yellow.trafficone_mini_yellow");

	enum class EPartMesh : uint8 { Cylinder, Cube, SmallCone };

	/** One piece of a prop, in the prop's frame: cm, X along its yaw, Z up from
	 *  the ground at its centre. A basic shape is placed by its centre and
	 *  sized in cm; the cone mesh is placed by the centre of its base, unscaled. */
	struct FPart
	{
		EPartMesh Mesh;
		FVector Location;
		FRotator Rotation = FRotator::ZeroRotator;
		FVector SizeCm = FVector(100.0);
	};

	struct FPropClassDef
	{
		const TCHAR* Name;
		TArray<FPart> Parts;
	};

	/** Light barrier: three 3 cm legs from a 0.5 m radius to a head at 1.05 m. */
	TArray<FPart> TripodParts()
	{
		TArray<FPart> Parts;
		const FVector Apex(0.0, 0.0, 105.0);
		for (int32 K = 0; K < 3; ++K)
		{
			const double A = FMath::DegreesToRadians(120.0 * K);
			const FVector Foot(50.0 * FMath::Cos(A), 50.0 * FMath::Sin(A), 0.0);
			const FVector Leg = Apex - Foot;
			Parts.Add({EPartMesh::Cylinder, (Apex + Foot) * 0.5,
				FRotationMatrix::MakeFromZ(Leg.GetSafeNormal()).Rotator(), FVector(3.0, 3.0, Leg.Size())});
		}
		Parts.Add({EPartMesh::Cube, FVector(0.0, 0.0, 110.0), FRotator::ZeroRotator, FVector(12.0, 8.0, 10.0)});
		return Parts;
	}

	/** Four stacks of tyres, 60 cm across and 60 cm tall, side by side along X. */
	TArray<FPart> TyreWallParts()
	{
		TArray<FPart> Parts;
		for (int32 I = 0; I < 4; ++I)
		{
			Parts.Add({EPartMesh::Cylinder, FVector(-90.0 + 60.0 * I, 0.0, 30.0), FRotator::ZeroRotator,
				FVector(60.0, 60.0, 60.0)});
		}
		return Parts;
	}

	/** Four small cones nested, each 6 cm above the one below: about 50 cm tall. */
	TArray<FPart> ConeStackParts()
	{
		TArray<FPart> Parts;
		for (int32 I = 0; I < 4; ++I)
		{
			Parts.Add({EPartMesh::SmallCone, FVector(0.0, 0.0, 6.0 * I)});
		}
		return Parts;
	}

	/** Names and footprints match tools/envgen/catalogue.py. */
	const TArray<FPropClassDef>& Classes()
	{
		static const TArray<FPropClassDef> Defs = {
			{TEXT("bollard"), {{EPartMesh::Cylinder, FVector(0.0, 0.0, 37.5), FRotator::ZeroRotator, FVector(20.0, 20.0, 75.0)}}},
			{TEXT("bin"), {{EPartMesh::Cylinder, FVector(0.0, 0.0, 32.5), FRotator::ZeroRotator, FVector(45.0, 45.0, 65.0)}}},
			{TEXT("tripod"), TripodParts()},
			{TEXT("tyre_wall"), TyreWallParts()},
			{TEXT("cone_stack"), ConeStackParts()},
		};
		return Defs;
	}

	const FPropClassDef* FindClass(const FString& Name)
	{
		for (const FPropClassDef& Def : Classes())
		{
			if (Name == Def.Name) return &Def;
		}
		return nullptr;
	}

	// Ground is searched for this far above and below the floor's plane.
	constexpr double GroundSearchHalfHeightCm = 20000.0;
}

AFSDSEnvironmentActor::AFSDSEnvironmentActor()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

TArray<FString> AFSDSEnvironmentActor::KnownClasses()
{
	TArray<FString> Names;
	for (const FPropClassDef& Def : Classes()) Names.Add(Def.Name);
	return Names;
}

UInstancedStaticMeshComponent* AFSDSEnvironmentActor::PartComponent(UStaticMesh* Mesh)
{
	if (TObjectPtr<UInstancedStaticMeshComponent>* Found = Parts.Find(Mesh))
	{
		return Found->Get();
	}
	UInstancedStaticMeshComponent* C = NewObject<UInstancedStaticMeshComponent>(this);
	// Movable first: a Static component refuses SetStaticMesh at runtime.
	C->SetMobility(EComponentMobility::Movable);
	C->SetStaticMesh(Mesh);
	C->SetCollisionProfileName(FSDSCollision::PropProfile);
	C->SetupAttachment(RootComponent);
	C->RegisterComponent();
	Parts.Add(Mesh, C);
	return C;
}

void AFSDSEnvironmentActor::Clear()
{
	for (auto& Pair : Parts)
	{
		if (Pair.Value) Pair.Value->ClearInstances();
	}
	Built.Reset();
	NumBuilt = 0;
	NumSkipped = 0;
	UnknownClasses.Reset();
	TransformHash.Reset();
}

int32 AFSDSEnvironmentActor::Build(const FFSDSEnvironment& Env)
{
	Clear();
	UWorld* World = GetWorld();
	if (!World) return 0;

	UStaticMesh* Meshes[3] = {
		LoadObject<UStaticMesh>(nullptr, CylinderPath),
		LoadObject<UStaticMesh>(nullptr, CubePath),
		LoadObject<UStaticMesh>(nullptr, SmallConePath),
	};
	for (int32 I = 0; I < 3; ++I)
	{
		if (!Meshes[I])
		{
			UE_LOG(LogTemp, Error, TEXT("FSDS Environment: mesh %d of the prop kit is missing; props using it are skipped"), I);
		}
	}

	FCollisionQueryParams Params(SCENE_QUERY_STAT(FSDSEnvironmentGround), /*bTraceComplex=*/true, this);
	const FCollisionObjectQueryParams GroundOnly(ECC_WorldStatic);
	FString HashInput;

	for (const FFSDSEnvProp& Prop : Env.Props)
	{
		const FPropClassDef* Def = FindClass(Prop.Class);
		if (!Def)
		{
			UnknownClasses.AddUnique(Prop.Class);
			++NumSkipped;
			continue;
		}

		// Stand on the ground, found the way the cones find it: a WorldStatic
		// query, so props never stand on other props.
		const FVector2D Xy = FSDSEnvironment::CsvToUeCm(Prop.X, Prop.Y);
		FHitResult Hit;
		const bool bGround = World->LineTraceSingleByObjectType(Hit,
			FVector(Xy, GroundSearchHalfHeightCm), FVector(Xy, -GroundSearchHalfHeightCm), GroundOnly, Params);
		if (!bGround)
		{
			++NumSkipped;
			continue;
		}
		const double YawDeg = FSDSEnvironment::CsvYawToUeDeg(Prop.YawDeg);
		const FTransform PropXf(FRotator(0.0, YawDeg, 0.0), Hit.ImpactPoint);

		bool bPlaced = false;
		for (const FPart& Part : Def->Parts)
		{
			UStaticMesh* Mesh = Meshes[(int32)Part.Mesh];
			if (!Mesh) continue;
			FTransform Local;
			if (Part.Mesh == EPartMesh::SmallCone)
			{
				// Put the base of the mesh's bounds on Part.Location.
				const FBox B = Mesh->GetBoundingBox();
				const FVector Pivot(-B.GetCenter().X, -B.GetCenter().Y, -B.Min.Z);
				Local = FTransform(Part.Rotation, Part.Location + Part.Rotation.RotateVector(Pivot));
			}
			else
			{
				Local = FTransform(Part.Rotation, Part.Location, Part.SizeCm / 100.0);
			}
			PartComponent(Mesh)->AddInstance(Local * PropXf, /*bWorldSpace=*/true);
			bPlaced = true;
		}
		if (!bPlaced)
		{
			++NumSkipped;
			continue;
		}

		Built.FindOrAdd(Prop.Class)++;
		++NumBuilt;
		const FVector L = PropXf.GetLocation();
		HashInput += FString::Printf(TEXT("%s,%lld,%lld,%lld,%lld;"), *Prop.Class,
			(int64)FMath::RoundToDouble(L.X * 10.0), (int64)FMath::RoundToDouble(L.Y * 10.0),
			(int64)FMath::RoundToDouble(L.Z * 10.0), (int64)FMath::RoundToDouble(YawDeg * 10.0));
	}

	FTCHARToUTF8 Utf8(*HashInput);
	uint8 Digest[FSHA1::DigestSize];
	FSHA1::HashBuffer(Utf8.Get(), Utf8.Length(), Digest);
	TransformHash = BytesToHex(Digest, FSHA1::DigestSize).ToLower();

	Built.KeySort(TLess<FString>());
	UE_LOG(LogTemp, Log, TEXT("FSDS Environment: built %d props (%d skipped%s), transforms sha1 %s"),
		NumBuilt, NumSkipped,
		UnknownClasses.Num() ? *FString::Printf(TEXT("; unknown classes: %s"), *FString::Join(UnknownClasses, TEXT(", "))) : TEXT(""),
		*TransformHash);
	return NumBuilt;
}
