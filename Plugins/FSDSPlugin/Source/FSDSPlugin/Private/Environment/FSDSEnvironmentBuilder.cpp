#include "Environment/FSDSEnvironmentBuilder.h"

#include "Components/StaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "FSDSCollision.h"
#include "FSDSReferee.h"
#include "Misc/SecureHash.h"

namespace
{
	// Engine basic shapes are 100 cm across with their pivot at the centre;
	// /Engine/BasicShapes is cooked (Config/DefaultGame.ini).
	const TCHAR* CylinderPath = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
	const TCHAR* CubePath = TEXT("/Engine/BasicShapes/Cube.Cube");
	const TCHAR* SpherePath = TEXT("/Engine/BasicShapes/Sphere.Sphere");
	// The small cone the spawner uses for yellow cones: 228 x 228 x 325 mm.
	const TCHAR* SmallConePath =
		TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_yellow.trafficone_mini_yellow");

	enum class EPartMesh : uint8 { Cylinder, Cube, SmallCone, Sphere };

	/** One piece of a primitive prop, in the prop's frame: cm, X along its yaw,
	 *  Z up from the ground at its centre. A basic shape is placed by its centre
	 *  and sized in cm; the cone mesh is placed by the centre of its base,
	 *  unscaled. Stencil is the material the LiDAR sees (FSDSPropStencil). */
	struct FPart
	{
		EPartMesh Mesh;
		FVector Location;
		FRotator Rotation = FRotator::ZeroRotator;
		FVector SizeCm = FVector(100.0);
		uint8 Stencil = FSDSConeStencil::None;
	};

	/**
	 * A real (photoreal) mesh for a class, used instead of the primitives when
	 * one of Paths loads. Scanned assets from the marketplace can't be
	 * committed to this public repo, so they live under /Game/ThirdParty/Props
	 * (git-ignored) on the machines that have them; see docs/environment_props.md.
	 *
	 * The mesh is scaled uniformly until its bounds fit FitCm (0 = that axis is
	 * free) and stood on the ground by the bottom of its bounds; Copies of it go
	 * side by side along X, SpacingCm apart, centred on the prop.
	 */
	struct FRealMesh
	{
		TArray<const TCHAR*> Paths;
		FVector FitCm;
		int32 Copies = 1;
		double SpacingCm = 0.0;
		// None = the LiDAR reads the mesh's own colour (its textures).
		uint8 Stencil = FSDSConeStencil::None;
	};

	struct FPropClassDef
	{
		const TCHAR* Name;
		TArray<FPart> Parts;
		FRealMesh Real;
		/** Seen by the LiDAR only (FSDSPropLidarOnly): a phantom to the car.
		 *  For anything under 0.30 m a wheel could reach, e.g. a sidewalk's
		 *  kerb (roadmap rule 2, docs/collision_matrix.md). */
		bool bLidarOnly = false;
	};

	/** Bollard: a 20 cm post, 75 cm tall, with a 10 cm retroreflective band near the top. */
	TArray<FPart> BollardParts()
	{
		return {
			{EPartMesh::Cylinder, FVector(0.0, 0.0, 37.5), FRotator::ZeroRotator, FVector(20.0, 20.0, 75.0),
				FSDSPropStencil::DarkPlastic},
			// A hair wider than the post, so its surface is the one the LiDAR sees.
			{EPartMesh::Cylinder, FVector(0.0, 0.0, 60.0), FRotator::ZeroRotator, FVector(20.4, 20.4, 10.0),
				FSDSPropStencil::Retroreflective},
		};
	}

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
				FRotationMatrix::MakeFromZ(Leg.GetSafeNormal()).Rotator(), FVector(3.0, 3.0, Leg.Size()),
				FSDSPropStencil::Aluminium});
		}
		Parts.Add({EPartMesh::Cube, FVector(0.0, 0.0, 110.0), FRotator::ZeroRotator, FVector(12.0, 8.0, 10.0),
			FSDSPropStencil::DarkPlastic});
		return Parts;
	}

	/** Four stacks of tyres, 60 cm across and 60 cm tall, side by side along X. */
	TArray<FPart> TyreWallParts()
	{
		TArray<FPart> Parts;
		for (int32 I = 0; I < 4; ++I)
		{
			Parts.Add({EPartMesh::Cylinder, FVector(-90.0 + 60.0 * I, 0.0, 30.0), FRotator::ZeroRotator,
				FVector(60.0, 60.0, 60.0), FSDSPropStencil::Rubber});
		}
		return Parts;
	}

	/** Four small cones nested, each 6 cm above the one below: about 50 cm tall. */
	TArray<FPart> ConeStackParts()
	{
		TArray<FPart> Parts;
		for (int32 I = 0; I < 4; ++I)
		{
			Parts.Add({EPartMesh::SmallCone, FVector(0.0, 0.0, 6.0 * I), FRotator::ZeroRotator, FVector(100.0),
				FSDSConeStencil::Yellow});
		}
		return Parts;
	}

	// --- Venue classes (docs/venues.md): what stands around a parking lot. ---

	/** Tree: a 30 cm trunk 2.5 m tall under a 3 m round canopy (top at 5.3 m). */
	TArray<FPart> TreeParts()
	{
		return {
			{EPartMesh::Cylinder, FVector(0.0, 0.0, 125.0), FRotator::ZeroRotator, FVector(30.0, 30.0, 250.0)},
			{EPartMesh::Sphere, FVector(0.0, 0.0, 380.0), FRotator::ZeroRotator, FVector(300.0)},
		};
	}

	/** Street lamp: a 16 cm pole 6 m tall, an arm along X and a head at its end. */
	TArray<FPart> LampPostParts()
	{
		return {
			{EPartMesh::Cylinder, FVector(0.0, 0.0, 300.0), FRotator::ZeroRotator, FVector(16.0, 16.0, 600.0),
				FSDSPropStencil::Aluminium},
			{EPartMesh::Cube, FVector(60.0, 0.0, 592.0), FRotator::ZeroRotator, FVector(120.0, 10.0, 10.0),
				FSDSPropStencil::Aluminium},
			{EPartMesh::Cube, FVector(110.0, 0.0, 580.0), FRotator::ZeroRotator, FVector(45.0, 22.0, 12.0),
				FSDSPropStencil::DarkPlastic},
		};
	}

	/** A parked car facing X: 4.4 x 1.8 m body, a cabin, four wheels; 1.5 m tall. */
	TArray<FPart> ParkedCarParts()
	{
		TArray<FPart> Parts = {
			{EPartMesh::Cube, FVector(0.0, 0.0, 65.0), FRotator::ZeroRotator, FVector(440.0, 180.0, 70.0)},
			{EPartMesh::Cube, FVector(-20.0, 0.0, 125.0), FRotator::ZeroRotator, FVector(240.0, 165.0, 50.0)},
		};
		for (int32 I = 0; I < 4; ++I)
		{
			const double X = I < 2 ? 135.0 : -135.0;
			const double Y = (I % 2) ? 80.0 : -80.0;
			// A cylinder's axis is Z: roll it onto Y for a wheel.
			Parts.Add({EPartMesh::Cylinder, FVector(X, Y, 32.0), FRotator(0.0, 0.0, 90.0), FVector(64.0, 64.0, 22.0),
				FSDSPropStencil::Rubber});
		}
		return Parts;
	}

	/** Bench along X: 1.8 m seat at 45 cm, a back rest, two legs. */
	TArray<FPart> BenchParts()
	{
		return {
			{EPartMesh::Cube, FVector(0.0, 0.0, 45.0), FRotator::ZeroRotator, FVector(180.0, 45.0, 6.0)},
			{EPartMesh::Cube, FVector(0.0, -20.0, 70.0), FRotator::ZeroRotator, FVector(180.0, 5.0, 40.0)},
			{EPartMesh::Cube, FVector(-75.0, 0.0, 21.0), FRotator::ZeroRotator, FVector(6.0, 40.0, 42.0),
				FSDSPropStencil::Aluminium},
			{EPartMesh::Cube, FVector(75.0, 0.0, 21.0), FRotator::ZeroRotator, FVector(6.0, 40.0, 42.0),
				FSDSPropStencil::Aluminium},
		};
	}

	/** Crowd barrier along X: a 2.5 m steel panel 1.1 m tall, top and bottom
	 *  rails and upright bars between two end posts, on two 0.5 m feet. The
	 *  LiDAR sees through the gaps; every bar is solid to the car. */
	TArray<FPart> FenceParts()
	{
		TArray<FPart> Parts;
		for (const double X : {-122.0, 122.0})
		{
			Parts.Add({EPartMesh::Cylinder, FVector(X, 0.0, 57.0), FRotator::ZeroRotator, FVector(4.0, 4.0, 106.0),
				FSDSPropStencil::Aluminium});                                                     // post
			Parts.Add({EPartMesh::Cube, FVector(X, 0.0, 2.0), FRotator::ZeroRotator, FVector(6.0, 50.0, 4.0),
				FSDSPropStencil::Aluminium});                                                     // foot
		}
		Parts.Add({EPartMesh::Cube, FVector(0.0, 0.0, 108.0), FRotator::ZeroRotator, FVector(248.0, 4.0, 4.0),
			FSDSPropStencil::Aluminium});                                                         // top rail
		Parts.Add({EPartMesh::Cube, FVector(0.0, 0.0, 22.0), FRotator::ZeroRotator, FVector(248.0, 4.0, 4.0),
			FSDSPropStencil::Aluminium});                                                         // bottom rail
		for (int32 I = 0; I < 7; ++I)
		{
			Parts.Add({EPartMesh::Cylinder, FVector(-90.0 + 30.0 * I, 0.0, 65.0), FRotator::ZeroRotator,
				FVector(2.0, 2.0, 82.0), FSDSPropStencil::Aluminium});                            // bars
		}
		return Parts;
	}

	/** Earth bank along X, 10 m long: a 5 m square beam rolled 45 deg and sunk,
	 *  so a ridge about 2.9 m tall and 5.9 m wide shows. Its box collision is
	 *  exactly what's drawn (rule 1). A stand-in for a hill until terrain (P3). */
	TArray<FPart> BermParts()
	{
		return {
			{EPartMesh::Cube, FVector(0.0, 0.0, -60.0), FRotator(0.0, 0.0, 45.0), FVector(1000.0, 500.0, 500.0)},
		};
	}

	/** Names and footprints match tools/envgen/catalogue.py. */
	const TArray<FPropClassDef>& Classes()
	{
		static const TArray<FPropClassDef> Defs = {
			{TEXT("bollard"), BollardParts(),
				{{TEXT("/Game/ThirdParty/Props/SM_Bollard.SM_Bollard")}, FVector(20.0, 20.0, 75.0)}},
			{TEXT("bin"),
				{{EPartMesh::Cylinder, FVector(0.0, 0.0, 32.5), FRotator::ZeroRotator, FVector(45.0, 45.0, 65.0),
					FSDSPropStencil::DarkPlastic}},
				{{TEXT("/Game/ThirdParty/Props/SM_Bin.SM_Bin")}, FVector(50.0, 50.0, 65.0)}},
			{TEXT("tripod"), TripodParts(),
				{{TEXT("/Game/ThirdParty/Props/SM_Tripod.SM_Tripod")}, FVector(100.0, 100.0, 115.0)}},
			// The repo's own textured tyre stack, unless a scanned one is installed.
			{TEXT("tyre_wall"), TyreWallParts(),
				{{TEXT("/Game/ThirdParty/Props/SM_TyreStack.SM_TyreStack"),
				  TEXT("/Game/RaceCourse/Model/Environment/tire_stacked.tire_stacked")},
				 FVector(60.0, 60.0, 0.0), 4, 60.0, FSDSPropStencil::Rubber}},
			// Already a real mesh: the sim's own small cone.
			{TEXT("cone_stack"), ConeStackParts(), {}},

			// Venues (docs/venues.md).
			{TEXT("tree"), TreeParts(), {{TEXT("/Game/ThirdParty/Props/SM_Tree.SM_Tree")}, FVector(300.0, 300.0, 530.0)}},
			{TEXT("lamp_post"), LampPostParts(), {{TEXT("/Game/ThirdParty/Props/SM_LampPost.SM_LampPost")}, FVector(0.0, 0.0, 600.0)}},
			{TEXT("parked_car"), ParkedCarParts(), {{TEXT("/Game/ThirdParty/Props/SM_ParkedCar.SM_ParkedCar")}, FVector(440.0, 180.0, 0.0)}},
			{TEXT("bench"), BenchParts(), {{TEXT("/Game/ThirdParty/Props/SM_Bench.SM_Bench")}, FVector(180.0, 50.0, 90.0)}},
			// 2 m x 2 m of sidewalk, 20 cm tall: rows of them make a pavement.
			{TEXT("sidewalk"),
				{{EPartMesh::Cube, FVector(0.0, 0.0, 10.0), FRotator::ZeroRotator, FVector(200.0, 200.0, 20.0)}},
				{}, /*bLidarOnly=*/true},
			{TEXT("fence"), FenceParts(), {}},
			{TEXT("berm"), BermParts(), {}},
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

	/** The first of Paths that loads, without warning about the ones that don't. */
	UStaticMesh* LoadFirst(const TArray<const TCHAR*>& Paths, FString& OutPath)
	{
		for (const TCHAR* Path : Paths)
		{
			if (UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, Path, nullptr, LOAD_NoWarn | LOAD_Quiet))
			{
				OutPath = Path;
				return Mesh;
			}
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

void AFSDSEnvironmentActor::AddPart(UStaticMesh* Mesh, uint8 Stencil, const FTransform& WorldXf, bool bLidarOnly)
{
	// One plain component per part, not instanced: a material needs the
	// "used with instanced static meshes" flag to render instanced, and the
	// repo's tyre and cone materials (and most marketplace ones) don't have
	// it, so instanced props rendered with the engine's grey default material.
	// A track has at most a few hundred parts.
	UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this);
	// Movable first: a Static component refuses SetStaticMesh at runtime.
	C->SetMobility(EComponentMobility::Movable);
	C->SetStaticMesh(Mesh);
	C->SetCollisionProfileName(bLidarOnly ? FSDSCollision::PropLidarOnlyProfile : FSDSCollision::PropProfile);
	C->SetupAttachment(RootComponent);
	C->SetWorldTransform(WorldXf);
	C->RegisterComponent();
	// The LiDAR's reflectance LUT reads this (FSDSPropStencil); untagged
	// parts read their own colour.
	if (Stencil != FSDSConeStencil::None)
	{
		C->SetRenderCustomDepth(true);
		C->SetCustomDepthStencilValue(Stencil);
	}
	Parts.Add(C);
}

void AFSDSEnvironmentActor::Clear()
{
	for (UStaticMeshComponent* C : Parts)
	{
		if (C) C->DestroyComponent();
	}
	Parts.Reset();
	Built.Reset();
	MeshSources.Reset();
	LoggedMeshes.Reset();
	NumBuilt = 0;
	NumSkipped = 0;
	UnknownClasses.Reset();
	TransformHash.Reset();
}

void AFSDSEnvironmentActor::PlaceRealMesh(UStaticMesh* Mesh, const FVector& FitCm, int32 Copies, double SpacingCm,
	uint8 Stencil, const FTransform& PropXf)
{
	const FBox B = Mesh->GetBoundingBox();
	const FVector Size = B.GetSize();
	// Uniform scale: the largest that keeps every constrained axis within FitCm.
	double Scale = TNumericLimits<double>::Max();
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		if (FitCm[Axis] > 0.0 && Size[Axis] > KINDA_SMALL_NUMBER)
		{
			Scale = FMath::Min(Scale, FitCm[Axis] / Size[Axis]);
		}
	}
	if (Scale == TNumericLimits<double>::Max()) Scale = 1.0;
	if (!LoggedMeshes.Contains(Mesh))
	{
		LoggedMeshes.Add(Mesh);
		UE_LOG(LogTemp, Log, TEXT("FSDS Environment: %s bounds %.1f x %.1f x %.1f cm (min z %.1f), scaled %.3f to fit %.0f x %.0f x %.0f"),
			*Mesh->GetPathName(), Size.X, Size.Y, Size.Z, B.Min.Z, Scale, FitCm.X, FitCm.Y, FitCm.Z);
	}
	// Centre the bounds on the prop and stand them on the ground.
	const FVector Pivot = FVector(-B.GetCenter().X, -B.GetCenter().Y, -B.Min.Z) * Scale;
	const int32 N = FMath::Max(Copies, 1);
	for (int32 I = 0; I < N; ++I)
	{
		const FVector Offset((I - (N - 1) * 0.5) * SpacingCm, 0.0, 0.0);
		const FTransform Local(FRotator::ZeroRotator, Offset + Pivot, FVector(Scale));
		AddPart(Mesh, Stencil, Local * PropXf);
	}
}

int32 AFSDSEnvironmentActor::Build(const FFSDSEnvironment& Env)
{
	Clear();
	UWorld* World = GetWorld();
	if (!World) return 0;

	UStaticMesh* Meshes[4] = {
		LoadObject<UStaticMesh>(nullptr, CylinderPath),
		LoadObject<UStaticMesh>(nullptr, CubePath),
		LoadObject<UStaticMesh>(nullptr, SmallConePath),
		LoadObject<UStaticMesh>(nullptr, SpherePath),
	};
	for (int32 I = 0; I < 4; ++I)
	{
		if (!Meshes[I])
		{
			UE_LOG(LogTemp, Error, TEXT("FSDS Environment: mesh %d of the prop kit is missing; props using it are skipped"), I);
		}
	}

	// Which classes have a real mesh installed, resolved once per build.
	TMap<FString, UStaticMesh*> RealMeshes;
	for (const FPropClassDef& Def : Classes())
	{
		FString Path;
		UStaticMesh* Real = Def.Real.Paths.Num() ? LoadFirst(Def.Real.Paths, Path) : nullptr;
		RealMeshes.Add(Def.Name, Real);
		MeshSources.Add(Def.Name, Real ? Path : FString(TEXT("primitives")));
	}
	MeshSources.KeySort(TLess<FString>());

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
		if (UStaticMesh* Real = RealMeshes.FindRef(Def->Name))
		{
			PlaceRealMesh(Real, Def->Real.FitCm, Def->Real.Copies, Def->Real.SpacingCm, Def->Real.Stencil, PropXf);
			bPlaced = true;
		}
		else for (const FPart& Part : Def->Parts)
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
			AddPart(Mesh, Part.Stencil, Local * PropXf, Def->bLidarOnly);
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
