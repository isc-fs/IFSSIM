#include "Test/FSDSCollisionMatrixCheck.h"

#include "FSDSCollision.h"
#include "FSDSVehiclePawn.h"
#include "Vehicles/FSDSWheeledVehicleMovementComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/CollisionProfile.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace
{
	const FName TestPropTag(TEXT("FSDSCollisionTest"));
	constexpr double M2CM = 100.0;

	struct FTestProp
	{
		const TCHAR* Name;
		const TCHAR* Profile;
		ECollisionChannel Type;
		bool bSolid;
		double AheadOffsetM, LeftM;     // AheadM + AheadOffsetM ahead of the car, LeftM to its left
		double SizeX, SizeY, SizeZ;     // m, along / across the car's heading, up

		// Set by Spawn.
		TWeakObjectPtr<AStaticMeshActor> Actor;
		double AheadM = 0.0;
		FVector BaseUe = FVector::ZeroVector;   // footprint centre, on the ground
		FVector FwdUe = FVector::ForwardVector; // the car's heading at spawn
	};

	// Game thread only, like everything that touches it.
	FTestProp GProps[] = {
		{ TEXT("solid"),      FSDSCollision::PropProfile,          FSDSCollision::PropChannel,          true,  2.0, 2.5, 1.0, 1.0, 0.60 },
		{ TEXT("lidar_only"), FSDSCollision::PropLidarOnlyProfile, FSDSCollision::PropLidarOnlyChannel, false, 0.0, 0.0, 0.4, 3.0, 0.15 },
	};

	bool IsTestProp(const AActor* A) { return A && A->Tags.Contains(TestPropTag); }

	/** Ground under XY: the same object-type WorldStatic query the road probe
	 *  and the cone snap make. */
	bool GroundAt(UWorld* World, const FVector2D& XY, const AActor* Ignore, FHitResult& Hit)
	{
		FCollisionQueryParams Params(SCENE_QUERY_STAT(FSDSCollisionMatrixGround), false, Ignore);
		return World->LineTraceSingleByObjectType(Hit,
			FVector(XY.X, XY.Y, 20000.0), FVector(XY.X, XY.Y, -20000.0),
			FCollisionObjectQueryParams(ECC_WorldStatic), Params);
	}

	void AddCheck(TArray<TSharedPtr<FJsonValue>>& Out, bool& bAllOk,
		const FString& Name, bool bOk, const FString& Detail)
	{
		TSharedRef<FJsonObject> C = MakeShared<FJsonObject>();
		C->SetStringField(TEXT("name"), Name);
		C->SetBoolField(TEXT("ok"), bOk);
		C->SetStringField(TEXT("detail"), Detail);
		Out.Add(MakeShared<FJsonValueObject>(C));
		bAllOk &= bOk;
	}

	FString HitName(const FHitResult& Hit, bool bHit)
	{
		if (!bHit) return TEXT("nothing");
		const AActor* A = Hit.GetActor();
		const FString Who = !A ? FString(TEXT("?"))
			: IsTestProp(A) ? FString::Printf(TEXT("the test prop (%s)"), *A->GetName())
			: A->GetName();
		return FString::Printf(TEXT("%s at z=%.3f m"), *Who, Hit.ImpactPoint.Z / M2CM);
	}

	const TCHAR* ResponseName(ECollisionResponse R)
	{
		switch (R)
		{
		case ECR_Ignore:  return TEXT("Ignore");
		case ECR_Overlap: return TEXT("Overlap");
		case ECR_Block:   return TEXT("Block");
		default:          return TEXT("?");
		}
	}

	FString ToJson(const TSharedRef<FJsonObject>& Obj)
	{
		FString Json;
		TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
			TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&Json);
		FJsonSerializer::Serialize(Obj, Writer);
		return Json;
	}
}

int32 FSDSCollisionMatrixCheck::Clear(UWorld* World)
{
	int32 N = 0;
	if (World)
	{
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			if (IsTestProp(*It))
			{
				It->Destroy();
				++N;
			}
		}
	}
	for (FTestProp& P : GProps) P.Actor.Reset();
	return N;
}

FString FSDSCollisionMatrixCheck::Spawn(AFSDSVehiclePawn* Pawn, double AheadM)
{
	UWorld* World = Pawn ? Pawn->GetWorld() : nullptr;
	if (!World) return TEXT("{\"ok\":false,\"error\":\"no vehicle\"}");
	Clear(World);

	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (!Cube) return TEXT("{\"ok\":false,\"error\":\"/Engine/BasicShapes/Cube is not available\"}");

	// The car's heading, flat. UE's frame is left-handed, so "left" of
	// forward F is (F.Y, -F.X).
	const FVector Fwd = Pawn->GetActorForwardVector().GetSafeNormal2D();
	const FVector Left(Fwd.Y, -Fwd.X, 0.0);
	const FVector CarXY(Pawn->GetActorLocation().X, Pawn->GetActorLocation().Y, 0.0);

	for (FTestProp& P : GProps)
	{
		P.AheadM = AheadM + P.AheadOffsetM;
		P.FwdUe = Fwd;
		const FVector XY = CarXY + Fwd * (P.AheadM * M2CM) + Left * (P.LeftM * M2CM);
		FHitResult Ground;
		if (!GroundAt(World, FVector2D(XY.X, XY.Y), Pawn, Ground))
		{
			Clear(World);
			return FString::Printf(TEXT("{\"ok\":false,\"error\":\"no ground under the %s prop; move the car\"}"), P.Name);
		}
		P.BaseUe = Ground.ImpactPoint;

		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		// The engine cube is 1 m, centred on its origin.
		const FVector Centre = P.BaseUe + FVector(0, 0, P.SizeZ * M2CM * 0.5);
		AStaticMeshActor* A = World->SpawnActor<AStaticMeshActor>(
			AStaticMeshActor::StaticClass(), Centre, FRotator(0.0, Fwd.Rotation().Yaw, 0.0), SpawnParams);
		if (!A)
		{
			Clear(World);
			return FString::Printf(TEXT("{\"ok\":false,\"error\":\"could not spawn the %s prop\"}"), P.Name);
		}
		// Movable first: a Static component refuses SetStaticMesh and scaling
		// at runtime (see FSDSTestTerrain.cpp).
		A->SetMobility(EComponentMobility::Movable);
		A->Tags.Add(TestPropTag);
		UStaticMeshComponent* MC = A->GetStaticMeshComponent();
		MC->SetMobility(EComponentMobility::Movable);
		MC->SetStaticMesh(Cube);
		MC->SetWorldScale3D(FVector(P.SizeX, P.SizeY, P.SizeZ));
		MC->SetCollisionProfileName(P.Profile);
		P.Actor = A;
	}
	return FString();
}

FString FSDSCollisionMatrixCheck::Check(AFSDSVehiclePawn* Pawn)
{
	UWorld* World = Pawn ? Pawn->GetWorld() : nullptr;
	if (!World) return TEXT("{\"ok\":false,\"error\":\"no vehicle\"}");
	for (const FTestProp& P : GProps)
	{
		if (!P.Actor.IsValid()) return TEXT("{\"ok\":false,\"error\":\"no test props; spawn them first\"}");
	}

	TArray<TSharedPtr<FJsonValue>> Checks;
	bool bOk = true;

	// The ini's channel names sit in the slots FSDSCollision.h names.
	{
		const FName PropName = UCollisionProfile::Get()->ReturnChannelNameFromContainerIndex(FSDSCollision::PropChannel);
		const FName LidarName = UCollisionProfile::Get()->ReturnChannelNameFromContainerIndex(FSDSCollision::PropLidarOnlyChannel);
		AddCheck(Checks, bOk, TEXT("channels"),
			PropName == FSDSCollision::PropChannelName && LidarName == FSDSCollision::PropLidarOnlyChannelName,
			FString::Printf(TEXT("GameTraceChannel1=%s, GameTraceChannel2=%s"), *PropName.ToString(), *LidarName.ToString()));
	}

	const FCollisionResponseContainer& WheelResponses = Pawn->VehicleMovement->WheelTraceCollisionResponses;
	USkeletalMeshComponent* Chassis = Pawn->GetMesh();
	FCollisionResponseTemplate ConeProfile;
	const bool bHaveConeProfile = UCollisionProfile::Get()->GetProfileTemplate(FName(TEXT("PhysicsActor")), ConeProfile);

	for (const FTestProp& P : GProps)
	{
		AStaticMeshActor* A = P.Actor.Get();
		UStaticMeshComponent* MC = A->GetStaticMeshComponent();
		const ECollisionChannel Type = MC->GetCollisionObjectType();
		AddCheck(Checks, bOk, FString::Printf(TEXT("%s: object type"), P.Name), Type == P.Type,
			UCollisionProfile::Get()->ReturnChannelNameFromContainerIndex(Type).ToString());

		const FVector Top = P.BaseUe + FVector(0, 0, P.SizeZ * M2CM + 100.0);
		const FVector Below = P.BaseUe - FVector(0, 0, 100.0);

		// Control: an unfiltered WorldDynamic trace straight down must hit the
		// solid prop. If it does not, the prop is not in the scene queries yet
		// and every "ignores it" result below means nothing.
		if (P.bSolid)
		{
			FCollisionQueryParams Params(SCENE_QUERY_STAT(FSDSCollisionMatrixControl), false, Pawn);
			FHitResult Hit;
			const bool bHit = World->LineTraceSingleByChannel(Hit, Top, Below, ECC_WorldDynamic, Params);
			AddCheck(Checks, bOk, TEXT("solid: in the scene queries (control)"),
				bHit && Hit.GetActor() == A, TEXT("unfiltered trace hit ") + HitName(Hit, bHit));
		}

		// Chaos wheel trace: the WorldDynamic channel with the car's own wheel
		// responses, straight down through the prop.
		{
			FCollisionQueryParams Params(SCENE_QUERY_STAT(FSDSCollisionMatrixWheel), false, Pawn);
			FCollisionResponseParams Responses;
			Responses.CollisionResponse = WheelResponses;
			FHitResult Hit;
			const bool bHit = World->LineTraceSingleByChannel(Hit, Top, Below, ECC_WorldDynamic, Params, Responses);
			AddCheck(Checks, bOk, FString::Printf(TEXT("%s: wheel trace ignores it"), P.Name),
				bHit && !IsTestProp(Hit.GetActor()), TEXT("hit ") + HitName(Hit, bHit));
		}

		// Ground query (road probe, cone snap): object-type WorldStatic.
		{
			FHitResult Hit;
			const bool bHit = GroundAt(World, FVector2D(P.BaseUe.X, P.BaseUe.Y), Pawn, Hit);
			AddCheck(Checks, bOk, FString::Printf(TEXT("%s: ground query ignores it"), P.Name),
				bHit && !IsTestProp(Hit.GetActor()), TEXT("hit ") + HitName(Hit, bHit));
		}

		// CPU LiDAR: Visibility, simple collision, default responses, aimed at
		// the prop from 2 m in front of it. The ray starts as high above the
		// ground there as it ends above the ground at the prop, so on a slope
		// it follows the surface instead of running into it.
		{
			FCollisionQueryParams Params(SCENE_QUERY_STAT(FSDSCollisionMatrixLidar), false, Pawn);
			Params.bReturnPhysicalMaterial = false;
			const double HalfHeight = P.SizeZ * M2CM * 0.5;
			const FVector Mid = P.BaseUe + FVector(0, 0, HalfHeight);
			FVector From = Mid - P.FwdUe * (P.SizeX * M2CM * 0.5 + 200.0);
			FHitResult FromGround;
			if (GroundAt(World, FVector2D(From.X, From.Y), Pawn, FromGround))
			{
				From.Z = FromGround.ImpactPoint.Z + HalfHeight;
			}
			FHitResult Hit;
			const bool bHit = World->LineTraceSingleByChannel(Hit, From, Mid, ECC_Visibility, Params);
			AddCheck(Checks, bOk, FString::Printf(TEXT("%s: CPU LiDAR sees it"), P.Name),
				bHit && Hit.GetActor() == A, TEXT("hit ") + HitName(Hit, bHit));
		}

		// Physics against the car and the cones. A contact needs both sides to
		// Block each other, and a query-only body has no contacts at all.
		{
			const bool bPhysics = MC->GetCollisionEnabled() == ECollisionEnabled::QueryAndPhysics;
			const ECollisionResponse PropToCar = MC->GetCollisionResponseToChannel(ECC_Vehicle);
			const ECollisionResponse CarToProp = Chassis ? Chassis->GetCollisionResponseToChannel(Type) : ECR_Ignore;
			const bool bCarBlocked = bPhysics && PropToCar == ECR_Block && CarToProp == ECR_Block;
			AddCheck(Checks, bOk, FString::Printf(TEXT("%s: %s the car"), P.Name, P.bSolid ? TEXT("stops") : TEXT("lets through")),
				bCarBlocked == P.bSolid,
				FString::Printf(TEXT("physics %s, prop->Vehicle %s, car(%s)->prop %s"),
					bPhysics ? TEXT("on") : TEXT("off"), ResponseName(PropToCar),
					Chassis ? *Chassis->GetCollisionProfileName().ToString() : TEXT("-"), ResponseName(CarToProp)));

			const ECollisionResponse PropToCone = MC->GetCollisionResponseToChannel(ECC_PhysicsBody);
			const ECollisionResponse ConeToProp = bHaveConeProfile ? ConeProfile.ResponseToChannels.GetResponse(Type) : ECR_Ignore;
			const bool bConeBlocked = bPhysics && PropToCone == ECR_Block && ConeToProp == ECR_Block;
			AddCheck(Checks, bOk, FString::Printf(TEXT("%s: %s cones"), P.Name, P.bSolid ? TEXT("stops") : TEXT("lets through")),
				bConeBlocked == P.bSolid,
				FString::Printf(TEXT("prop->PhysicsBody %s, PhysicsActor->prop %s"),
					ResponseName(PropToCone), ResponseName(ConeToProp)));
		}
	}

	TSharedRef<FJsonObject> Out = MakeShared<FJsonObject>();
	Out->SetBoolField(TEXT("ok"), bOk);
	TArray<TSharedPtr<FJsonValue>> PropsJson;
	for (const FTestProp& P : GProps)
	{
		TSharedRef<FJsonObject> J = MakeShared<FJsonObject>();
		J->SetStringField(TEXT("name"), P.Name);
		J->SetNumberField(TEXT("ahead_m"), P.AheadM);
		J->SetNumberField(TEXT("left_m"), P.LeftM);
		TArray<TSharedPtr<FJsonValue>> Size = {
			MakeShared<FJsonValueNumber>(P.SizeX), MakeShared<FJsonValueNumber>(P.SizeY), MakeShared<FJsonValueNumber>(P.SizeZ) };
		J->SetArrayField(TEXT("size_m"), Size);
		J->SetNumberField(TEXT("ground_z_m"), P.BaseUe.Z / M2CM);
		PropsJson.Add(MakeShared<FJsonValueObject>(J));
	}
	Out->SetArrayField(TEXT("props"), PropsJson);
	Out->SetArrayField(TEXT("checks"), Checks);
	return ToJson(Out);
}
