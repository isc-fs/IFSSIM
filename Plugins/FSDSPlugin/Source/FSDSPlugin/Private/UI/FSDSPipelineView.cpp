#include "UI/FSDSPipelineView.h"

#include "FSDSCoordinates.h"
#include "FSDSVehiclePawn.h"

#include "Components/LineBatchComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "Misc/ConfigCacheIni.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace PipelineViewUi
{
	static const TCHAR* Section = TEXT("FSDSPipelineView");

	// The car moving off locks the anchor; the car standing still frees it
	// (with SLAM's map empty: a new session).
	constexpr double MovingMps = 0.5;
	constexpr double StillMps = 0.1;
	constexpr double StaleS = 2.0;

	// Heights above the ground (cm), so lines don't flicker in the asphalt.
	constexpr double PathZ = 6.0;
	constexpr double ChainZ = 4.0;
	constexpr double MapZ = 3.0;
	constexpr double DetZ = 8.0;

	const FLinearColor MapColor(0.95f, 0.95f, 0.95f, 1.f);
	const FLinearColor DetColor(1.f, 0.f, 1.f, 1.f);              // as the pipeline's /Conos_raw markers
	const FLinearColor PathColor(0.15f, 0.95f, 0.35f, 1.f);
	const FLinearColor LeftColor(0.2f, 0.45f, 1.f, 1.f);
	const FLinearColor RightColor(1.f, 0.85f, 0.1f, 1.f);
	const FLinearColor PoseColor(1.f, 0.35f, 0.9f, 1.f);

	void Points(const TSharedPtr<FJsonObject>& O, const TCHAR* Field, TArray<FVector2D>& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
		if (!O->TryGetArrayField(Field, Arr)) return;
		Out.Reserve(Arr->Num());
		for (const TSharedPtr<FJsonValue>& V : *Arr)
		{
			const TArray<TSharedPtr<FJsonValue>>* XY = nullptr;
			if (V.IsValid() && V->TryGetArray(XY) && XY->Num() >= 2)
			{
				Out.Add(FVector2D((*XY)[0]->AsNumber(), (*XY)[1]->AsNumber()));
			}
		}
	}

	FVector2D Rotate(const FVector2D& P, double Yaw)
	{
		const double C = FMath::Cos(Yaw), S = FMath::Sin(Yaw);
		return FVector2D(C * P.X - S * P.Y, S * P.X + C * P.Y);
	}

	double EnuYaw(const FQuat& UE)
	{
		const FQuat Q = FSDSCoord::UEQuatToENU(UE);
		return FMath::Atan2(2.0 * (Q.W * Q.Z + Q.X * Q.Y), 1.0 - 2.0 * (Q.Y * Q.Y + Q.Z * Q.Z));
	}
}

FFSDSPipelineView& FFSDSPipelineView::Get()
{
	static FFSDSPipelineView Instance;
	return Instance;
}

void FFSDSPipelineView::Init(UWorld* InWorld, AFSDSVehiclePawn* InPawn)
{
	Shutdown();
	World = InWorld;
	Pawn = InPawn;
	bAnchored = bAnchorLocked = false;
	TickHandle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateRaw(this, &FFSDSPipelineView::Tick));
}

void FFSDSPipelineView::Shutdown()
{
	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}
	if (AActor* A = Holder.Get())
	{
		A->Destroy();
	}
	Holder.Reset();
	Lines.Reset();
	World.Reset();
	Pawn.Reset();
	bDrawn = false;
}

void FFSDSPipelineView::Receive(const FString& JsonLine)
{
	TSharedPtr<FJsonObject> O;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonLine);
	if (!FJsonSerializer::Deserialize(Reader, O) || !O.IsValid()) return;

	// A replay's recorded pose comes on lines of its own, 30 a second.
	const TArray<TSharedPtr<FJsonValue>>* Truth = nullptr;
	if (O->TryGetArrayField(TEXT("truth"), Truth) && Truth->Num() >= 4)
	{
		FScopeLock L(&Lock);
		TruthPrev = TruthLast;
		TruthLast.At = FPlatformTime::Seconds();
		TruthLast.Enu = FVector((*Truth)[0]->AsNumber(), (*Truth)[1]->AsNumber(), (*Truth)[2]->AsNumber());
		TruthLast.Yaw = (*Truth)[3]->AsNumber();
		++TruthCount;
		if (!O->HasField(TEXT("map"))) return;
	}

	FSnapshot S;
	const TArray<TSharedPtr<FJsonValue>>* Pose = nullptr;
	if (O->TryGetArrayField(TEXT("pose"), Pose) && Pose->Num() >= 3)
	{
		S.bHasPose = true;
		S.Pose = FVector2D((*Pose)[0]->AsNumber(), (*Pose)[1]->AsNumber());
		S.Yaw = (*Pose)[2]->AsNumber();
	}
	PipelineViewUi::Points(O, TEXT("map"), S.Map);
	PipelineViewUi::Points(O, TEXT("det"), S.Det);
	PipelineViewUi::Points(O, TEXT("path"), S.Path);
	PipelineViewUi::Points(O, TEXT("left"), S.Left);
	PipelineViewUi::Points(O, TEXT("right"), S.Right);

	FScopeLock L(&Lock);
	Pending = MoveTemp(S);
	bPendingNew = true;
	LastReceived = FPlatformTime::Seconds();
}

bool FFSDSPipelineView::IsShown() const
{
	if (ShownCache < 0)
	{
		bool bOn = true;
		if (GConfig) GConfig->GetBool(PipelineViewUi::Section, TEXT("Shown"), bOn, GGameUserSettingsIni);
		ShownCache = bOn ? 1 : 0;
	}
	return ShownCache == 1;
}

void FFSDSPipelineView::SetShown(bool bOn)
{
	ShownCache = bOn ? 1 : 0;
	if (!bOn) Clear();
	else
	{
		FScopeLock L(&Lock);
		bPendingNew = true;   // redraw the last one now
	}
	if (!GConfig) return;
	GConfig->SetBool(PipelineViewUi::Section, TEXT("Shown"), bOn, GGameUserSettingsIni);
	GConfig->Flush(false, GGameUserSettingsIni);
}

bool FFSDSPipelineView::IsReceiving() const
{
	FScopeLock L(&Lock);
	return LastReceived > 0.0 && FPlatformTime::Seconds() - LastReceived < PipelineViewUi::StaleS;
}

bool FFSDSPipelineView::Tick(float Dt)
{
	if (bReplay) ApplyReplayPose();   // every frame, not only on a new snapshot

	FSnapshot S;
	{
		FScopeLock L(&Lock);
		const bool bStale = LastReceived <= 0.0 || FPlatformTime::Seconds() - LastReceived >= PipelineViewUi::StaleS;
		if (bStale)
		{
			// The relay stopped (the stack is down, or the session ended).
			bPendingNew = false;
			if (bDrawn) Clear();
			bAnchored = bAnchorLocked = false;
			return true;
		}
		if (!bPendingNew) return true;
		bPendingNew = false;
		S = Pending;
	}

	const AFSDSVehiclePawn* P = Pawn.Get();
	if (!P) return true;
	const AFSDSVehiclePawn::FCarState Car = P->GetCarState();

	// The car's speed: its own, or from its motion (a replay moves it with physics off).
	double Speed = FMath::Abs(Car.Speed);
	{
		const FVector Enu3 = FSDSCoord::UEToENU(Car.Position);
		const FVector2D Enu(Enu3.X, Enu3.Y);
		const double Now = FPlatformTime::Seconds();
		if (LastCarAt > 0.0 && Now - LastCarAt > 1e-3)
		{
			Speed = FMath::Max(Speed, FVector2D::Distance(Enu, LastCarEnu) / (Now - LastCarAt));
		}
		LastCarEnu = Enu;
		LastCarAt = Now;
	}

	// The anchor, while the car stands at the start (see the header).
	if (S.bHasPose)
	{
		if (bAnchorLocked && S.Map.Num() == 0 && Speed < PipelineViewUi::StillMps)
		{
			bAnchorLocked = false;   // a new session
		}
		if (!bAnchorLocked)
		{
			const FVector CarEnu = FSDSCoord::UEToENU(Car.Position);
			const double CarYaw = PipelineViewUi::EnuYaw(Car.Orientation);
			AnchorYaw = CarYaw - S.Yaw;
			AnchorEnu = FVector2D(CarEnu.X, CarEnu.Y) - PipelineViewUi::Rotate(S.Pose, AnchorYaw);
			bAnchored = true;
			if (Speed > PipelineViewUi::MovingMps) bAnchorLocked = true;
		}
	}

	// Only while the pipeline drives: the relay keeps sending its last state
	// after a run, which would linger over the next manual drive.
	if (IsShown() && (P->IsApiControlEnabled() || bReplay)) Draw(S);
	else if (bDrawn) Clear();
	return true;
}

FVector FFSDSPipelineView::MapToWorld(const FVector2D& P) const
{
	const FVector2D Enu = AnchorEnu + PipelineViewUi::Rotate(P, AnchorYaw);
	FVector UE = FSDSCoord::ENUToUE(FVector(Enu.X, Enu.Y, 0.0));
	UE.Z = GroundZ;
	return UE;
}

void FFSDSPipelineView::Draw(const FSnapshot& S)
{
	UWorld* W = World.Get();
	const AFSDSVehiclePawn* P = Pawn.Get();
	if (!W || !P) return;

	if (!Lines.IsValid())
	{
		FActorSpawnParameters Params;
		Params.ObjectFlags |= RF_Transient;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		AActor* A = W->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
		if (!A) return;
		ULineBatchComponent* L = NewObject<ULineBatchComponent>(A, TEXT("PipelineViewLines"));
		L->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		L->SetGenerateOverlapEvents(false);
		L->CastShadow = false;
		L->bHiddenInSceneCapture = true;   // the sim's cameras don't see it
		L->bCalculateAccurateBounds = false;
		A->SetRootComponent(L);
		L->RegisterComponent();
		Holder = A;
		Lines = L;
	}
	ULineBatchComponent* L = Lines.Get();
	L->Flush();

	const AFSDSVehiclePawn::FCarState Car = P->GetCarState();

	// The ground under the car: tracks are flat, so one height for all.
	{
		FHitResult Hit;
		FCollisionQueryParams Q(SCENE_QUERY_STAT(FSDSPipelineViewGround), false, P);
		const FVector From = Car.Position + FVector(0, 0, 50);
		if (W->LineTraceSingleByChannel(Hit, From, From - FVector(0, 0, 400), ECC_Visibility, Q))
		{
			GroundZ = Hit.ImpactPoint.Z;
		}
		else
		{
			GroundZ = Car.Position.Z - 30.0;
		}
	}

	TArray<FBatchedLine> Out;
	Out.Reserve(16 * S.Map.Num() + 8 * S.Det.Num() + S.Path.Num() + S.Left.Num() + S.Right.Num() + 8);
	auto Line = [&Out](const FVector& A, const FVector& B, const FLinearColor& C, float Thick)
	{
		Out.Add(FBatchedLine(A, B, C, 0.f, Thick, SDPG_World));
	};
	auto Ring = [&Line](const FVector& Centre, double RadiusCm, const FLinearColor& C, float Thick, int32 Sides)
	{
		for (int32 i = 0; i < Sides; ++i)
		{
			const double A0 = 2.0 * PI * i / Sides, A1 = 2.0 * PI * (i + 1) / Sides;
			Line(Centre + FVector(FMath::Cos(A0), FMath::Sin(A0), 0) * RadiusCm,
				Centre + FVector(FMath::Cos(A1), FMath::Sin(A1), 0) * RadiusCm, C, Thick);
		}
	};
	auto Strip = [&Line, this](const TArray<FVector2D>& Pts, double Z, const FLinearColor& C, float Thick)
	{
		for (int32 i = 0; i + 1 < Pts.Num(); ++i)
		{
			Line(MapToWorld(Pts[i]) + FVector(0, 0, Z), MapToWorld(Pts[i + 1]) + FVector(0, 0, Z), C, Thick);
		}
	};

	if (bAnchored)
	{
		// The SLAM map: a ring and a post where the pipeline thinks each cone is.
		for (const FVector2D& M : S.Map)
		{
			const FVector At = MapToWorld(M) + FVector(0, 0, PipelineViewUi::MapZ);
			Ring(At, 30.0, PipelineViewUi::MapColor, 2.5f, 10);
			Line(At, At + FVector(0, 0, 45.0), PipelineViewUi::MapColor, 2.5f);
		}
		Strip(S.Left, PipelineViewUi::ChainZ, PipelineViewUi::LeftColor, 3.f);
		Strip(S.Right, PipelineViewUi::ChainZ, PipelineViewUi::RightColor, 3.f);
		Strip(S.Path, PipelineViewUi::PathZ, PipelineViewUi::PathColor, 6.f);

		// Where SLAM puts the car: an arrow from its estimated position.
		if (S.bHasPose)
		{
			const FVector From = MapToWorld(S.Pose) + FVector(0, 0, 20.0);
			const FVector2D Ahead = S.Pose + FVector2D(FMath::Cos(S.Yaw), FMath::Sin(S.Yaw)) * 2.0;
			const FVector To = MapToWorld(Ahead) + FVector(0, 0, 20.0);
			Line(From, To, PipelineViewUi::PoseColor, 5.f);
			const FVector Back = (From - To).GetSafeNormal() * 50.0;
			const FVector Side = FVector::CrossProduct(Back, FVector::UpVector).GetSafeNormal() * 30.0;
			Line(To, To + Back + Side, PipelineViewUi::PoseColor, 5.f);
			Line(To, To + Back - Side, PipelineViewUi::PoseColor, 5.f);
		}
	}

	// This scan's detections, in the car's frame (x forward, y left): on the car's true pose.
	{
		const FVector CarEnu = FSDSCoord::UEToENU(Car.Position);
		const double CarYaw = PipelineViewUi::EnuYaw(Car.Orientation);
		for (const FVector2D& D : S.Det)
		{
			const FVector2D Enu = FVector2D(CarEnu.X, CarEnu.Y) + PipelineViewUi::Rotate(D, CarYaw);
			FVector At = FSDSCoord::ENUToUE(FVector(Enu.X, Enu.Y, 0.0));
			At.Z = GroundZ + PipelineViewUi::DetZ;
			Ring(At, 18.0, PipelineViewUi::DetColor, 3.f, 8);
			Line(At + FVector(-18, -18, 0), At + FVector(18, 18, 0), PipelineViewUi::DetColor, 3.f);
			Line(At + FVector(-18, 18, 0), At + FVector(18, -18, 0), PipelineViewUi::DetColor, 3.f);
		}
	}

	if (Out.Num() > 0) L->DrawLines(Out);
	bDrawn = true;
}

void FFSDSPipelineView::BeginReplay()
{
	if (bReplay) return;
	{
		FScopeLock L(&Lock);
		TruthCount = 0;
	}
	bAnchored = bAnchorLocked = false;
	if (AFSDSVehiclePawn* P = Pawn.Get())
	{
		if (USkeletalMeshComponent* Mesh = P->GetMesh())
		{
			bPhysicsWas = Mesh->IsSimulatingPhysics();
			Mesh->SetSimulatePhysics(false);   // moved by hand along the recording
		}
	}
	bReplay = true;
}

void FFSDSPipelineView::EndReplay()
{
	if (!bReplay) return;
	bReplay = false;
	if (AFSDSVehiclePawn* P = Pawn.Get())
	{
		if (USkeletalMeshComponent* Mesh = P->GetMesh())
		{
			Mesh->SetSimulatePhysics(bPhysicsWas);
			if (bPhysicsWas)
			{
				Mesh->SetPhysicsLinearVelocity(FVector::ZeroVector);
				Mesh->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
			}
		}
	}
	Clear();
}

bool FFSDSPipelineView::HasReplayPose() const
{
	FScopeLock L(&Lock);
	return TruthCount > 0 && FPlatformTime::Seconds() - TruthLast.At < 2.0;
}

void FFSDSPipelineView::ApplyReplayPose()
{
	FTruth A, B;
	int32 N = 0;
	{
		FScopeLock L(&Lock);
		A = TruthPrev;
		B = TruthLast;
		N = TruthCount;
	}
	AFSDSVehiclePawn* P = Pawn.Get();
	if (!P || N == 0) return;

	// Between the last two poses, over the time the last one took to come:
	// at most one sample (a thirtieth of a second) behind, and smooth.
	FVector Enu = B.Enu;
	double Yaw = B.Yaw;
	if (N >= 2 && B.At > A.At)
	{
		const double Alpha = FMath::Clamp((FPlatformTime::Seconds() - B.At) / (B.At - A.At), 0.0, 1.0);
		Enu = FMath::Lerp(A.Enu, B.Enu, Alpha);
		Yaw = A.Yaw + FMath::FindDeltaAngleRadians(A.Yaw, B.Yaw) * Alpha;
	}
	// ENU heading (0 = east, counter-clockwise) to UE yaw (0 = north, clockwise).
	const FRotator Rot(0.0, 90.0 - FMath::RadiansToDegrees(Yaw), 0.0);
	P->SetActorLocationAndRotation(FSDSCoord::ENUToUE(Enu), Rot, false, nullptr, ETeleportType::TeleportPhysics);
}

void FFSDSPipelineView::Clear()
{
	if (ULineBatchComponent* L = Lines.Get()) L->Flush();
	bDrawn = false;
}
