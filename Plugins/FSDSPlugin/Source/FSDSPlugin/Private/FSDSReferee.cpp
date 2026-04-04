#include "FSDSReferee.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "GameFramework/Pawn.h"

AFSDSReferee::AFSDSReferee()
{
	PrimaryActorTick.bCanEverTick = true;

	// Root component
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RefereeRoot"));

	// Finish line trigger — positioned later from orange cones
	FinishLineTrigger = CreateDefaultSubobject<UBoxComponent>(TEXT("FinishLineTrigger"));
	FinishLineTrigger->SetupAttachment(RootComponent);
	FinishLineTrigger->SetBoxExtent(FVector(100.f, 500.f, 200.f)); // 1m deep, 10m wide, 4m tall
	FinishLineTrigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	FinishLineTrigger->SetCollisionResponseToAllChannels(ECR_Overlap);
	FinishLineTrigger->SetGenerateOverlapEvents(true);
	FinishLineTrigger->SetVisibility(false);
	FinishLineTrigger->SetHiddenInGame(true);
}

void AFSDSReferee::BeginPlay()
{
	Super::BeginPlay();

	// Bind finish line overlap
	FinishLineTrigger->OnComponentBeginOverlap.AddDynamic(this, &AFSDSReferee::OnFinishLineOverlap);

	UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Initialized (cone hit threshold: %.0f cm)"), ConeHitThreshold);
}

void AFSDSReferee::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Check all registered cones for displacement (knocked over)
	for (auto& Pair : ConeOriginalPositions)
	{
		AActor* ConeActor = Pair.Key;
		if (!ConeActor || !IsValid(ConeActor)) continue;
		if (HitCones.Contains(ConeActor)) continue; // Already counted

		FVector CurrentPos = ConeActor->GetActorLocation();
		FVector OriginalPos = Pair.Value;
		float Displacement = FVector::Dist(CurrentPos, OriginalPos);

		if (Displacement > ConeHitThreshold)
		{
			HitCones.Add(ConeActor);
			State.DooCounter++;
			UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Cone displaced %.1f cm (DOO count: %d)"),
				Displacement, State.DooCounter);
		}
	}

	// Debounce: reset finish zone flag when vehicle exits
	if (bVehicleInsideFinishZone)
	{
		// Check if vehicle has left the trigger zone
		TArray<AActor*> OverlappingActors;
		FinishLineTrigger->GetOverlappingActors(OverlappingActors);
		bool bStillInside = false;
		for (AActor* A : OverlappingActors)
		{
			if (Cast<APawn>(A))
			{
				bStillInside = true;
				break;
			}
		}
		if (!bStillInside)
		{
			bVehicleInsideFinishZone = false;
		}
	}
}

int32 AFSDSReferee::ConeHit(FString ConeName)
{
	State.DooCounter++;
	UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Cone hit '%s' (DOO count: %d)"), *ConeName, State.DooCounter);
	return State.DooCounter;
}

int32 AFSDSReferee::LapCompleted(float LapTime)
{
	State.Laps.Add(LapTime);
	UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Lap %d completed in %.3f seconds"), State.Laps.Num(), LapTime);
	return State.Laps.Num();
}

void AFSDSReferee::AppendYellowCone(FTransform ConeTransform)
{
	AppendCone(ConeTransform, EFSDSConeColor::Yellow);
}

void AFSDSReferee::AppendBlueCone(FTransform ConeTransform)
{
	AppendCone(ConeTransform, EFSDSConeColor::Blue);
}

void AFSDSReferee::AppendBigOrangeCone(FTransform ConeTransform)
{
	AppendCone(ConeTransform, EFSDSConeColor::OrangeLarge);
}

void AFSDSReferee::AppendSmallOrangeCone(FTransform ConeTransform)
{
	AppendCone(ConeTransform, EFSDSConeColor::OrangeSmall);
}

void AFSDSReferee::LoadStartPos(FVector Pos)
{
	State.CarStartLocation = FVector2D(Pos.X, Pos.Y);
	UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Start position set to (%.1f, %.1f)"), Pos.X, Pos.Y);
}

void AFSDSReferee::RegisterConeActor(AActor* ConeActor, EFSDSConeColor Color)
{
	if (!ConeActor) return;

	// Store original position for displacement tracking
	ConeOriginalPositions.Add(ConeActor, ConeActor->GetActorLocation());

	// Enable physics on the cone so it can be knocked over
	UStaticMeshComponent* MeshComp = nullptr;
	if (AStaticMeshActor* SMA = Cast<AStaticMeshActor>(ConeActor))
	{
		MeshComp = SMA->GetStaticMeshComponent();
	}
	else
	{
		MeshComp = ConeActor->FindComponentByClass<UStaticMeshComponent>();
	}

	if (MeshComp)
	{
		MeshComp->SetSimulatePhysics(true);
		MeshComp->SetMassOverrideInKg(NAME_None, 1.0f); // ~1 kg traffic cone
		MeshComp->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		MeshComp->SetCollisionResponseToAllChannels(ECR_Block);
		MeshComp->SetGenerateOverlapEvents(true);
	}

	// If this is a big orange cone, update finish line
	if (Color == EFSDSConeColor::OrangeLarge)
	{
		// Collect all big orange cone positions
		TArray<FVector> OrangePositions;
		for (const auto& Cone : State.Cones)
		{
			if (Cone.Color == EFSDSConeColor::OrangeLarge)
			{
				OrangePositions.Add(FVector(Cone.Location.X, Cone.Location.Y, 0.f));
			}
		}

		if (OrangePositions.Num() >= 2)
		{
			// Compute finish line center and direction
			FVector Sum = FVector::ZeroVector;
			for (const FVector& P : OrangePositions)
			{
				Sum += P;
			}
			FinishLineCenter = Sum / OrangePositions.Num();

			// Direction perpendicular to the line between first two orange cones
			FVector LineDir = (OrangePositions[1] - OrangePositions[0]).GetSafeNormal();
			FinishLineDirection = FVector(-LineDir.Y, LineDir.X, 0.f); // 90° rotation

			// Position and orient the finish line trigger
			float LineWidth = FVector::Dist(OrangePositions[0], OrangePositions[1]);
			FinishLineTrigger->SetWorldLocation(FVector(FinishLineCenter.X, FinishLineCenter.Y, 100.f));
			FinishLineTrigger->SetBoxExtent(FVector(100.f, LineWidth / 2.f + 100.f, 200.f));
			FinishLineTrigger->SetWorldRotation(LineDir.Rotation());

			bFinishLineValid = true;
			UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Finish line at (%.0f, %.0f) width=%.0f cm, %d orange cones"),
				FinishLineCenter.X, FinishLineCenter.Y, LineWidth, OrangePositions.Num());
		}
	}
}

void AFSDSReferee::ResetState()
{
	State.DooCounter = 0;
	State.Laps.Empty();
	State.Cones.Empty();
	ConeOriginalPositions.Empty();
	HitCones.Empty();
	bLapTimerRunning = false;
	bVehicleInsideFinishZone = false;
	bFinishLineValid = false;
	LapStartTime = 0.0;
	UE_LOG(LogTemp, Log, TEXT("FSDS Referee: State reset"));
}

void AFSDSReferee::AppendCone(FTransform Transform, EFSDSConeColor Color)
{
	FFSDSCone Cone;
	Cone.Location = FVector2D(Transform.GetTranslation().X, Transform.GetTranslation().Y);
	Cone.Color = Color;
	State.Cones.Add(Cone);
}

void AFSDSReferee::OnConeOverlap(UPrimitiveComponent* OverlappedComp, AActor* OtherActor,
	UPrimitiveComponent* OtherComp, int32 OtherBodyIndex,
	bool bFromSweep, const FHitResult& SweepResult)
{
	// Not used currently — displacement check in Tick is more reliable
}

void AFSDSReferee::SetEventType(EFSDSEventType Type, int32 NumLaps)
{
	State.EventType = Type;
	State.bFinished = false;

	switch (Type)
	{
	case EFSDSEventType::Acceleration:
		State.RequiredLaps = 1; // Single run
		ConeHitThreshold = 15.0f;
		UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Event = Acceleration (1 run)"));
		break;
	case EFSDSEventType::Skidpad:
		State.RequiredLaps = 4; // 2 right + 2 left laps
		ConeHitThreshold = 15.0f;
		UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Event = Skidpad (4 crossings)"));
		break;
	case EFSDSEventType::Autocross:
		State.RequiredLaps = 1; // Single lap
		ConeHitThreshold = 15.0f;
		UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Event = Autocross (1 lap)"));
		break;
	case EFSDSEventType::Trackdrive:
	default:
		State.RequiredLaps = FMath::Max(1, NumLaps);
		ConeHitThreshold = 15.0f;
		UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Event = Trackdrive (%d laps)"), State.RequiredLaps);
		break;
	}
}

void AFSDSReferee::OnFinishLineOverlap(UPrimitiveComponent* OverlappedComp, AActor* OtherActor,
	UPrimitiveComponent* OtherComp, int32 OtherBodyIndex,
	bool bFromSweep, const FHitResult& SweepResult)
{
	if (!bFinishLineValid) return;
	if (State.bFinished) return; // Event already complete

	// Only trigger on the vehicle pawn
	APawn* VehiclePawn = Cast<APawn>(OtherActor);
	if (!VehiclePawn) return;

	// Debounce: don't re-trigger while still in zone
	if (bVehicleInsideFinishZone) return;
	bVehicleInsideFinishZone = true;

	double CurrentTime = GetWorld()->GetTimeSeconds();

	if (!bLapTimerRunning)
	{
		// First crossing — start the timer
		LapStartTime = CurrentTime;
		bLapTimerRunning = true;
		UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Lap timer started at %.2f s (%s)"),
			CurrentTime, *UEnum::GetValueAsString(State.EventType));
	}
	else
	{
		// Subsequent crossing — record lap/run
		float LapTime = (float)(CurrentTime - LapStartTime);

		// Minimum time to avoid false triggers
		float MinTime = (State.EventType == EFSDSEventType::Acceleration) ? 1.0f : 3.0f;

		if (LapTime > MinTime)
		{
			LapCompleted(LapTime);
			LapStartTime = CurrentTime; // Start next lap

			// Check if event is finished
			if (State.Laps.Num() >= State.RequiredLaps)
			{
				State.bFinished = true;
				UE_LOG(LogTemp, Log, TEXT("FSDS Referee: EVENT FINISHED — %d/%d laps, DOO=%d"),
					State.Laps.Num(), State.RequiredLaps, State.DooCounter);
			}
		}
	}
}
