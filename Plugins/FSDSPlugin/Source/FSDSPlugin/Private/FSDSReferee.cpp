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

	// Wait for physics to settle before recording cone positions
	if (!bPositionsRecorded)
	{
		PositionSnapshotTimer += DeltaTime;
		if (PositionSnapshotTimer >= PositionSnapshotDelay)
		{
			// Snapshot current positions as "original" (after physics settled)
			for (auto& Pair : ConeOriginalPositions)
			{
				if (Pair.Key && IsValid(Pair.Key))
				{
					Pair.Value = Pair.Key->GetActorLocation();
				}
			}
			bPositionsRecorded = true;
			UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Cone positions recorded after %.1fs settle"), PositionSnapshotDelay);
		}
	}

	// Check all registered cones for displacement (only after positions recorded)
	if (bPositionsRecorded)
	{
		for (auto& Pair : ConeOriginalPositions)
		{
			AActor* ConeActor = Pair.Key;
			if (!ConeActor || !IsValid(ConeActor)) continue;
			if (HitCones.Contains(ConeActor)) continue;

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
	}

	// Out-of-bounds detection (throttled to avoid per-frame cost)
	OffTrackTimer += DeltaTime;
	if (OffTrackTimer >= OffTrackCheckInterval && State.Cones.Num() > 10)
	{
		OffTrackTimer = 0.f;

		// Find the vehicle pawn
		APawn* Vehicle = GetWorld()->GetFirstPlayerController() ?
			GetWorld()->GetFirstPlayerController()->GetPawn() : nullptr;

		if (Vehicle)
		{
			FVector VehiclePos = Vehicle->GetActorLocation();
			FVector2D CarPos2D(VehiclePos.X, VehiclePos.Y);

			bool bCurrentlyOffTrack = !IsInsideTrack(CarPos2D);

			if (bCurrentlyOffTrack && !bWasOffTrack)
			{
				// Transition: on-track → off-track
				State.OffTrackCounter++;
				UE_LOG(LogTemp, Log, TEXT("FSDS Referee: OFF TRACK (OC count: %d)"), State.OffTrackCounter);
			}
			bWasOffTrack = bCurrentlyOffTrack;
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
		// Set collision profile BEFORE enabling physics (order matters in Chaos)
		MeshComp->SetCollisionProfileName(TEXT("PhysicsActor"));
		MeshComp->SetSimulatePhysics(true);
		MeshComp->SetMassOverrideInKg(NAME_None, 1.0f); // ~1 kg traffic cone
		MeshComp->SetGenerateOverlapEvents(true);

		// Allow Chaos to sleep this cone as soon as it settles.
		// Without this, all cones simulate every physics tick even when stationary.
		if (FBodyInstance* BI = MeshComp->GetBodyInstance())
		{
			BI->SleepFamily = ESleepFamily::Sensitive;
		}
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
	State.OffTrackCounter = 0;
	State.Laps.Empty();
	State.Cones.Empty();
	ConeOriginalPositions.Empty();
	HitCones.Empty();
	bLapTimerRunning = false;
	bVehicleInsideFinishZone = false;
	bFinishLineValid = false;
	bWasOffTrack = false;
	bPositionsRecorded = false;
	PositionSnapshotTimer = 0.f;
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
				UE_LOG(LogTemp, Log, TEXT("FSDS Referee: EVENT FINISHED — %d/%d laps, DOO=%d, OC=%d"),
					State.Laps.Num(), State.RequiredLaps, State.DooCounter, State.OffTrackCounter);
			}
		}
	}
}

float AFSDSReferee::DistToNearestCone(FVector2D Point, EFSDSConeColor Color) const
{
	float MinDist = TNumericLimits<float>::Max();
	for (const FFSDSCone& Cone : State.Cones)
	{
		if (Cone.Color != Color) continue;
		float Dist = FVector2D::Distance(Point, Cone.Location);
		if (Dist < MinDist) MinDist = Dist;
	}
	return MinDist;
}

bool AFSDSReferee::IsInsideTrack(FVector2D Point) const
{
	// Strategy: find the two nearest blue cones and two nearest yellow cones.
	// The car is "on track" if it's closer to the track centerline than
	// to the outer boundary. We approximate this by checking that the car
	// is between the blue and yellow cone lines.
	//
	// Simple approach: find the nearest blue and nearest yellow cone.
	// If both are within a reasonable distance AND the car is between them,
	// it's on track. If either is very far, the car has left the track.

	float DistBlue = DistToNearestCone(Point, EFSDSConeColor::Blue);
	float DistYellow = DistToNearestCone(Point, EFSDSConeColor::Yellow);

	// Track width tolerance: if both boundaries are within max track width, car is near track
	// Typical FS track width: 3-5m = 300-500cm. Allow some margin.
	float MaxTrackWidth = 600.f; // 6m in cm — generous limit

	// If either cone line is very far, car has left the track
	if (DistBlue > MaxTrackWidth && DistYellow > MaxTrackWidth)
	{
		return false; // Far from both boundaries
	}

	// Find the two nearest cones (one blue, one yellow) and check
	// if the car is between them using a perpendicular distance approach.
	// Simplified: car is on track if min(distBlue, distYellow) < MaxTrackWidth/2
	// AND the sum of distances is reasonable (less than track width + margin)
	float SumDist = DistBlue + DistYellow;
	if (SumDist > MaxTrackWidth * 1.5f)
	{
		return false; // Too far from both sides combined
	}

	return true;
}
