#include "FSDSReferee.h"

AFSDSReferee::AFSDSReferee()
{
	PrimaryActorTick.bCanEverTick = true;
}

void AFSDSReferee::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(LogTemp, Log, TEXT("FSDS Referee: Initialized"));
}

void AFSDSReferee::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
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

void AFSDSReferee::AppendCone(FTransform Transform, EFSDSConeColor Color)
{
	FFSDSCone Cone;
	Cone.Location = FVector2D(Transform.GetTranslation().X, Transform.GetTranslation().Y);
	Cone.Color = Color;
	State.Cones.Add(Cone);
}
