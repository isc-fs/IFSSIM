#include "Sensors/FSDSDistanceSensor.h"
#include "Engine/World.h"

UFSDSDistanceSensor::UFSDSDistanceSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSDistanceSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	UWorld* World = GetWorld();
	if (!Owner || !World) return;

	FDistanceOutput Output;
	Output.Timestamp = FPlatformTime::Cycles64();
	Output.MinDistance = MinRange / 100.f;
	Output.MaxDistance = MaxRange / 100.f;

	FTransform OwnerTransform = Owner->GetActorTransform();
	FVector Start = OwnerTransform.TransformPosition(SensorOffset);
	FVector Direction = OwnerTransform.TransformVector(SensorRotation.Vector());
	if (Direction.IsNearlyZero()) Direction = Owner->GetActorForwardVector();
	FVector End = Start + Direction * MaxRange;

	FCollisionQueryParams Params;
	Params.AddIgnoredActor(Owner);

	FHitResult Hit;
	if (World->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Params))
	{
		float DistCm = (Hit.ImpactPoint - Start).Size();
		if (DistCm >= MinRange)
		{
			Output.Distance = DistCm / 100.f; // cm to meters
		}
	}
	else
	{
		Output.Distance = -1.f; // No hit
	}

	CachedOutput = Output;
}
