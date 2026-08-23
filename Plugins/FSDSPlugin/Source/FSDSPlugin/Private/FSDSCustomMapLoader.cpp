#include "FSDSCustomMapLoader.h"
#include "FSDSRandom.h"
#include <cstdlib>
#include <cmath>

static FString GCustomMapPath;

bool UFSDSCustomMapLoader::FileLoadString(FString FileName, FString& OutText)
{
	UE_LOG(LogTemp, Log, TEXT("FSDS MapLoader: Loading file %s"), *FileName);
	return FFileHelper::LoadFileToString(OutText, *FileName);
}

TArray<FString> UFSDSCustomMapLoader::ProcessFile(FString Data, TArray<FTransform>& BlueCones, TArray<FTransform>& YellowCones, TArray<FTransform>& BigOrangeCones)
{
	srand((unsigned)time(NULL));

	TArray<FString> Lines;
	FString Left, Right = Data;

	while (Right.Split(TEXT("\n"), &Left, &Right))
	{
		Lines.Add(Left);
	}
	Lines.Add(Right);

	// Deterministic cone yaw for this load. Cone meshes are not perfectly
	// symmetric, so yaw changes which facets the LiDAR sees and therefore the
	// returned points — unseeded, every load produced different perception
	// input. Seeded once per load so the whole track replays identically.
	FRandomStream MapConeYaw = FSDSRandom::MakeStream(TEXT("CustomMapLoader.coneYaw"));

	for (FString& Line : Lines)
	{
		FString Type, Value;

		Line.Split(TEXT(","), &Type, &Line);
		Line.Split(TEXT(","), &Value, &Line);
		float X = FCString::Atof(*Value) * 100.f;
		Line.Split(TEXT(","), &Value, &Line);
		float Y = FCString::Atof(*Value) * 100.f;

		// Skip remaining fields (heading, variances)
		FTransform Transform(
			// was C stdlib rand(): unseeded, and shares global state with anything
			// else in the process that calls it. Now a named deterministic stream.
			FRotator(0.f, MapConeYaw.GetFraction() * 360.f, 0.f),
			FVector(X, -Y, 5.f),
			FVector(1.f, 1.f, 1.f)
		);

		if (Type == TEXT("yellow"))
			YellowCones.Add(Transform);
		else if (Type == TEXT("blue"))
			BlueCones.Add(Transform);
		else if (Type == TEXT("big_orange"))
			BigOrangeCones.Add(Transform);
	}

	return Lines;
}

FTransform UFSDSCustomMapLoader::GetFinishTransform(TArray<FTransform> BigOrangeCones)
{
	FVector Location(0.f, 0.f, 0.f);

	for (FTransform& Cone : BigOrangeCones)
	{
		Location.X += Cone.GetLocation().X;
		Location.Y += Cone.GetLocation().Y;
	}

	if (BigOrangeCones.Num() > 0)
	{
		Location.X /= BigOrangeCones.Num();
		Location.Y /= BigOrangeCones.Num();
	}

	float Angle = 0.f;
	if (BigOrangeCones.Num() == 2)
	{
		Angle = (FMath::Atan2(
			BigOrangeCones[0].GetLocation().Y - BigOrangeCones[1].GetLocation().Y,
			BigOrangeCones[0].GetLocation().X - BigOrangeCones[1].GetLocation().X
		) - PI / 2.f) * 180.f / PI;
	}

	return FTransform(FRotator(0.f, Angle, 0.f), Location, FVector(1.f, 1.f, 1.f));
}

void UFSDSCustomMapLoader::SetCustomMapPath(FString Path)
{
	GCustomMapPath = Path;
}

FString UFSDSCustomMapLoader::GetCustomMapPath()
{
	return GCustomMapPath;
}
