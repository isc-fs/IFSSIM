#include "Environment/FSDSEnvironment.h"

#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
	// FJsonObject's TryGet*Field convert between types (the string "1" reads
	// as the number 1, the number 5 as the string "5"). A sidecar is generated
	// by a program, so a value of the wrong type is a bug to report, not one
	// to paper over.
	bool StrictNumber(const FJsonObject& Obj, const TCHAR* Key, double& Out)
	{
		const TSharedPtr<FJsonValue> V = Obj.TryGetField(Key);
		if (!V.IsValid() || V->Type != EJson::Number) return false;
		Out = V->AsNumber();
		return true;
	}

	bool StrictString(const FJsonObject& Obj, const TCHAR* Key, FString& Out)
	{
		const TSharedPtr<FJsonValue> V = Obj.TryGetField(Key);
		if (!V.IsValid() || V->Type != EJson::String) return false;
		Out = V->AsString();
		return true;
	}

	bool OnlyKnownKeys(const FJsonObject& Obj, std::initializer_list<const TCHAR*> Known,
		const FString& Where, FString& OutError)
	{
		for (const auto& Pair : Obj.Values)
		{
			bool bKnown = false;
			for (const TCHAR* K : Known)
			{
				if (Pair.Key == K) { bKnown = true; break; }
			}
			if (!bKnown)
			{
				OutError = FString::Printf(TEXT("%s: unknown key '%s'"), *Where, *Pair.Key);
				return false;
			}
		}
		return true;
	}

	bool ReadCoord(const FJsonObject& Obj, const TCHAR* Key, const FString& Where,
		double& Out, FString& OutError)
	{
		if (!StrictNumber(Obj, Key, Out))
		{
			OutError = FString::Printf(TEXT("%s: '%s' must be a number"), *Where, Key);
			return false;
		}
		if (FMath::Abs(Out) > FSDSEnvironment::MaxCoordM)
		{
			OutError = FString::Printf(TEXT("%s: '%s' = %.1f m is outside +-%.0f m (cm instead of m?)"),
				*Where, Key, Out, FSDSEnvironment::MaxCoordM);
			return false;
		}
		return true;
	}
}

FString FSDSEnvironment::SidecarPathFor(const FString& TrackCsvPath)
{
	return FPaths::Combine(FPaths::GetPath(TrackCsvPath),
		FPaths::GetBaseFilename(TrackCsvPath) + TEXT(".env.json"));
}

bool FSDSEnvironment::Parse(const FString& Json, const FString& SourcePath,
	FFSDSEnvironment& Out, FString& OutError)
{
	Out = FFSDSEnvironment();
	FFSDSEnvironment Env;
	Env.SidecarPath = SourcePath;

	TSharedPtr<FJsonObject> Root;
	TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
	if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
	{
		OutError = TEXT("not a JSON object");
		return false;
	}
	if (!OnlyKnownKeys(*Root, { TEXT("format"), TEXT("seed"), TEXT("profile"), TEXT("ground"), TEXT("props") },
		TEXT("sidecar"), OutError))
	{
		return false;
	}

	FString Format;
	if (!StrictString(*Root, TEXT("format"), Format) || Format != FormatV1)
	{
		OutError = FString::Printf(TEXT("'format' must be \"%s\" (got \"%s\")"), FormatV1, *Format);
		return false;
	}

	double SeedVal = 0.0;
	if (!StrictNumber(*Root, TEXT("seed"), SeedVal) || SeedVal != FMath::FloorToDouble(SeedVal) || SeedVal < 0.0)
	{
		OutError = TEXT("'seed' must be a non-negative integer");
		return false;
	}
	Env.Seed = (int64)SeedVal;

	if (!StrictString(*Root, TEXT("profile"), Env.Profile) || Env.Profile.IsEmpty())
	{
		OutError = TEXT("'profile' must be a non-empty string");
		return false;
	}

	const TSharedPtr<FJsonObject>* Ground = nullptr;
	if (Root->HasField(TEXT("ground")))
	{
		if (!Root->TryGetObjectField(TEXT("ground"), Ground))
		{
			OutError = TEXT("'ground' must be an object");
			return false;
		}
		if (!OnlyKnownKeys(**Ground, { TEXT("extent") }, TEXT("ground"), OutError)) return false;
		const TSharedPtr<FJsonObject>* Extent = nullptr;
		if (!(*Ground)->TryGetObjectField(TEXT("extent"), Extent))
		{
			OutError = TEXT("ground: 'extent' must be an object");
			return false;
		}
		if (!OnlyKnownKeys(**Extent, { TEXT("x_min"), TEXT("y_min"), TEXT("x_max"), TEXT("y_max") },
			TEXT("ground.extent"), OutError))
		{
			return false;
		}
		const FString Where = TEXT("ground.extent");
		if (!ReadCoord(**Extent, TEXT("x_min"), Where, Env.GroundXMin, OutError) ||
			!ReadCoord(**Extent, TEXT("y_min"), Where, Env.GroundYMin, OutError) ||
			!ReadCoord(**Extent, TEXT("x_max"), Where, Env.GroundXMax, OutError) ||
			!ReadCoord(**Extent, TEXT("y_max"), Where, Env.GroundYMax, OutError))
		{
			return false;
		}
		if (Env.GroundXMin >= Env.GroundXMax || Env.GroundYMin >= Env.GroundYMax)
		{
			OutError = TEXT("ground.extent: min must be below max on both axes");
			return false;
		}
		Env.bHasGroundExtent = true;
	}

	if (Root->HasField(TEXT("props")))
	{
		const TArray<TSharedPtr<FJsonValue>>* Props = nullptr;
		if (!Root->TryGetArrayField(TEXT("props"), Props))
		{
			OutError = TEXT("'props' must be an array");
			return false;
		}
		for (int32 i = 0; i < Props->Num(); ++i)
		{
			const FString Where = FString::Printf(TEXT("props[%d]"), i);
			const TSharedPtr<FJsonObject>* PropObj = nullptr;
			if (!(*Props)[i].IsValid() || !(*Props)[i]->TryGetObject(PropObj))
			{
				OutError = Where + TEXT(": must be an object");
				return false;
			}
			if (!OnlyKnownKeys(**PropObj, { TEXT("class"), TEXT("x"), TEXT("y"), TEXT("yaw_deg") }, Where, OutError))
			{
				return false;
			}
			FFSDSEnvProp Prop;
			if (!StrictString(**PropObj, TEXT("class"), Prop.Class) || Prop.Class.IsEmpty())
			{
				OutError = Where + TEXT(": 'class' must be a non-empty string");
				return false;
			}
			if (!ReadCoord(**PropObj, TEXT("x"), Where, Prop.X, OutError) ||
				!ReadCoord(**PropObj, TEXT("y"), Where, Prop.Y, OutError))
			{
				return false;
			}
			if ((*PropObj)->HasField(TEXT("yaw_deg")) && !StrictNumber(**PropObj, TEXT("yaw_deg"), Prop.YawDeg))
			{
				OutError = Where + TEXT(": 'yaw_deg' must be a number");
				return false;
			}
			Env.Props.Add(MoveTemp(Prop));
		}
	}

	Out = MoveTemp(Env);
	return true;
}

bool FSDSEnvironment::Load(const FString& Path, FFSDSEnvironment& Out, FString& OutError)
{
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *Path))
	{
		Out = FFSDSEnvironment();
		OutError = TEXT("cannot read file");
		return false;
	}
	return Parse(Text, Path, Out, OutError);
}
