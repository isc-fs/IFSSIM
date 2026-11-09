#include "Rendering/FSDSQualityPreset.h"

#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "Containers/Ticker.h"
#include "EngineUtils.h"
#include "Environment/FSDSDaylight.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace
{
	struct FSetting { const TCHAR* Name; const TCHAR* Showcase; };

	// The showcase values. Sim is whatever the ini files set, captured the
	// first time showcase is applied and put back by Apply(Sim).
	const FSetting Settings[] = {
		{ TEXT("r.ScreenPercentage"),                 TEXT("100") },
		{ TEXT("sg.ViewDistanceQuality"),             TEXT("3") },
		{ TEXT("sg.AntiAliasingQuality"),             TEXT("3") },
		{ TEXT("sg.ShadowQuality"),                   TEXT("3") },
		{ TEXT("sg.GlobalIlluminationQuality"),       TEXT("3") },
		{ TEXT("sg.ReflectionQuality"),               TEXT("3") },
		{ TEXT("sg.PostProcessQuality"),              TEXT("3") },
		{ TEXT("sg.TextureQuality"),                  TEXT("3") },
		{ TEXT("sg.EffectsQuality"),                  TEXT("3") },
		{ TEXT("sg.FoliageQuality"),                  TEXT("3") },
		{ TEXT("sg.ShadingQuality"),                  TEXT("3") },
		{ TEXT("r.DynamicGlobalIlluminationMethod"),  TEXT("1") },   // Lumen
		{ TEXT("r.ReflectionMethod"),                 TEXT("1") },   // Lumen
		{ TEXT("r.EyeAdaptationQuality"),             TEXT("2") },   // auto exposure
	};

	TMap<FString, FString> SimValues;
	FSDSQuality::EPreset Active = FSDSQuality::EPreset::Sim;

	void Set(const TCHAR* Name, const FString& Value)
	{
		if (IConsoleVariable* V = IConsoleManager::Get().FindConsoleVariable(Name))
		{
			// SetByCode outranks the ini files, so the preset holds.
			V->Set(*Value, ECVF_SetByCode);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("FSDS Quality: no console variable %s"), Name);
		}
	}

	/** Showcase daylight in World: one AFSDSDaylight, or none for sim. */
	void SetDaylight(UWorld* World, bool bOn)
	{
		if (!World || !World->IsGameWorld()) return;
		bool bHave = false;
		for (TActorIterator<AFSDSDaylight> It(World); It; ++It)
		{
			if (bOn) bHave = true;
			else It->Destroy();
		}
		if (bOn && !bHave) World->SpawnActor<AFSDSDaylight>();
	}

	FAutoConsoleCommandWithWorldAndArgs GQualityCmd(
		TEXT("FSDS.Quality"),
		TEXT("FSDS.Quality sim|showcase - rendering preset (docs/rendering_presets.md)"),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World) {
			if (Args.Num() != 1 || !FSDSQuality::ApplyByName(Args[0], World))
			{
				UE_LOG(LogTemp, Warning, TEXT("FSDS Quality: usage FSDS.Quality sim|showcase (now %s)"),
					FSDSQuality::Name(FSDSQuality::Current()));
			}
		}));

	// A still, taken once lighting has settled: Lumen gathers its bounce light
	// over a few seconds, and a shot on the first frames comes out darker and
	// noisier than what the sim shows a moment later.
	FAutoConsoleCommandWithWorldAndArgs GShotCmd(
		TEXT("FSDS.Shot"),
		TEXT("FSDS.Shot [seconds=5] [WxH=3840x2160] - high-resolution screenshot after the delay (wall clock)"),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World) {
			const float Delay = Args.Num() > 0 ? FCString::Atof(*Args[0]) : 5.f;
			const FString Size = Args.Num() > 1 ? Args[1] : FString(TEXT("3840x2160"));
			int32 W = 0, H = 0;
			FString Ws, Hs;
			if (!World || Delay < 0.f || Delay > 600.f || !Size.Split(TEXT("x"), &Ws, &Hs)
				|| (W = FCString::Atoi(*Ws)) < 16 || (H = FCString::Atoi(*Hs)) < 16 || W > 15360 || H > 8640)
			{
				UE_LOG(LogTemp, Warning, TEXT("FSDS Shot: usage FSDS.Shot [seconds 0..600] [WxH up to 15360x8640]"));
				return;
			}
			TWeakObjectPtr<UWorld> Weak(World);
			const FString Cmd = FString::Printf(TEXT("HighResShot %dx%d"), W, H);
			// Wall-clock, not a world timer: the sim sits paused until a client
			// connects, and paused worlds don't run their timers.
			FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([Weak, Cmd](float) {
				// HighResShot belongs to the game viewport, not the engine.
				if (Weak.IsValid() && GEngine && GEngine->GameViewport)
				{
					GEngine->GameViewport->ConsoleCommand(Cmd);
					UE_LOG(LogTemp, Log, TEXT("FSDS Shot: taken (%s)"), *Cmd);
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("FSDS Shot: no game viewport, nothing taken"));
				}
				return false;   // once
			}), FMath::Max(Delay, 0.01f));
			UE_LOG(LogTemp, Log, TEXT("FSDS Shot: %s in %.1f s"), *Cmd, Delay);
		}));
}

namespace FSDSQuality
{
	const TCHAR* Name(EPreset Preset)
	{
		return Preset == EPreset::Showcase ? TEXT("showcase") : TEXT("sim");
	}

	EPreset Current() { return Active; }

	void Apply(EPreset Preset, UWorld* World)
	{
		SetDaylight(World, Preset == EPreset::Showcase);
		if (Preset == Active) return;
		if (Preset == EPreset::Showcase)
		{
			for (const FSetting& S : Settings)
			{
				if (IConsoleVariable* V = IConsoleManager::Get().FindConsoleVariable(S.Name))
				{
					SimValues.FindOrAdd(S.Name, V->GetString());
				}
				Set(S.Name, S.Showcase);
			}
		}
		else
		{
			for (const FSetting& S : Settings)
			{
				if (const FString* V = SimValues.Find(S.Name)) Set(S.Name, *V);
			}
		}
		Active = Preset;
		UE_LOG(LogTemp, Log, TEXT("FSDS Quality: %s preset%s"), Name(Preset),
			Preset == EPreset::Showcase ? TEXT(" (demos and renders; slower, don't benchmark on it)") : TEXT(""));
	}

	bool ApplyByName(const FString& InName, UWorld* World)
	{
		if (InName.Equals(TEXT("showcase"), ESearchCase::IgnoreCase)) { Apply(EPreset::Showcase, World); return true; }
		if (InName.Equals(TEXT("sim"), ESearchCase::IgnoreCase)) { Apply(EPreset::Sim, World); return true; }
		return false;
	}

	void ApplyCommandLine(UWorld* World)
	{
		if (FParse::Param(FCommandLine::Get(), TEXT("showcase"))) Apply(EPreset::Showcase, World);
	}
}
