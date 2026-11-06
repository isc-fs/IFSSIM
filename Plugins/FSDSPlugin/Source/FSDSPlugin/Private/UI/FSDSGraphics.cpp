#include "UI/FSDSGraphics.h"

#include "DynamicRHI.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/ConfigCacheIni.h"

#define LOCTEXT_NAMESPACE "FSDSGraphics"

// Qualified everywhere (GraphicsUi::...): other UI files share the unity blob.
namespace GraphicsUi
{
	const TCHAR* Section = TEXT("FSDSGraphics");

	// Let each try settle (shaders, streaming) before measuring it.
	constexpr double SettleS = 1.5;
	constexpr double SampleS = 2.0;

	void SetCVar(const TCHAR* Name, int32 Value)
	{
		if (IConsoleVariable* V = IConsoleManager::Get().FindConsoleVariable(Name))
		{
			V->Set(Value, ECVF_SetByCode);
		}
	}

	/** The scalability groups' levels for a quality (DefaultScalability.ini defines 0 and 1; 2 and 3 are the engine's). */
	struct FLevels { int32 Shadow, PostProcess, AntiAliasing, Shading, Texture, Effects; };
	FLevels LevelsFor(FFSDSGraphics::EQuality Q)
	{
		switch (Q)
		{
		case FFSDSGraphics::EQuality::High:   return {3, 3, 3, 3, 3, 3};
		case FFSDSGraphics::EQuality::Medium: return {2, 1, 1, 1, 1, 1};
		default:                              return {1, 0, 0, 0, 0, 0};   // Config/DefaultEngine.ini's
		}
	}

	double Percentile(TArray<double> V, double P)
	{
		if (V.Num() == 0) return 0.0;
		V.Sort();
		return V[FMath::Clamp(FMath::FloorToInt(P * (V.Num() - 1)), 0, V.Num() - 1)];
	}
}

FFSDSGraphics& FFSDSGraphics::Get()
{
	static FFSDSGraphics Instance;
	return Instance;
}

FText FFSDSGraphics::QualityName(EQuality Q)
{
	switch (Q)
	{
	case EQuality::High:   return LOCTEXT("High", "High");
	case EQuality::Medium: return LOCTEXT("Medium", "Medium");
	default:               return LOCTEXT("Low", "Low");
	}
}

void FFSDSGraphics::Init(UWorld* InWorld)
{
	World = InWorld;
	bViewDrawn = true;
	if (GConfig)
	{
		int32 Q = 0, Pct = Settings.ResolutionPct;
		bool bHide = false;
		GConfig->GetInt(GraphicsUi::Section, TEXT("Quality"), Q, GGameUserSettingsIni);
		GConfig->GetInt(GraphicsUi::Section, TEXT("ResolutionPct"), Pct, GGameUserSettingsIni);
		GConfig->GetBool(GraphicsUi::Section, TEXT("HideViewWhileAutonomous"), bHide, GGameUserSettingsIni);
		Settings.Quality = (EQuality)FMath::Clamp(Q, 0, 2);
		Settings.ResolutionPct = (Pct == 50 || Pct == 75 || Pct == 100) ? Pct : 25;
		Settings.bHideViewWhileAutonomous = bHide;
	}
	Apply();
}

void FFSDSGraphics::Shutdown()
{
	if (TestTicker.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TestTicker);
		TestTicker.Reset();
	}
	TestIndex = INDEX_NONE;
	OnDone = nullptr;
	SetViewDrawn(true);
	World.Reset();
}

void FFSDSGraphics::Apply() const
{
	const GraphicsUi::FLevels L = GraphicsUi::LevelsFor(Settings.Quality);
	GraphicsUi::SetCVar(TEXT("sg.ShadowQuality"), L.Shadow);
	GraphicsUi::SetCVar(TEXT("sg.PostProcessQuality"), L.PostProcess);
	GraphicsUi::SetCVar(TEXT("sg.AntiAliasingQuality"), L.AntiAliasing);
	GraphicsUi::SetCVar(TEXT("sg.ShadingQuality"), L.Shading);
	GraphicsUi::SetCVar(TEXT("sg.TextureQuality"), L.Texture);
	GraphicsUi::SetCVar(TEXT("sg.EffectsQuality"), L.Effects);
	GraphicsUi::SetCVar(TEXT("r.ScreenPercentage"), Settings.ResolutionPct);
	UE_LOG(LogTemp, Log, TEXT("FSDS Graphics: %s, %d%% resolution%s"),
		*QualityName(Settings.Quality).ToString(), Settings.ResolutionPct,
		Settings.bHideViewWhileAutonomous ? TEXT(", view hidden while the pipeline drives") : TEXT(""));
}

void FFSDSGraphics::Save() const
{
	if (!GConfig) return;
	GConfig->SetInt(GraphicsUi::Section, TEXT("Quality"), (int32)Settings.Quality, GGameUserSettingsIni);
	GConfig->SetInt(GraphicsUi::Section, TEXT("ResolutionPct"), Settings.ResolutionPct, GGameUserSettingsIni);
	GConfig->SetBool(GraphicsUi::Section, TEXT("HideViewWhileAutonomous"), Settings.bHideViewWhileAutonomous, GGameUserSettingsIni);
	GConfig->Flush(false, GGameUserSettingsIni);
}

void FFSDSGraphics::SetQuality(EQuality Q)
{
	Settings.Quality = Q;
	Apply();
	Save();
}

void FFSDSGraphics::SetResolution(int32 Pct)
{
	Settings.ResolutionPct = FMath::Clamp(Pct, 25, 100);
	Apply();
	Save();
}

void FFSDSGraphics::SetHideViewWhileAutonomous(bool bHide)
{
	Settings.bHideViewWhileAutonomous = bHide;
	Save();
}

void FFSDSGraphics::SetViewDrawn(bool bDrawn)
{
	if (bDrawn == bViewDrawn) return;
	UWorld* W = World.Get();
	UGameViewportClient* Viewport = W ? W->GetGameViewport() : nullptr;
	if (!Viewport) return;
	Viewport->bDisableWorldRendering = !bDrawn;
	bViewDrawn = bDrawn;
	UE_LOG(LogTemp, Log, TEXT("FSDS Graphics: 3D view %s"), bDrawn ? TEXT("drawn") : TEXT("hidden while the pipeline drives"));
}

// ---------------------------------------------------------------------------
// Recommend

void FFSDSGraphics::Recommend(TFunction<void(const FString&)> Done)
{
	if (IsRecommending()) return;
	Before = Settings;
	OnDone = MoveTemp(Done);
	// The best looking first.
	Tries = {
		{EQuality::High, 100}, {EQuality::High, 75}, {EQuality::Medium, 75},
		{EQuality::Medium, 50}, {EQuality::Low, 50}, {EQuality::Low, 25},
	};
	TestIndex = 0;
	Settings.Quality = Tries[0].Quality;
	Settings.ResolutionPct = Tries[0].ResolutionPct;
	Apply();
	TryStartS = FPlatformTime::Seconds();
	GpuMs.Reset();
	TestTicker = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateRaw(this, &FFSDSGraphics::TickRecommend));
}

FString FFSDSGraphics::RecommendProgress() const
{
	if (!IsRecommending() || !Tries.IsValidIndex(TestIndex)) return FString();
	return FString::Printf(TEXT("Trying %s at %d%% (%d of %d)..."),
		*QualityName(Tries[TestIndex].Quality).ToString(), Tries[TestIndex].ResolutionPct, TestIndex + 1, Tries.Num());
}

bool FFSDSGraphics::TickRecommend(float)
{
	const double Now = FPlatformTime::Seconds();
	const double Age = Now - TryStartS;
	if (Age >= GraphicsUi::SettleS)
	{
		// The GPU's own time for the last frame, not the frame rate: the fixed
		// 60 fps step hides any headroom.
		const double Ms = FPlatformTime::ToMilliseconds64(RHIGetGPUFrameCycles());
		if (Ms > 0.0) GpuMs.Add(Ms);
	}
	if (Age < GraphicsUi::SettleS + GraphicsUi::SampleS) return true;

	// p90: the LiDAR's scans make every few frames heavier, and those are the
	// ones that would miss the frame.
	const double P90 = GraphicsUi::Percentile(GpuMs, 0.9);
	const FTry T = Tries[TestIndex];
	UE_LOG(LogTemp, Log, TEXT("FSDS Graphics: %s at %d%%: GPU %.1f ms per frame (p90 of %d)"),
		*QualityName(T.Quality).ToString(), T.ResolutionPct, P90, GpuMs.Num());
	const bool bLast = TestIndex == Tries.Num() - 1;
	if ((P90 > 0.0 && P90 <= RecommendBudgetMs) || bLast)
	{
		// Keep it (or the cheapest, if nothing fits).
		Settings.Quality = T.Quality;
		Settings.ResolutionPct = T.ResolutionPct;
		Settings.bHideViewWhileAutonomous = Before.bHideViewWhileAutonomous || (P90 > RecommendBudgetMs);
		Apply();
		Save();
		const FString Msg = P90 > RecommendBudgetMs
			? FString::Printf(TEXT("Even the cheapest view takes %.1f ms of the GPU's 16.7 ms per frame: Low at 25%%, with the view hidden while the pipeline drives. Check the HUD's Real time line during a run."), P90)
			: FString::Printf(TEXT("%s at %d%%: the GPU takes %.1f ms of each 16.7 ms frame, leaving room for the pipeline."),
				*QualityName(T.Quality).ToString(), T.ResolutionPct, P90);
		TestTicker.Reset();   // returning false removes it
		TestIndex = INDEX_NONE;
		TFunction<void(const FString&)> Done = MoveTemp(OnDone);
		if (Done) Done(Msg);
		return false;
	}
	// Too heavy: the next one down.
	++TestIndex;
	Settings.Quality = Tries[TestIndex].Quality;
	Settings.ResolutionPct = Tries[TestIndex].ResolutionPct;
	Apply();
	TryStartS = Now;
	GpuMs.Reset();
	return true;
}

#undef LOCTEXT_NAMESPACE
