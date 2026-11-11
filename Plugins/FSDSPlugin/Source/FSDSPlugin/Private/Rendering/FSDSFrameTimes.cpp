#include "Rendering/FSDSFrameTimes.h"

#include "Containers/Ticker.h"
#include "DynamicRHI.h"
#include "HAL/PlatformTime.h"
#include "Misc/CoreMisc.h"
#include "RenderTimer.h"

namespace
{
	constexpr double ReportEveryS = 5.0;

	struct FTotals
	{
		int32 Frames = 0;
		double Frame = 0.0, Game = 0.0, Render = 0.0, Rhi = 0.0, Gpu = 0.0;
		double FrameMax = 0.0, GameMax = 0.0, RenderMax = 0.0, GpuMax = 0.0;
	};

	FTSTicker::FDelegateHandle Handle;
	FTotals Totals;
	double WindowStart = 0.0;
	double LastTick = 0.0;

	bool Tick(float /*DeltaTime: may be a fixed step, so the wall clock is used*/)
	{
		// The module loads before the engine knows it is an editor, so ask here.
		if (GIsEditor || IsRunningCommandlet()) return true;
		const double Now = FPlatformTime::Seconds();
		if (LastTick == 0.0)
		{
			LastTick = WindowStart = Now;
			return true;
		}
		const double FrameMs = (Now - LastTick) * 1000.0;
		LastTick = Now;

		// The previous frame's times, as `stat unit` reads them.
		const double GameMs = FPlatformTime::ToMilliseconds(GGameThreadTime);
		const double RenderMs = FPlatformTime::ToMilliseconds(GRenderThreadTime);
		const double RhiMs = FPlatformTime::ToMilliseconds(GRHIThreadTime);
		const double GpuMs = FPlatformTime::ToMilliseconds(RHIGetGPUFrameCycles(0));

		FTotals& T = Totals;
		++T.Frames;
		T.Frame += FrameMs;   T.FrameMax = FMath::Max(T.FrameMax, FrameMs);
		T.Game += GameMs;     T.GameMax = FMath::Max(T.GameMax, GameMs);
		T.Render += RenderMs; T.RenderMax = FMath::Max(T.RenderMax, RenderMs);
		T.Rhi += RhiMs;
		T.Gpu += GpuMs;       T.GpuMax = FMath::Max(T.GpuMax, GpuMs);

		const double WindowS = Now - WindowStart;
		if (WindowS >= ReportEveryS)
		{
			const double N = double(T.Frames);
			UE_LOG(LogTemp, Log,
				TEXT("FSDS Frame: %d frames in %.1f s (%.1f FPS) | mean ms: frame %.2f, game %.2f, render %.2f, rhi %.2f, gpu %.2f | max ms: frame %.2f, game %.2f, render %.2f, gpu %.2f"),
				T.Frames, WindowS, N / WindowS,
				T.Frame / N, T.Game / N, T.Render / N, T.Rhi / N, T.Gpu / N,
				T.FrameMax, T.GameMax, T.RenderMax, T.GpuMax);
			Totals = FTotals();
			WindowStart = Now;
		}
		return true;
	}
}

namespace FSDSFrameTimes
{
	void Start()
	{
		if (Handle.IsValid()) return;
		Totals = FTotals();
		WindowStart = LastTick = 0.0;
		Handle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateStatic(&Tick));
	}

	void Stop()
	{
		if (!Handle.IsValid()) return;
		FTSTicker::GetCoreTicker().RemoveTicker(Handle);
		Handle.Reset();
	}
}
