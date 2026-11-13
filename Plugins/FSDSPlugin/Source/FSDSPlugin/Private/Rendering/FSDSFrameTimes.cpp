#include "Rendering/FSDSFrameTimes.h"

#include "Containers/Ticker.h"
#include "DynamicRHI.h"
#include "HAL/PlatformTime.h"
#include "Misc/CoreMisc.h"
#include "RenderTimer.h"

// Named, not anonymous: in a unity build an anonymous namespace's names
// are visible to the other files in the blob and trip -Wshadow there.
namespace FSDSFrameTimesPrivate
{
	constexpr double ReportEveryS = 5.0;

	struct FTotals
	{
		int32 Frames = 0;
		double Frame = 0.0, Game = 0.0, Render = 0.0, Rhi = 0.0, Gpu = 0.0;
		double FrameMax = 0.0, GameMax = 0.0, RenderMax = 0.0, GpuMax = 0.0;
	};

	// Frames since the LiDAR's last scan: 0, 1, ..., LidarBins-1 (and later).
	// Render and GPU times lag the game thread by a frame or two, so binning
	// by distance from the scan shows the cost whatever the lag is.
	constexpr int32 LidarBins = 6;
	struct FLidarBin
	{
		int32 Frames = 0;
		double Render = 0.0, Gpu = 0.0;
	};

	FTSTicker::FDelegateHandle Handle;
	FTotals Totals;
	FLidarBin LidarBin[LidarBins];
	bool bLidarScanPending = false;
	bool bLidarSeen = false;
	int32 FramesSinceLidarScan = LidarBins;
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

		if (bLidarScanPending)
		{
			FramesSinceLidarScan = 0;
			bLidarScanPending = false;
		}
		else if (FramesSinceLidarScan < LidarBins - 1)
		{
			++FramesSinceLidarScan;
		}
		FLidarBin& B = LidarBin[FMath::Min(FramesSinceLidarScan, LidarBins - 1)];
		++B.Frames;
		B.Render += RenderMs;
		B.Gpu += GpuMs;

		const double WindowS = Now - WindowStart;
		if (WindowS >= ReportEveryS)
		{
			const double N = double(T.Frames);
			UE_LOG(LogTemp, Log,
				TEXT("FSDS Frame: %d frames in %.1f s (%.1f FPS) | mean ms: frame %.2f, game %.2f, render %.2f, rhi %.2f, gpu %.2f | max ms: frame %.2f, game %.2f, render %.2f, gpu %.2f"),
				T.Frames, WindowS, N / WindowS,
				T.Frame / N, T.Game / N, T.Render / N, T.Rhi / N, T.Gpu / N,
				T.FrameMax, T.GameMax, T.RenderMax, T.GpuMax);
			if (bLidarSeen)
			{
				// Frames k after the scan frame: "k:render/gpu (frames)".
				FString Bins;
				for (int32 K = 0; K < LidarBins; ++K)
				{
					const FLidarBin& Bk = LidarBin[K];
					const double Nk = FMath::Max(1.0, double(Bk.Frames));
					Bins += FString::Printf(TEXT(" %d%s:%.2f/%.2f(%d)"),
						K, K == LidarBins - 1 ? TEXT("+") : TEXT(""),
						Bk.Render / Nk, Bk.Gpu / Nk, Bk.Frames);
				}
				UE_LOG(LogTemp, Log,
					TEXT("FSDS Frame LiDAR: render/gpu ms by frames since scan:%s"), *Bins);
			}
			for (FLidarBin& Bk : LidarBin) Bk = FLidarBin();
			Totals = FTotals();
			WindowStart = Now;
		}
		return true;
	}
}

namespace FSDSFrameTimes
{
	namespace P = FSDSFrameTimesPrivate;

	void Start()
	{
		if (P::Handle.IsValid()) return;
		P::Totals = P::FTotals();
		P::WindowStart = P::LastTick = 0.0;
		P::Handle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateStatic(&P::Tick));
	}

	void NoteLidarScan()
	{
		P::bLidarScanPending = true;
		P::bLidarSeen = true;
	}

	void Stop()
	{
		if (!P::Handle.IsValid()) return;
		FTSTicker::GetCoreTicker().RemoveTicker(P::Handle);
		P::Handle.Reset();
	}
}
