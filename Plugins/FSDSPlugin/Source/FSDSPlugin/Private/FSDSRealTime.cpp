#include "FSDSRealTime.h"

#include "Engine/World.h"
#include "HAL/PlatformTime.h"

namespace
{
	// The factor and fps are averaged over this much real time.
	constexpr double WindowS = 2.0;
	// ... and only reported once this much is in the window.
	constexpr double MinSpanS = 0.5;
	// A gap this long between frames is a pause, a load or a debugger, not a
	// slow frame: start over rather than count it.
	constexpr double MaxGapS = 0.5;
}

FFSDSRealTime& FFSDSRealTime::Get()
{
	static FFSDSRealTime Instance;
	return Instance;
}

void FFSDSRealTime::Start(UWorld* InWorld)
{
	Stop();
	World = InWorld;
	Restart();
	TickHandle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateRaw(this, &FFSDSRealTime::Tick));
}

void FFSDSRealTime::Stop()
{
	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}
	World.Reset();
	Restart();
}

void FFSDSRealTime::Restart()
{
	Window.Reset();
	FactorValue.store(0.0);
	FpsValue.store(0.0);
}

bool FFSDSRealTime::Tick(float)
{
	const UWorld* W = World.Get();
	if (!W || W->IsPaused())
	{
		Restart();
		return true;
	}
	const FSample Now{W->GetTimeSeconds(), FPlatformTime::Seconds()};
	if (Window.Num() > 0)
	{
		const FSample& Last = Window.Last();
		// A gap, or the clock going back (a new world, a reset): start over.
		if (Now.Wall - Last.Wall > MaxGapS || Now.Sim < Last.Sim) Restart();
		// Several ticks in one frame (the core ticker can run without the world
		// ticking): keep the first.
		else if (Now.Sim == Last.Sim) return true;
	}
	Window.Add(Now);
	int32 Drop = 0;
	while (Drop < Window.Num() - 2 && Now.Wall - Window[Drop + 1].Wall >= WindowS) ++Drop;
	if (Drop > 0) Window.RemoveAt(0, Drop, EAllowShrinking::No);

	const FSample& First = Window[0];
	const double Wall = Now.Wall - First.Wall;
	if (Window.Num() < 3 || Wall < MinSpanS) return true;
	FactorValue.store((Now.Sim - First.Sim) / Wall);
	FpsValue.store((Window.Num() - 1) / Wall);
	return true;
}
