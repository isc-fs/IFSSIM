#include "FSDSRealTime.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "FSDSSettings.h"
#include "FSDSVehiclePawn.h"
#include "HAL/PlatformProcess.h"
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
	// The pacer sleeps until this close to the deadline, then yields: a sleep
	// can wake a little late, and a late frame is time the rate can't win back.
	constexpr double SpinS = 0.0015;
}

void FFSDSFramePacer::Wait(double PeriodS)
{
	double Now = FPlatformTime::Seconds();
	// The first frame, or one that took more than a period already: start
	// the count from here rather than wait, or try to catch up.
	if (Deadline <= 0.0 || Now > Deadline + PeriodS)
	{
		Deadline = Now;
		return;
	}
	Deadline += PeriodS;
	for (;;)
	{
		Now = FPlatformTime::Seconds();
		const double Left = Deadline - Now;
		if (Left <= 0.0) break;
		if (Left > SpinS) FPlatformProcess::SleepNoStats((float)(Left - SpinS));
		else FPlatformProcess::YieldThread();
	}
}

FFSDSRealTime& FFSDSRealTime::Get()
{
	static FFSDSRealTime Instance;
	return Instance;
}

void FFSDSRealTime::Start(UWorld* InWorld, AFSDSVehiclePawn* InPawn)
{
	Stop();
	World = InWorld;
	Pawn = InPawn;
	SetSpeedFactor(FFSDSSettings::Get().TimingSpeedFactor);
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
	Pawn.Reset();
	bPacing.store(false);
	Restart();
}

void FFSDSRealTime::SetSpeedFactor(double InSpeed)
{
	const double S = FMath::Clamp(InSpeed, MinSpeed, 1.0);
	if (S == Speed.load()) return;
	Speed.store(S);
	UE_LOG(LogTemp, Log, TEXT("FSDS Timing: %s while the pipeline drives"),
		S >= 1.0 ? TEXT("real time") : *FString::Printf(TEXT("slow motion, %.2fx real time"), S));
}

void FFSDSRealTime::Restart()
{
	Window.Reset();
	FactorValue.store(0.0);
	FpsValue.store(0.0);
	Pacer.Reset();
}

bool FFSDSRealTime::Tick(float)
{
	const UWorld* W = World.Get();
	if (!W || W->IsPaused())
	{
		bPacing.store(false);
		Restart();
		return true;
	}

	// Slow motion: space the frames out while the pipeline has the car.
	const AFSDSVehiclePawn* P = Pawn.Get();
	const double S = Speed.load();
	const bool bPace = S < 1.0 && P && P->IsApiControlEnabled();
	if (bPace != bPacing.load())
	{
		bPacing.store(bPace);
		Restart();   // a new target: measure afresh
	}
	if (bPace)
	{
		const double StepS = GEngine && GEngine->bUseFixedFrameRate && GEngine->FixedFrameRate > 0.f
			? 1.0 / GEngine->FixedFrameRate : 1.0 / 60.0;
		Pacer.Wait(StepS / S);
	}

	const FSample Now{W->GetTimeSeconds(), FPlatformTime::Seconds()};
	if (Window.Num() > 0)
	{
		const FSample& Last = Window.Last();
		// A gap, or the clock going back (a new world, a reset): start over.
		// Slow motion's own gaps are expected: allow up to a 0.05x frame.
		const double MaxGap = bPace ? FMath::Max(MaxGapS, 2.0 / (60.0 * S)) : MaxGapS;
		if (Now.Wall - Last.Wall > MaxGap || Now.Sim < Last.Sim) Restart();
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
