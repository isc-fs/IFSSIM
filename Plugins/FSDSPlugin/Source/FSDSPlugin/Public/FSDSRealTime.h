#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"

#include <atomic>

class UWorld;

/**
 * How fast the simulation runs against the real clock: the real-time factor,
 * simulated seconds per real second, over the last couple of seconds.
 *
 * Every frame advances the simulation by exactly 1/FixedFrameRate
 * (Config/DefaultEngine.ini), so a frame the PC can't render in time makes the
 * simulation slower than real time instead of taking a bigger step. The
 * pipeline runs on the simulation's clock (/clock, use_sim_time) but computes
 * in real time, so below 1.0 it gets more time than it would on the car: a
 * run's timing results are only trustworthy near 1.0.
 *
 * Sampled on the core ticker, which runs while the game is paused; a pause or
 * a long hitch starts the window over, so neither reads as a slow sim. Read
 * from any thread (the HUD on the game thread, getSimStatus on the RPC one).
 */
class FSDSPLUGIN_API FFSDSRealTime
{
public:
	static FFSDSRealTime& Get();

	/** Start sampling World's clock; Stop when the world goes. */
	void Start(UWorld* InWorld);
	void Stop();

	/** Simulated seconds per real second over the window; 0 while unknown (paused, starting). */
	double Factor() const { return FactorValue.load(); }
	/** Frames per real second over the window; 0 while unknown. */
	double Fps() const { return FpsValue.load(); }
	/** Whether the factor is known, i.e. the simulation is running and has been for a moment. */
	bool IsKnown() const { return FactorValue.load() > 0.0; }

	/** At or above this, a run's timing results hold (the HUD shows it green). */
	static constexpr double GoodFactor = 0.95;
	/** Below this the HUD shows red: the pipeline gets a quarter more time, or more. */
	static constexpr double PoorFactor = 0.8;

private:
	bool Tick(float DeltaTime);
	void Restart();

	struct FSample { double Sim; double Wall; };

	TWeakObjectPtr<UWorld> World;
	FTSTicker::FDelegateHandle TickHandle;
	TArray<FSample> Window;   // oldest first, at most WindowS of real time
	std::atomic<double> FactorValue{0.0};
	std::atomic<double> FpsValue{0.0};
};
