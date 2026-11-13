#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"

#include <atomic>

class AFSDSVehiclePawn;
class UWorld;

/**
 * Holds a frame to a fixed period of real time: Wait() sleeps until one
 * period after the last frame's end, so the rate holds on average instead of
 * drifting with each frame's own time. A frame that already took longer than
 * a period isn't made up for: the rate can only be capped, never raised.
 */
struct FSDSPLUGIN_API FFSDSFramePacer
{
	void Wait(double PeriodS);
	void Reset() { Deadline = 0.0; }

private:
	double Deadline = 0.0;   // FPlatformTime::Seconds() the current frame should end at
};

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
 * It also holds the speed the simulation should run at while the pipeline
 * drives (docs/sim_timing.md):
 * - 1, real time (the default), for testing the car's computer: the factor
 *   should stay at 1, and the HUD warns when it doesn't;
 * - slow motion (0.5, 0.25, 0.1), for comparing algorithms: frames are spaced
 *   out so the simulation runs at that fraction of real time, and the pipeline
 *   gets 2, 4 or 10 times its real time per simulated second. Every frame is
 *   still the same fixed physics step, and sensors run on simulation time, so
 *   only the pipeline's time budget changes.
 * The speed applies only while the pipeline has the car (API control); manual
 * driving is always real time.
 *
 * Sampled on the core ticker, which runs while the game is paused; a pause or
 * a long hitch starts the window over, so neither reads as a slow sim. Read
 * from any thread (the HUD on the game thread, getSimStatus on the RPC one).
 */
class FSDSPLUGIN_API FFSDSRealTime
{
public:
	static FFSDSRealTime& Get();

	/** Start sampling World's clock (and pacing while Pawn is under API control); Stop when the world goes. */
	void Start(UWorld* InWorld, AFSDSVehiclePawn* InPawn);
	void Stop();

	/** Simulated seconds per real second over the window; 0 while unknown (paused, starting). */
	double Factor() const { return FactorValue.load(); }
	/** Frames per real second over the window; 0 while unknown. */
	double Fps() const { return FpsValue.load(); }
	/** Whether the factor is known, i.e. the simulation is running and has been for a moment. */
	bool IsKnown() const { return FactorValue.load() > 0.0; }

	/** The speed while the pipeline drives: 1 for real time, or a slow motion. */
	double SpeedFactor() const { return Speed.load(); }
	/** Clamped to [MinSpeed, 1]. Applies from the next frame. */
	void SetSpeedFactor(double InSpeed);
	/** Whether frames are being spaced out now: a slow motion, and the pipeline has the car. */
	bool IsPacing() const { return bPacing.load(); }
	/** The factor the simulation should reach now: the slow motion while pacing, else 1. */
	double TargetFactor() const { return IsPacing() ? SpeedFactor() : 1.0; }

	/** The slow motions the menu offers; any value in [MinSpeed, 1] works over RPC. */
	static constexpr double MinSpeed = 0.05;
	/** At or above this share of the target, a run's timing results hold (the HUD shows it plainly). */
	static constexpr double GoodFactor = 0.95;
	/** Below this share the HUD shows red: the pipeline gets a quarter more time, or more. */
	static constexpr double PoorFactor = 0.8;

private:
	bool Tick(float DeltaTime);
	void Restart();

	struct FSample { double Sim; double Wall; };

	TWeakObjectPtr<UWorld> World;
	TWeakObjectPtr<AFSDSVehiclePawn> Pawn;
	FTSTicker::FDelegateHandle TickHandle;
	TArray<FSample> Window;   // oldest first, at most WindowS of real time
	std::atomic<double> FactorValue{0.0};
	std::atomic<double> FpsValue{0.0};
	std::atomic<double> Speed{1.0};
	std::atomic<bool> bPacing{false};
	FFSDSFramePacer Pacer;
};
