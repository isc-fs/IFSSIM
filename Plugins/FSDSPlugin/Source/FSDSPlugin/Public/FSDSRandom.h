#pragma once

#include "CoreMinimal.h"
#include "Math/RandomStream.h"

/**
 * Deterministic randomness for the simulator.
 *
 * WHY
 * ---
 * Everything stochastic in the sim used process-global randomness:
 * FMath::FRand / FMath::RandRange for sensor noise and cone yaw, C stdlib
 * rand() in the map loader, and the GPU LiDAR path reseeding from
 * FPlatformTime::Cycles() on every scan. All of it is seeded by the clock, so
 * two runs of the same scenario were never the same run.
 *
 * For a platform whose purpose is comparing algorithms that is fatal: any
 * difference measured between two runs is a mixture of the change under test
 * and the noise draw, with no way to separate them.
 *
 * THE MODEL
 * ---------
 * One scenario seed, many independent streams. Each stream is seeded
 * `ScenarioSeed ^ Hash(name)`, so:
 *
 *   - a run is reproduced exactly by reusing the scenario seed;
 *   - streams are INDEPENDENT — adding a cone does not shift the LiDAR noise
 *     sequence, and enabling a sensor does not shift anyone else's. Sharing one
 *     stream would couple them, so an unrelated change would silently alter
 *     every noise draw and look like a real effect;
 *   - a deliberately different draw is one seed away, which is what makes
 *     N-seed repeats (and honest statistics) possible.
 *
 * USAGE
 * -----
 *   FRandomStream Stream = FSDSRandom::MakeStream(TEXT("Lidar1.noise"));
 *   const float N = FSDSRandom::StandardNormal(Stream);
 *
 * Name streams stably. Renaming a stream changes its draw sequence and breaks
 * comparability with previously recorded runs.
 */
namespace FSDSRandom
{
	/**
	 * Set the scenario seed for this run. Call once at startup, before any
	 * stream is made. Seed 0 is reserved to mean "non-reproducible": it makes
	 * MakeStream fall back to a clock-derived seed and logs a warning, so a run
	 * that cannot be reproduced says so rather than pretending.
	 */
	FSDSPLUGIN_API void SetScenarioSeed(int32 InSeed);

	/** The active scenario seed (0 when unseeded / non-reproducible). */
	FSDSPLUGIN_API int32 GetScenarioSeed();

	/** True when this run is reproducible from its seed. */
	FSDSPLUGIN_API bool IsDeterministic();

	/**
	 * Increments every time the scenario seed is (re)set.
	 *
	 * Streams are cached by their owners — a sensor seeds once and keeps
	 * drawing from that stream. On a scenario RESET those caches must be
	 * rebuilt, or run 2 continues run 1's sequence from wherever it stopped
	 * and is not a repeat at all. Owners store the generation they seeded for
	 * and re-seed when it changes.
	 */
	FSDSPLUGIN_API uint32 GetGeneration();

	/**
	 * A stream for a named source of randomness, derived from the scenario
	 * seed. Independent of every other stream.
	 */
	FSDSPLUGIN_API FRandomStream MakeStream(const FString& StreamName);

	/**
	 * A uint32 seed for a named stream — for consumers that need a raw seed
	 * rather than an FRandomStream, e.g. the GPU LiDAR shader uniform.
	 * `Salt` distinguishes successive uses (pass the scan counter) so a
	 * per-scan seed stays reproducible instead of coming from the clock.
	 */
	FSDSPLUGIN_API uint32 MakeSeed(const FString& StreamName, uint32 Salt = 0);

	/** One sample from N(0,1) via Box-Muller, drawn from `Stream`. */
	FSDSPLUGIN_API float StandardNormal(FRandomStream& Stream);
}
