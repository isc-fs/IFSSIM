#include "FSDSRandom.h"

#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace
{
	int32  GScenarioSeed = 0;
	bool   GWarnedUnseeded = false;
	uint32 GGeneration = 0;   // bumped on every (re)seed; see GetGeneration()

	/**
	 * FNV-1a over the stream name. Any stable hash works; what matters is that
	 * it is deterministic across runs and platforms. FString::GetTypeHash is
	 * NOT usable here — it is not guaranteed stable between engine versions,
	 * which would silently change every stream's sequence on an engine upgrade.
	 */
	uint32 HashStreamName(const FString& Name)
	{
		uint32 Hash = 2166136261u;
		for (TCHAR C : Name)
		{
			Hash ^= static_cast<uint32>(C);
			Hash *= 16777619u;
		}
		return Hash;
	}
}

void FSDSRandom::SetScenarioSeed(int32 InSeed)
{
	// -fsds.seed=N overrides settings.json. This is what makes N-seed repeats
	// possible without editing config between runs — a batch runner sweeps the
	// seed on the command line and every run is otherwise byte-identical in
	// configuration, which is exactly the property a paired comparison needs.
	int32 CmdSeed = 0;
	if (FParse::Value(FCommandLine::Get(), TEXT("fsds.seed="), CmdSeed))
	{
		UE_LOG(LogTemp, Log,
			TEXT("FSDS: scenario seed overridden from the command line: %d (settings.json said %d)"),
			CmdSeed, InSeed);
		InSeed = CmdSeed;
	}

	GScenarioSeed = InSeed;
	GWarnedUnseeded = false;
	++GGeneration;

	if (InSeed != 0)
	{
		UE_LOG(LogTemp, Log,
			TEXT("FSDS: RNG seeded — scenario seed %d. This run is reproducible: "
				 "re-run with the same seed to get the same noise, dropouts and cone yaw."),
			InSeed);
	}
	else
	{
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS: RNG UNSEEDED (scenario seed 0) — noise, dropouts and cone yaw are "
				 "clock-derived. This run CANNOT be reproduced and must not be compared "
				 "against another. Set ScenarioSeed in settings.json."));
	}
}

int32 FSDSRandom::GetScenarioSeed()
{
	return GScenarioSeed;
}

bool FSDSRandom::IsDeterministic()
{
	return GScenarioSeed != 0;
}

uint32 FSDSRandom::GetGeneration()
{
	return GGeneration;
}

uint32 FSDSRandom::MakeSeed(const FString& StreamName, uint32 Salt)
{
	if (GScenarioSeed == 0)
	{
		// Unseeded: fall back to the clock so behaviour matches the old
		// non-reproducible default rather than every stream collapsing to the
		// same constant sequence — which would be a far more confusing bug.
		if (!GWarnedUnseeded)
		{
			GWarnedUnseeded = true;
			UE_LOG(LogTemp, Warning,
				TEXT("FSDS: stream '%s' requested with no scenario seed — using the clock. "
					 "Results are not reproducible."), *StreamName);
		}
		return static_cast<uint32>(FPlatformTime::Cycles()) ^ (Salt * 2654435761u);
	}

	// Mix the salt through the multiplier as well so successive salts land far
	// apart in the sequence rather than in adjacent, correlated states.
	return static_cast<uint32>(GScenarioSeed) ^ HashStreamName(StreamName) ^ (Salt * 2654435761u);
}

FRandomStream FSDSRandom::MakeStream(const FString& StreamName)
{
	return FRandomStream(static_cast<int32>(MakeSeed(StreamName)));
}

float FSDSRandom::StandardNormal(FRandomStream& Stream)
{
	// Box-Muller. Cap u1 away from zero so Loge stays finite on the unlucky
	// sample. Matches the previous FSDSNoise::RandStandardNormal exactly except
	// that the draws come from `Stream` instead of process-global state.
	const float u1 = FMath::Max(Stream.GetFraction(), 1e-7f);
	const float u2 = Stream.GetFraction();
	return FMath::Sqrt(-2.f * FMath::Loge(u1)) * FMath::Cos(2.f * PI * u2);
}
