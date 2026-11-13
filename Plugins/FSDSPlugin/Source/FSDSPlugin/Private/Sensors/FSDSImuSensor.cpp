#include "Sensors/FSDSImuSensor.h"
#include "FSDSRandom.h"
#include "Misc/Parse.h"
#include "Misc/CommandLine.h"
#include "FSDSSensorNoise.h"
#include "FSDSVehiclePawn.h"

using FSDSNoise::RandStandardNormal;
using FSDSNoise::StepOrnsteinUhlenbeck;

UFSDSImuSensor::UFSDSImuSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSImuSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	if (!Owner) return;

	AFSDSVehiclePawn* Pawn = Cast<AFSDSVehiclePawn>(Owner);
	const FFSDSPlantOutput* Plant = Pawn ? &Pawn->GetPlantState() : nullptr;

	// PLANT FIRST, and REFUSE to publish if it failed.
	//
	// The previous guard was `if (RootPrim && RootPrim->IsSimulatingPhysics())`
	// wrapped around the gyro only, with the accelerometer outside it entirely.
	// A dead plant therefore published a well-formed, correctly-covarianced
	// ZERO at full rate with nothing in the log — indistinguishable downstream
	// from a stationary car. Silence is safer than a confident lie.
	if (!Plant || !Plant->bPlantOk)
	{
		UE_LOG(LogTemp, Error,
			TEXT("FSDS IMU: plant state unavailable — not publishing. A zeroed IMU "
			     "looks exactly like a stationary car to the EKF."));
		return;
	}

	// GYRO truth — body-frame angular velocity from the plant, converted back to
	// the UE wire convention.
	//
	// Angular velocity is an AXIAL vector, so the map is (x,y,z) -> (-x,y,-z),
	// NOT the (x,-y,z) used for velocities. The rule is an involution, so the
	// same expression converts in both directions. Using the polar rule here
	// mirrors yaw, which SLAM then integrates into a mirrored attitude — this
	// project has shipped that bug before.
	const FVector TruthGyro(
		-Plant->OmegaBody[0],
		 Plant->OmegaBody[1],
		-Plant->OmegaBody[2]);

	// ACCELEROMETER truth — PROPER acceleration from the plant.
	//
	// The previous code differenced a WORLD velocity and then rotated it,
	// dropping the Coriolis term (-omega x v) that body-frame rates require,
	// and added gravity as a world-Z constant. That missing term was the
	// documented root cause of lateral velocity drifting during sustained
	// cornering; the plant computes proper acceleration correctly.
	//
	// UNIT ROUND-TRIP, and it is deliberate: the plant works in m/s^2, while
	// this sensor's contract with the frame packers is cm/s^2 (they divide by
	// 100 at FSDSUdpBroadcaster.cpp and FSDSRpcServer.cpp). Converting up here
	// keeps the wire format unchanged. Removing BOTH conversions is a separate,
	// explicit change — doing half of it silently rescales every accelerometer
	// reading by 100. Every accel noise, bias, LSB and range below is in cm/s^2
	// for the same reason (the pawn scales the m/s^2 settings by 100).
	const FVector TruthAccelCm(
		 Plant->AccelProper[0] * 100.0,
		-Plant->AccelProper[1] * 100.0,
		 Plant->AccelProper[2] * 100.0);

	const FQuat TickOrientation = Owner->GetActorQuat();
	PreviousVelocity = Owner->GetVelocity();   // kept for the reset-spike guard

	UWorld* World = GetWorld();
	const double NowSec = World ? World->GetTimeSeconds() : 0.0;

	// A new session (the sim clock went back) restarts the sample grid; the
	// noise stream re-seeds itself on the generation bump (Noise()). So does a
	// gap of more than a quarter second (the plant was down): interpolating
	// across it would invent a trajectory, so the grid restarts at this tick.
	if (bHavePrevTruth && (NowSec < PrevTruthTimeSec || NowSec - PrevTruthTimeSec > 0.25))
	{
		ResetSampleClock();
	}

	if (SampleRateHz <= 0.f)
	{
		// Legacy: one sample per tick, stamped with the tick.
		if (DeltaTime > 0.f)
		{
			EmitSample(NowSec, TruthAccelCm, TruthGyro, TickOrientation, DeltaTime, true);
		}
	}
	else
	{
		// Emit every slot of the sample grid that fell inside this tick:
		// (previous tick, this tick]. The truth for a slot is interpolated
		// between the two ticks' plant states, so the stream is continuous;
		// each sample gets its own noise draw.
		const double SamplePeriod = 1.0 / SampleRateHz;
		const int64 NowSlot = FMath::FloorToInt64(NowSec / SamplePeriod + 1e-9);
		// No more than one second of samples per tick, e.g. after a pause or
		// a hitch: the stream would drop them anyway.
		const int64 MaxSlotsPerTick = FMath::Max<int64>(1, FMath::CeilToInt64(SampleRateHz));
		int64 FirstSlot = (LastSampleSlot < 0) ? NowSlot : LastSampleSlot + 1;
		FirstSlot = FMath::Max(FirstSlot, NowSlot - MaxSlotsPerTick + 1);

		const int32 AccelDivider = (AccelOdrHz > 0.f)
			? FMath::Max(1, FMath::RoundToInt(SampleRateHz / AccelOdrHz))
			: 1;
		const double TickSpan = NowSec - PrevTruthTimeSec;

		for (int64 SampleSlot = FirstSlot; SampleSlot <= NowSlot; ++SampleSlot)
		{
			const double SlotSec = double(SampleSlot) * SamplePeriod;
			FVector SlotAccelCm = TruthAccelCm;
			FVector SlotGyro = TruthGyro;
			if (bHavePrevTruth && TickSpan > 0.0)
			{
				const double SlotFrac = FMath::Clamp((SlotSec - PrevTruthTimeSec) / TickSpan, 0.0, 1.0);
				SlotAccelCm = FMath::Lerp(PrevTruthAccelCm, TruthAccelCm, SlotFrac);
				SlotGyro = FMath::Lerp(PrevTruthGyro, TruthGyro, SlotFrac);
			}
			// uDV reads the accel every 400 Hz wake, but its 100 Hz ODR only
			// refreshes the data registers every 4th read: the car's /imu
			// repeats each accel value 4 times. Same here, on the slot grid.
			const bool bAccelUpdate = !bHaveHeldAccel || (SampleSlot % AccelDivider) == 0;
			EmitSample(SlotSec, SlotAccelCm, SlotGyro, TickOrientation, float(SamplePeriod), bAccelUpdate);
		}
		if (NowSlot >= FirstSlot)
		{
			LastSampleSlot = NowSlot;
		}
	}

	bHavePrevTruth = true;
	PrevTruthTimeSec = NowSec;
	PrevTruthAccelCm = TruthAccelCm;
	PrevTruthGyro = TruthGyro;
}

void UFSDSImuSensor::ResetSampleClock()
{
	PendingSamples.Reset();
	LastSampleSlot = -1;
	bHavePrevTruth = false;
	PrevTruthTimeSec = 0.0;
	bHaveHeldAccel = false;
}

void UFSDSImuSensor::ConsumeSamples(TArray<FImuOutput>& Out)
{
	Out.Append(PendingSamples);
	PendingSamples.Reset();
}

namespace FSDSImuSensorPrivate
{
	/** Clamp to ±Range (0 = unlimited), then round to a multiple of Lsb (0 = none). */
	static double Digitise(double Value, double Lsb, double Range)
	{
		if (Range > 0.0) Value = FMath::Clamp(Value, -Range, Range);
		if (Lsb > 0.0) Value = FMath::RoundToDouble(Value / Lsb) * Lsb;
		return Value;
	}
}

void UFSDSImuSensor::EmitSample(double SampleTimeSec, const FVector& TrueAccelCm, const FVector& TrueGyro,
	const FQuat& SampleOrientation, float SampleDt, bool bAccelUpdate)
{
	using FSDSImuSensorPrivate::Digitise;

	// Noise() first: it re-seeds on a new session and draws that session's
	// turn-on offsets, which must be in place before this sample uses them.
	FRandomStream& ImuStream = Noise();

	// UNIT CONVERSION — the accelerometer only. Noise/bias settings are SI
	// (m/s^2) and the signal is cm/s^2 here (see the round-trip note in
	// TickComponent). Applying an m/s^2 sigma to the cm/s^2 signal made the
	// published noise 100x too small once (measured 0.00024 against 0.024).
	// The gyro needs no equivalent: rad/s is already the published unit.
	constexpr float AccelMToCm = 100.f;

	// Bias drift — Ornstein–Uhlenbeck, stepped per sample. Bounded long-run stddev.
	if (SampleDt > 0.f)
	{
		if (AccelBiasStd > 0.f)
		{
			const float AccelBiasStdCm = AccelBiasStd * AccelMToCm;
			AccelBias.X = StepOrnsteinUhlenbeck(ImuStream, AccelBias.X, SampleDt, AccelBiasTau, AccelBiasStdCm);
			AccelBias.Y = StepOrnsteinUhlenbeck(ImuStream, AccelBias.Y, SampleDt, AccelBiasTau, AccelBiasStdCm);
			AccelBias.Z = StepOrnsteinUhlenbeck(ImuStream, AccelBias.Z, SampleDt, AccelBiasTau, AccelBiasStdCm);
		}
		if (GyroBiasStd > 0.f)
		{
			GyroBias.X = StepOrnsteinUhlenbeck(ImuStream, GyroBias.X, SampleDt, GyroBiasTau, GyroBiasStd);
			GyroBias.Y = StepOrnsteinUhlenbeck(ImuStream, GyroBias.Y, SampleDt, GyroBiasTau, GyroBiasStd);
			GyroBias.Z = StepOrnsteinUhlenbeck(ImuStream, GyroBias.Z, SampleDt, GyroBiasTau, GyroBiasStd);
		}
	}

	FImuOutput Output;
	Output.Timestamp = FPlatformTime::Cycles64();
	Output.SimTimeSec = SampleTimeSec;
	Output.Orientation = SampleOrientation;

	// Gyro: every sample. Per-axis σ are magnitudes, so the UE/REP-103 sign
	// flips between the two conventions do not change which σ goes where.
	const FVector GyroSigma = GyroNoiseStdXYZ.IsNearlyZero() ? FVector(GyroNoiseStd) : GyroNoiseStdXYZ;
	const FVector GyroNoisy = TrueGyro + GyroBias + GyroTurnOnBias + FVector(
		GyroSigma.X * RandStandardNormal(ImuStream),
		GyroSigma.Y * RandStandardNormal(ImuStream),
		GyroSigma.Z * RandStandardNormal(ImuStream));
	Output.AngularVelocity = FVector(
		Digitise(GyroNoisy.X, GyroLsb, GyroRange),
		Digitise(GyroNoisy.Y, GyroLsb, GyroRange),
		Digitise(GyroNoisy.Z, GyroLsb, GyroRange));

	// Accel: only when its data registers refresh; held in between.
	if (bAccelUpdate)
	{
		const FVector AccelSigmaCm =
			(AccelNoiseStdXYZ.IsNearlyZero() ? FVector(AccelNoiseStd) : AccelNoiseStdXYZ) * AccelMToCm;
		const FVector AccelNoisyCm = TrueAccelCm + AccelBias + AccelTurnOnBias + FVector(
			AccelSigmaCm.X * RandStandardNormal(ImuStream),
			AccelSigmaCm.Y * RandStandardNormal(ImuStream),
			AccelSigmaCm.Z * RandStandardNormal(ImuStream));
		HeldAccelCm = FVector(
			Digitise(AccelNoisyCm.X, AccelLsb, AccelRange),
			Digitise(AccelNoisyCm.Y, AccelLsb, AccelRange),
			Digitise(AccelNoisyCm.Z, AccelLsb, AccelRange));
		bHaveHeldAccel = true;
	}
	Output.LinearAcceleration = HeldAccelCm;

	CachedOutput = Output;

	// Bounded: if nobody drains the queue (no sensor stream), keep two seconds.
	const int32 MaxPending = FMath::Max(1, FMath::CeilToInt(FMath::Max(SampleRateHz, 1.f) * 2.f));
	if (PendingSamples.Num() >= MaxPending)
	{
		PendingSamples.RemoveAt(0, PendingSamples.Num() - MaxPending + 1, EAllowShrinking::No);
	}
	PendingSamples.Add(Output);
}

FRandomStream& UFSDSImuSensor::Noise()
{
	// DETERMINISM PROBE (fsds.probeNoise). Logs the first N raw draws straight
	// from this sensor's stream, BEFORE they touch physics, the bridge, or the
	// republishing that duplicates samples and flattens timestamps.
	//
	// This exists because /imu turned out to be unusable for answering "does the
	// same seed reproduce the same noise": it mixes signal with noise, arrives
	// resampled with duplicates, and its header stamps neither advance per
	// sample nor reset across a PIE restart. Three attempts to measure
	// determinism downstream all failed on the observable, not on the sim.
	//
	// Compare two fresh sessions: identical lines => seeding reproduces.
	// Enable with -fsds.probeNoise on the command line, or by flipping the
	// default below for a quick local check.
	static bool bProbe = FParse::Param(FCommandLine::Get(), TEXT("fsds.probeNoise"));
	if (bProbe && ProbeDrawsLogged < 8)
	{
		// Peek WITHOUT consuming: copy the stream, draw from the copy. Drawing
		// from the live stream here would change the sequence the sensor then
		// uses, i.e. the probe would alter what it is measuring.
		FRandomStream Peek = (NoiseStreamGeneration == FSDSRandom::GetGeneration())
			? NoiseStream
			: FSDSRandom::MakeStream(TEXT("Imu.noise"));
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS PROBE imu.noise[%d] seed=%d gen=%u draw=%.9f"),
			ProbeDrawsLogged, FSDSRandom::GetScenarioSeed(),
			FSDSRandom::GetGeneration(), Peek.GetFraction());
		++ProbeDrawsLogged;
	}

	// Re-seed on generation change, not just once: a scenario reset must
	// restart the sequence, otherwise run 2 continues run 1 from wherever it
	// happened to stop. Generation 0 means the seed has not been set yet.
	const uint32 Gen = FSDSRandom::GetGeneration();
	if (NoiseStreamGeneration != Gen)
	{
		NoiseStream = FSDSRandom::MakeStream(TEXT("Imu.noise"));
		NoiseStreamGeneration = Gen;
		// Bias is persistent O-U state, not just a draw. Leaving it across a
		// reset would start the repeat mid-drift with the previous run's
		// accumulated bias — the run would not be a repeat.
		AccelBias = FVector::ZeroVector;
		GyroBias  = FVector::ZeroVector;
		// Turn-on offsets: one constant draw per session. The BMI088's zero
		// offset changes from one power-up to the next (and with range and
		// temperature); the car's /imu is not bias-corrected (uDV applies its
		// boot calibration only to its own attitude filter), so the pipeline
		// sees it raw. AccelTurnOnBiasStd is already cm/s^2 (set by the pawn).
		AccelTurnOnBias = FVector(
			AccelTurnOnBiasStd * RandStandardNormal(NoiseStream),
			AccelTurnOnBiasStd * RandStandardNormal(NoiseStream),
			AccelTurnOnBiasStd * RandStandardNormal(NoiseStream));
		GyroTurnOnBias = FVector(
			GyroTurnOnBiasStd * RandStandardNormal(NoiseStream),
			GyroTurnOnBiasStd * RandStandardNormal(NoiseStream),
			GyroTurnOnBiasStd * RandStandardNormal(NoiseStream));
	}
	return NoiseStream;
}
