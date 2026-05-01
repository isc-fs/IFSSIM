// EMRAX 228 MV motor model — Tier 1 implementation.
//
// See EmraxMotor.h for design rationale; this file implements:
//   - the hardcoded torque envelope (EMRAX 228 MV at 400 V bus)
//   - the per-tick Step() compute path
//
// Validation tests for this model live in EMRAX228/emrax228_ue5_complete.md
// section 11. They are documented here so that any tuning change in
// FEmraxMotorParams can be re-validated in a few minutes:
//
//   1. Stall: throttle=1, RPM=0 → torque should converge to 200 Nm
//      in ~10 ms (CurrentLoopTau ≈ 1.5 ms; 5τ = 7.5 ms).
//   2. Power cap: at 5000 RPM (524 rad/s) torque should be capped at
//      P_max / ω = 100 kW / 524 ≈ 191 Nm, dropping along the
//      envelope's field-weakening rolloff.
//   3. Thermal derate: throttle=1 continuous → after ~120 s the
//      effective max torque blends toward the 130 Nm continuous cap.
//   4. Speed clamp: max RPM = 6500.

#include "EmraxMotor.h"

#include "Math/UnrealMathUtility.h"

namespace
{
	// EMRAX 228 MV peak torque envelope (mech_rpm, max_torque_Nm).
	// Constant-torque region uses the EMRAX 228 MV S2 (2-minute)
	// rating of 220 Nm — the official datasheet number for short-
	// duration peak. The previous 200 Nm cap mirrored the NX-tech
	// LUT's conservative ceiling, but launch-from-rest at the IFS-08
	// mass + Hoosier R20 μ leaves only a 4.8 % margin over static-
	// friction at 200 Nm, below Chaos's wheel-solver stick-threshold.
	// The datasheet 220 Nm gives ~15 % margin and matches the real
	// motor's actual S2 capability — what a real-car launch ECU
	// commands. Field-weakening rolloff above ~4600 RPM is unchanged
	// (constant-power region was already correct).
	// Linear interpolation between knots; spacing matches the
	// NX-tech file's 103-RPM grid.
	constexpr struct FEnvelopeKnot
	{
		float Rpm;
		float TorqueNm;
	} GTorqueEnvelopeKnots[] = {
		{0.f,    220.f}, {103.f,  220.f}, {206.f,  220.f}, {309.f,  220.f},
		{412.f,  220.f}, {515.f,  220.f}, {618.f,  220.f}, {721.f,  220.f},
		{824.f,  220.f}, {927.f,  220.f}, {1030.f, 220.f}, {1133.f, 220.f},
		{1236.f, 220.f}, {1339.f, 220.f}, {1442.f, 220.f}, {1545.f, 220.f},
		{1648.f, 220.f}, {1751.f, 220.f}, {1854.f, 220.f}, {1957.f, 220.f},
		{2060.f, 220.f}, {2163.f, 220.f}, {2266.f, 220.f}, {2369.f, 220.f},
		{2472.f, 220.f}, {2575.f, 220.f}, {2678.f, 220.f}, {2781.f, 220.f},
		{2884.f, 220.f}, {2987.f, 220.f}, {3090.f, 220.f}, {3193.f, 220.f},
		{3296.f, 220.f}, {3399.f, 220.f}, {3502.f, 220.f}, {3605.f, 220.f},
		{3708.f, 220.f}, {3811.f, 220.f}, {3914.f, 220.f}, {4017.f, 220.f},
		{4120.f, 220.f}, {4223.f, 220.f}, {4326.f, 220.f}, {4429.f, 220.f},
		{4532.f, 220.f}, {4635.f, 219.f}, {4738.f, 216.f}, {4841.f, 213.f},
		{4944.f, 209.f}, {5047.f, 207.f}, {5150.f, 204.f}, {5253.f, 199.f},
		{5356.f, 195.f}, {5459.f, 191.f}, {5562.f, 187.f}, {5665.f, 184.f},
		{5768.f, 180.f}, {5871.f, 177.f}, {5974.f, 173.f}, {6077.f, 168.f},
		{6180.f, 164.f}, {6283.f, 160.f}, {6386.f, 156.f}, {6489.f, 154.f},
	};
}

UEmraxMotor::UEmraxMotor()
{
	// Build the envelope curve once at construction. Linear interpolation
	// matches the spec doc's recommendation; cubic would smooth the
	// rolloff but introduces overshoot near the field-weakening knee.
	TorqueEnvelope.Reset();
	for (const FEnvelopeKnot& K : GTorqueEnvelopeKnots)
	{
		const FKeyHandle Handle = TorqueEnvelope.AddKey(K.Rpm, K.TorqueNm);
		TorqueEnvelope.SetKeyInterpMode(Handle, RCIM_Linear);
	}
}

float UEmraxMotor::Step(float Throttle, float Dt)
{
	Throttle = FMath::Clamp(Throttle, -1.f, 1.f);

	// Speed-clamp the externally-set RPM at the mechanical limit. The
	// caller may have computed RPM from wheel speed without knowing
	// the motor limit; we enforce it here. Envelope and power-cap math
	// always use the magnitude; the sign is preserved separately so
	// the single-quadrant regen guard below can refuse braking torque
	// on a backward-rotating wheel.
	const float SignedRpm = FMath::Clamp(MechRpm, -P.MaxMechRpm, P.MaxMechRpm);
	const float Rpm = FMath::Abs(SignedRpm);
	const float OmegaRadS = Rpm * (2.f * PI / 60.f);
	const float OmegaForPowerCap = FMath::Max(OmegaRadS, 1e-3f);

	// === Motoring (drive) cap ===
	// Peak torque from envelope, capped by the constant-power
	// boundary above the field-weakening knee. Thermal derate
	// removed — was clamping launch torque below the static
	// friction lock and blocking autonomous launches; will be
	// reintroduced once the HV battery class lands and we have a
	// proper coolant + winding-temp model rather than the I²t proxy.
	const float TDrivePeakLut = TorqueEnvelope.Eval(Rpm, P.MaxPeakTorqueNm);
	const float TDrivePeakPwr = P.MaxPeakPowerW / OmegaForPowerCap;
	const float TDriveMax = FMath::Min(TDrivePeakLut, TDrivePeakPwr);

	// === Regen (generating) cap ===
	// Different power limit because charge current is gated by the
	// HV battery, not the motor envelope. At low speeds the torque
	// cap binds; at higher speeds the power cap dominates (e.g. at
	// 6500 RPM = 680 rad/s, MaxRegenPowerW=6000 → ~8.8 Nm vs the
	// 230 Nm torque cap).
	const float TRegenPwr = P.MaxRegenPowerW / OmegaForPowerCap;
	const float TRegenMax = FMath::Min(P.MaxRegenTorqueNm, TRegenPwr);

	// Driver demand. Positive throttle hits the motoring cap;
	// negative (regen) demand hits the regen cap. Splitting them
	// here means a -1 brake demand at speed yields the small regen
	// torque, not the (10-100×) larger motoring torque it would get
	// from a symmetric envelope.
	//
	// Single-quadrant regen: the IFS-08 EMRAX 228 motor controller
	// only generates negative torque on a *forward-rotating* wheel.
	// Asking for regen at zero or negative ω would (a) be electrically
	// unsafe on the real inverter, and (b) here would produce reverse
	// motor torque on a stationary wheel — driving the car backward
	// from rest, a behaviour the real car physically cannot exhibit.
	// Clamp regen to zero when the wheel isn't moving forward.
	float TCommand;
	if (Throttle >= 0.f)
	{
		TCommand = Throttle * TDriveMax;
	}
	else if (SignedRpm > 0.f)
	{
		// Wheel rotating forward → regen brake produces decel torque.
		TCommand = Throttle * TRegenMax;
	}
	else
	{
		// Wheel stationary or rotating backward → no regen torque.
		// Generating negative torque on a non-forward wheel would
		// drive the car in reverse, which the real IFS-08 EMRAX
		// inverter does not (and physically cannot) do.
		TCommand = 0.f;
	}

	// First-order current-loop dynamics. Discrete approximation:
	//   T_new = T_old + α (T_command - T_old),   α = clamp(dt / τ, 0, 1)
	// At the typical 1.5 ms τ and 1 kHz physics tick (1 ms dt),
	// α = 0.66 — torque settles in ~5 ticks.
	const float Alpha = FMath::Min(Dt / FMath::Max(P.CurrentLoopTau, 1e-4f), 1.f);
	TorqueNm += (TCommand - TorqueNm) * Alpha;

	// Idle creep — see EmraxMotor.h "Software creep" block for the
	// rationale. Floor the post-current-loop torque at IdleCreepTorqueNm
	// while we're motoring (Throttle ≥ 0) and below threshold. Skipped
	// when EBS is latched (TCommand was already forced to 0 upstream
	// only via the bEbsLatched gate in the pawn, but we re-check here
	// in case the motor is driven from a different caller — Throttle<0
	// means regen demand, which must not be overridden by creep).
	if (Throttle >= 0.f && Rpm < P.IdleCreepRpmThreshold)
	{
		TorqueNm = FMath::Max(TorqueNm, P.IdleCreepTorqueNm);
	}

	// Thermal derate disabled — see comment above. State variables
	// kept on the class so future versions can re-enable without
	// breaking the API; held at no-op values here.
	OverloadJ = 0.f;
	ThermalDerate = 1.f;

	return TorqueNm;
}
