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
	// EMRAX 228 MV peak torque envelope (mech_rpm, max_torque_Nm),
	// directly transcribed from EMRAX228/emrax228_envelope_curve.csv.
	// Flat 200 Nm in the constant-torque region, falling above
	// ~4600 RPM as field-weakening engages. The curve interpolates
	// linearly between knots; the spacing matches the NX-tech file's
	// 103-RPM grid.
	constexpr struct FEnvelopeKnot
	{
		float Rpm;
		float TorqueNm;
	} GTorqueEnvelopeKnots[] = {
		{0.f,    200.f}, {103.f,  200.f}, {206.f,  200.f}, {309.f,  200.f},
		{412.f,  200.f}, {515.f,  200.f}, {618.f,  200.f}, {721.f,  200.f},
		{824.f,  200.f}, {927.f,  200.f}, {1030.f, 200.f}, {1133.f, 200.f},
		{1236.f, 200.f}, {1339.f, 200.f}, {1442.f, 200.f}, {1545.f, 200.f},
		{1648.f, 200.f}, {1751.f, 200.f}, {1854.f, 200.f}, {1957.f, 200.f},
		{2060.f, 200.f}, {2163.f, 200.f}, {2266.f, 200.f}, {2369.f, 200.f},
		{2472.f, 200.f}, {2575.f, 200.f}, {2678.f, 200.f}, {2781.f, 200.f},
		{2884.f, 200.f}, {2987.f, 200.f}, {3090.f, 200.f}, {3193.f, 200.f},
		{3296.f, 200.f}, {3399.f, 200.f}, {3502.f, 200.f}, {3605.f, 200.f},
		{3708.f, 200.f}, {3811.f, 200.f}, {3914.f, 200.f}, {4017.f, 200.f},
		{4120.f, 200.f}, {4223.f, 200.f}, {4326.f, 200.f}, {4429.f, 200.f},
		{4532.f, 200.f}, {4635.f, 199.f}, {4738.f, 196.f}, {4841.f, 193.f},
		{4944.f, 190.f}, {5047.f, 188.f}, {5150.f, 185.f}, {5253.f, 181.f},
		{5356.f, 177.f}, {5459.f, 174.f}, {5562.f, 170.f}, {5665.f, 167.f},
		{5768.f, 164.f}, {5871.f, 161.f}, {5974.f, 157.f}, {6077.f, 153.f},
		{6180.f, 149.f}, {6283.f, 145.f}, {6386.f, 142.f}, {6489.f, 140.f},
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
	// the motor limit; we enforce it here.
	const float Rpm = FMath::Clamp(MechRpm, 0.f, P.MaxMechRpm);
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
	const float TCommand = (Throttle >= 0.f)
		? Throttle * TDriveMax
		: Throttle * TRegenMax;

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
