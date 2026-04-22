#pragma once

#include "CoreMinimal.h"

/**
 * Pacejka Magic Formula '96 coefficients (pure lateral + pure longitudinal).
 *
 * MF: F(x) = D * sin( C * atan( B*x - E*(B*x - atan(B*x)) ) )
 *   B — stiffness factor   (sets initial slope = B*C*D)
 *   C — shape factor       (controls peak width)
 *   D — peak value         (= TireMu for normalized-to-friction output)
 *   E — curvature factor   (< 0 keeps post-peak softer for slicks)
 *
 * Defaults are estimated from TTC open-wheel category data for a
 * Hoosier 16.0x7.5-10 R20 slick.  Tune via settings.json → Pacejka block.
 */
struct FFSDSPacejkaCoeffs
{
	// --- Lateral (cornering) ---
	float LatB = 10.0f;   // stiffness  (1/rad) — sets cornering stiffness slope
	float LatC =  1.9f;   // shape      — controls how quickly peak falls off
	float LatE = -1.5f;   // curvature  — negative = gradual post-peak falloff

	// --- Longitudinal (drive / brake) ---
	float LonB = 12.0f;   // stiffness  — longitudinal slip stiffness
	float LonC =  1.7f;   // shape
	float LonE = -0.5f;   // curvature
};

/**
 * Stateless helpers for evaluating the Pacejka formula and baking the
 * resulting curves into Chaos vehicle wheel slip graphs.
 */
namespace FSDSPacejka
{
	/**
	 * Evaluate lateral friction coefficient at the given slip angle.
	 * @param C         Pacejka coefficients
	 * @param SlipDeg   Slip angle in degrees (|α|, always non-negative)
	 * @param PeakMu    Peak friction coefficient (= TireMu from settings)
	 * @return          Friction coefficient in [0, PeakMu]
	 */
	float EvalLateral(const FFSDSPacejkaCoeffs& C, float SlipDeg, float PeakMu);

	/**
	 * Evaluate longitudinal friction coefficient at the given slip ratio.
	 * @param C          Pacejka coefficients
	 * @param SlipRatio  Slip ratio κ in [0, 1]
	 * @param PeakMu     Peak friction coefficient
	 * @return           Friction coefficient in [0, PeakMu]
	 */
	float EvalLongitudinal(const FFSDSPacejkaCoeffs& C, float SlipRatio, float PeakMu);

	/**
	 * Bake Pacejka curves into a Chaos wheel's LateralSlipGraph and
	 * LongitudinalSlipGraph, replacing the flat FrictionForceMultiplier model.
	 *
	 * LateralSlipGraph    — X: slip angle (degrees), Y: friction coefficient
	 * LongitudinalSlipGraph — X: slip ratio [0,1],   Y: friction coefficient
	 *
	 * FrictionForceMultiplier is also set to PeakMu so the wheel falls back
	 * correctly if Chaos ignores the slip graphs for any reason.
	 */
	void BakeToWheel(class UChaosVehicleWheel* Wheel,
	                 const FFSDSPacejkaCoeffs& C,
	                 float PeakMu);
}
