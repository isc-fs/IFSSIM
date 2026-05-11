#pragma once

#include "CoreMinimal.h"
#include "ChaosVehicleWheel.h"

/**
 * Pacejka Magic Formula '96 lateral coefficients.
 *
 * MF: F(α) = D * sin( C * atan( B*α - E*(B*α - atan(B*α)) ) )
 *   B — stiffness factor   (sets initial cornering stiffness slope = B*C*D)
 *   C — shape factor       (controls peak width)
 *   D — peak value         (= TireMu, set via settings.json)
 *   E — curvature factor   (< 0 = gradual post-peak falloff, good for slicks)
 *
 * Note: longitudinal slip graph requires a source-built UE5 engine; the
 * binary Epic Games Launcher install only exposes LateralSlipGraph on
 * UChaosVehicleWheel. Longitudinal grip is capped by FrictionForceMultiplier.
 *
 * Tune via settings.json → VehiclePhysics → Pacejka block.
 */
struct FFSDSPacejkaCoeffs
{
	float LatB = 10.0f;
	float LatC =  1.9f;
	float LatE = -1.5f;

	// Kept for forward-compatibility if longitudinal graph is added later
	float LonB = 12.0f;
	float LonC =  1.7f;
	float LonE = -0.5f;
};

namespace FSDSPacejka
{
	// Evaluate lateral friction coefficient at slip angle in degrees.
	float EvalLateral(const FFSDSPacejkaCoeffs& C, float SlipDeg, float PeakMu);

	// Bake LateralSlipGraph and set FrictionForceMultiplier on the given wheel.
	void BakeToWheel(UChaosVehicleWheel* Wheel,
	                 const FFSDSPacejkaCoeffs& C,
	                 float PeakMu);
}
