#pragma once

#include "CoreMinimal.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "FSDSWheeledVehicleMovementComponent.generated.h"

/**
 * Chaos wheeled vehicle movement component that can push wheel configuration
 * to the physics thread AFTER the vehicle has been created.
 *
 * WHY THIS EXISTS
 * ---------------
 * Chaos builds each physics-side wheel from the wheel class's CLASS DEFAULT
 * OBJECT, not from the per-instance UChaosVehicleWheel objects:
 *
 *   ChaosWheeledVehicleMovementComponent.cpp:1412
 *     UChaosVehicleWheel* Wheel = WheelSetups[WheelIdx].WheelClass.GetDefaultObject();
 *
 * That runs inside CreateVehicle() from OnCreatePhysicsState() — at component
 * registration, BEFORE BeginPlay. So anything written to Wheels[i] afterwards
 * (which is where AFSDSVehiclePawn applies settings.json: TireMu, the Pacejka
 * bake, MaxBrakeTorque, radius, steer limit) lands on an object the solver
 * never reads.
 *
 * That is why much of settings.json's VehiclePhysics block behaved as
 * decoration. It was never unsupported by Chaos — it was written to the wrong
 * object at the wrong time.
 *
 * TWO TIERS, DELIBERATELY
 * -----------------------
 * Tier 1 - ApplyWheelConfigToPhysics(). Uses the engine's PUBLIC per-field
 * setters (SetWheelRadius, SetWheelFrictionMultiplier, SetWheelMaxBrakeTorque,
 * SetWheelMaxSteerAngle, SetSuspensionParams). Each writes straight into
 * VehicleSimulationPT->PVehicle->Wheels[i] under the correct physics-write
 * guard — verified in engine source. This path uses only public API, cannot
 * leave the wheel half-initialised, and covers most of what settings.json
 * actually sets. Prefer it.
 *
 * Tier 2 - PushFullWheelConfigToPhysics(). Re-runs InitializeWheel /
 * InitializeSuspension, mirroring SetWheelClass (engine :2330-2355) without
 * swapping the wheel object. Needed ONLY for fields with no per-field setter,
 * principally the baked Pacejka LateralSlipGraph (the public API exposes a
 * scalar multiplier on the curve, not the curve itself) plus wheel width, mass
 * and cornering stiffness.
 *
 * Tier 2 carries real risk: FillWheelSetup computes TorqueRatio "later after
 * all wheel info is known", so a partial re-init may leave derived state
 * stale. It is isolated here so that if it misbehaves, Tier 1 still stands on
 * its own and the fallback is to bake the slip graph onto the CDO in the
 * wheel class constructor instead.
 *
 * VERIFY, DO NOT ASSUME
 * ---------------------
 * The failure mode this guards against is silent: config that never reaches
 * physics yields a car that drives, just not the car that was configured.
 * VerifyWheelConfigApplied() reads the solver back and logs an Error on
 * mismatch, so a regression is loud rather than a season of confusing data.
 */
UCLASS()
class FSDSPLUGIN_API UFSDSWheeledVehicleMovementComponent : public UChaosWheeledVehicleMovementComponent
{
	GENERATED_BODY()

public:
	/**
	 * Tier 1: push every field that has a public per-field setter from
	 * Wheels[WheelIndex] into the solver. Safe, public API only.
	 *
	 * @return true if the physics body and simulation existed to write to.
	 */
	bool ApplyWheelConfigToPhysics(int32 WheelIndex);

	/**
	 * Tier 2: full re-initialisation of the wheel + suspension config, which
	 * additionally carries the baked slip curve. See the class comment for the
	 * risk this takes on.
	 */
	bool PushFullWheelConfigToPhysics(int32 WheelIndex);

	/**
	 * Apply to every wheel. When bFullReinit is true each wheel also goes
	 * through Tier 2.
	 *
	 * @return number of wheels successfully written.
	 */
	int32 ApplyAllWheelConfigsToPhysics(bool bFullReinit = false);

	/**
	 * Read the solver's live wheel state back and compare against the
	 * game-thread Wheels[WheelIndex]. Logs Error listing each mismatched field.
	 *
	 * @return true when the solver agrees with the configuration.
	 */
	bool VerifyWheelConfigApplied(int32 WheelIndex, float Tolerance = 0.5f) const;

	/** VerifyWheelConfigApplied across all wheels; logs a single summary. */
	bool VerifyAllWheelConfigsApplied(float Tolerance = 0.5f) const;

	/**
	 * Log every wheel parameter this project never explicitly chose.
	 *
	 * Chaos's wheel class ships twelve tuning fields that FSDSWheelFront/Rear
	 * do not set, so they run at UChaosVehicleWheel's constructor defaults —
	 * values picked for arcade game handling, not for an IFS-08. They are
	 * invisible in this repo precisely BECAUSE they are absent from it: you
	 * cannot grep for a value that is never written.
	 *
	 * Two of them are physically significant and neither was a decision:
	 *   RollbarScaling      0.15  — a live anti-roll bar
	 *   MaxHandBrakeTorque  3000 N.m/wheel — and the handbrake channel IS the
	 *                       EBS on this car, so this is the number the entire
	 *                       emergency-braking case rests on
	 *
	 * This logs actual runtime values rather than changing them. Picking real
	 * ones needs IFS-08 numbers the simulator does not have, and inventing them
	 * would repeat the mistake that produced the 2.3x spring-rate divergence.
	 * Making them visible is the prerequisite for choosing them.
	 */
	void LogInheritedWheelDefaults() const;
};
