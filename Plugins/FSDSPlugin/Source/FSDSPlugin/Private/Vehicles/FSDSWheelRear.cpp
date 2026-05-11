#include "Vehicles/FSDSWheelRear.h"

UFSDSWheelRear::UFSDSWheelRear()
{
	AxleType = EAxleType::Rear;
	bAffectedByHandbrake = true;
	bAffectedBySteering = false;

	// AFSDSVehiclePawn::Tick injects per-wheel drive torque from the
	// EMRAX motor model via VehicleMovement->SetDriveTorque(). That call
	// writes to ExternalDriveTorque on the wheel, which is *only*
	// applied if the wheel's combine method is Override or Additive —
	// the Chaos default (None) silently discards it. We use Additive
	// here (vs Override) so internal brake torques (handbrake / EBS)
	// still reach the wheel; we keep the engine's internal drive
	// torque effectively at zero by passing throttle=0 to
	// VehicleMovement->SetThrottleInput() in the pawn Tick.
	ExternalTorqueCombineMethod = ETorqueCombineMethod::Additive;

	// IFS-08: Hoosier 16.0x7.5-10 R20
	WheelRadius = 20.f;       // 200mm tire radius
	WheelWidth = 19.f;        // 7.5 inch = 190mm
	MaxSteerAngle = 0.f;

	// Per-wheel total mass — IFS-08 corner is ≈10 kg total (Hoosier R20
	// 16×7.5-10 ≈ 6 kg + 10″ Mg rim ≈ 3 kg + brake/hub residual ≈ 1 kg).
	// Halving the wheel mass to 5 kg was a numerical workaround to halve
	// rotational inertia at launch — reverted because the proper fix
	// for launch-from-rest belongs in the autonomy's launch state
	// machine (open-loop full throttle until v_launch_done), not in
	// fudged wheel physics that the real car would inherit.
	WheelMass = 10.f;

	// IFS-08: 35mm ride height, 300 lbs/in rear springs
	SuspensionMaxRaise = 3.5f;
	SuspensionMaxDrop = 3.5f;
	SuspensionDampingRatio = 1.5f;

	// Use the actual per-wheel vertical load (Fz) for grip calculation
	// instead of Chaos's default 50/50 blend with the resting load.
	// Chaos's ChaosWheeledVehicleMovementComponent.cpp:653 does:
	//   ForceMagnitude = WheelLoadRatio * ActualFz + (1-WheelLoadRatio) * RestingForce
	// At 0.5 (default), each wheel sees half its real Fz averaged with
	// the static one-quarter-mass restingForce. That's a Chaos arcade-
	// handling helper — at 1.0 it uses the true dynamic Fz, which is
	// what we want for a physics-truthful FS simulation. Load transfer
	// during launch then properly increases rear-axle grip.
	WheelLoadRatio = 1.0f;

	// Hoosier R20 slick friction — peak μ ≈ 1.45 on dry tarmac, 1.4 is
	// the conservative default we run against. Reverted from the 1.0
	// hack we tried during the launch-debug rabbit hole: lowering μ to
	// give the EMRAX margin past static friction was a sim-only trick
	// that the real car does not benefit from. The truthful answer to
	// launch-from-rest is the autonomy's launch state machine (open-
	// loop full throttle), not detuned tire grip.
	FrictionForceMultiplier = 1.4f;

	// Brake channel = motor regen (EMRAX 228 on rear axle). Per-wheel
	// peak brake torque is sized to the max motor regen torque referred
	// to the wheel: T_motor_max × GearRatio × Efficiency / 2 wheels =
	// 230 × 2.909 × 0.92 / 2 ≈ 308 Nm. Driver/autonomy "brake" input
	// (0-1) requests a fraction of this, scaled down in the pawn Tick
	// by the cell-current-limited regen power cap (see
	// AFSDSVehiclePawn::ApplyRegenBrake). Settings can override via
	// MaxRegenTorque but this is the sane default.
	MaxBrakeTorque = 310.f;
}
