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

	// Travel: MaxDrop keeps the static wheel position (ride height) of the
	// old 52.5 kN/m rate. Static compression 758.1 N / 20184 N/m = 3.76 cm,
	// was 1.44 cm, so MaxDrop 3.5 + 2.31 = 5.81 cm. See FSDSWheelFront.cpp.
	SuspensionMaxRaise = 3.5f;  // cm
	SuspensionMaxDrop = 5.81f;  // cm
	SuspensionDampingRatio = 1.5f;   // overwritten from settings.json SuspensionDamping (AFSDSVehiclePawn::ApplyTireModelToWheelCDOs)

	// FALLBACK ONLY. SpringRate, SuspensionDampingRatio, WheelMass and
	// SuspensionMaxDrop are overwritten from settings.json (SpringRate*,
	// DampingRatio*, WheelMass*) by AFSDSVehiclePawn::ApplyTireModelToWheelCDOs
	// before CreateVehicle(); MaxDrop is recomputed there from the rate so the
	// static ride height stays put. These literals equal the FSDSSettings.h defaults.
	// Wheel rate 20184 N/m -> property 201.84 (x100 into N/m), the same as
	// the front. Was 300 lb/in = 52538 N/m, the workbook's spring rate taken
	// as a wheel rate. See FSDSWheelFront.cpp for the units and the
	// measurement against the car.
	SpringRate = 201.84f;

	// No anti-roll bar: see FSDSWheelFront.cpp.
	RollbarScaling = 0.f;

	// The tyre solver reads its load from the RESTING force, not the spring.
	// Chaos's ChaosWheeledVehicleMovementComponent.cpp:665 does:
	//   ForceMagnitude = WheelLoadRatio * SpringForce + (1-WheelLoadRatio) * RestingForce
	// and the tyre's grip circle and (via SetWheelSlipGraphMultiplier) its
	// lateral force are built on ForceMagnitude. SpringForce is the "simple"
	// suspension's spring rate x compression, but the car is physically held
	// up by the suspension CONSTRAINT (stiffness 0.25 x rate, its own travel
	// limits), so the two disagree: at rest the four spring forces summed to
	// 9 / 40 / 69 / 93 % of the weight at 150 / 220 / 275 / 350 kg (measured
	// 2026-11-24, road roughness off, either car body). The 220 kg car's
	// tyres got 40 % of their load and spun up from half throttle.
	//
	// At 0 the solver gets RestingForce (sprung mass x g, right at rest), and
	// AFSDSVehiclePawn::ApplyTyreLoads scales grip every tick by the load
	// transfer, aero included (see there). WAS 1.0, on the reasoning that it
	// is "the true dynamic Fz"; it is the dynamic number of a spring that is
	// not the one carrying the car.
	WheelLoadRatio = 0.0f;

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
