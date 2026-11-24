#include "Vehicles/FSDSWheelFront.h"

UFSDSWheelFront::UFSDSWheelFront()
{
	AxleType = EAxleType::Front;
	// IFS-08 EBS is pneumatic, routed to all four calipers (not just the
	// rear like a conventional handbrake). The sim models EBS via the
	// Chaos handbrake channel, so front wheels must also respond to it.
	bAffectedByHandbrake = true;
	bAffectedBySteering = true;

	// Match FSDSWheelRear: SetDriveTorque()/SetBrakeTorque() write to the
	// wheel's ExternalDriveTorque/ExternalBrakeTorque, which the solver only
	// reads when the combine method is Override or Additive. The Chaos default
	// (None) silently DISCARDS them — see WheelSystem.cpp's combine block.
	//
	// This is a no-op today because the car is RWD and no torque is ever sent
	// to the front wheels. It is set here so that when it is — four in-wheel
	// motors, torque vectoring, front regen — the torque actually arrives.
	// Without it the first 4WD experiment would silently be an RWD experiment,
	// with a plausible-looking result and no error anywhere.
	//
	// Additive rather than Override, for the same reason as the rear: internal
	// brake torques (handbrake / EBS, which these wheels DO respond to) must
	// still reach the wheel.
	ExternalTorqueCombineMethod = ETorqueCombineMethod::Additive;

	// IFS-08: Hoosier 16.0x7.5-10 R20
	WheelRadius = 20.f;       // 200mm tire radius
	WheelWidth = 19.f;        // 7.5 inch = 190mm
	// 18.2 deg at the road wheel: the only lock measured on this car
	// (tools/steering_check), and the angle the autonomy controller means by a
	// full steering command (control params max_steer_deg). settings.json
	// overrides this at runtime; it is set here too so the class default and
	// the file agree.
	//
	// It was 22.4, described as the tyre's peak-grip angle. A road-wheel angle
	// is not a slip angle, and at 22.4 every command turned the wheels 1.23x
	// further than the controller intended, so the car cut the inside of
	// corners. Settled in #462.
	MaxSteerAngle = 18.2f;

	// Per-wheel total mass. See FSDSWheelRear.cpp for the IFS-08 corner
	// breakdown — Hoosier R20 16×7.5-10 ≈ 6 kg + 10″ Mg rim ≈ 3 kg +
	// brake/hub residual ≈ 1 kg. 10 kg matches the spec total per
	// corner; the earlier 5 kg figure was a numerical workaround to
	// halve rotational inertia at launch and is no longer used (the
	// proper fix is in the launch state machine, not in the wheel
	// physics).
	WheelMass = 10.f;

	// Travel. Chaos measures spring compression from full droop, so a softer
	// spring sits further into its travel at rest. MaxDrop is sized to keep
	// the static wheel position (and so the ride height and the LiDAR's
	// height above the ground) where it was with the old 61 kN/m rate:
	// static compression 590.8 N / 20184 N/m = 2.93 cm, was 0.96 cm, so
	// MaxDrop 3.5 + 1.96 = 5.46 cm. Bump travel from rest is unchanged.
	SuspensionMaxRaise = 3.5f;  // cm
	SuspensionMaxDrop = 5.46f;  // cm
	SuspensionDampingRatio = 1.5f;   // overwritten from settings.json SuspensionDamping (AFSDSVehiclePawn::ApplyTireModelToWheelCDOs)

	// Wheel rate. UNITS ARE A TRAP: the UPROPERTY is documented "N/m" but
	// ChaosVehicleWheel.h:392 FillSuspensionSetup does SpringRate =
	// Chaos::MToCm(SpringRate) (x100), and SuspensionSystem.cpp:48 computes
	//   StiffnessForce = SpringDisplacement[cm] * SpringRate
	// in Chaos force units (kg*cm/s^2 = 0.01 N). Net effect:
	//
	//     property value x 100 = wheel rate in N/m
	//
	// 20184 N/m per corner -> property 201.84. This is the plant spec's
	// wheel rate (matlab/spec/cars/ifs08.m on dev-manual: HeaveStiffness
	// 80737 N/m from the recorded 2.9-3.0 Hz ride frequency), and the car
	// agrees: in the 2026-09-20 manual bag the LiDAR ground plane gives
	// ~0.6-0.8 deg of pitch per g and ~1.5-1.8 deg of roll per g. The old
	// rate, 350 lb/in = 61294 N/m, was the workbook's SPRING rate taken as a
	// wheel rate (the hardpoints give a motion ratio of 0.83, so even as a
	// spring it would be a 42 kN/m wheel rate). It made a 5 Hz ride and a
	// car about 2x too stiff in pitch and 4x too stiff in roll (0.30 and
	// 0.36 deg/g).
	//
	// Same rate front and rear: the spec only fixes the total, and the
	// measured pitch and roll cannot split it.
	//
	// Set HERE, in the wheel class constructor, because Chaos builds its
	// physics wheels from the CLASS DEFAULT OBJECT in CreateVehicle() before
	// BeginPlay. Pushing suspension to a live vehicle instead is what
	// launched the car into the air upside down on an earlier attempt.
	SpringRate = 201.84f;

	// Chaos's anti-roll bar: each axle adds RollbarScaling x (left - right
	// spring force) to one side and takes it off the other
	// (ChaosWheeledVehicleMovementComponent.cpp:705), so axle roll stiffness
	// = springs x (1 + 2 x RollbarScaling). The engine default 0.15 was never
	// chosen. The springs alone give 15.0 / 14.3 kN*m/rad front/rear and
	// 1.6 deg of roll per g, which is what the car shows, so no bar. Read
	// from the wheel class of the axle, so it is set on both.
	RollbarScaling = 0.f;

	// See FSDSWheelRear.cpp for rationale — use real per-wheel Fz, not
	// Chaos's default 50/50 blend with resting load.
	WheelLoadRatio = 1.0f;

	// Hoosier R20 slick friction
	FrictionForceMultiplier = 1.65f;

	// IFS-08 has no hydraulic service brake — the only retarding channel
	// on the front axle is aero drag. The brake input channel models
	// motor regen, which is rear-axle (drive) only; EBS via handbrake is
	// also rear-axle (pneumatic). Zero out the Chaos default (1500 Nm).
	MaxBrakeTorque = 0.f;
}
