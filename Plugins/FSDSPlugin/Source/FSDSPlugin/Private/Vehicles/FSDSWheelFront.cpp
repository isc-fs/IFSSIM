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
	// Clamped to the tyre's peak-grip angle, not to a geometry estimate.
	// characterise_plant.m sweeps constant steer at 8 m/s: lateral
	// acceleration peaks at 22.4 deg (1.336 g) and FALLS to 1.268 g by
	// 28 deg, while yaw/kinematic collapses 0.873 -> 0.651. Past the peak
	// more lock buys less turn, which inverts the sign of the path
	// controller's feedback: it runs wide, adds lock, turns less, adds more.
	// That is the plow this repo kept diagnosing as a gain problem.
	//
	// Nothing is lost by the clamp — every angle it removes produces less
	// curvature than 22.4 deg already does.
	//
	// The 28 deg it replaces was never measured; the comment here said
	// "FS typical steering geometry". 22.4 is measured, but measured
	// against a SHAPE-FITTED Pacejka, not tyre data — so it moves with the
	// tyre model. Re-run characterise_plant.m after any Pacejka change.
	MaxSteerAngle = 22.4f;

	// Per-wheel total mass. See FSDSWheelRear.cpp for the IFS-08 corner
	// breakdown — Hoosier R20 16×7.5-10 ≈ 6 kg + 10″ Mg rim ≈ 3 kg +
	// brake/hub residual ≈ 1 kg. 10 kg matches the spec total per
	// corner; the earlier 5 kg figure was a numerical workaround to
	// halve rotational inertia at launch and is no longer used (the
	// proper fix is in the launch state machine, not in the wheel
	// physics).
	WheelMass = 10.f;

	// IFS-08: 35mm ride height, 350 lbs/in front springs
	SuspensionMaxRaise = 3.5f;  // 35mm
	SuspensionMaxDrop = 3.5f;
	SuspensionDampingRatio = 1.5f;

	// Wheel rate. UNITS ARE A TRAP: the UPROPERTY is documented "N/m" but
	// ChaosVehicleWheel.h:392 FillSuspensionSetup does SpringRate =
	// Chaos::MToCm(SpringRate) (x100), and SuspensionSystem.cpp:48 computes
	//   StiffnessForce = SpringDisplacement[cm] * SpringRate
	// in Chaos force units (kg*cm/s^2 = 0.01 N). Net effect:
	//
	//     property value x 100 = wheel rate in N/m
	//
	// 350 lb/in = 61294 N/m -> property 612.9.
	//
	// This was never set, so Chaos ran on its engine default of 250, i.e.
	// 25 kN/m per wheel. Meanwhile ComputeTireLoadsParametric was using
	// FFSDSVehicleSettings::HeaveStiffness = 227600 N/m ("sum of 4 wheel
	// rates") = 56.9 kN/m per corner. The car had TWO different suspension
	// stiffnesses depending on which model you asked, differing by 2.3x.
	//
	// Cross-check that 612.9 is right rather than merely plausible:
	//   2 x 61294 (front, 350 lb/in) + 2 x 52538 (rear, 300 lb/in)
	//     = 227665 N/m ~= the declared HeaveStiffness of 227600.
	// The lb/in figures and HeaveStiffness are independent declarations in
	// this repo and they agree to 0.03%, so these are true WHEEL rates with
	// the motion ratio already folded in — not spring rates that would still
	// need multiplying by MR^2.
	//
	// Sanity: sprung mass ~59 kg/corner gives a 5.1 Hz ride frequency (FS
	// cars with aero run 3-5 Hz) and 1.2 cm static deflection out of 7 cm of
	// travel. The old 250 gave 3.3 Hz and 2.7 cm.
	//
	// Set HERE, in the wheel class constructor, because Chaos builds its
	// physics wheels from the CLASS DEFAULT OBJECT in CreateVehicle() before
	// BeginPlay. Pushing suspension to a live vehicle instead is what
	// launched the car into the air upside down on an earlier attempt.
	SpringRate = 612.9f;

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
