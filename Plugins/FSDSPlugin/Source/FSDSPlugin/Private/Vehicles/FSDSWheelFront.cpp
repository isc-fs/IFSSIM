#include "Vehicles/FSDSWheelFront.h"

UFSDSWheelFront::UFSDSWheelFront()
{
	AxleType = EAxleType::Front;
	// IFS-08 EBS is pneumatic, routed to all four calipers (not just the
	// rear like a conventional handbrake). The sim models EBS via the
	// Chaos handbrake channel, so front wheels must also respond to it.
	bAffectedByHandbrake = true;
	bAffectedBySteering = true;

	// IFS-08: Hoosier 16.0x7.5-10 R20
	WheelRadius = 20.f;       // 200mm tire radius
	WheelWidth = 19.f;        // 7.5 inch = 190mm
	MaxSteerAngle = 28.f;     // FS typical steering geometry

	// Per-wheel rotating-mass total. See FSDSWheelRear.cpp for the
	// physical breakdown — same 10 kg per corner, since front and rear
	// share the same Hoosier 16×7.5-10 / 10″ Mg rim package on the
	// IFS-08. Default 20 kg is overstated for an FS car and was making
	// Chaos's wheel solver too sluggish to break out of ω=0 at launch.
	WheelMass = 10.f;

	// IFS-08: 35mm ride height, 350 lbs/in front springs
	SuspensionMaxRaise = 3.5f;  // 35mm
	SuspensionMaxDrop = 3.5f;
	SuspensionDampingRatio = 1.5f;

	// Hoosier R20 slick friction
	FrictionForceMultiplier = 1.65f;

	// IFS-08 has no hydraulic service brake — the only retarding channel
	// on the front axle is aero drag. The brake input channel models
	// motor regen, which is rear-axle (drive) only; EBS via handbrake is
	// also rear-axle (pneumatic). Zero out the Chaos default (1500 Nm).
	MaxBrakeTorque = 0.f;
}
