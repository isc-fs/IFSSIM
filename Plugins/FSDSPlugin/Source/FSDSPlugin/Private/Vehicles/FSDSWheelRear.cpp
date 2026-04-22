#include "Vehicles/FSDSWheelRear.h"

UFSDSWheelRear::UFSDSWheelRear()
{
	AxleType = EAxleType::Rear;
	bAffectedByHandbrake = true;
	bAffectedBySteering = false;

	// IFS-08: Hoosier 16.0x7.5-10 R20
	WheelRadius = 20.f;       // 200mm tire radius
	WheelWidth = 19.f;        // 7.5 inch = 190mm
	MaxSteerAngle = 0.f;

	// IFS-08: 35mm ride height, 300 lbs/in rear springs
	SuspensionMaxRaise = 3.5f;
	SuspensionMaxDrop = 3.5f;
	SuspensionDampingRatio = 1.5f;

	// Hoosier R20 slick friction
	FrictionForceMultiplier = 1.65f;

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
