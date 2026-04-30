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

	// Per-wheel rotating-mass total. Chaos's default 20 kg is roughly an
	// SUV/sedan figure; the IFS-08 corner is ≈10 kg total (Hoosier R20
	// 16×7.5-10 ≈ 6 kg + 10″ Mg rim ≈ 3 kg + brake/hub residual ≈ 1 kg
	// rotating). Halving the wheel mass halves rotational inertia
	// (Chaos uses I = 0.5·m·r²), which is the right physical number for
	// our corner *and* doubles the per-tick `ExcessTorque/Inertia` term
	// in WheelSystem — the wheel's slip-omega moves out of the
	// numerically-degenerate ω=0 state in half the ticks, which is what
	// was pinning the Chaos solver at standstill under full launch
	// torque even though drive force exceeded available grip.
	WheelMass = 10.f;

	// IFS-08: 35mm ride height, 300 lbs/in rear springs
	SuspensionMaxRaise = 3.5f;
	SuspensionMaxDrop = 3.5f;
	SuspensionDampingRatio = 1.5f;

	// Hoosier R20 slick friction. Was 1.65 prior to 2026-04-30, but
	// in combination with the EMRAX 228 motor model it pushed the
	// effective static friction above the launch torque the powertrain
	// can deliver — the rear axle locked at standstill regardless of
	// throttle, blocking autonomous launches. Real Hoosier R20 peak μ
	// is ~1.45 on warm dry tarmac; 1.4 leaves a small margin against
	// peak tire friction while still letting the EMRAX (≤ 200 Nm
	// shaft × 2.909 gear / 0.228 m wheel = ≤ 2553 N/axle force) break
	// the rear-axle static lock at full throttle.
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
