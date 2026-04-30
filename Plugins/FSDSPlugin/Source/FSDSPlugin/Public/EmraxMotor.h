// EMRAX 228 MV (LC variant) motor model — Tier 1.
//
// Purpose: replace UE5 Chaos's ICE-style engine simulation with an
// EV-correct torque-vs-speed envelope, first-order current-loop lag,
// and I²t thermal derate. The IFS-08 runs an EMRAX 228 MV at 400 V
// bus; this class encapsulates that motor's behaviour so we can:
//
//   - feed the right shaft torque to the wheels (no idle, no
//     transmission torque-converter shenanigans)
//   - report a vehicle-frame RPM that matches actual wheel motion
//     (zero at standstill, scales linearly with vehicle speed)
//   - extend later with the high-voltage battery pack (current draw
//     in this class will become I_dc → battery V × I and SoC drain)
//
// Architectural note: the EMRAX-228 spec doc (EMRAX228/emrax228_ue5_complete.md)
// integrates rotor dynamics internally (OmegaRadS += NetTorque/J × dt).
// In our pipeline Chaos already simulates wheel rotation, and the
// stiff EV driveline locks motor RPM to wheel_omega × gear_ratio, so
// integrating rotor dynamics here would create a duplicate state
// that drifts from Chaos's. Instead the caller (FSDSVehiclePawn)
// drives the RPM each tick via SetMechRpm(), and Step() reads from
// it without integrating. The thermal model and current-loop lag
// remain authoritative here.

#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "Curves/CurveFloat.h"
#include "EmraxMotor.generated.h"

USTRUCT(BlueprintType)
struct FEmraxMotorParams
{
	GENERATED_BODY()

	// --- Datasheet (EMRAX 228 MV / LC at 400 V bus, NX-tech traj file) ---

	/** Pole pairs. 10 for EMRAX 228. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Datasheet")
	float PolePairs = 10.f;

	/** Rotor moment of inertia (kg·m²). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Datasheet")
	float RotorInertia = 0.02521f;

	/** Mass of the motor itself (kg). LC variant. Informational; not used in physics. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Datasheet")
	float MassKg = 13.5f;

	/** Hard mechanical RPM limit. Speed-clamp at this value. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Datasheet")
	float MaxMechRpm = 6500.f;

	/** Torque constant (Nm per A_RMS). For EMRAX 228 MV: 0.61. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Datasheet")
	float KtNmPerArms = 0.61f;

	// --- NX-tech caps at 400 V bus (MV variant), motoring (drive) regime ---

	/** Peak power cap from the NX-tech LUT — 100 kW at 400 V (vs 124 kW
	 *  at 630 V per datasheet). The spec doc walks through why this
	 *  number is bus-voltage-dependent; section 2 in the markdown. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Datasheet")
	float MaxPeakPowerW = 100000.f;

	/** Peak torque (S2 2-min rating). NX file uses 200 Nm conservative
	 *  cap; datasheet S2 is 220 Nm. Envelope curve below caps at 200. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Datasheet")
	float MaxPeakTorqueNm = 220.f;

	/** Continuous (S1) torque, LC cooling. Once thermal budget is
	 *  consumed, max torque blends from peak down to this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Datasheet")
	float ContTorqueNm = 130.f;

	/** Continuous (S1) power, LC cooling. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Datasheet")
	float ContPowerW = 75000.f;

	// --- Regen (generating) regime ---
	//
	// Regenerative braking is gated by the BATTERY cell-input current
	// limit, not by the motor's mechanical envelope. The EMRAX 228 can
	// dump well over 100 kW into a load, but the IFS-08 accumulator
	// (~140 cells) will fault out at a few kW of total charge power at
	// the per-cell input current limit. So regen torque caps at
	// MaxRegenPowerW / motor_ω rather than at MaxPeakPowerW / motor_ω.
	// Once the HV battery class lands these will be derived from the
	// pack model (cell voltage, SoC, max charge current) instead of
	// being constants here.

	/** Battery cell-input-current-limited regen power (W).
	 *  Default 6000 W matches the IFS-08 accumulator's per-cell
	 *  charge current limit at nominal voltage. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Regen")
	float MaxRegenPowerW = 6000.f;

	/** Peak regen torque cap (Nm). Symmetric with motoring at low
	 *  speed; the power cap above starts to dominate at ~250 RPM
	 *  (= 26 rad/s = 6000 W / 230 Nm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Regen")
	float MaxRegenTorqueNm = 230.f;

	// --- Inverter / controller dynamics ---

	/** First-order current-loop time constant (s). EMRAX-class
	 *  controllers settle in ~1.5 ms; tighten if launches feel rubbery. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controller")
	float CurrentLoopTau = 0.0015f;

	// --- Software "creep" (off-throttle idle torque) ---
	//
	// Real EMRAX has no idle — at throttle=0 it produces literally zero
	// torque. But Chaos's wheel solver gets numerically frozen at the
	// degenerate state ω_wheel=0, v_chassis=0, T_drive=0: the friction
	// circle clips the chassis force to AvailableGrip, and `ExcessTorque`
	// for wheel spin-up is near-zero too, so a parked car can stay
	// pinned indefinitely even when the controller commands full
	// throttle. The OLD Chaos engine masked this because EngineIdleRPM
	// =1200 always fed a few Nm into the system. We replicate that with
	// a software creep: a small constant shaft torque applied while the
	// motor is below `IdleCreepRpmThreshold` and the driver is not
	// braking, identical to the "creep mode" that consumer EVs program
	// into their inverter for the same drive-feel reason.
	//
	// Default 5 Nm shaft × 2.909 gear × 0.92 eff / 2 wheels ≈ 6.7 Nm
	// per rear wheel, well below static-friction torque (≈242 Nm), so
	// it can never roll the parked car against locked brakes — it only
	// keeps ω just off zero so the wheel solver stays out of the
	// degenerate state.

	/** Off-throttle idle torque (Nm at the shaft). Applied whenever
	 *  Throttle ≥ 0 and |MechRpm| < IdleCreepRpmThreshold, regardless
	 *  of envelope/power caps. Set to 0 to disable. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controller")
	float IdleCreepTorqueNm = 5.f;

	/** Above this MechRpm the idle creep is no longer applied (the
	 *  wheel solver is already well out of the ω=0 degenerate state). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Controller")
	float IdleCreepRpmThreshold = 100.f;

	// --- I²t thermal model ---

	/** Total I²t budget before thermal_derate hits the floor (J of
	 *  excess copper loss). 1e6 ≈ 120 s at peak overload, matching
	 *  the datasheet's S2 2-min rating. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thermal")
	float OverloadBudgetJ = 1.0e6f;

	/** Cooling rate (W of overload-budget recovery while not at
	 *  overload). 8 kW for LC at 6 l/min coolant flow. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thermal")
	float CoolingRateW = 8000.f;
};

/**
 * EMRAX 228 motor model. Owned by AFSDSVehiclePawn; one per car.
 *
 * Lifecycle:
 *   1. Constructor builds a default-EMRAX-228 envelope curve from the
 *      hard-coded coefficients in EmraxMotor.cpp. No external asset
 *      needed; future work can swap to a UCurveFloat from an asset.
 *   2. Caller (FSDSVehiclePawn::Tick) each tick:
 *        Motor->SetMechRpm(motor_rpm_from_wheel_omega)
 *        const float ShaftTorqueNm = Motor->Step(throttle, dt)
 *        VehicleMovement->SetDriveTorque(WHEEL_RL, ShaftTorqueNm/2)
 *        VehicleMovement->SetDriveTorque(WHEEL_RR, ShaftTorqueNm/2)
 *   3. Bridge's getCarState reports Motor->GetMechRpm() instead of
 *      Chaos's GetEngineRotationSpeed() (which floors at idle=1200).
 */
UCLASS(BlueprintType, Blueprintable, EditInlineNew, DefaultToInstanced)
class FSDSPLUGIN_API UEmraxMotor : public UObject
{
	GENERATED_BODY()

public:
	UEmraxMotor();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Motor")
	FEmraxMotorParams P;

	// --- Externally-driven state ---

	/** Mechanical RPM, set by the caller from the actual driveline.
	 *  Updated each tick from `wheel_omega × gear_ratio`. */
	UPROPERTY(BlueprintReadOnly, Category = "Motor")
	float MechRpm = 0.f;

	// --- Internally-tracked state ---

	/** Latest output shaft torque (Nm). Updated by Step(). */
	UPROPERTY(BlueprintReadOnly, Category = "Motor")
	float TorqueNm = 0.f;

	/** Accumulated I²t excess (J). Counts up while in overload, decays
	 *  at CoolingRateW while at or below continuous. */
	UPROPERTY(BlueprintReadOnly, Category = "Motor")
	float OverloadJ = 0.f;

	/** [0..1]. 1 = no derate (peak available), 0 = full derate (only
	 *  continuous available). Blends linearly with OverloadJ. */
	UPROPERTY(BlueprintReadOnly, Category = "Motor")
	float ThermalDerate = 1.f;

	// --- API ---

	/** Push the current RPM into the model. Caller computes from
	 *  wheel angular velocity × gear ratio (or vehicle speed × gear
	 *  ratio / wheel radius). Step() reads this each call. */
	UFUNCTION(BlueprintCallable, Category = "Motor")
	void SetMechRpm(float Rpm) { MechRpm = FMath::Abs(Rpm); }

	/**
	 * Compute the shaft torque the motor delivers this step.
	 *
	 * @param Throttle  Driver demand in [-1, 1]. Negative = regen
	 *                  request (motor as generator). The torque
	 *                  envelope is symmetric in this version — full
	 *                  regen torque equals full drive torque.
	 * @param Dt        Tick interval (s). 1.5 ms current-loop lag is
	 *                  the only first-order state advanced here.
	 * @return Shaft torque (Nm). Positive = forward drive, negative = regen.
	 */
	UFUNCTION(BlueprintCallable, Category = "Motor")
	float Step(float Throttle, float Dt);

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Motor")
	float GetMechRpm() const { return MechRpm; }

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Motor")
	float GetTorqueNm() const { return TorqueNm; }

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Motor")
	float GetThermalDerate() const { return ThermalDerate; }

	UFUNCTION(BlueprintCallable, Category = "Motor")
	void Reset()
	{
		MechRpm = 0.f;
		TorqueNm = 0.f;
		OverloadJ = 0.f;
		ThermalDerate = 1.f;
	}

private:
	/** Peak torque envelope as a function of mechanical RPM. Built in
	 *  the constructor from the hard-coded EMRAX-228 envelope CSV
	 *  (see EmraxMotor.cpp). Non-UPROPERTY because FRichCurve isn't
	 *  reflectable; it's value-copied so this class stays self-contained. */
	FRichCurve TorqueEnvelope;
};
