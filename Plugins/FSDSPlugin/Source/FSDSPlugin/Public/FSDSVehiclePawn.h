#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "Camera/CameraComponent.h"
#include "Sensors/FSDSCameraSensor.h"
#include "Sensors/FSDSLidarSensor.h"
#include "Sensors/FSDSImuSensor.h"
#include "Sensors/FSDSGpsSensor.h"
#include "Sensors/FSDSGssSensor.h"
#include "Sensors/FSDSDistanceSensor.h"
#include "Sensors/FSDSBarometerSensor.h"
#include "Sensors/FSDSMagnetometerSensor.h"
#include "Vehicles/FSDSWheelFront.h"
#include "Vehicles/FSDSWheelRear.h"
#include "EmraxMotor.h"
#include "FSDSVehiclePawn.generated.h"

/**
 * FSDS Vehicle Pawn — Chaos Physics wheeled vehicle.
 * Uses AWheeledVehiclePawn with FormulaMesh skeletal mesh.
 * Falls back to simple movement if skeletal mesh fails to load.
 */
UCLASS()
class FSDSPLUGIN_API AFSDSVehiclePawn : public AWheeledVehiclePawn
{
	GENERATED_BODY()

public:
	AFSDSVehiclePawn();

	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual void BeginPlay() override;

	// --- Programmatic control ---

	struct FCarControls
	{
		float Throttle = 0.f;
		float Steering = 0.f;
		float Brake = 0.f;
		bool bHandbrake = false;
		bool bIsManualGear = false;
		int32 ManualGear = 0;
		bool bGearImmediate = true;
	};

	struct FCarState
	{
		float Speed = 0.f;
		int32 Gear = 0;
		float RPM = 0.f;
		float MaxRPM = 0.f;
		bool bHandbrake = false;
		FVector Position = FVector::ZeroVector;
		FQuat Orientation = FQuat::Identity;
		FVector LinearVelocity = FVector::ZeroVector;
		FVector AngularVelocity = FVector::ZeroVector;
		FVector LinearAcceleration = FVector::ZeroVector;
		uint64 Timestamp = 0;
	};

	void SetCarControls(const FCarControls& Controls);
	FCarControls GetCarControls() const;
	FCarState GetCarState() const;
	void SetApiControlEnabled(bool bEnabled) { bApiControlEnabled = bEnabled; }

	// Per-wheel vertical load. Order matches WheelSetups: FL, FR, RL, RR.
	// Always returned in Newtons. Negative loads (wheel-lift) are
	// clamped to 0 by the parametric path so callers can sanity-check
	// `Total < Mass·g` to detect a lifted-wheel condition; the truth
	// path passes through whatever Chaos's solver computed.
	struct FTireLoads
	{
		float FL = 0.f;
		float FR = 0.f;
		float RL = 0.f;
		float RR = 0.f;
		float Total() const { return FL + FR + RL + RR; }
	};

	/**
	 * Closed-form Milliken decomposition of Fz per wheel.
	 *
	 * Decomposes the vertical load into the five physical contributions:
	 *   1. Static (mass distribution at rest)
	 *   2. Longitudinal (m·ax·h_cg/L, split L/R 50/50)
	 *   3. Lateral geometric (via roll-centre heights, instantaneous)
	 *   4. Lateral elastic (via roll angle, depends on dynamic state)
	 *   5. Heave + pitch (suspended-mass displacement)
	 *
	 * Conventions (ISO 8855):
	 *   x forward, y left, z up
	 *   ay > 0 → accel toward LEFT (i.e. right turn) → R wheels load
	 *   ax > 0 → accelerating → rear loads
	 *   phi > 0 → right side rises (roll to the right)
	 *   theta > 0 → nose up (squat-equivalent)
	 *   z > 0 → suspended mass risen above equilibrium
	 *
	 * Pulls m_total / L / wf / h_cg / track / roll-centres / stiffnesses
	 * directly from the cached settings struct, so this is the same set
	 * of facts the Chaos solver was configured against.
	 */
	FTireLoads ComputeTireLoadsParametric(float ax, float ay, float phi, float theta, float z) const;

	/**
	 * Read the truth Fz per wheel as Chaos's solver currently has it
	 * (the same value being used to compute longitudinal/lateral grip
	 * this tick). Reads UChaosWheeledVehicleMovementComponent's
	 * per-wheel state — no parametric model, just the live signal.
	 */
	FTireLoads GetTireLoadsTruth() const;

	// EBS analog: clamps the handbrake and locks all input channels (API +
	// keyboard) until explicitly released. The real IFS-08 EBS is a
	// pneumatic rear-axle brake that can only be reset manually, not by
	// the autonomy stack — while latched, no throttle/brake/steering
	// input takes effect and the handbrake stays engaged.
	void ActivateEbs();
	void ReleaseEbs();
	bool IsEbsLatched() const { return bEbsLatched; }

	// --- Components ---

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UChaosWheeledVehicleMovementComponent* VehicleMovement;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	USpringArmComponent* SpringArm;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UCameraComponent* FollowCamera;

	/** EMRAX 228 motor model. Replaces Chaos's ICE-style EngineSetup
	 *  with an EV-correct envelope-curve + thermal-derate model.
	 *  See EmraxMotor.h for details. Instantiated in BeginPlay. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Instanced, Category = "Powertrain")
	UEmraxMotor* Motor = nullptr;

	// --- Sensors ---

	/** Multiple cameras from settings.json */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	TMap<FString, UFSDSCameraSensor*> Cameras;

	/** Get a camera by name (returns nullptr if not found) */
	UFSDSCameraSensor* GetCamera(const FString& Name) const;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSLidarSensor* LidarSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSImuSensor* ImuSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSGpsSensor* GpsSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSGssSensor* GssSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSDistanceSensor* DistanceSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSBarometerSensor* BarometerSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSMagnetometerSensor* MagnetometerSensor;

	/** Create sensors from settings.json config (call in BeginPlay) */
	void SetupSensorsFromSettings();

	UPROPERTY()
	UFloatingPawnMovement* FallbackMovement = nullptr;

	UPROPERTY()
	class UBoxComponent* PhysicsBox = nullptr;

private:
	void SetupVehicleMovement();
	void OnThrottleInput(float Value);
	void OnSteeringInput(float Value);
	void OnBrakeInput(float Value);
	void OnHandbrakePressed();
	void OnHandbrakeReleased();

	FCarControls CurrentControls;
	bool bApiControlEnabled = false;
	bool bChaosVehicleActive = false;
	bool bEbsLatched = false;

	// Regen brake limits — shadow of FFSDSVehiclePhysics values, captured
	// from settings at construction so the Tick can apply the cell-input-
	// current-limited power cap without reparsing settings every frame.
	// Brake channel = regen: 0-1 input maps to 0-MaxRegenTorque at the
	// motor, then Tick caps by MaxRegenPower/ω_motor. See ApplyRegenBrake.
	float MaxRegenTorque = 230.f;  // Nm at motor
	float MaxRegenPower = 6000.f;  // Watts — hardware cell-current limit
	float GearRatio = 2.909f;      // motor → rear axle
	float WheelRadius = 0.2f;      // m

	// Vehicle-dynamics shadow of FFSDSVehiclePhysics — same write-once
	// pattern as the regen fields above. Consumed by
	// ComputeTireLoadsParametric. Defaults match FFSDSVehiclePhysics.
	float Mass = 290.f;                  // kg
	float Wheelbase = 1.627f;            // m
	float WeightDistFront = 0.438f;
	float CoGHeight = 0.300f;            // m
	float TrackFront = 1.220f;           // m
	float TrackRear = 1.190f;            // m
	float RollCenterFront = 0.040f;      // m
	float RollCenterRear = 0.060f;       // m
	float RollStiffnessFront = 27000.f;  // Nm/rad
	float RollStiffnessRear = 22000.f;   // Nm/rad
	float HeaveStiffness = 227600.f;     // N/m
	float PitchStiffness = 155600.f;     // Nm/rad

	FVector PreviousVelocity = FVector::ZeroVector;
	FVector CurrentAcceleration = FVector::ZeroVector;

	// IFS-08 Aerodynamics
	float CdA = 0.95f;               // Drag coefficient * frontal area (m²)
	float ClA = 3.0f;                // Downforce coefficient * plan area (m²)
	float AeroBalanceFront = 0.45f;   // 45% downforce on front axle
	void ApplyAeroForces();
};
