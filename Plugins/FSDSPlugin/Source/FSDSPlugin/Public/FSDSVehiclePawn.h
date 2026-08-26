#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "Vehicles/FSDSWheeledVehicleMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "Camera/CameraComponent.h"
#include "Sensors/FSDSLidarSensor.h"
#include "Sensors/FSDSImuSensor.h"
#include "Sensors/FSDSGpsSensor.h"
#include "Sensors/FSDSGssSensor.h"
#include "Sensors/FSDSDistanceSensor.h"
#include "Sensors/FSDSBarometerSensor.h"
#include "Sensors/FSDSMagnetometerSensor.h"
#include "Vehicles/FSDSWheelFront.h"
#include "Plant/FSDSPlant.h"
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
	// Takes an FObjectInitializer so the constructor can substitute
	// UFSDSWheeledVehicleMovementComponent for the stock Chaos component —
	// the movement component is a default subobject created by
	// AWheeledVehiclePawn, so its class can only be changed here.
	AFSDSVehiclePawn(const FObjectInitializer& ObjectInitializer);

	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual void BeginPlay() override;

	// --- Programmatic control ---

	// Vehicle command channels. Each field maps to one physical actuator
	// path on the IFS-08, deliberately separated so future control work
	// (steering dynamics #149, brake split #150, traction control #151)
	// has a clean wedge point between command and actuator.
	//
	//   Throttle  [0, 1]   — motor drive demand. Maps to EMRAX shaft
	//                        torque via the envelope curve. Positive only;
	//                        regen is a separate channel below.
	//   Regen     [0, 1]   — motor regen brake demand. Same envelope class
	//                        but capped by HV battery cell-input current,
	//                        not by motor power. Rear axle only (RWD).
	//   Steering  [-1, 1]  — normalized front-wheel steering target.
	//                        Currently applied instantly via SetSteeringInput;
	//                        future #149 wedges servo dynamics here.
	//   bHandbrake bool    — EBS latch request. Operator-only release per
	//                        FS-DV T 14.8.4. Server enforces latched
	//                        semantics (ActivateEbs / ReleaseEbs).
	//
	// Throttle and Regen can both be > 0 in the same tick (legacy semantics
	// from the flat CarControls API): Tick folds them as net = Throttle -
	// Regen at the motor. New callers should command only one at a time.
	struct FCarControls
	{
		float Throttle = 0.f;
		float Steering = 0.f;
		float Regen = 0.f;
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
	/** The plant behind the interface. Chaos today; an FMU later. Not a
	 *  UPROPERTY because IFSDSPlant is a plain C++ interface, deliberately —
	 *  it must be implementable without dragging in UObject machinery. */
	TUniquePtr<IFSDSPlant> Plant;

	/** Refreshed once per Tick, read by everything downstream. */
	FFSDSPlantOutput PlantState;

	/** The FMU running alongside Chaos in Plant.Type="shadow". Stepped with
	 *  the same inputs, read by nothing — so it cannot change behaviour. */
	/** True when the FMU is integrating the car and Chaos has been stood down.
	 *  The mesh is then a KINEMATIC TARGET driven from PlantState, not a
	 *  simulated body. */
	bool bFmuDrivesPawn = false;

	/** Vertical datum shift between the plant's body origin and the mesh's
	 *  origin, in cm. The plant reports the CoG; the mesh origin is wherever
	 *  the artist put it, and under Chaos the car rested at z ~= 0.03 m while
	 *  the plant says 0.30. Writing the plant's z straight onto the mesh
	 *  floats the car by the difference. Captured once, from the pose Chaos
	 *  had the car in at the moment of the swap. */
	double PlantToMeshZCm = 0.0;
	bool   bZDatumCaptured = false;
	double PawnZAtSwapCm = 0.0;

	/** Contact impulses accumulated since the last plant step, in CONTRACT
	 *  units (N*s, world ENU) with the moment already taken about the body
	 *  origin. Converted to a force by dividing by the step, and cleared
	 *  once consumed — an impulse applied twice is a force that doubles with
	 *  frame rate. */
	FVector PendingContactImpulse = FVector::ZeroVector;
	FVector PendingContactMoment  = FVector::ZeroVector;
	FVector PendingContactPoint   = FVector::ZeroVector;
	int32   PendingContactCount   = 0;

	/** Write the plant's pose onto the mesh. Only called when the FMU drives. */
	void DrivePawnFromPlant();

	/** Pose the wheel bones from the plant.
	 *
	 *  Chaos's vehicle anim node used to do this, and deactivating Chaos took
	 *  it with it — leaving the wheels in their bind pose while the chassis
	 *  moved correctly, which reads on screen as a car not touching the
	 *  ground. The plant knows where its wheels are; this puts them there. */
	void PoseWheelsFromPlant(float DeltaTime);

	/** Build the plant-driven wheel meshes and hide the skeletal ones. */
	void CreatePlantWheels();

	/** Wheels drawn from the plant, in FL/FR/RL/RR order.
	 *
	 *  The wheels ARE plant state — omega, steer, suspension travel and
	 *  contact all come off the Wheels bus — so the platform's only job is to
	 *  draw what the plant reports. Chaos's animation node drawing them
	 *  instead put two different simulators into one picture: a chassis from
	 *  the plant and wheels from Chaos. */
	UPROPERTY()
	TArray<TObjectPtr<UStaticMeshComponent>> PlantWheels;

	/** Accumulated spin per wheel, radians, integrated from the plant's omega.
	 *  Chaos snaps wheel speed to ground speed, so its wheels always look like
	 *  they are rolling; these stop when the plant says the wheel has. */
	double WheelSpinRad[FSDS_NUM_WHEELS] = {0,0,0,0};

	/** Throttles the wheel-gap report to once a second. */
	double WheelReportAccum = 0.0;



	TUniquePtr<IFSDSPlant> ShadowPlant;
	FFSDSPlantOutput ShadowState;
	int64 ShadowSteps = 0;
	/** Each plant's own pose on the first shadow step. The FMU begins at its
	 *  own initial condition while the Chaos car spawns on the start gate, so
	 *  absolute positions are not comparable and their difference would be a
	 *  large meaningless constant. Divergence is measured between DISPLACEMENTS
	 *  from these origins. */
	bool   bShadowOriginSet = false;
	double ShadowOriginFmu[3] = {0,0,0};
	double ShadowOriginChaos[3] = {0,0,0};
	double ShadowYaw0Fmu = 0.0;
	double ShadowYaw0Chaos = 0.0;
	/** Reference pose on the PREVIOUS step, to catch teleports. */
	double ShadowPrevChaosPos[3] = {0,0,0};
	int32  ShadowRelatches = 0;
	double ShadowWorstPosErrM = 0.0;
	double ShadowWorstYawErrDeg = 0.0;
	double ShadowSumPosErrM = 0.0;
	double ShadowNextLogTime = 0.0;

	/** Fill the road bus by probing terrain under each wheel.
	 *
	 *  Deliberately independent of Chaos: it traces from the wheel BONES, not
	 *  from FWheelStatus::ContactPoint, because Chaos's contact results vanish
	 *  in Phase 6 and a probe that depends on them would have to be rewritten
	 *  exactly when it is load-bearing. */
	void ProbeRoad(FFSDSPlantInput& In) const;

	/** Step the shadow plant and accumulate divergence against PlantState. */
	void StepShadowPlant(const FFSDSPlantInput& In);


	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UFSDSWheeledVehicleMovementComponent* VehicleMovement;

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

	// FSDSCameraSensor was removed in perf/strip-cameras. The real IFS-08
	// has no cameras and the autonomy pipeline never consumed any
	// /camera/* topics — keeping the SceneCaptureComponent2D in the tree
	// was the largest single source of per-frame GPU cost on the
	// mid-range gaming-laptop target. FollowCamera above is unrelated:
	// it's the editor/spectator chase view, not a sensor.

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
	// Writes the settings.json tire model into the wheel CLASS DEFAULT
	// OBJECTS. Must be called from the constructor, before the movement
	// component is configured: Chaos builds its physics wheels from the CDO
	// during CreateVehicle(), so anything applied later never reaches the
	// solver. See the implementation for why the Pacejka curve in particular
	// has no other route.
	void ApplyTireModelToWheelCDOs();

public:
	/**
	 * Push everything settings.json contributes to the wheels into the Chaos
	 * solver, then read it back and complain if it disagrees.
	 *
	 * MUST be called again after anything that rebuilds physics state.
	 * ResetVehicleState() destroys and recreates it, which re-runs
	 * CreateVehicle() and rebuilds every physics wheel from the CLASS DEFAULT
	 * OBJECT — silently discarding the runtime-pushed configuration.
	 */
	void ApplyWheelSettingsToSolver(bool bLogInherited = false);

	/**
	 * This tick's plant state, in the platform<->plant contract (SI, ISO 8855,
	 * ENU) rather than UE's left-handed centimetres.
	 *
	 * Consumers should migrate to this instead of reading the pawn or the Chaos
	 * component directly. Two reasons: the frame conversion then lives in ONE
	 * place, and when the plant becomes an FMU nothing downstream changes.
	 *
	 * Check bPlantOk. A failed step reports false; it does NOT return a
	 * well-formed zero, which is what the IsSimulatingPhysics guards do today.
	 */
	const FFSDSPlantOutput& GetPlantState() const { return PlantState; }

	/** Which plant is driving. "Chaos" today. */
	FString GetPlantName() const;

	/** Tell the plant(s) the car has been teleported.
	 *
	 *  Must be called from every reset path. A plant that integrates its own
	 *  state has no other way to know: the platform moving the mesh is
	 *  invisible to it, so without this it keeps driving from wherever it had
	 *  got to while the rest of the sim starts a fresh mission.
	 *
	 *  Position/Quat are in CONTRACT units (m, ENU, w-first quaternion), not
	 *  UE centimetres. */
	void ResetPlants(const double Position[3], const double Quat[4]);

	/** Vertical offset between the MESH origin and the PLANT's body origin,
	 *  in metres: plant_z = mesh_z + this.
	 *
	 *  The plant reports its CoG (build_chassis seeds pos to [0;0;CoGH]); the
	 *  mesh origin sits MeshOriginHeightM above the road. Both directions of
	 *  this conversion exist, and having them written out separately is how
	 *  they came to disagree: the plant->mesh path was corrected and the
	 *  mesh->plant path was not, so a teleport buried the plant's CoG by a
	 *  full CoG height. The suspension bottomed out, front and rear compressed
	 *  differently under their different static loads, and the car settled
	 *  PITCHED — measured as a 3.85 m/s^2 longitudinal "bias" that the EKF
	 *  then calibrated in, after which SLAM never produced a pose and the
	 *  watchdog fired. One function, used by both directions. */
	static double PlantMeshZOffsetM();

	/** Report a contact impulse the car just delivered, recovered from the
	 *  OTHER body.
	 *
	 *  It has to come from the other side. When the FMU drives, the car's mesh
	 *  is kinematic, and a kinematic body's own OnComponentHit reports a
	 *  NormalImpulse of zero — there is no solver reaction on a body the
	 *  solver does not integrate. The cone IS simulating, so its hit carries
	 *  the real impulse, and Newton's third law supplies the car's.
	 *
	 *  ImpulseUe is the impulse ON THE OTHER BODY in UE units (kg*cm/s);
	 *  PointUe is the world contact point in cm. Both are converted and
	 *  negated here, once, rather than at each call site. */
	void ReportContactImpulse(const FVector& ImpulseUe, const FVector& PointUe);

	/** Tear the plants down deterministically. Waiting for the pawn to be
	 *  garbage-collected is too late: PIE restarts BeginPlay on a new pawn
	 *  while the old one is still alive, and the FMU is one-instance-per
	 *  -process. */
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

	/** Probe the ground under ONE point. Public and single-point so it can be
	 *  tested against known geometry: the wheel loop below is a caller, not
	 *  the unit. A probe only ever exercised through four wheel bones on flat
	 *  ground cannot be told apart from a stub returning zero.
	 *
	 *  StartCm is a world UE position; the trace runs down from above it.
	 *  Outputs are CONTRACT units — height in m, normal in ENU with +y LEFT. */
	bool ProbeRoadAt(const FVector& StartCm, double& OutHeightM, double OutNormal[3]) const;

	/** Fit a plane to five rays around one point, so the platform reports the
	 *  surface the tyre actually sits on and can say how well a plane
	 *  describes it.
	 *
	 *  OutResidual is the RMS distance of the hits from the fitted plane, in
	 *  metres — the contract's "so the plant can DETECT a bad fit rather than
	 *  trust it". It is kRoadResidualNotFitted (negative, so it cannot be
	 *  mistaken for a good measurement) when fewer than three rays hit and
	 *  there is no plane to fit. */
	bool ProbeRoadPatch(const FVector& CentreCm, double& OutHeightM,
	                    double OutNormal[3], double& OutResidualM) const;

	/** Reported when no plane could be fitted. Negative because an RMS never
	 *  is, so a consumer cannot silently read it as "perfectly flat" — which
	 *  is exactly what a 0.0 here used to invite. */
	static constexpr double kRoadResidualNotFitted = -1.0;

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
