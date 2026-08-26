#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "FSDSPacejkaTireModel.h"

/**
 * FSDS Settings — Parses settings.json for vehicle and sensor configuration.
 * Compatible with the original FSDS settings format (camera fields ignored).
 *
 * Camera sensors were removed in PR perf/strip-cameras (2026-05): the real
 * IFS-08 carries no cameras and the autonomy pipeline never consumed any
 * `camera` topics in production. Stripping them removed the
 * SceneCaptureComponent2D render pass from every frame (largest single
 * source of GPU cost on the mid-range gaming-laptop target).
 */

struct FFSDSSensorSettings
{
	FString Name;
	int32 SensorType = 0;
	bool bEnabled = true;
	FVector Position = FVector::ZeroVector;
	FRotator Rotation = FRotator::ZeroRotator;

	// LiDAR specific
	int32 NumberOfChannels = 4;
	int32 PointsPerSecond = 40960;
	float RotationsPerSecond = 10.f;
	float VerticalFOVUpper = 0.f;
	float VerticalFOVLower = -25.f;
	float HorizontalFOVStart = 0.f;
	float HorizontalFOVEnd = 359.f;
	float MaxRange = 100.f; // meters
	bool bDrawDebugPoints = false;

	// Per-channel max-range overrides (meters). Real LiDARs (e.g. Hesai
	// ATX_S01 datasheet Appendix A.1.1) have per-beam laser-power
	// variance — outer/edge channels typically reach shorter than the
	// central beams. When this array is populated and its length
	// matches NumberOfChannels, the LiDAR sensor uses ChannelMaxRange
	// per-channel instead of the global MaxRange above. When empty
	// (default), all channels fall back to MaxRange — preserves
	// existing settings.json behaviour bit-for-bit.
	//
	// Length validation happens in FSDSLidarSensor::OnSettingsApplied:
	// any mismatch logs a warning and falls back to the global value.
	TArray<float> PerChannelMaxRangeM;

	// LiDAR path: "cpu" (default, ParallelFor + Chaos line traces) or
	// "gpu" (depth-render + compute decode, #223). Read by
	// FSDSLidarSensor::BeginPlay; switching at runtime requires a PIE
	// stop/start. Unknown values fall back to "cpu" with a warning log.
	FString LidarPath = TEXT("cpu");

	// Noise parameters (apply to all sensor types, 0 = no noise).
	// All values are SI: m, m/s, m/s², rad/s. The plugin converts to its
	// internal cm/s²-based accel signal when applying AccelNoiseStd /
	// AccelBiasStd (see FSDSVehiclePawn::SetupSensorsFromSettings);
	// the bridge receives accel in m/s² (UEVelocityToENU) so covariance
	// uses these values as-is.
	//
	// Bias is modelled as an Ornstein–Uhlenbeck process: AccelBiasStd /
	// GyroBiasStd are the *long-run* steady-state stddevs (the bound),
	// AccelBiasTau / GyroBiasTau the correlation time in seconds. Defaults
	// to a BMI088-class τ=100 s.
	float GpsPositionNoiseStd = 0.f;  // m
	float GpsVelocityNoiseStd = 0.f;  // m/s
	float AccelNoiseStd = 0.f;        // m/s²
	float GyroNoiseStd = 0.f;         // rad/s
	float AccelBiasStd = 0.f;         // m/s² steady-state σ
	float GyroBiasStd = 0.f;          // rad/s steady-state σ
	float AccelBiasTau = 100.f;       // s
	float GyroBiasTau = 100.f;        // s
	float VelocityNoiseStd = 0.f;     // m/s (GSS)
	float RangeNoiseStd = 0.f;        // m (LiDAR)
	float DropoutRate = 0.f;          // [0,1] (LiDAR)
};

struct FFSDSVehiclePhysics
{
	float Mass = 290.f;
	FString Drivetrain = TEXT("RWD"); // RWD, FWD, AWD
	float WheelRadius = 0.200f;      // meters
	float WheelWidth = 0.190f;       // meters
	float MaxSteerAngle = 22.4f;     // degrees
	// Resistive wheel torque Crr*Fz*Rw. ASSUMED, not measured on the IFS-08.
	//
	// CONSUMED BY THE FMU PLANT, NOT BY CHAOS. Chaos has no equivalent knob we
	// drive, so this field changes nothing in a Plant.Type="chaos" run — it is
	// here because ifssim_params.m mirrors this struct field-for-field and a
	// silent divergence between the two is the bug that mirror exists to stop.
	//
	// It reaches the FMU by being BAKED IN AT EXPORT: export_plant_fmu.m reads
	// settings.json into IFSSIM_Crr and Simulink freezes it into the .fmu. So
	// editing this value does NOT change an already-exported FMU. Re-export
	// (matlab/plant/export_plant_fmu.m) or the number here and the number the
	// plant actually uses will quietly disagree.
	float RollingResistance = 0.020f;
	float MotorMaxTorque = 230.f;    // Nm (at motor)
	float MotorMaxPower = 80000.f;   // Watts
	// Regen braking limits. The IFS-08 has no hydraulic service brake —
	// braking on the drive wheels is motor regen only, capped by the
	// battery's max cell input current. MaxRegenTorque defaults to the
	// same motor peak (no extra headroom on the negative-torque side);
	// MaxRegenPower is the cell-limited cap and is the binding limit
	// at typical driving speeds.
	float MaxRegenTorque = 230.f;    // Nm (at motor, negative side)
	float MaxRegenPower = 6000.f;    // Watts — cell input current limit
	float GearRatio = 2.909f;
	float DrivetrainEfficiency = 0.92f;
	float CdA = 0.95f;              // drag
	float ClA = 3.0f;               // downforce (positive = down)
	float AeroBalanceFront = 0.45f;
	float TireMu = 1.65f;
	float WeightDistFront = 0.438f;
	float CoGHeight = 0.344f;        // meters
	float SuspensionDamping = 1.5f;
	TArray<float> MotorRPM;          // RPM points
	TArray<float> MotorTorque;       // Nm at motor for each RPM

	// --- Vehicle dynamics (load transfer) ---
	// Geometry + stiffness fields consumed by AFSDSVehiclePawn::
	// ComputeTireLoadsParametric. Defaults are the IFS-08 values; can be
	// overridden per-car via settings.json. See settings.json for the
	// JSON keys (Wheelbase, TrackFront, TrackRear, RollCenter*,
	// RollStiffness*, HeaveStiffness, PitchStiffness).
	float Wheelbase = 1.627f;            // m
	float TrackFront = 1.220f;           // m
	float TrackRear = 1.190f;            // m
	float RollCenterFront = 0.040f;      // m
	float RollCenterRear = 0.060f;       // m
	float RollStiffnessFront = 27000.f;  // Nm/rad
	float RollStiffnessRear = 22000.f;   // Nm/rad
	float HeaveStiffness = 227600.f;     // N/m (sum of 4 wheel rates)
	float PitchStiffness = 155600.f;     // Nm/rad

	// Pacejka Magic Formula '96 tire coefficients.
	// Applied to Chaos LateralSlipGraph / LongitudinalSlipGraph at BeginPlay.
	// See FSDSPacejkaTireModel.h for coefficient definitions.
	FFSDSPacejkaCoeffs Pacejka;
};

struct FFSDSVehicleSettings
{
	FString Name = TEXT("FSCar");
	FString VehicleType = TEXT("PhysXCar");
	bool bEnableCollisions = true;
	bool bAllowAPIAlways = true;
	bool bAutoCreate = true;

	FFSDSVehiclePhysics Physics;
	TMap<FString, FFSDSSensorSettings> Sensors;
};

class FSDSPLUGIN_API FFSDSSettings
{
public:
	static FFSDSSettings& Get();

	/** Load settings from the given JSON string */
	bool LoadFromString(const FString& JsonString);

	/** Load settings from a file path */
	bool LoadFromFile(const FString& FilePath);

	/** Try to find and load settings.json from standard locations */
	bool AutoLoad();

	/** Get the raw JSON string */
	const FString& GetSettingsString() const { return SettingsString; }

	// --- Settings ---
	float SettingsVersion = 1.2f;
	FString SimMode = TEXT("Car");
	FString ViewMode = TEXT("SpringArmChase");
	float ClockSpeed = 1.0f;
	FString SpectatorServerPassword;

	// Scenario seed for all stochastic sources (sensor noise, LiDAR dropout,
	// cone yaw). Same seed => same run.
	//
	// Defaults to 1, i.e. REPRODUCIBLE BY DEFAULT — a validation platform
	// should not depend on someone remembering to enable determinism, and a
	// silently non-reproducible run is the failure mode this whole change
	// exists to remove. Override with "ScenarioSeed": N in settings.json to
	// draw a different sample; set 0 to opt out entirely, in which case
	// randomness falls back to the clock and the run logs itself as
	// non-reproducible. See FSDSRandom.h.
	int32 ScenarioSeed = 1;

	// --- Plant selection -------------------------------------------------
	//
	// Which implementation of IFSDSPlant stands behind the vehicle.
	//
	//   "chaos"  — Chaos observes the car it is already integrating. Default,
	//              and the only validated reference.
	//   "shadow" — Chaos still DRIVES; the FMU is stepped alongside it with
	//              the same inputs and the divergence is logged. Behaviour is
	//              unchanged by construction, because nothing downstream reads
	//              the shadow. This is how parity gets measured before the
	//              kinematic swap, per docs/fmu_plant_migration.md Phase 6.
	//   "fmu"    — the FMU is the plant. NOT yet the authoritative driver: the
	//              pawn is still Chaos-integrated, so selecting this makes the
	//              sensors read a car the mesh is not flying. Only useful for
	//              bring-up until the Phase 6 kinematic swap lands.
	//
	// Kept out of VehiclePhysics on purpose. This is not a property of the
	// car; it is a choice about which simulator runs it.
	FString PlantType = TEXT("chaos");

	/** Path to the .fmu for "shadow"/"fmu". Relative paths resolve against the
	 *  project directory. Empty means fall back to chaos, loudly. */
	FString PlantFmuPath;

	/** Metres above the wheel centre to start each road probe, and metres
	 *  below to end it. The defaults straddle a 0.2 m wheel with room for
	 *  suspension travel without reaching through thin geometry. */
	float RoadProbeUpM   = 0.6f;
	float RoadProbeDownM = 1.2f;

	/** Surface friction handed to the plant where the probe cannot tell.
	 *  Distinct from VehiclePhysics.TireMu, which is the tyre's own limit. */
	float RoadDefaultMu = 1.4f;

	/** Half-span of the road probe's sample pattern, metres. Sized to the tyre
	 *  contact patch: sampling much wider than the patch would report a plane
	 *  the tyre never touches, and much narrower makes the fit noise-limited. */
	float RoadProbeSpanM = 0.08f;

	/** Height of the SKELETAL MESH ORIGIN above the road when the car rests,
	 *  metres. The plant reports its CoG, so the mesh offset is
	 *  -(CoGHeight - MeshOriginHeight).
	 *
	 *  ZERO, and the reason is worth stating because the obvious answer is
	 *  wrong. Under Chaos the chassis rested at 0.029 m, and anchoring to that
	 *  seems right — but Chaos's animation node was posing the wheels DOWN by
	 *  the suspension compression at the same time. With that posing gone the
	 *  wheels sit at their bind pose, exactly one radius below the mesh
	 *  origin, so the origin belongs on the road plane.
	 *
	 *  Using 0.029 here lifted the whole car by that much: measured as a
	 *  +2.8 cm gap between every tyre and the road. The right reference is the
	 *  WHEEL contact, not the chassis height — a chassis-height match can be
	 *  exact while the car visibly hovers, which is what happened. */
	float MeshOriginHeightM = 0.0f;

	/** Shadow mode: force the shadow's state equal to the reference's every
	 *  step, so the comparison is "same state, same inputs, same response?"
	 *  rather than "how far apart do they drift?".
	 *
	 *  The drift question cannot be answered by an open-loop shadow at all —
	 *  the controller is steering the REFERENCE, so the shadow diverges
	 *  without bound however good it is, and the number measures the
	 *  experiment rather than the plant. Set false only to watch a shadow run
	 *  free, which is a demo, not a measurement. */
	bool bShadowSync = true;

	TMap<FString, FFSDSVehicleSettings> Vehicles;

	/** Get the first (default) vehicle settings */
	const FFSDSVehicleSettings* GetDefaultVehicle() const;

private:
	FFSDSSettings() = default;

	void ParseVehicle(const FString& Name, TSharedPtr<FJsonObject> VehicleObj);
	void ParseSensor(const FString& Name, TSharedPtr<FJsonObject> SensorObj, FFSDSVehicleSettings& Vehicle);

	FString SettingsString;

	static FFSDSSettings* Instance;
};
