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
