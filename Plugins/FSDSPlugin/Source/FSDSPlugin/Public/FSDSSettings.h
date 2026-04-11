#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

/**
 * FSDS Settings — Parses settings.json for vehicle, sensor, and camera configuration.
 * Compatible with the original FSDS settings format.
 */

enum class EFSDSImageType : uint8
{
	Scene = 0,
	DepthPlanner = 1,
	DepthPerspective = 2,
	DepthVis = 3,
	DisparityNormalized = 4,
	Segmentation = 5,
	SurfaceNormals = 6,
	Infrared = 7
};

struct FFSDSCaptureSettings
{
	EFSDSImageType ImageType = EFSDSImageType::Scene;
	int32 Width = 785;
	int32 Height = 785;
	float FOV_Degrees = 90.f;

	// Auto-exposure
	float AutoExposureSpeed = 100.f;
	float AutoExposureBias = 0.f;
	float AutoExposureMaxBrightness = 0.64f;
	float AutoExposureMinBrightness = 0.03f;

	// Motion blur
	float MotionBlurAmount = 0.f; // 0 = disabled (default for sim)

	// Gamma
	float TargetGamma = 1.0f;

	// Projection
	bool bOrthographic = false;
	float OrthoWidth = 5.12f;
};

struct FFSDSGimbalSettings
{
	bool bEnabled = false;
	float Stabilization = 0.f; // 0 = no stabilization, 1 = full
	FRotator Rotation = FRotator::ZeroRotator;
};

struct FFSDSNoiseSettings
{
	bool bEnabled = false;
	float RandContrib = 0.f;
	float RandSpeed = 1.f;
	float RandSize = 1.f;
	float RandDensity = 1.f;
	float HorzWaveContrib = 0.f;
	float HorzWaveStrength = 0.f;
	float HorzWaveVertSize = 0.f;
	float HorzWaveScreenSize = 0.f;
	float HorzNoiseLinesContrib = 0.f;
	float HorzDistortionContrib = 0.f;
	float HorzDistortionStrength = 0.f;
};

struct FFSDSCameraSettings
{
	FString Name;
	FVector Position = FVector::ZeroVector; // meters
	FRotator Rotation = FRotator::ZeroRotator;
	TArray<FFSDSCaptureSettings> CaptureSettings;
	FFSDSGimbalSettings Gimbal;
	TMap<int32, FFSDSNoiseSettings> NoiseSettings; // per image type
};

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

	// Noise parameters (apply to all sensor types, 0 = no noise)
	float GpsPositionNoiseStd = 0.f;  // meters
	float GpsVelocityNoiseStd = 0.f;  // m/s
	float AccelNoiseStd = 0.f;        // cm/s²
	float GyroNoiseStd = 0.f;         // rad/s
	float AccelBiasStd = 0.f;         // cm/s² per sqrt(s)
	float GyroBiasStd = 0.f;          // rad/s per sqrt(s)
	float VelocityNoiseStd = 0.f;     // m/s (GSS)
	float RangeNoiseStd = 0.f;        // cm (LiDAR)
	float DropoutRate = 0.f;          // [0,1] (LiDAR)
};

struct FFSDSVehiclePhysics
{
	float Mass = 290.f;
	FString Drivetrain = TEXT("RWD"); // RWD, FWD, AWD
	float WheelRadius = 0.200f;      // meters
	float WheelWidth = 0.190f;       // meters
	float MaxSteerAngle = 28.f;      // degrees
	float MotorMaxTorque = 230.f;    // Nm (at motor)
	float MotorMaxPower = 80000.f;   // Watts
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
	TMap<FString, FFSDSCameraSettings> Cameras;
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

	TMap<FString, FFSDSVehicleSettings> Vehicles;

	/** Get the first (default) vehicle settings */
	const FFSDSVehicleSettings* GetDefaultVehicle() const;

private:
	FFSDSSettings() = default;

	void ParseVehicle(const FString& Name, TSharedPtr<FJsonObject> VehicleObj);
	void ParseSensor(const FString& Name, TSharedPtr<FJsonObject> SensorObj, FFSDSVehicleSettings& Vehicle);
	void ParseCamera(const FString& Name, TSharedPtr<FJsonObject> CameraObj, FFSDSVehicleSettings& Vehicle);

	FString SettingsString;

	static FFSDSSettings* Instance;
};
