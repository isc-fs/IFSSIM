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
};

struct FFSDSCameraSettings
{
	FString Name;
	FVector Position = FVector::ZeroVector; // meters
	FRotator Rotation = FRotator::ZeroRotator;
	TArray<FFSDSCaptureSettings> CaptureSettings;
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
};

struct FFSDSVehicleSettings
{
	FString Name = TEXT("FSCar");
	FString VehicleType = TEXT("PhysXCar");
	bool bEnableCollisions = true;
	bool bAllowAPIAlways = true;
	bool bAutoCreate = true;

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
