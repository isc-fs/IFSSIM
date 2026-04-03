#include "FSDSSettings.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

FFSDSSettings* FFSDSSettings::Instance = nullptr;

FFSDSSettings& FFSDSSettings::Get()
{
	if (!Instance)
	{
		Instance = new FFSDSSettings();
	}
	return *Instance;
}

bool FFSDSSettings::AutoLoad()
{
	// Search order (matching original FSDS):
	// 1. Next to executable (LaunchDir)
	// 2. Project root
	// 3. User AppData
	TArray<FString> SearchPaths = {
		FPaths::Combine(FPaths::LaunchDir(), TEXT("settings.json")),
		FPaths::Combine(FPaths::ProjectDir(), TEXT("settings.json")),
		FPaths::Combine(FPaths::Combine(FPlatformProcess::UserSettingsDir(), TEXT("IFSSIM")), TEXT("settings.json"))
	};

	for (const FString& Path : SearchPaths)
	{
		if (FPaths::FileExists(Path))
		{
			UE_LOG(LogTemp, Log, TEXT("FSDS Settings: Loading from %s"), *Path);
			return LoadFromFile(Path);
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("FSDS Settings: No settings.json found. Using defaults."));
	// Create default vehicle
	FFSDSVehicleSettings DefaultVehicle;
	DefaultVehicle.Name = TEXT("FSCar");

	// Default sensors
	FFSDSSensorSettings ImuSensor;
	ImuSensor.Name = TEXT("Imu"); ImuSensor.SensorType = 2; ImuSensor.bEnabled = true;
	DefaultVehicle.Sensors.Add(TEXT("Imu"), ImuSensor);

	FFSDSSensorSettings GpsSensor;
	GpsSensor.Name = TEXT("Gps"); GpsSensor.SensorType = 3; GpsSensor.bEnabled = true;
	DefaultVehicle.Sensors.Add(TEXT("Gps"), GpsSensor);

	FFSDSSensorSettings LidarSensor;
	LidarSensor.Name = TEXT("Lidar1"); LidarSensor.SensorType = 6; LidarSensor.bEnabled = true;
	DefaultVehicle.Sensors.Add(TEXT("Lidar1"), LidarSensor);

	FFSDSSensorSettings GssSensor;
	GssSensor.Name = TEXT("GSS"); GssSensor.SensorType = 7; GssSensor.bEnabled = true;
	DefaultVehicle.Sensors.Add(TEXT("GSS"), GssSensor);

	// Default camera
	FFSDSCameraSettings Cam;
	Cam.Name = TEXT("cam1");
	Cam.Position = FVector(1.6f, 0.f, -0.2f);
	FFSDSCaptureSettings Cap;
	Cap.ImageType = EFSDSImageType::Scene;
	Cap.Width = 785; Cap.Height = 785; Cap.FOV_Degrees = 90.f;
	Cam.CaptureSettings.Add(Cap);
	DefaultVehicle.Cameras.Add(TEXT("cam1"), Cam);

	Vehicles.Add(TEXT("FSCar"), DefaultVehicle);
	return false;
}

bool FFSDSSettings::LoadFromFile(const FString& FilePath)
{
	FString JsonString;
	if (!FFileHelper::LoadFileToString(JsonString, *FilePath))
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS Settings: Failed to read %s"), *FilePath);
		return false;
	}
	return LoadFromString(JsonString);
}

bool FFSDSSettings::LoadFromString(const FString& JsonString)
{
	SettingsString = JsonString;

	TSharedPtr<FJsonObject> Root;
	TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
	if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS Settings: Failed to parse JSON"));
		return false;
	}

	// Root settings
	Root->TryGetNumberField(TEXT("SettingsVersion"), SettingsVersion);
	Root->TryGetStringField(TEXT("SimMode"), SimMode);
	Root->TryGetStringField(TEXT("ViewMode"), ViewMode);
	Root->TryGetNumberField(TEXT("ClockSpeed"), ClockSpeed);
	Root->TryGetStringField(TEXT("SpectatorServerPassword"), SpectatorServerPassword);

	// Vehicles
	const TSharedPtr<FJsonObject>* VehiclesObj;
	if (Root->TryGetObjectField(TEXT("Vehicles"), VehiclesObj))
	{
		for (auto& Pair : (*VehiclesObj)->Values)
		{
			const TSharedPtr<FJsonObject>* VehicleObj;
			if (Pair.Value->TryGetObject(VehicleObj))
			{
				ParseVehicle(Pair.Key, *VehicleObj);
			}
		}
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS Settings: Loaded — %d vehicles"), Vehicles.Num());
	for (auto& V : Vehicles)
	{
		UE_LOG(LogTemp, Log, TEXT("  Vehicle '%s': %d sensors, %d cameras"),
			*V.Key, V.Value.Sensors.Num(), V.Value.Cameras.Num());
	}

	return true;
}

void FFSDSSettings::ParseVehicle(const FString& Name, TSharedPtr<FJsonObject> VehicleObj)
{
	FFSDSVehicleSettings Vehicle;
	Vehicle.Name = Name;

	VehicleObj->TryGetStringField(TEXT("VehicleType"), Vehicle.VehicleType);
	VehicleObj->TryGetBoolField(TEXT("EnableCollisions"), Vehicle.bEnableCollisions);
	VehicleObj->TryGetBoolField(TEXT("AllowAPIAlways"), Vehicle.bAllowAPIAlways);
	VehicleObj->TryGetBoolField(TEXT("AutoCreate"), Vehicle.bAutoCreate);

	// Sensors
	const TSharedPtr<FJsonObject>* SensorsObj;
	if (VehicleObj->TryGetObjectField(TEXT("Sensors"), SensorsObj))
	{
		for (auto& Pair : (*SensorsObj)->Values)
		{
			const TSharedPtr<FJsonObject>* SensorObj;
			if (Pair.Value->TryGetObject(SensorObj))
			{
				ParseSensor(Pair.Key, *SensorObj, Vehicle);
			}
		}
	}

	// Cameras
	const TSharedPtr<FJsonObject>* CamerasObj;
	if (VehicleObj->TryGetObjectField(TEXT("Cameras"), CamerasObj))
	{
		for (auto& Pair : (*CamerasObj)->Values)
		{
			const TSharedPtr<FJsonObject>* CameraObj;
			if (Pair.Value->TryGetObject(CameraObj))
			{
				ParseCamera(Pair.Key, *CameraObj, Vehicle);
			}
		}
	}

	Vehicles.Add(Name, Vehicle);
}

void FFSDSSettings::ParseSensor(const FString& Name, TSharedPtr<FJsonObject> SensorObj, FFSDSVehicleSettings& Vehicle)
{
	FFSDSSensorSettings Sensor;
	Sensor.Name = Name;

	int32 SensorType = 0;
	SensorObj->TryGetNumberField(TEXT("SensorType"), SensorType);
	Sensor.SensorType = SensorType;
	SensorObj->TryGetBoolField(TEXT("Enabled"), Sensor.bEnabled);

	// Position (meters in settings, stored as meters)
	double X = 0, Y = 0, Z = 0;
	SensorObj->TryGetNumberField(TEXT("X"), X); Sensor.Position.X = X;
	SensorObj->TryGetNumberField(TEXT("Y"), Y); Sensor.Position.Y = Y;
	SensorObj->TryGetNumberField(TEXT("Z"), Z); Sensor.Position.Z = Z;

	double Roll = 0, Pitch = 0, Yaw = 0;
	SensorObj->TryGetNumberField(TEXT("Roll"), Roll); Sensor.Rotation.Roll = Roll;
	SensorObj->TryGetNumberField(TEXT("Pitch"), Pitch); Sensor.Rotation.Pitch = Pitch;
	SensorObj->TryGetNumberField(TEXT("Yaw"), Yaw); Sensor.Rotation.Yaw = Yaw;

	// LiDAR specific
	int32 IntVal;
	double DblVal;
	if (SensorObj->TryGetNumberField(TEXT("NumberOfChannels"), IntVal)) Sensor.NumberOfChannels = IntVal;
	if (SensorObj->TryGetNumberField(TEXT("PointsPerSecond"), IntVal)) Sensor.PointsPerSecond = IntVal;
	if (SensorObj->TryGetNumberField(TEXT("RotationsPerSecond"), DblVal)) Sensor.RotationsPerSecond = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("VerticalFOVUpper"), DblVal)) Sensor.VerticalFOVUpper = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("VerticalFOVLower"), DblVal)) Sensor.VerticalFOVLower = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("HorizontalFOVStart"), DblVal)) Sensor.HorizontalFOVStart = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("HorizontalFOVEnd"), DblVal)) Sensor.HorizontalFOVEnd = DblVal;
	SensorObj->TryGetBoolField(TEXT("DrawDebugPoints"), Sensor.bDrawDebugPoints);

	Vehicle.Sensors.Add(Name, Sensor);
}

void FFSDSSettings::ParseCamera(const FString& Name, TSharedPtr<FJsonObject> CameraObj, FFSDSVehicleSettings& Vehicle)
{
	FFSDSCameraSettings Camera;
	Camera.Name = Name;

	// Position (meters)
	double X = 0, Y = 0, Z = 0;
	CameraObj->TryGetNumberField(TEXT("X"), X); Camera.Position.X = X;
	CameraObj->TryGetNumberField(TEXT("Y"), Y); Camera.Position.Y = Y;
	CameraObj->TryGetNumberField(TEXT("Z"), Z); Camera.Position.Z = Z;

	double Roll = 0, Pitch = 0, Yaw = 0;
	CameraObj->TryGetNumberField(TEXT("Roll"), Roll); Camera.Rotation.Roll = Roll;
	CameraObj->TryGetNumberField(TEXT("Pitch"), Pitch); Camera.Rotation.Pitch = Pitch;
	CameraObj->TryGetNumberField(TEXT("Yaw"), Yaw); Camera.Rotation.Yaw = Yaw;

	// Capture settings
	const TArray<TSharedPtr<FJsonValue>>* CaptureArray;
	if (CameraObj->TryGetArrayField(TEXT("CaptureSettings"), CaptureArray))
	{
		for (auto& CaptureVal : *CaptureArray)
		{
			TSharedPtr<FJsonObject> CapObj = CaptureVal->AsObject();
			if (!CapObj.IsValid()) continue;

			FFSDSCaptureSettings Cap;
			int32 ImgType = 0;
			CapObj->TryGetNumberField(TEXT("ImageType"), ImgType);
			Cap.ImageType = static_cast<EFSDSImageType>(ImgType);

			int32 W = 785, H = 785;
			CapObj->TryGetNumberField(TEXT("Width"), W); Cap.Width = W;
			CapObj->TryGetNumberField(TEXT("Height"), H); Cap.Height = H;

			double FOV = 90.0;
			CapObj->TryGetNumberField(TEXT("FOV_Degrees"), FOV); Cap.FOV_Degrees = FOV;

			Camera.CaptureSettings.Add(Cap);
		}
	}

	// Default capture settings if none provided
	if (Camera.CaptureSettings.Num() == 0)
	{
		FFSDSCaptureSettings DefaultCap;
		Camera.CaptureSettings.Add(DefaultCap);
	}

	Vehicle.Cameras.Add(Name, Camera);
}

const FFSDSVehicleSettings* FFSDSSettings::GetDefaultVehicle() const
{
	if (Vehicles.Num() == 0) return nullptr;

	// Prefer "FSCar"
	if (Vehicles.Contains(TEXT("FSCar")))
		return &Vehicles[TEXT("FSCar")];

	// Return first vehicle
	for (auto& Pair : Vehicles)
		return &Pair.Value;

	return nullptr;
}
