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

	// VehiclePhysics
	const TSharedPtr<FJsonObject>* PhysicsObj;
	if (VehicleObj->TryGetObjectField(TEXT("VehiclePhysics"), PhysicsObj))
	{
		double DblVal;
		FString StrVal;
		auto& P = Vehicle.Physics;

		if ((*PhysicsObj)->TryGetNumberField(TEXT("Mass"), DblVal)) P.Mass = DblVal;
		if ((*PhysicsObj)->TryGetStringField(TEXT("Drivetrain"), StrVal)) P.Drivetrain = StrVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("WheelRadius"), DblVal)) P.WheelRadius = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("WheelWidth"), DblVal)) P.WheelWidth = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("MaxSteerAngle"), DblVal)) P.MaxSteerAngle = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("MotorMaxTorque"), DblVal)) P.MotorMaxTorque = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("MotorMaxPower"), DblVal)) P.MotorMaxPower = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("MaxRegenTorque"), DblVal)) P.MaxRegenTorque = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("MaxRegenPower"), DblVal)) P.MaxRegenPower = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("GearRatio"), DblVal)) P.GearRatio = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("DrivetrainEfficiency"), DblVal)) P.DrivetrainEfficiency = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("CdA"), DblVal)) P.CdA = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("ClA"), DblVal)) P.ClA = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("AeroBalanceFront"), DblVal)) P.AeroBalanceFront = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("TireMu"), DblVal)) P.TireMu = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("WeightDistFront"), DblVal)) P.WeightDistFront = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("CoGHeight"), DblVal)) P.CoGHeight = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("SuspensionDamping"), DblVal)) P.SuspensionDamping = DblVal;

		// Motor torque curve arrays
		const TArray<TSharedPtr<FJsonValue>>* RPMArr;
		const TArray<TSharedPtr<FJsonValue>>* TorqueArr;
		if ((*PhysicsObj)->TryGetArrayField(TEXT("MotorRPM"), RPMArr))
		{
			for (auto& V : *RPMArr) P.MotorRPM.Add(V->AsNumber());
		}
		if ((*PhysicsObj)->TryGetArrayField(TEXT("MotorTorque"), TorqueArr))
		{
			for (auto& V : *TorqueArr) P.MotorTorque.Add(V->AsNumber());
		}

		// Pacejka Magic Formula coefficients (optional block)
		const TSharedPtr<FJsonObject>* PacejkaObj;
		if ((*PhysicsObj)->TryGetObjectField(TEXT("Pacejka"), PacejkaObj))
		{
			double D;
			if ((*PacejkaObj)->TryGetNumberField(TEXT("LatB"), D)) P.Pacejka.LatB = (float)D;
			if ((*PacejkaObj)->TryGetNumberField(TEXT("LatC"), D)) P.Pacejka.LatC = (float)D;
			if ((*PacejkaObj)->TryGetNumberField(TEXT("LatE"), D)) P.Pacejka.LatE = (float)D;
			if ((*PacejkaObj)->TryGetNumberField(TEXT("LonB"), D)) P.Pacejka.LonB = (float)D;
			if ((*PacejkaObj)->TryGetNumberField(TEXT("LonC"), D)) P.Pacejka.LonC = (float)D;
			if ((*PacejkaObj)->TryGetNumberField(TEXT("LonE"), D)) P.Pacejka.LonE = (float)D;
			UE_LOG(LogTemp, Log, TEXT("FSDS Settings: Pacejka loaded — lat(B=%.1f C=%.2f E=%.2f) lon(B=%.1f C=%.2f E=%.2f)"),
				P.Pacejka.LatB, P.Pacejka.LatC, P.Pacejka.LatE,
				P.Pacejka.LonB, P.Pacejka.LonC, P.Pacejka.LonE);
		}

		UE_LOG(LogTemp, Log, TEXT("FSDS Settings: VehiclePhysics loaded — Mass=%.0f, %s, Motor=%.0fNm/%.0fW, GR=%.3f, CdA=%.2f, ClA=%.1f, mu=%.2f"),
			P.Mass, *P.Drivetrain, P.MotorMaxTorque, P.MotorMaxPower, P.GearRatio, P.CdA, P.ClA, P.TireMu);
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

	// Noise parameters (all sensor types)
	if (SensorObj->TryGetNumberField(TEXT("GpsPositionNoiseStd"), DblVal)) Sensor.GpsPositionNoiseStd = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("GpsVelocityNoiseStd"), DblVal)) Sensor.GpsVelocityNoiseStd = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("AccelNoiseStd"), DblVal)) Sensor.AccelNoiseStd = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("GyroNoiseStd"), DblVal)) Sensor.GyroNoiseStd = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("AccelBiasStd"), DblVal)) Sensor.AccelBiasStd = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("GyroBiasStd"), DblVal)) Sensor.GyroBiasStd = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("AccelBiasTau"), DblVal)) Sensor.AccelBiasTau = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("GyroBiasTau"), DblVal)) Sensor.GyroBiasTau = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("VelocityNoiseStd"), DblVal)) Sensor.VelocityNoiseStd = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("RangeNoiseStd"), DblVal)) Sensor.RangeNoiseStd = DblVal;
	if (SensorObj->TryGetNumberField(TEXT("DropoutRate"), DblVal)) Sensor.DropoutRate = DblVal;

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

			// Auto-exposure
			double DblVal;
			if (CapObj->TryGetNumberField(TEXT("AutoExposureSpeed"), DblVal)) Cap.AutoExposureSpeed = DblVal;
			if (CapObj->TryGetNumberField(TEXT("AutoExposureBias"), DblVal)) Cap.AutoExposureBias = DblVal;
			if (CapObj->TryGetNumberField(TEXT("AutoExposureMaxBrightness"), DblVal)) Cap.AutoExposureMaxBrightness = DblVal;
			if (CapObj->TryGetNumberField(TEXT("AutoExposureMinBrightness"), DblVal)) Cap.AutoExposureMinBrightness = DblVal;

			// Motion blur
			if (CapObj->TryGetNumberField(TEXT("MotionBlurAmount"), DblVal)) Cap.MotionBlurAmount = DblVal;

			// Gamma
			if (CapObj->TryGetNumberField(TEXT("TargetGamma"), DblVal)) Cap.TargetGamma = DblVal;

			// Projection
			bool bOrtho = false;
			if (CapObj->TryGetBoolField(TEXT("ProjectionMode"), bOrtho)) Cap.bOrthographic = bOrtho;
			if (CapObj->TryGetNumberField(TEXT("OrthoWidth"), DblVal)) Cap.OrthoWidth = DblVal;

			Camera.CaptureSettings.Add(Cap);
		}
	}

	// Gimbal settings
	const TSharedPtr<FJsonObject>* GimbalObj;
	if (CameraObj->TryGetObjectField(TEXT("Gimbal"), GimbalObj))
	{
		(*GimbalObj)->TryGetBoolField(TEXT("Enabled"), Camera.Gimbal.bEnabled);
		double Stab = 0;
		if ((*GimbalObj)->TryGetNumberField(TEXT("Stabilization"), Stab)) Camera.Gimbal.Stabilization = Stab;
	}

	// Noise settings (per image type)
	const TSharedPtr<FJsonObject>* NoiseObj;
	if (CameraObj->TryGetObjectField(TEXT("NoiseSettings"), NoiseObj))
	{
		for (auto& NoisePair : (*NoiseObj)->Values)
		{
			int32 ImgType = FCString::Atoi(*NoisePair.Key);
			const TSharedPtr<FJsonObject>* NObj;
			if (NoisePair.Value->TryGetObject(NObj))
			{
				FFSDSNoiseSettings Noise;
				(*NObj)->TryGetBoolField(TEXT("Enabled"), Noise.bEnabled);
				double D;
				if ((*NObj)->TryGetNumberField(TEXT("RandContrib"), D)) Noise.RandContrib = D;
				if ((*NObj)->TryGetNumberField(TEXT("RandSpeed"), D)) Noise.RandSpeed = D;
				if ((*NObj)->TryGetNumberField(TEXT("RandSize"), D)) Noise.RandSize = D;
				if ((*NObj)->TryGetNumberField(TEXT("RandDensity"), D)) Noise.RandDensity = D;
				if ((*NObj)->TryGetNumberField(TEXT("HorzWaveContrib"), D)) Noise.HorzWaveContrib = D;
				if ((*NObj)->TryGetNumberField(TEXT("HorzWaveStrength"), D)) Noise.HorzWaveStrength = D;
				if ((*NObj)->TryGetNumberField(TEXT("HorzWaveVertSize"), D)) Noise.HorzWaveVertSize = D;
				if ((*NObj)->TryGetNumberField(TEXT("HorzWaveScreenSize"), D)) Noise.HorzWaveScreenSize = D;
				if ((*NObj)->TryGetNumberField(TEXT("HorzNoiseLinesContrib"), D)) Noise.HorzNoiseLinesContrib = D;
				if ((*NObj)->TryGetNumberField(TEXT("HorzDistortionContrib"), D)) Noise.HorzDistortionContrib = D;
				if ((*NObj)->TryGetNumberField(TEXT("HorzDistortionStrength"), D)) Noise.HorzDistortionStrength = D;
				Camera.NoiseSettings.Add(ImgType, Noise);
			}
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
