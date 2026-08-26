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
	{
		double SeedVal = 0.0;
		if (Root->TryGetNumberField(TEXT("ScenarioSeed"), SeedVal)) ScenarioSeed = (int32)SeedVal;
	}
	Root->TryGetStringField(TEXT("SpectatorServerPassword"), SpectatorServerPassword);

	// Plant selection. A top-level "Plant" block, because which simulator
	// runs the car is not a property of the car.
	const TSharedPtr<FJsonObject>* PlantObj;
	if (Root->TryGetObjectField(TEXT("Plant"), PlantObj))
	{
		double PlantDbl = 0.0;
		(*PlantObj)->TryGetStringField(TEXT("Type"), PlantType);
		(*PlantObj)->TryGetStringField(TEXT("FmuPath"), PlantFmuPath);
		if ((*PlantObj)->TryGetNumberField(TEXT("RoadProbeUpM"), PlantDbl))   RoadProbeUpM   = (float)PlantDbl;
		if ((*PlantObj)->TryGetNumberField(TEXT("RoadProbeDownM"), PlantDbl)) RoadProbeDownM = (float)PlantDbl;
		if ((*PlantObj)->TryGetNumberField(TEXT("RoadDefaultMu"), PlantDbl))  RoadDefaultMu  = (float)PlantDbl;

		PlantType = PlantType.ToLower();
		if (PlantType != TEXT("chaos") && PlantType != TEXT("shadow") && PlantType != TEXT("fmu"))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FSDS Settings: Plant.Type '%s' not recognised (chaos|shadow|fmu) — using chaos"),
				*PlantType);
			PlantType = TEXT("chaos");
		}
		UE_LOG(LogTemp, Log, TEXT("FSDS Settings: Plant type=%s fmu='%s' probe=+%.2f/-%.2f m mu=%.2f"),
			*PlantType, *PlantFmuPath, RoadProbeUpM, RoadProbeDownM, RoadDefaultMu);
	}

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
		UE_LOG(LogTemp, Log, TEXT("  Vehicle '%s': %d sensors"),
			*V.Key, V.Value.Sensors.Num());
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

	// Cameras block in settings.json is silently ignored — camera sensors
	// were removed in perf/strip-cameras (the real IFS-08 has no cameras
	// and the autonomy never consumed /camera/* topics). Leaving the
	// JSON keys behind is harmless; the parser just skips them.

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
		if ((*PhysicsObj)->TryGetNumberField(TEXT("RollingResistance"), DblVal)) P.RollingResistance = DblVal;
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

		// Vehicle dynamics fields consumed by ComputeTireLoadsParametric.
		// All optional — defaults in FFSDSVehiclePhysics are the IFS-08 numbers.
		if ((*PhysicsObj)->TryGetNumberField(TEXT("Wheelbase"), DblVal)) P.Wheelbase = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("TrackFront"), DblVal)) P.TrackFront = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("TrackRear"), DblVal)) P.TrackRear = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("RollCenterFront"), DblVal)) P.RollCenterFront = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("RollCenterRear"), DblVal)) P.RollCenterRear = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("RollStiffnessFront"), DblVal)) P.RollStiffnessFront = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("RollStiffnessRear"), DblVal)) P.RollStiffnessRear = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("HeaveStiffness"), DblVal)) P.HeaveStiffness = DblVal;
		if ((*PhysicsObj)->TryGetNumberField(TEXT("PitchStiffness"), DblVal)) P.PitchStiffness = DblVal;

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
	if (SensorObj->TryGetNumberField(TEXT("MaxRange"), DblVal)) Sensor.MaxRange = DblVal;
	SensorObj->TryGetBoolField(TEXT("DrawDebugPoints"), Sensor.bDrawDebugPoints);

	// Per-channel max-range override (Hesai datasheet App. A.1.1 style).
	// Format: "PerChannelMaxRangeM": [r0, r1, ..., r{NumChannels-1}]
	// in metres. Length must equal NumberOfChannels — validation lives
	// in FSDSLidarSensor::OnSettingsApplied; mismatches log a warning
	// and the global MaxRange is used instead.
	{
		const TArray<TSharedPtr<FJsonValue>>* RangeArr = nullptr;
		if (SensorObj->TryGetArrayField(TEXT("PerChannelMaxRangeM"), RangeArr) && RangeArr)
		{
			Sensor.PerChannelMaxRangeM.Reset(RangeArr->Num());
			for (const TSharedPtr<FJsonValue>& Entry : *RangeArr)
			{
				double V = 0.0;
				if (Entry->TryGetNumber(V))
				{
					Sensor.PerChannelMaxRangeM.Add((float)V);
				}
			}
		}
	}

	// LidarPath: "cpu" (default) or "gpu" — see #223. Only honoured when
	// SensorType==6 (LiDAR); read by FSDSLidarSensor::BeginPlay. We
	// store it on every sensor's settings struct uniformly because
	// ParseSensor is shared across types; non-LiDAR sensors ignore it.
	{
		FString PathVal;
		if (SensorObj->TryGetStringField(TEXT("LidarPath"), PathVal)) Sensor.LidarPath = PathVal;
	}

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
