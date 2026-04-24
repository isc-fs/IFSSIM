#include "FSDSVehiclePawn.h"
#include "FSDSPacejkaTireModel.h"
#include "Components/InputComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/BoxComponent.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Engine/World.h"
#include "UObject/ConstructorHelpers.h"

AFSDSVehiclePawn::AFSDSVehiclePawn()
{
	PrimaryActorTick.bCanEverTick = true;

	// Get the Chaos vehicle movement component
	VehicleMovement = CastChecked<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

	// Try to load the Formula Student skeletal mesh
	static ConstructorHelpers::FObjectFinder<USkeletalMesh> CarMesh(
		TEXT("/Game/Vehicle/TechnionCar/FormulaMesh.FormulaMesh"));

	if (CarMesh.Succeeded())
	{
		GetMesh()->SetSkeletalMesh(CarMesh.Object);
		GetMesh()->SetSimulatePhysics(true);

		// Load physics asset
		static ConstructorHelpers::FObjectFinder<UPhysicsAsset> PhysAsset(
			TEXT("/Game/Vehicle/TechnionCar/FormulaMesh_PhysicsAsset.FormulaMesh_PhysicsAsset"));
		if (PhysAsset.Succeeded())
		{
			GetMesh()->SetPhysicsAsset(PhysAsset.Object);
			UE_LOG(LogTemp, Log, TEXT("FSDS: PhysicsAsset loaded"));
		}

		// Note: FormulaAnim was a UE4 PhysX animation BP — removed (incompatible with Chaos)
		// Wheel animation will come from Chaos vehicle component automatically

		// Check if skeleton has wheel bones AND a valid skeleton
		if (CarMesh.Object->GetSkeleton() != nullptr &&
			CarMesh.Object->GetRefSkeleton().FindBoneIndex(FName("WheelFL")) != INDEX_NONE)
		{
			// Material assignment is deferred to BeginPlay where we create
			// a dynamic material instance with a visible color

			bChaosVehicleActive = true;
			UE_LOG(LogTemp, Log, TEXT("FSDS: FormulaMesh loaded with wheel bones — Chaos vehicle ACTIVE"));
		}
		else
		{
			bChaosVehicleActive = false;
			UE_LOG(LogTemp, Warning, TEXT("FSDS: FormulaMesh loaded but NO wheel bones found. Bones in skeleton:"));
			const FReferenceSkeleton& RefSkel = CarMesh.Object->GetRefSkeleton();
			for (int32 i = 0; i < FMath::Min(RefSkel.GetRawBoneNum(), 20); i++)
			{
				UE_LOG(LogTemp, Warning, TEXT("  Bone[%d]: %s"), i, *RefSkel.GetBoneName(i).ToString());
			}
		}
	}
	else
	{
		// Fallback: use a simple cube mesh
		static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
		if (CubeMesh.Succeeded())
		{
			UStaticMeshComponent* PlaceholderMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PlaceholderMesh"));
			PlaceholderMesh->SetupAttachment(GetMesh());
			PlaceholderMesh->SetStaticMesh(CubeMesh.Object);
			PlaceholderMesh->SetRelativeScale3D(FVector(4.0f, 2.0f, 1.2f));
			PlaceholderMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		}

		bChaosVehicleActive = false;
		UE_LOG(LogTemp, Warning, TEXT("FSDS: FormulaMesh not found — using placeholder cube. Chaos vehicle disabled."));
	}

	// Configure Chaos vehicle physics only if skeleton is valid
	if (bChaosVehicleActive)
	{
		SetupVehicleMovement();
	}
	else if (VehicleMovement)
	{
		// Skeleton failed to load — disable Chaos to prevent crash
		VehicleMovement->Deactivate();
		GetMesh()->SetSimulatePhysics(false);

		// Add FloatingPawnMovement as minimal fallback
		FallbackMovement = CreateDefaultSubobject<UFloatingPawnMovement>(TEXT("FallbackMovement"));
		FallbackMovement->MaxSpeed = 2000.f;
		FallbackMovement->Acceleration = 4000.f;
		FallbackMovement->Deceleration = 8000.f;

		UE_LOG(LogTemp, Warning, TEXT("FSDS: Skeleton failed — Chaos disabled, using FloatingPawnMovement"));
	}

	// Spring arm for chase camera
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(GetRootComponent());
	SpringArm->TargetArmLength = 600.f;
	SpringArm->SetRelativeLocation(FVector(-50.f, 0.f, 200.f));
	SpringArm->SetRelativeRotation(FRotator(-15.f, 0.f, 0.f));
	SpringArm->bUsePawnControlRotation = false;
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;
	SpringArm->bInheritYaw = true;
	SpringArm->bDoCollisionTest = false;

	// Follow camera
	FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
	FollowCamera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);

	// --- Sensors (non-camera — cameras created in BeginPlay from settings) ---
	LidarSensor = CreateDefaultSubobject<UFSDSLidarSensor>(TEXT("LidarSensor"));
	ImuSensor = CreateDefaultSubobject<UFSDSImuSensor>(TEXT("ImuSensor"));
	GpsSensor = CreateDefaultSubobject<UFSDSGpsSensor>(TEXT("GpsSensor"));
	GssSensor = CreateDefaultSubobject<UFSDSGssSensor>(TEXT("GssSensor"));
	DistanceSensor = CreateDefaultSubobject<UFSDSDistanceSensor>(TEXT("DistanceSensor"));
	BarometerSensor = CreateDefaultSubobject<UFSDSBarometerSensor>(TEXT("BarometerSensor"));
	MagnetometerSensor = CreateDefaultSubobject<UFSDSMagnetometerSensor>(TEXT("MagnetometerSensor"));
}

void AFSDSVehiclePawn::SetupVehicleMovement()
{
	if (!VehicleMovement) return;

	// === IFS-08 EMRAX 228 Powertrain — electric, motor-side semantics ===
	// Motor: 230 Nm stall, 240 Nm peak (1-3183 RPM), 80 kW from ~3183 RPM
	// up, 6500 RPM redline. Single-speed chain reduction 2.909 (32T/11T),
	// 92% drivetrain efficiency.
	//
	// MaxTorque is MOTOR-side (pre-gearbox) and TorqueCurve keys are MOTOR
	// RPM. Chaos applies ForwardGearRatio × FinalRatio × TransmissionEff
	// internally to get wheel torque. Previous implementation baked the
	// gear ratio and efficiency into MaxTorque=643 while leaving gear=1
	// and the default TransmissionEfficiency=0.9 — the latter silently
	// stole 10% on top, so the sim was delivering 579 Nm at the wheel
	// instead of the 643 Nm spec. Moving to motor-side semantics makes
	// the numbers match the EMRAX datasheet and the team's own tuning.
	VehicleMovement->EngineSetup.MaxRPM = 6500.f;
	VehicleMovement->EngineSetup.MaxTorque = 240.f; // motor-side peak
	// ICE leftovers — neutralise them for an electric drivetrain:
	//  - EngineIdleRPM 1200 → 0: no idle, motor sits at 0 until commanded.
	//  - EngineBrakeEffect 0.05 → 0: no compression/crank drag when
	//    throttle=0. Retardation is pure drag + regen (fix/22).
	//  - EngineRevUpMOI 5.0 → 0.05: EMRAX rotor inertia is ~0.02 kg·m².
	//    The default is tuned for a V8 (≈100× heavier), which artificially
	//    slows the motor's response to throttle changes.
	//  - EngineRevDownRate 600 → 10000: with idle=0 and no engine-brake
	//    the ramp-down rate barely matters, but big value keeps Chaos
	//    from dragging RPM down artificially when throttle=0.
	VehicleMovement->EngineSetup.EngineIdleRPM = 0.f;
	VehicleMovement->EngineSetup.EngineBrakeEffect = 0.f;
	VehicleMovement->EngineSetup.EngineRevUpMOI = 0.05f;
	VehicleMovement->EngineSetup.EngineRevDownRate = 10000.f;

	FRichCurve* TorqueCurve = VehicleMovement->EngineSetup.TorqueCurve.GetRichCurve();
	TorqueCurve->Reset();
	// Motor-side normalized (T(rpm) / 240 Nm). Flat 1.0 up to the
	// power-limited transition ω_t = 80000/240 ≈ 333 rad/s ≈ 3183 RPM,
	// then T = 80 kW / ω i.e. normalized = 764000 / (RPM × 240).
	TorqueCurve->AddKey(0.f,    0.958f);  // 230 Nm stall
	TorqueCurve->AddKey(1000.f, 1.000f);  // 240 Nm peak
	TorqueCurve->AddKey(2000.f, 1.000f);
	TorqueCurve->AddKey(3000.f, 1.000f);
	TorqueCurve->AddKey(4000.f, 0.796f);  // 80 kW / (4000·2π/60) = 191 Nm → 0.796
	TorqueCurve->AddKey(5000.f, 0.637f);
	TorqueCurve->AddKey(6000.f, 0.531f);
	TorqueCurve->AddKey(6500.f, 0.490f);  // redline

	// --- Transmission: single-speed chain reduction ---
	// Chaos handles the gear ratio and efficiency natively — the torque
	// curve above is motor-side.
	VehicleMovement->TransmissionSetup.bUseAutomaticGears = false;
	VehicleMovement->TransmissionSetup.GearChangeTime = 0.0f;
	VehicleMovement->TransmissionSetup.ForwardGearRatios.Reset();
	VehicleMovement->TransmissionSetup.ForwardGearRatios.Add(2.909f); // chain 32/11
	VehicleMovement->TransmissionSetup.ReverseGearRatios.Reset();
	VehicleMovement->TransmissionSetup.ReverseGearRatios.Add(2.909f);
	VehicleMovement->TransmissionSetup.FinalRatio = 1.0f;
	VehicleMovement->TransmissionSetup.TransmissionEfficiency = 0.92f;
	// Single-gear shifts must never fire — set the thresholds out of reach.
	VehicleMovement->TransmissionSetup.ChangeUpRPM = 99999.f;
	VehicleMovement->TransmissionSetup.ChangeDownRPM = 0.f;

	// --- Differential (RWD) ---
	VehicleMovement->DifferentialSetup.DifferentialType = EVehicleDifferential::RearWheelDrive;

	// --- Steering curve (speed-dependent) ---
	FRichCurve* SteeringCurve = VehicleMovement->SteeringSetup.SteeringCurve.GetRichCurve();
	SteeringCurve->Reset();
	SteeringCurve->AddKey(0.f, 1.0f);    // Full lock at standstill
	SteeringCurve->AddKey(60.f, 0.8f);   // 80% at 60 km/h
	SteeringCurve->AddKey(120.f, 0.6f);  // 60% at 120 km/h

	// --- Wheels ---
	VehicleMovement->WheelSetups.SetNum(4);

	VehicleMovement->WheelSetups[0].WheelClass = UFSDSWheelFront::StaticClass();
	VehicleMovement->WheelSetups[0].BoneName = FName("WheelFL");
	VehicleMovement->WheelSetups[0].AdditionalOffset = FVector(0.f, -8.f, 0.f);

	VehicleMovement->WheelSetups[1].WheelClass = UFSDSWheelFront::StaticClass();
	VehicleMovement->WheelSetups[1].BoneName = FName("WheelFR");
	VehicleMovement->WheelSetups[1].AdditionalOffset = FVector(0.f, 8.f, 0.f);

	VehicleMovement->WheelSetups[2].WheelClass = UFSDSWheelRear::StaticClass();
	VehicleMovement->WheelSetups[2].BoneName = FName("WheelRL");
	VehicleMovement->WheelSetups[2].AdditionalOffset = FVector(0.f, -8.f, 0.f);

	VehicleMovement->WheelSetups[3].WheelClass = UFSDSWheelRear::StaticClass();
	VehicleMovement->WheelSetups[3].BoneName = FName("WheelRR");
	VehicleMovement->WheelSetups[3].AdditionalOffset = FVector(0.f, 8.f, 0.f);

	// === IFS-08 Mass & Inertia ===
	// IFS-08 driving mass (corner-weight-measured):
	//   Car    210 kg   Driver ≈ 65 kg  → total 275 kg
	// Susp_Geometry sheet (CAD authoritative):
	//   Wheelbase  1600 mm
	//   CoG height  300 mm
	//   Weight dist front 0.438 → CoG is 0.562 · 1600 = 899 mm from
	//   the front axle, i.e. 99 mm rearward of the wheelbase midpoint
	//   (800 mm) → -9.9 cm in UE X relative to the mesh center.
	// Settings.json overrides all three of these via ApplyPhysicsSettings;
	// the hardcoded values below are only used if settings.json is
	// missing.
	VehicleMovement->Mass = 275.f;
	VehicleMovement->InertiaTensorScale = FVector(1.0f, 1.4f, 1.1f);
	VehicleMovement->CenterOfMassOverride = FVector(-10.f, 0.f, 0.f);
	VehicleMovement->bEnableCenterOfMassOverride = true;
}

UFSDSCameraSensor* AFSDSVehiclePawn::GetCamera(const FString& Name) const
{
	const auto* Found = Cameras.Find(Name);
	return Found ? *Found : nullptr;
}

void AFSDSVehiclePawn::SetupSensorsFromSettings()
{
	const FFSDSVehicleSettings* VehicleSettings = FFSDSSettings::Get().GetDefaultVehicle();
	if (!VehicleSettings)
	{
		// Create a default camera if no settings
		UFSDSCameraSensor* DefaultCam = NewObject<UFSDSCameraSensor>(this, FName("cam1"));
		DefaultCam->SetupAttachment(GetRootComponent());
		DefaultCam->SetRelativeLocation(FVector(160.f, 0.f, 50.f));
		DefaultCam->RegisterComponent();
		Cameras.Add(TEXT("cam1"), DefaultCam);
		UE_LOG(LogTemp, Log, TEXT("FSDS: Created default camera 'cam1'"));
		return;
	}

	// Create cameras from settings
	for (auto& CamPair : VehicleSettings->Cameras)
	{
		const FFSDSCameraSettings& CamSettings = CamPair.Value;

		UFSDSCameraSensor* Cam = NewObject<UFSDSCameraSensor>(this, FName(*CamPair.Key));
		Cam->SetupAttachment(GetRootComponent());

		// Position: settings in meters, UE in cm. Convention is UE-native
		// (X = forward, Y = right, Z = up); matches the LiDAR mount block
		// above. Previously the camera path negated Z with the comment
		// "Z is inverted in settings (negative = up)" — an AirSim/NED
		// hangover — which meant settings.json had to bury one sign flip
		// only for cameras while LiDAR was up-positive. Harmonised.
		Cam->SetRelativeLocation(FVector(
			CamSettings.Position.X * 100.f,
			CamSettings.Position.Y * 100.f,
			CamSettings.Position.Z * 100.f
		));
		Cam->SetRelativeRotation(CamSettings.Rotation);

		// Configure from capture settings
		if (CamSettings.CaptureSettings.Num() > 0)
		{
			Cam->Configure(CamPair.Key, CamSettings.CaptureSettings[0]);
		}

		Cam->RegisterComponent();
		Cameras.Add(CamPair.Key, Cam);

		UE_LOG(LogTemp, Log, TEXT("FSDS: Created camera '%s' at (%.0f, %.0f, %.0f)cm"),
			*CamPair.Key,
			CamSettings.Position.X * 100.f,
			CamSettings.Position.Y * 100.f,
			CamSettings.Position.Z * 100.f);
	}

	// Configure LiDAR from settings
	if (LidarSensor)
	{
		for (auto& SensorPair : VehicleSettings->Sensors)
		{
			if (SensorPair.Value.SensorType == 6 && SensorPair.Value.bEnabled)
			{
				LidarSensor->NumberOfChannels = SensorPair.Value.NumberOfChannels;
				LidarSensor->PointsPerSecond = SensorPair.Value.PointsPerSecond;
				LidarSensor->RotationsPerSecond = SensorPair.Value.RotationsPerSecond;
				LidarSensor->VerticalFOVUpper = SensorPair.Value.VerticalFOVUpper;
				LidarSensor->VerticalFOVLower = SensorPair.Value.VerticalFOVLower;
				LidarSensor->HorizontalFOVStart = SensorPair.Value.HorizontalFOVStart;
				LidarSensor->HorizontalFOVEnd = SensorPair.Value.HorizontalFOVEnd;
				LidarSensor->SensorOffset = SensorPair.Value.Position * 100.f; // meters to cm
				LidarSensor->RangeNoiseStd = SensorPair.Value.RangeNoiseStd;
				LidarSensor->DropoutRate = SensorPair.Value.DropoutRate;
				break;
			}
		}
	}

	// Configure noise from settings — GPS (SensorType 3)
	if (GpsSensor)
	{
		for (auto& SensorPair : VehicleSettings->Sensors)
		{
			if (SensorPair.Value.SensorType == 3 && SensorPair.Value.bEnabled)
			{
				GpsSensor->GpsPositionNoiseStd = SensorPair.Value.GpsPositionNoiseStd;
				GpsSensor->GpsVelocityNoiseStd = SensorPair.Value.GpsVelocityNoiseStd;
				break;
			}
		}
	}

	// Configure noise from settings — IMU (SensorType 2).
	// Settings are in SI (m/s², rad/s). The IMU sensor's
	// Output.LinearAcceleration is in cm/s² (gravity is added as
	// `WorldAccel.Z += 980.f`, matching UE's cm-based units), so the
	// accel-side values need a ×100 bump to match that scale. Gyro
	// values are rad/s on both sides and go through unchanged.
	if (ImuSensor)
	{
		for (auto& SensorPair : VehicleSettings->Sensors)
		{
			if (SensorPair.Value.SensorType == 2 && SensorPair.Value.bEnabled)
			{
				ImuSensor->AccelNoiseStd = SensorPair.Value.AccelNoiseStd * 100.f;
				ImuSensor->GyroNoiseStd = SensorPair.Value.GyroNoiseStd;
				ImuSensor->AccelBiasStd = SensorPair.Value.AccelBiasStd * 100.f;
				ImuSensor->GyroBiasStd = SensorPair.Value.GyroBiasStd;
				break;
			}
		}
	}

	// Configure noise from settings — GSS (SensorType 7)
	if (GssSensor)
	{
		for (auto& SensorPair : VehicleSettings->Sensors)
		{
			if (SensorPair.Value.SensorType == 7 && SensorPair.Value.bEnabled)
			{
				GssSensor->VelocityNoiseStd = SensorPair.Value.VelocityNoiseStd;
				break;
			}
		}
	}

	// Apply vehicle physics from settings (overrides hardcoded IFS-08 defaults)
	if (VehicleSettings && VehicleMovement && bChaosVehicleActive)
	{
		auto& P = VehicleSettings->Physics;

		// Mass
		VehicleMovement->Mass = P.Mass;

		// Drivetrain
		if (P.Drivetrain == TEXT("RWD"))
			VehicleMovement->DifferentialSetup.DifferentialType = EVehicleDifferential::RearWheelDrive;
		else if (P.Drivetrain == TEXT("FWD"))
			VehicleMovement->DifferentialSetup.DifferentialType = EVehicleDifferential::FrontWheelDrive;
		else if (P.Drivetrain == TEXT("AWD"))
		{
			VehicleMovement->DifferentialSetup.DifferentialType = EVehicleDifferential::AllWheelDrive;
			VehicleMovement->DifferentialSetup.FrontRearSplit = P.WeightDistFront;
		}

		// Motor torque curve from settings — MOTOR-side values (pre-gear).
		// Chaos multiplies by ForwardGearRatio × FinalRatio × TransmissionEff
		// internally, so we just set MaxTorque and the curve at motor RPM.
		// Gear ratio and efficiency land on TransmissionSetup below.
		if (P.MotorRPM.Num() > 0 && P.MotorRPM.Num() == P.MotorTorque.Num())
		{
			float PeakMotorTorque = 0.f;
			for (float T : P.MotorTorque)
			{
				if (T > PeakMotorTorque) PeakMotorTorque = T;
			}

			VehicleMovement->EngineSetup.MaxTorque = PeakMotorTorque;
			FRichCurve* TC = VehicleMovement->EngineSetup.TorqueCurve.GetRichCurve();
			TC->Reset();

			for (int32 i = 0; i < P.MotorRPM.Num(); i++)
			{
				float MotorT = P.MotorTorque[i];
				// Power limit: T = min(T, P_max / ω_motor) — both motor-side.
				float MotorOmega = P.MotorRPM[i] * 2.f * PI / 60.f;
				if (MotorOmega > 1.f)
				{
					float PowerLimitT = P.MotorMaxPower / MotorOmega;
					MotorT = FMath::Min(MotorT, PowerLimitT);
				}
				float Normalized = (PeakMotorTorque > 0.f) ? MotorT / PeakMotorTorque : 0.f;
				TC->AddKey(P.MotorRPM[i], Normalized);
			}

			UE_LOG(LogTemp, Log, TEXT("FSDS: Motor curve from settings — %d points, peak %.0f Nm at motor (→ %.0f Nm at wheel after %.2f gear × %.2f eff)"),
				P.MotorRPM.Num(), PeakMotorTorque,
				PeakMotorTorque * P.GearRatio * P.DrivetrainEfficiency,
				P.GearRatio, P.DrivetrainEfficiency);
		}

		// Transmission: single-speed chain reduction driven by settings.
		// Overwrites the ForwardGearRatios[0] that SetupVehicleMovement set
		// to the hardcoded default so the user's settings.json gear ratio
		// actually takes effect.
		if (VehicleMovement->TransmissionSetup.ForwardGearRatios.Num() > 0)
		{
			VehicleMovement->TransmissionSetup.ForwardGearRatios[0] = P.GearRatio;
		}
		if (VehicleMovement->TransmissionSetup.ReverseGearRatios.Num() > 0)
		{
			VehicleMovement->TransmissionSetup.ReverseGearRatios[0] = P.GearRatio;
		}
		VehicleMovement->TransmissionSetup.TransmissionEfficiency = P.DrivetrainEfficiency;

		// Aero
		CdA = P.CdA;
		ClA = P.ClA;
		AeroBalanceFront = P.AeroBalanceFront;

		// Regen parameters — shadowed for per-tick power-cap math.
		MaxRegenTorque = P.MaxRegenTorque;
		MaxRegenPower = P.MaxRegenPower;
		GearRatio = P.GearRatio;
		WheelRadius = P.WheelRadius;

		// Size the rear-wheel brake torque to the max motor regen
		// referred to the wheel. Brake input (0-1) then represents a
		// fraction of max motor regen; the Tick caps further by
		// MaxRegenPower/ω_motor. Front wheels keep MaxBrakeTorque=0
		// from the class default — no hydraulic service brake on the
		// real car.
		float RearPerWheelMax = (P.MaxRegenTorque * P.GearRatio * P.DrivetrainEfficiency) / 2.f;
		for (int32 i = 0; i < VehicleMovement->Wheels.Num(); i++)
		{
			UChaosVehicleWheel* W = VehicleMovement->Wheels[i];
			if (!W) continue;
			// RL=2, RR=3 per the WheelSetups order in SetupVehicleMovement
			if (i == 2 || i == 3) W->MaxBrakeTorque = RearPerWheelMax;

			// Pacejka Magic Formula — bake lateral and longitudinal slip curves
			// into each wheel, replacing the flat FrictionForceMultiplier model.
			FSDSPacejka::BakeToWheel(W, P.Pacejka, P.TireMu);
		}

		UE_LOG(LogTemp, Log, TEXT("FSDS: Physics from settings — %.0fkg %s, motor %.0fNm/%.0fW, regen %.0fNm/%.0fW, mu=%.2f"),
			P.Mass, *P.Drivetrain, P.MotorMaxTorque, P.MotorMaxPower, P.MaxRegenTorque, P.MaxRegenPower, P.TireMu);
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS: Configured %d cameras, LiDAR, IMU, GPS, GSS from settings (with noise)"),
		Cameras.Num());
}

void AFSDSVehiclePawn::BeginPlay()
{
	Super::BeginPlay();

	// Load settings and create cameras
	FFSDSSettings::Get().AutoLoad();
	SetupSensorsFromSettings();

	// Load and apply FSDS car materials at runtime.
	// Materials are loaded from /Game/Vehicle/TechnionCar/Materials/ (cooked in pak).
	// Some materials reference legacy SuvCar automotive textures that were removed;
	// those will return null and fall back to mat_british_green.
	if (GetMesh() && GetMesh()->GetSkeletalMeshAsset())
	{
		const FString MatPath = TEXT("/Game/Vehicle/TechnionCar/Materials/");

		// Load all car materials — some may be null if their texture deps were removed
		UMaterialInterface* GreenMat    = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_british_green.mat_british_green"));
		UMaterialInterface* RedMat      = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "m_red_real_formula_mat.m_red_real_formula_mat"));
		UMaterialInterface* CarbonMat   = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_carbonFiber_Formula.mat_carbonFiber_Formula"));
		UMaterialInterface* ChassisMat  = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_chassis_Formula.mat_chassis_Formula"));
		UMaterialInterface* YellowMat   = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_yellow_Formula.mat_yellow_Formula"));
		UMaterialInterface* DashMat     = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_dashboard_Formula.mat_dashboard_Formula"));
		UMaterialInterface* JuntsMat    = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_junts_Formula.mat_junts_Formula"));
		UMaterialInterface* NoseMat     = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "nose_red_mat.nose_red_mat"));

		// Best available fallback: use GreenMat if chassis/red also fail
		UMaterialInterface* FallbackMat = ChassisMat ? ChassisMat : (RedMat ? RedMat : GreenMat);

		UE_LOG(LogTemp, Log, TEXT("FSDS Materials: green=%s red=%s carbon=%s chassis=%s yellow=%s dash=%s junts=%s nose=%s"),
			GreenMat  ? TEXT("OK") : TEXT("FAIL"),
			RedMat    ? TEXT("OK") : TEXT("FAIL"),
			CarbonMat ? TEXT("OK") : TEXT("FAIL"),
			ChassisMat? TEXT("OK") : TEXT("FAIL"),
			YellowMat ? TEXT("OK") : TEXT("FAIL"),
			DashMat   ? TEXT("OK") : TEXT("FAIL"),
			JuntsMat  ? TEXT("OK") : TEXT("FAIL"),
			NoseMat   ? TEXT("OK") : TEXT("FAIL"));

		int32 NumMaterials = GetMesh()->GetNumMaterials();
		USkeletalMesh* SkelMesh = GetMesh()->GetSkeletalMeshAsset();

		// Count null vs valid embedded material refs in the skeleton asset
		int32 ValidMats = 0, NullMats = 0;
		for (int32 i = 0; i < NumMaterials; i++)
		{
			if (SkelMesh->GetMaterials()[i].MaterialInterface) ValidMats++; else NullMats++;
		}
		UE_LOG(LogTemp, Log, TEXT("FSDS: %d material slots — %d embedded valid, %d embedded null"), NumMaterials, ValidMats, NullMats);

		// Apply materials slot by slot.
		// Slot assignment priority (highest first):
		//   1. Specific runtime material that loaded successfully
		//   2. Embedded material from the skeleton asset (if valid)
		//   3. FallbackMat (chassis > red > green) — ensures no slot is grey
		int32 Fixed = 0;
		for (int32 i = 0; i < NumMaterials; i++)
		{
			// Element 112: body panel → British Racing Green (or fallback)
			if (i == 112)
			{
				GetMesh()->SetMaterial(i, GreenMat ? GreenMat : FallbackMat);
				continue;
			}

			// Try the embedded skeleton material reference first
			UMaterialInterface* ExistingMat = SkelMesh->GetMaterials()[i].MaterialInterface;
			if (ExistingMat)
			{
				GetMesh()->SetMaterial(i, ExistingMat);
			}
			else
			{
				// No embedded material — use fallback so slot isn't grey
				GetMesh()->SetMaterial(i, FallbackMat);
				Fixed++;
			}
		}
		UE_LOG(LogTemp, Log, TEXT("FSDS: Applied materials — %d from skeleton, %d null→fallback, elem112→green"),
			ValidMats, Fixed);
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS: Vehicle pawn spawned at %s (Chaos: %s)"),
		*GetActorLocation().ToString(),
		bChaosVehicleActive ? TEXT("YES") : TEXT("NO - fallback mode"));
}

void AFSDSVehiclePawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Acceleration tracking
	FVector CurrentVelocity = GetVelocity();
	if (DeltaTime > 0.f)
	{
		CurrentAcceleration = (CurrentVelocity - PreviousVelocity) / DeltaTime;
	}
	PreviousVelocity = CurrentVelocity;

	// Apply aerodynamic forces
	if (bChaosVehicleActive)
	{
		ApplyAeroForces();
	}

	// Apply controls
	if (bChaosVehicleActive && VehicleMovement)
	{
		// Chaos vehicle mode
		VehicleMovement->SetThrottleInput(CurrentControls.Throttle);
		VehicleMovement->SetSteeringInput(CurrentControls.Steering);

		// Brake channel = motor regen, power-capped by the battery's
		// cell input current limit. Driver/autonomy `Brake` input is a
		// fraction of max motor regen torque; we scale it down by the
		// ratio between the power-limited torque at current motor ω and
		// the full motor torque. At low speeds the cap doesn't bind
		// (plenty of torque headroom); at high speeds scale < 1 so the
		// actual decel scales with 1/v (constant power shape).
		float EffectiveBrake = CurrentControls.Brake;
		if (EffectiveBrake > 0.f && MaxRegenTorque > 0.f)
		{
			float VFwd = FMath::Abs(FVector::DotProduct(GetVelocity(), GetActorForwardVector())) * 0.01f;  // cm/s -> m/s
			float OmegaMotor = (VFwd / FMath::Max(WheelRadius, 0.01f)) * GearRatio;  // rad/s
			if (OmegaMotor > 0.1f)
			{
				float TPowerLimited = MaxRegenPower / OmegaMotor;  // Nm at motor
				float Scale = FMath::Min(1.f, TPowerLimited / MaxRegenTorque);
				EffectiveBrake *= Scale;
			}
		}
		VehicleMovement->SetBrakeInput(EffectiveBrake);
		VehicleMovement->SetHandbrakeInput(CurrentControls.bHandbrake);

		// Electric: always in gear 1 (single speed)
		VehicleMovement->SetTargetGear(1, true);
	}
	else if (FallbackMovement)
	{
		// Fallback: FloatingPawnMovement (no physics, no gravity)
		if (FMath::Abs(CurrentControls.Throttle) > 0.01f)
		{
			AddMovementInput(GetActorForwardVector(), CurrentControls.Throttle);
		}
		if (FMath::Abs(CurrentControls.Steering) > 0.01f)
		{
			FRotator NewRot = GetActorRotation();
			NewRot.Yaw += CurrentControls.Steering * 2.0f;
			SetActorRotation(NewRot);
		}
	}
}

void AFSDSVehiclePawn::ApplyAeroForces()
{
	USkeletalMeshComponent* VehicleMesh = GetMesh();
	if (!VehicleMesh || !VehicleMesh->IsSimulatingPhysics()) return;

	FVector Velocity = GetVelocity(); // cm/s
	float SpeedMs = Velocity.Size() / 100.f; // m/s

	if (SpeedMs < 1.0f) return; // No aero below 1 m/s

	const float Rho = 1.225f; // Air density kg/m³
	float Q = 0.5f * Rho * SpeedMs * SpeedMs; // Dynamic pressure (Pa)

	// Drag force (opposing velocity, in Newtons)
	float Fdrag = Q * CdA;
	FVector DragForce = -Velocity.GetSafeNormal() * Fdrag * 100.f; // N → UE force units (mass*cm/s²)

	// Downforce (negative Z in world, in Newtons)
	float Fdown = Q * ClA;

	// Apply drag at CoG
	VehicleMesh->AddForce(DragForce, NAME_None, false);

	// Apply downforce split front/rear at approximate axle positions
	// Wheelbase ~1627mm in UE X. Front axle at +813mm from center, rear at -813mm
	FTransform ActorTransform = GetActorTransform();
	FVector FrontAxleLocal(813.f, 0.f, 0.f); // cm, local space
	FVector RearAxleLocal(-813.f, 0.f, 0.f);
	FVector FrontAxleWorld = ActorTransform.TransformPosition(FrontAxleLocal);
	FVector RearAxleWorld = ActorTransform.TransformPosition(RearAxleLocal);

	FVector DownDir = FVector(0.f, 0.f, -1.f) * 100.f; // N → UE force
	VehicleMesh->AddForceAtLocation(DownDir * Fdown * AeroBalanceFront, FrontAxleWorld, NAME_None);
	VehicleMesh->AddForceAtLocation(DownDir * Fdown * (1.f - AeroBalanceFront), RearAxleWorld, NAME_None);
}

void AFSDSVehiclePawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis("MoveForward", this, &AFSDSVehiclePawn::OnThrottleInput);
	PlayerInputComponent->BindAxis("MoveRight", this, &AFSDSVehiclePawn::OnSteeringInput);
	PlayerInputComponent->BindAxis("Brake", this, &AFSDSVehiclePawn::OnBrakeInput);

	PlayerInputComponent->BindAction("Handbrake", IE_Pressed, this, &AFSDSVehiclePawn::OnHandbrakePressed);
	PlayerInputComponent->BindAction("Handbrake", IE_Released, this, &AFSDSVehiclePawn::OnHandbrakeReleased);
}

void AFSDSVehiclePawn::OnThrottleInput(float Value)
{
	if (bEbsLatched) return;
	if (!bApiControlEnabled)
		CurrentControls.Throttle = FMath::Clamp(Value, -1.f, 1.f);
}

void AFSDSVehiclePawn::OnSteeringInput(float Value)
{
	if (bEbsLatched) return;
	if (!bApiControlEnabled)
		CurrentControls.Steering = FMath::Clamp(Value, -1.f, 1.f);
}

void AFSDSVehiclePawn::OnBrakeInput(float Value)
{
	if (bEbsLatched) return;
	if (!bApiControlEnabled)
		CurrentControls.Brake = FMath::Clamp(Value, 0.f, 1.f);
}

void AFSDSVehiclePawn::OnHandbrakePressed()
{
	if (bEbsLatched) return;
	if (!bApiControlEnabled) CurrentControls.bHandbrake = true;
}

void AFSDSVehiclePawn::OnHandbrakeReleased()
{
	if (bEbsLatched) return;
	if (!bApiControlEnabled) CurrentControls.bHandbrake = false;
}

void AFSDSVehiclePawn::SetCarControls(const FCarControls& Controls)
{
	// While EBS is latched the autonomy cannot drive the car — it must
	// be released explicitly (ReleaseEbs, or a reset).
	if (bEbsLatched) return;
	CurrentControls = Controls;
}

void AFSDSVehiclePawn::ActivateEbs()
{
	// Zero the drive and steering channels, engage handbrake, and lock
	// all inputs. The handbrake is the sim analog of the real car's
	// pneumatic EBS clamp — full rear-axle brake until manually reset.
	CurrentControls.Throttle = 0.f;
	CurrentControls.Steering = 0.f;
	CurrentControls.Brake = 0.f;
	CurrentControls.bHandbrake = true;
	bEbsLatched = true;
	bApiControlEnabled = false;
}

void AFSDSVehiclePawn::ReleaseEbs()
{
	// Unlatch and hand control back to whoever wants it. Called on sim
	// reset or explicit operator release; the autonomy stack itself is
	// NEVER able to invoke this (mirrors the real car — driver only).
	CurrentControls.bHandbrake = false;
	bEbsLatched = false;
}

AFSDSVehiclePawn::FCarControls AFSDSVehiclePawn::GetCarControls() const
{
	return CurrentControls;
}

AFSDSVehiclePawn::FCarState AFSDSVehiclePawn::GetCarState() const
{
	FCarState State;

	State.Speed = GetVelocity().Size() / 100.f;
	State.Position = GetActorLocation();
	State.Orientation = GetActorQuat();
	State.LinearVelocity = GetVelocity();
	State.AngularVelocity = GetMesh() ? GetMesh()->GetPhysicsAngularVelocityInRadians() : FVector::ZeroVector;
	State.LinearAcceleration = CurrentAcceleration;
	State.bHandbrake = CurrentControls.bHandbrake;

	if (VehicleMovement)
	{
		State.Gear = VehicleMovement->GetCurrentGear();
		State.RPM = VehicleMovement->GetEngineRotationSpeed();
		State.MaxRPM = VehicleMovement->GetEngineMaxRotationSpeed();
	}

	// Regen snapshot — compute from current speed and brake request so it
	// tracks whatever the Tick is applying right now. All values are at
	// the motor (pre-gear) side to match datasheet semantics.
	State.RegenMaxTorqueLimit = MaxRegenTorque;
	State.RegenMaxPowerLimit = MaxRegenPower;
	if (MaxRegenTorque > 0.f)
	{
		const float VFwd = FMath::Abs(FVector::DotProduct(GetVelocity(), GetActorForwardVector())) * 0.01f; // cm/s → m/s
		const float OmegaMotor = (VFwd / FMath::Max(WheelRadius, 0.01f)) * GearRatio; // rad/s
		float TAvail = MaxRegenTorque;
		if (OmegaMotor > 0.1f)
		{
			const float TPowerLimited = MaxRegenPower / OmegaMotor;
			TAvail = FMath::Min(MaxRegenTorque, TPowerLimited);
		}
		State.RegenAvailTorque = TAvail;
		State.RegenTorque = FMath::Clamp(CurrentControls.Brake, 0.f, 1.f) * TAvail;
		State.RegenPower = State.RegenTorque * OmegaMotor;
	}

	State.Timestamp = FPlatformTime::Cycles64();
	return State;
}
