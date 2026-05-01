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

	// === IFS-08 EMRAX 228 Powertrain ===
	// Motor: EMRAX 228, 230 Nm peak, 80 kW, 6500 RPM redline
	// Gear ratio: 2.909 (32/11), drivetrain efficiency: 92%
	// Torque at wheel = motor_torque * gear_ratio * efficiency
	// (Local constants were hardcoded but unused; the settings path
	//  overrides both below. Member GearRatio is shadowed by settings
	//  in ApplyPhysicsSettings.)

	// Chaos engine setup. We own the powertrain via UEmraxMotor and
	// override per-wheel drive torque each tick via SetDriveTorque
	// (see Tick() below) — Chaos's internally-computed engine torque
	// gets discarded at the wheel level. We DO NOT zero MaxTorque or
	// flatten the TorqueCurve here: doing so triggered NaN AABB
	// bounds during vehicle init on 2026-04-29 (Chaos's vehicle
	// simulator computes EngineRevDownRate × Sqr((Omega - idle/2) /
	// MaxOmega) and similar terms that go degenerate when the curve
	// collapses to zero; that NaN propagated into the skeletal mesh's
	// root-bone transform and the GJK collision iterator hit its
	// limit on the first physics tick). Leaving the original curve
	// in place is harmless because SetDriveTorqueOverride runs after
	// Chaos applies engine torque and replaces it.
	//
	// EngineIdleRPM = 0 IS still set: it's the change that actually
	// fixes our SLAM-visible bug (the 1200 RPM idle floor that fed
	// iSAM2 a 10 m/s velocity prior on a parked car). Setting only
	// idle to 0 (without zeroing MaxTorque/curve) leaves Chaos's
	// init math well-conditioned.
	VehicleMovement->EngineSetup.MaxRPM = 6500.f;
	VehicleMovement->EngineSetup.MaxTorque = 643.f; // Full EMRAX peak at wheel
	VehicleMovement->EngineSetup.EngineIdleRPM = 0.f;
	FRichCurve* TorqueCurve = VehicleMovement->EngineSetup.TorqueCurve.GetRichCurve();
	TorqueCurve->Reset();
	// EMRAX 228 torque curve (normalized, power-limited only) — kept
	// for Chaos's init even though SetDriveTorque overrides per-wheel
	// torque every tick.
	TorqueCurve->AddKey(0.f,    0.958f);
	TorqueCurve->AddKey(1000.f, 1.000f);
	TorqueCurve->AddKey(2000.f, 1.000f);
	TorqueCurve->AddKey(3000.f, 0.900f);
	TorqueCurve->AddKey(4000.f, 0.700f);
	TorqueCurve->AddKey(5000.f, 0.560f);
	TorqueCurve->AddKey(6000.f, 0.470f);
	TorqueCurve->AddKey(6500.f, 0.430f);

	// --- Transmission (single speed, electric) ---
	// Electric motor: single fixed gear, no shifting
	VehicleMovement->TransmissionSetup.bUseAutomaticGears = false;
	VehicleMovement->TransmissionSetup.GearChangeTime = 0.0f;
	VehicleMovement->TransmissionSetup.ForwardGearRatios.Reset();
	VehicleMovement->TransmissionSetup.ForwardGearRatios.Add(1.0f); // Single gear (reduction already in torque)
	VehicleMovement->TransmissionSetup.ReverseGearRatios.Reset();
	VehicleMovement->TransmissionSetup.ReverseGearRatios.Add(1.0f);
	VehicleMovement->TransmissionSetup.FinalRatio = 1.0f; // No additional reduction

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
	// Total mass: 290 kg (car 210 + driver 80)
	// Wheelbase: 1627 mm, weight dist front: 43.8%
	// CoG at 713mm from front axle = 813.5mm - 713mm = 100.5mm behind mesh center
	// CoG height: 344mm from ground
	VehicleMovement->Mass = 290.f;
	VehicleMovement->InertiaTensorScale = FVector(1.0f, 1.4f, 1.1f);
	// CoG offset: negative X = rearward (43.8% front means rear-biased)
	// Mesh center is roughly at wheelbase/2 = 813mm from front
	// CoG at 713mm from front → 100mm behind center → -10cm in UE X
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

		// Position: settings uses meters, UE uses cm
		Cam->SetRelativeLocation(FVector(
			CamSettings.Position.X * 100.f,
			CamSettings.Position.Y * 100.f,
			CamSettings.Position.Z * -100.f // Z is inverted in settings (negative = up)
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
			CamSettings.Position.Z * -100.f);
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

	// Configure noise from settings — IMU (SensorType 2)
	if (ImuSensor)
	{
		for (auto& SensorPair : VehicleSettings->Sensors)
		{
			if (SensorPair.Value.SensorType == 2 && SensorPair.Value.bEnabled)
			{
				ImuSensor->AccelNoiseStd = SensorPair.Value.AccelNoiseStd;
				ImuSensor->GyroNoiseStd = SensorPair.Value.GyroNoiseStd;
				ImuSensor->AccelBiasStd = SensorPair.Value.AccelBiasStd;
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

		// NOTE: a parametric CoG nudge to hit P.WeightDistFront is NOT
		// applied here. The PhysicsAsset (FormulaMesh_PhysicsAsset)
		// ships with a forward-biased authored CoM that runtime
		// overrides cannot fully correct (BodyInstance.COMNudge
		// saturates non-monotonically; UpdateMassProperties resets the
		// body mass to the asset value, blowing away VehicleMovement->
		// Mass). The proper fix is to author the physics asset itself
		// so its CoM and per-bone masses match the IFS-08 spec — see
		// the follow-up branch for that work. The load-transfer RPC
		// returns the *truth* signal as Chaos has it, so the autonomy
		// can still consume relative wheel loads correctly even while
		// the absolute distribution is biased forward.

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

		// Motor torque curve from settings (if provided)
		if (P.MotorRPM.Num() > 0 && P.MotorRPM.Num() == P.MotorTorque.Num())
		{
			float PeakWheelTorque = 0.f;
			for (float T : P.MotorTorque)
			{
				float WheelT = T * P.GearRatio * P.DrivetrainEfficiency;
				if (WheelT > PeakWheelTorque) PeakWheelTorque = WheelT;
			}

			VehicleMovement->EngineSetup.MaxTorque = PeakWheelTorque;
			FRichCurve* TC = VehicleMovement->EngineSetup.TorqueCurve.GetRichCurve();
			TC->Reset();

			for (int32 i = 0; i < P.MotorRPM.Num(); i++)
			{
				float WheelT = P.MotorTorque[i] * P.GearRatio * P.DrivetrainEfficiency;
				// Power limit: T = min(T, P_max / omega)
				float MotorOmega = P.MotorRPM[i] * 2.f * PI / 60.f;
				if (MotorOmega > 1.f)
				{
					float PowerLimitT = (P.MotorMaxPower / MotorOmega) * P.GearRatio * P.DrivetrainEfficiency;
					WheelT = FMath::Min(WheelT, PowerLimitT);
				}
				float Normalized = (PeakWheelTorque > 0.f) ? WheelT / PeakWheelTorque : 0.f;
				TC->AddKey(P.MotorRPM[i], Normalized);
			}

			UE_LOG(LogTemp, Log, TEXT("FSDS: Motor curve from settings — %d points, peak %.0f Nm at wheel"),
				P.MotorRPM.Num(), PeakWheelTorque);
		}

		// Aero
		CdA = P.CdA;
		ClA = P.ClA;
		AeroBalanceFront = P.AeroBalanceFront;

		// Regen parameters — shadowed for per-tick power-cap math.
		MaxRegenTorque = P.MaxRegenTorque;
		MaxRegenPower = P.MaxRegenPower;
		GearRatio = P.GearRatio;
		WheelRadius = P.WheelRadius;

		// Vehicle-dynamics shadow consumed by ComputeTireLoadsParametric.
		// Mirrors the same per-tick caching pattern as the regen fields
		// so the parametric load-transfer doesn't reparse settings on
		// every call.
		Mass = P.Mass;
		Wheelbase = P.Wheelbase;
		WeightDistFront = P.WeightDistFront;
		CoGHeight = P.CoGHeight;
		TrackFront = P.TrackFront;
		TrackRear = P.TrackRear;
		RollCenterFront = P.RollCenterFront;
		RollCenterRear = P.RollCenterRear;
		RollStiffnessFront = P.RollStiffnessFront;
		RollStiffnessRear = P.RollStiffnessRear;
		HeaveStiffness = P.HeaveStiffness;
		PitchStiffness = P.PitchStiffness;

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

	// Instantiate the EMRAX 228 motor model. We own the powertrain
	// from here on: ApplyPhysicsSettings() neutered Chaos's EngineSetup
	// (MaxTorque=0, EngineIdleRPM=0) and Tick below feeds per-wheel
	// drive torque from this object. Default FEmraxMotorParams matches
	// the EMRAX 228 MV / LC datasheet; we forward the regen caps from
	// settings.json so a user override (e.g. a bigger battery raising
	// MaxRegenPower) is honoured by the motor model too. The other
	// EMRAX parameters (envelope, peak power, thermal budget) are
	// motor-specific and stay at the class defaults.
	if (!Motor)
	{
		Motor = NewObject<UEmraxMotor>(this, TEXT("EmraxMotor"));
	}
	if (Motor)
	{
		Motor->P.MaxRegenPowerW = MaxRegenPower;
		Motor->P.MaxRegenTorqueNm = MaxRegenTorque;
	}

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
		// Throttle is held at zero for Chaos's internal engine. The
		// rear wheels' combine method is Additive (see FSDSWheelRear),
		// so SetDriveTorque() below adds the EMRAX-computed torque on
		// top of whatever Chaos's engine produces. By zeroing the
		// throttle input here we ensure the engine's contribution is
		// always zero and EMRAX is the sole drive-torque source —
		// without disabling Chaos's engine module entirely (which
		// caused NaN bounds during init in an earlier attempt).
		VehicleMovement->SetThrottleInput(0.f);
		VehicleMovement->SetSteeringInput(CurrentControls.Steering);

		// SetBrakeInput is left at 0: regenerative braking is the
		// EMRAX motor producing negative shaft torque (driven by the
		// `brake` channel folded into the motor command below), and
		// the IFS-08 has no front hydraulic friction brake on which
		// the rear-wheel SetBrakeInput would map cleanly. Mechanical
		// stopping power for emergencies comes from the handbrake +
		// EBS latch, both of which use SetHandbrakeInput.
		VehicleMovement->SetBrakeInput(0.f);
		VehicleMovement->SetHandbrakeInput(CurrentControls.bHandbrake);

		// Electric: always in gear 1 (single speed)
		VehicleMovement->SetTargetGear(1, true);

		// --- EMRAX 228 drive-torque override ----------------------
		// We bypass Chaos's engine entirely (MaxTorque was zeroed in
		// SetupVehicleMovement) and compute the shaft torque from
		// our motor model. The motor's RPM tracks actual wheel speed
		// × gear ratio so a parked car reads zero RPM (vs Chaos's
		// 1200 idle floor that fed the SLAM velocity prior with a
		// 10 m/s phantom motion on a stationary car).
		if (Motor)
		{
			// Vehicle longitudinal speed (cm/s → m/s).
			const float VFwdMs = FMath::Abs(FVector::DotProduct(
				GetVelocity(), GetActorForwardVector())) * 0.01f;
			// Wheel angular velocity assuming no slip, then geared
			// up to motor rotor speed. WheelRadius / GearRatio are
			// captured from settings in ApplyPhysicsSettings.
			const float WheelOmega = VFwdMs / FMath::Max(WheelRadius, 0.01f);
			const float MotorOmega = WheelOmega * GearRatio;
			const float MotorRpm = MotorOmega * (60.f / (2.f * PI));
			Motor->SetMechRpm(MotorRpm);

			// Combine throttle + brake into a single signed motor
			// command in [-1, 1]. Brake feeds regen on the motor side
			// (negative torque, capped by MaxRegenPowerW inside the
			// EMRAX class). Simultaneous throttle+brake (e.g. 0.5 +
			// 0.5) cancels out — physically a driver request for two
			// opposing forces; we honour the simpler "net demand"
			// interpretation.
			const float ThrottleCmd = bEbsLatched
				? 0.f
				: FMath::Clamp(
					CurrentControls.Throttle - CurrentControls.Brake,
					-1.f, 1.f);
			const float ShaftTorqueNm = Motor->Step(ThrottleCmd, DeltaTime);

			// Shaft → axle: torque multiplied by gear ratio and the
			// drivetrain efficiency we already cache from settings.
			// The 92% default mirrors the EMRAX-driven IFS-08
			// gearbox + chain losses. Sign carries through so a
			// negative ShaftTorqueNm (regen) becomes a negative
			// per-wheel torque — Chaos applies it as a decelerating
			// force on the rear axle, which is the actual physical
			// behaviour of regen on a RWD EV.
			constexpr float DrivetrainEfficiency = 0.92f;
			const float AxleTorqueNm = ShaftTorqueNm * GearRatio * DrivetrainEfficiency;
			const float PerWheelTorqueNm = 0.5f * AxleTorqueNm;

			// SetDriveTorque takes Nm; internally scales to Nm·cm.
			// Wheels 2 and 3 are RearLeft / RearRight (RWD), per the
			// WheelSetups order in SetupVehicleMovement.
			VehicleMovement->SetDriveTorque(PerWheelTorqueNm, 2);
			VehicleMovement->SetDriveTorque(PerWheelTorqueNm, 3);

			// Diagnostic — log every ~0.5 s while the throttle is
			// non-zero, so the Output Log shows whether the EMRAX path
			// is producing torque and at what magnitude. Easy to delete
			// once the live run is verified.
			static double LastLogT = 0.0;
			const double NowT = FPlatformTime::Seconds();
			if (FMath::Abs(ThrottleCmd) > 0.01f && (NowT - LastLogT) > 0.5)
			{
				LastLogT = NowT;
				UE_LOG(LogTemp, Log,
					TEXT("EMRAX: throttle_cmd=%.3f rpm=%.1f shaft=%.1f Nm "
					     "axle=%.1f Nm per_wheel=%.1f Nm vfwd=%.2f m/s"),
					ThrottleCmd, MotorRpm, ShaftTorqueNm,
					AxleTorqueNm, PerWheelTorqueNm, VFwdMs);
			}
		}
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
		// RPM source priority: prefer the EMRAX motor model (zero at
		// standstill, scales with actual wheel speed) over Chaos's
		// GetEngineRotationSpeed() which returns the ICE-style idle
		// floor (1200 RPM) even when the car isn't moving.
		// VehicleMovement still owns MaxRPM since the EMRAX motor
		// model's MaxMechRpm matches the EngineSetup.MaxRPM (6500)
		// and any future Blueprint UI that reads it stays in sync.
		if (Motor)
		{
			State.RPM = Motor->GetMechRpm();
		}
		else
		{
			State.RPM = VehicleMovement->GetEngineRotationSpeed();
		}
		State.MaxRPM = VehicleMovement->GetEngineMaxRotationSpeed();
	}

	State.Timestamp = FPlatformTime::Cycles64();
	return State;
}

// ────────────────────────────────────────────────────────────────────
// Tire-load decomposition — Milliken §5.3 / §18.3-4 closed form.
//
// Inputs are the chassis state the caller already has (longitudinal +
// lateral acceleration, roll/pitch angles, heave). All read from the
// per-tick cached settings (Mass, Wheelbase, …) so this matches the
// numbers Chaos was actually configured against — no second source of
// truth.
// ────────────────────────────────────────────────────────────────────
AFSDSVehiclePawn::FTireLoads
AFSDSVehiclePawn::ComputeTireLoadsParametric(
	float ax, float ay, float phi, float theta, float z) const
{
	constexpr float G = 9.81f;
	const float Wf = WeightDistFront;
	const float Wr = 1.f - Wf;

	// 1. Static loads (per wheel).
	const float Fz_static_total = Mass * G;
	const float Fz_static_f = 0.5f * Wf * Fz_static_total;
	const float Fz_static_r = 0.5f * Wr * Fz_static_total;

	// 2. Longitudinal transfer (Milliken §5.3): m·ax·h_cg / L. Total
	// between axles, then split 50/50 L/R within each axle.
	const float dFz_long_total = (Wheelbase > 1e-3f)
		? Mass * ax * CoGHeight / Wheelbase : 0.f;
	const float dFz_long_f_per_wheel = -dFz_long_total / 2.f;
	const float dFz_long_r_per_wheel = +dFz_long_total / 2.f;

	// 3. Lateral GEOMETRIC transfer (§18.3): instantaneous, transmitted
	// through the suspension links at the roll-centre height of each
	// axle. Per axle: ΔFz = m · w_i · ay · h_RC,i / t_i. ay > 0 (ISO
	// 8855: accel toward LEFT, i.e. RIGHT turn) loads R, unloads L.
	const float dFz_lat_geom_f = (TrackFront > 1e-3f)
		? Mass * Wf * ay * RollCenterFront / TrackFront : 0.f;
	const float dFz_lat_geom_r = (TrackRear > 1e-3f)
		? Mass * Wr * ay * RollCenterRear / TrackRear : 0.f;

	// 4. Lateral ELASTIC transfer (§18.4): driven by the dynamic roll
	// angle phi. Per axle: ΔFz = K_phi,i · phi / t_i. Same direction
	// as geometric in steady state, but lags it through the transient.
	const float dFz_lat_elast_f = (TrackFront > 1e-3f)
		? RollStiffnessFront * phi / TrackFront : 0.f;
	const float dFz_lat_elast_r = (TrackRear > 1e-3f)
		? RollStiffnessRear * phi / TrackRear : 0.f;

	// 5. Heave + pitch. Heave hits all 4 wheels equally (the only
	// contribution that does NOT conserve total weight — it represents
	// the suspended mass being above/below static equilibrium, so
	// total spring force on the ground genuinely changes). Pitch is a
	// CoG-centred moment: front wheels lose what rear wheels gain,
	// total conserved.
	const float dFz_heave = -HeaveStiffness * z / 4.f;
	const float dFz_pitch_f = (Wheelbase > 1e-3f)
		? -PitchStiffness * theta / (2.f * Wheelbase) : 0.f;
	const float dFz_pitch_r = -dFz_pitch_f;

	// Per-wheel sums. ay > 0 → L wheels (interior of right turn) lose,
	// R wheels (exterior) gain — sign captured in the +/- on the
	// geometric and elastic terms below. Front and rear share the same
	// L/R sign convention.
	float FL = Fz_static_f + dFz_long_f_per_wheel
	         - dFz_lat_geom_f - dFz_lat_elast_f
	         + dFz_heave + dFz_pitch_f;
	float FR = Fz_static_f + dFz_long_f_per_wheel
	         + dFz_lat_geom_f + dFz_lat_elast_f
	         + dFz_heave + dFz_pitch_f;
	float RL = Fz_static_r + dFz_long_r_per_wheel
	         - dFz_lat_geom_r - dFz_lat_elast_r
	         + dFz_heave + dFz_pitch_r;
	float RR = Fz_static_r + dFz_long_r_per_wheel
	         + dFz_lat_geom_r + dFz_lat_elast_r
	         + dFz_heave + dFz_pitch_r;

	// A wheel can't push the ground. Above the wheel-lift threshold the
	// quasi-static decomposition stops being valid (the lifted wheel's
	// share doesn't redistribute here); callers can detect this by
	// comparing Total() against Mass·G.
	FTireLoads Out;
	Out.FL = FMath::Max(0.f, FL);
	Out.FR = FMath::Max(0.f, FR);
	Out.RL = FMath::Max(0.f, RL);
	Out.RR = FMath::Max(0.f, RR);
	return Out;
}

// ────────────────────────────────────────────────────────────────────
// Truth Fz — read from Chaos's per-wheel state. SpringForce is the
// suspension spring force at the contact patch this tick; it equals
// the normal contact force at low suspension velocity (which covers
// effectively all driving-relevant cases). This is the value the wheel
// solver uses to compute available longitudinal/lateral grip in the
// same tick, so it's the canonical "what is Chaos seeing" signal.
// ────────────────────────────────────────────────────────────────────
AFSDSVehiclePawn::FTireLoads AFSDSVehiclePawn::GetTireLoadsTruth() const
{
	FTireLoads Out;
	if (!VehicleMovement) return Out;
	const int32 N = VehicleMovement->GetNumWheels();
	if (N < 4) return Out;
	// WheelSetups order: 0=FL, 1=FR, 2=RL, 3=RR (see SetupVehicleMovement).
	//
	// SpringForce is stored in Chaos's internal cm-based units
	// (kg·cm/s²), the same convention UE5 uses for distance everywhere.
	// The engine's own debug overlay applies CmToM (×0.01) to display it
	// in Newtons (see ChaosWheeledVehicleMovementComponent.cpp:1801).
	// We do the same here so callers always work in SI Newtons,
	// matching the parametric path. The ratio across all four wheels
	// stays correct either way (cm units factor out), but the absolute
	// numbers only line up with the parametric Fz once converted.
	constexpr float CmToM = 0.01f;
	Out.FL = VehicleMovement->GetWheelState(0).SpringForce * CmToM;
	Out.FR = VehicleMovement->GetWheelState(1).SpringForce * CmToM;
	Out.RL = VehicleMovement->GetWheelState(2).SpringForce * CmToM;
	Out.RR = VehicleMovement->GetWheelState(3).SpringForce * CmToM;
	return Out;
}
