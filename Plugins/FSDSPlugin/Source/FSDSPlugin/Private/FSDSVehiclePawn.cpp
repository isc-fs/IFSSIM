#include "FSDSVehiclePawn.h"
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
		TEXT("/FSDSPlugin/VehicleAdv/Cars/TechnionCar/FormulaMesh.FormulaMesh"));

	if (CarMesh.Succeeded())
	{
		GetMesh()->SetSkeletalMesh(CarMesh.Object);
		GetMesh()->SetSimulatePhysics(true);

		// Load physics asset
		static ConstructorHelpers::FObjectFinder<UPhysicsAsset> PhysAsset(
			TEXT("/FSDSPlugin/VehicleAdv/Cars/TechnionCar/FormulaMesh_PhysicsAsset.FormulaMesh_PhysicsAsset"));
		if (PhysAsset.Succeeded())
		{
			GetMesh()->SetPhysicsAsset(PhysAsset.Object);
			UE_LOG(LogTemp, Log, TEXT("FSDS: PhysicsAsset loaded"));
		}

		// Load animation blueprint if available
		static ConstructorHelpers::FClassFinder<UAnimInstance> AnimBP(
			TEXT("/FSDSPlugin/VehicleAdv/Cars/TechnionCar/FormulaAnim"));
		if (AnimBP.Succeeded())
		{
			GetMesh()->SetAnimInstanceClass(AnimBP.Class);
			UE_LOG(LogTemp, Log, TEXT("FSDS: FormulaAnim loaded"));
		}

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

		UE_LOG(LogTemp, Error, TEXT("FSDS: Skeleton failed — Chaos disabled, using FloatingPawnMovement"));
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
}

void AFSDSVehiclePawn::SetupVehicleMovement()
{
	if (!VehicleMovement) return;

	// --- Engine (matching FSDS/Colosseum) ---
	VehicleMovement->EngineSetup.MaxRPM = 5700.f;
	VehicleMovement->EngineSetup.MaxTorque = 500.f;
	FRichCurve* TorqueCurve = VehicleMovement->EngineSetup.TorqueCurve.GetRichCurve();
	TorqueCurve->Reset();
	TorqueCurve->AddKey(0.f, 400.f);
	TorqueCurve->AddKey(1890.f, 500.f);
	TorqueCurve->AddKey(5730.f, 400.f);

	// --- Transmission ---
	VehicleMovement->TransmissionSetup.bUseAutomaticGears = true;
	VehicleMovement->TransmissionSetup.GearChangeTime = 0.15f;

	// --- Differential ---
	VehicleMovement->DifferentialSetup.DifferentialType = EVehicleDifferential::AllWheelDrive;
	VehicleMovement->DifferentialSetup.FrontRearSplit = 0.65f;

	// --- Steering curve (speed-dependent) ---
	FRichCurve* SteeringCurve = VehicleMovement->SteeringSetup.SteeringCurve.GetRichCurve();
	SteeringCurve->Reset();
	SteeringCurve->AddKey(0.f, 1.0f);
	SteeringCurve->AddKey(40.f, 0.7f);
	SteeringCurve->AddKey(120.f, 0.6f);

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

	// --- Physics ---
	VehicleMovement->Mass = 300.f;
	VehicleMovement->InertiaTensorScale = FVector(1.0f, 1.333f, 1.2f);
	VehicleMovement->CenterOfMassOverride = FVector(8.f, 0.f, 0.f);
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
				break;
			}
		}
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS: Configured %d cameras, LiDAR, IMU, GPS, GSS from settings"),
		Cameras.Num());
}

void AFSDSVehiclePawn::BeginPlay()
{
	Super::BeginPlay();

	// Load settings and create cameras
	FFSDSSettings::Get().AutoLoad();
	SetupSensorsFromSettings();

	// Load and apply FSDS car materials at runtime
	if (GetMesh() && GetMesh()->GetSkeletalMeshAsset())
	{
		const FString MatPath = TEXT("/FSDSPlugin/VehicleAdv/Cars/TechnionCar/matreials_and_textures/");

		UMaterialInterface* RedMat = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "m_red_real_formula_mat.m_red_real_formula_mat"));
		UMaterialInterface* CarbonMat = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_carbonFiber_Formula.mat_carbonFiber_Formula"));
		UMaterialInterface* ChassisMat = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_chassis_Formula.mat_chassis_Formula"));
		UMaterialInterface* YellowMat = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_yellow_Formula.mat_yellow_Formula"));
		UMaterialInterface* DashMat = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_dashboard_Formula.mat_dashboard_Formula"));
		UMaterialInterface* JuntsMat = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "mat_junts_Formula.mat_junts_Formula"));
		UMaterialInterface* NoseMat = LoadObject<UMaterialInterface>(nullptr, *(MatPath + "nose_red_mat.nose_red_mat"));

		int32 NumMaterials = GetMesh()->GetNumMaterials();
		USkeletalMesh* SkelMesh = GetMesh()->GetSkeletalMeshAsset();

		// Log unique slot names to understand the mesh structure
		TSet<FString> UniqueNames;
		for (int32 i = 0; i < NumMaterials; i++)
		{
			FName SlotName = SkelMesh->GetMaterials()[i].MaterialSlotName;
			UniqueNames.Add(SlotName.ToString());
		}
		// Log unique imported names (these contain the original material references)
		TMap<FString, int32> ImportedNameCounts;
		for (int32 i = 0; i < NumMaterials; i++)
		{
			FName ImportedName = SkelMesh->GetMaterials()[i].ImportedMaterialSlotName;
			FString Key = ImportedName.ToString();
			if (ImportedNameCounts.Contains(Key))
				ImportedNameCounts[Key]++;
			else
				ImportedNameCounts.Add(Key, 1);
		}
		UE_LOG(LogTemp, Log, TEXT("FSDS: %d slots, %d unique imported names:"), NumMaterials, ImportedNameCounts.Num());
		for (auto& Pair : ImportedNameCounts)
		{
			UE_LOG(LogTemp, Log, TEXT("  '%s' (%d slots)"), *Pair.Key, Pair.Value);
		}

		// Check how many slots already have valid materials loaded
		// (the /AirSim/ mount point may have resolved them automatically)
		int32 ValidMats = 0;
		int32 NullMats = 0;
		for (int32 i = 0; i < NumMaterials; i++)
		{
			UMaterialInterface* ExistingMat = SkelMesh->GetMaterials()[i].MaterialInterface;
			if (ExistingMat)
				ValidMats++;
			else
				NullMats++;
		}
		UE_LOG(LogTemp, Log, TEXT("FSDS: Materials — %d valid, %d null (out of %d)"), ValidMats, NullMats, NumMaterials);

		// Keep valid materials, fill nulls with chassis (dark) material
		int32 Fixed = 0;
		for (int32 i = 0; i < NumMaterials; i++)
		{
			UMaterialInterface* ExistingMat = SkelMesh->GetMaterials()[i].MaterialInterface;
			if (ExistingMat)
			{
				// Material resolved from the asset — use it as-is
				GetMesh()->SetMaterial(i, ExistingMat);
			}
			else
			{
				// Null — fill with dark chassis material
				GetMesh()->SetMaterial(i, ChassisMat ? ChassisMat : RedMat);
				Fixed++;
			}
		}
		UE_LOG(LogTemp, Log, TEXT("FSDS: Kept %d original materials, filled %d null slots with chassis material"),
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

	// Apply controls
	if (bChaosVehicleActive && VehicleMovement)
	{
		// Chaos vehicle mode
		VehicleMovement->SetThrottleInput(CurrentControls.Throttle);
		VehicleMovement->SetSteeringInput(CurrentControls.Steering);
		VehicleMovement->SetBrakeInput(CurrentControls.Brake);
		VehicleMovement->SetHandbrakeInput(CurrentControls.bHandbrake);

		if (CurrentControls.bIsManualGear)
		{
			VehicleMovement->SetTargetGear(CurrentControls.ManualGear, CurrentControls.bGearImmediate);
		}
		else
		{
			VehicleMovement->SetUseAutomaticGears(true);
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
	if (!bApiControlEnabled)
		CurrentControls.Throttle = FMath::Clamp(Value, -1.f, 1.f);
}

void AFSDSVehiclePawn::OnSteeringInput(float Value)
{
	if (!bApiControlEnabled)
		CurrentControls.Steering = FMath::Clamp(Value, -1.f, 1.f);
}

void AFSDSVehiclePawn::OnBrakeInput(float Value)
{
	if (!bApiControlEnabled)
		CurrentControls.Brake = FMath::Clamp(Value, 0.f, 1.f);
}

void AFSDSVehiclePawn::OnHandbrakePressed()
{
	if (!bApiControlEnabled) CurrentControls.bHandbrake = true;
}

void AFSDSVehiclePawn::OnHandbrakeReleased()
{
	if (!bApiControlEnabled) CurrentControls.bHandbrake = false;
}

void AFSDSVehiclePawn::SetCarControls(const FCarControls& Controls)
{
	CurrentControls = Controls;
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

	State.Timestamp = FPlatformTime::Cycles64();
	return State;
}
