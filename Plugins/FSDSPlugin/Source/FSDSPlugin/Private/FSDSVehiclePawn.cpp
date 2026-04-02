#include "FSDSVehiclePawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "UObject/ConstructorHelpers.h"

AFSDSVehiclePawn::AFSDSVehiclePawn()
{
	PrimaryActorTick.bCanEverTick = true;

	// Get the Chaos vehicle movement component (created by parent)
	VehicleMovement = CastChecked<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

	// Configure engine torque curve so the vehicle can actually drive
	if (VehicleMovement)
	{
		VehicleMovement->EngineSetup.MaxTorque = 500.f;
		VehicleMovement->EngineSetup.MaxRPM = 6000.f;
		VehicleMovement->EngineSetup.EngineIdleRPM = 800.f;
		VehicleMovement->EngineSetup.EngineBrakeEffect = 0.1f;

		// Simple torque curve: full torque from 1000-4000 RPM, drops after
		FRichCurve* TorqueCurve = VehicleMovement->EngineSetup.TorqueCurve.GetRichCurve();
		TorqueCurve->Reset();
		TorqueCurve->AddKey(0.f, 0.5f);
		TorqueCurve->AddKey(1000.f, 0.8f);
		TorqueCurve->AddKey(3000.f, 1.0f);
		TorqueCurve->AddKey(4500.f, 0.9f);
		TorqueCurve->AddKey(6000.f, 0.7f);

		// Transmission
		VehicleMovement->TransmissionSetup.bUseAutomaticGears = true;
		VehicleMovement->TransmissionSetup.FinalRatio = 3.5f;

		// Mass
		VehicleMovement->Mass = 300.f; // Formula Student car ~300kg
	}

	// Set up a visible placeholder mesh (cube scaled to car proportions)
	// The skeletal mesh from GetMesh() is empty by default.
	// We use a simple static mesh as a visible body until a proper car model is imported.
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		// Create a static mesh component for the car body
		CarBodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CarBodyMesh"));
		CarBodyMesh->SetupAttachment(GetMesh());
		CarBodyMesh->SetStaticMesh(CubeMesh.Object);
		// Scale to approximate car size: ~4m long, ~2m wide, ~1.2m tall
		CarBodyMesh->SetRelativeScale3D(FVector(2.0f, 1.0f, 0.6f));
		CarBodyMesh->SetRelativeLocation(FVector(0.f, 0.f, 60.f));
		CarBodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
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

	// --- Sensors ---
	CameraSensor = CreateDefaultSubobject<UFSDSCameraSensor>(TEXT("CameraSensor"));
	CameraSensor->SetupAttachment(GetMesh());
	CameraSensor->SetRelativeLocation(FVector(160.f, 0.f, -20.f));

	LidarSensor = CreateDefaultSubobject<UFSDSLidarSensor>(TEXT("LidarSensor"));

	ImuSensor = CreateDefaultSubobject<UFSDSImuSensor>(TEXT("ImuSensor"));

	GpsSensor = CreateDefaultSubobject<UFSDSGpsSensor>(TEXT("GpsSensor"));

	GssSensor = CreateDefaultSubobject<UFSDSGssSensor>(TEXT("GssSensor"));
}

void AFSDSVehiclePawn::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(LogTemp, Log, TEXT("FSDS: Vehicle pawn spawned at %s"), *GetActorLocation().ToString());
}

void AFSDSVehiclePawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Calculate acceleration from velocity delta
	FVector CurrentVelocity = GetVelocity();
	if (DeltaTime > 0.f)
	{
		CurrentAcceleration = (CurrentVelocity - PreviousVelocity) / DeltaTime;
	}
	PreviousVelocity = CurrentVelocity;

	// Apply current controls to vehicle movement
	if (VehicleMovement)
	{
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

	// Direct physics force fallback (works even without wheel setup)
	// This ensures the car can be driven for testing purposes
	UPrimitiveComponent* RootPrim = Cast<UPrimitiveComponent>(GetRootComponent());
	if (RootPrim && RootPrim->IsSimulatingPhysics())
	{
		// Forward/backward force
		if (FMath::Abs(CurrentControls.Throttle) > 0.01f)
		{
			FVector ForwardForce = GetActorForwardVector() * CurrentControls.Throttle * 500000.f; // Newtons
			RootPrim->AddForce(ForwardForce);
		}

		// Steering torque
		if (FMath::Abs(CurrentControls.Steering) > 0.01f)
		{
			FVector SteeringTorque = FVector(0.f, 0.f, CurrentControls.Steering * 50000000.f);
			RootPrim->AddTorqueInRadians(SteeringTorque);
		}

		// Braking (damping)
		if (CurrentControls.Brake > 0.01f)
		{
			FVector Vel = GetVelocity();
			if (Vel.SizeSquared() > 1.f)
			{
				FVector BrakeForce = -Vel.GetSafeNormal() * CurrentControls.Brake * 300000.f;
				RootPrim->AddForce(BrakeForce);
			}
		}
	}
}

void AFSDSVehiclePawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	// Bind keyboard axes for manual driving
	PlayerInputComponent->BindAxis("MoveForward", this, &AFSDSVehiclePawn::OnThrottleInput);
	PlayerInputComponent->BindAxis("MoveRight", this, &AFSDSVehiclePawn::OnSteeringInput);
	PlayerInputComponent->BindAxis("Brake", this, &AFSDSVehiclePawn::OnBrakeInput);

	PlayerInputComponent->BindAction("Handbrake", IE_Pressed, this, &AFSDSVehiclePawn::OnHandbrakePressed);
	PlayerInputComponent->BindAction("Handbrake", IE_Released, this, &AFSDSVehiclePawn::OnHandbrakeReleased);
}

// --- Keyboard input handlers ---

void AFSDSVehiclePawn::OnThrottleInput(float Value)
{
	if (!bApiControlEnabled)
	{
		CurrentControls.Throttle = FMath::Clamp(Value, -1.f, 1.f);
	}
}

void AFSDSVehiclePawn::OnSteeringInput(float Value)
{
	if (!bApiControlEnabled)
	{
		CurrentControls.Steering = FMath::Clamp(Value, -1.f, 1.f);
	}
}

void AFSDSVehiclePawn::OnBrakeInput(float Value)
{
	if (!bApiControlEnabled)
	{
		CurrentControls.Brake = FMath::Clamp(Value, 0.f, 1.f);
	}
}

void AFSDSVehiclePawn::OnHandbrakePressed()
{
	if (!bApiControlEnabled)
	{
		CurrentControls.bHandbrake = true;
	}
}

void AFSDSVehiclePawn::OnHandbrakeReleased()
{
	if (!bApiControlEnabled)
	{
		CurrentControls.bHandbrake = false;
	}
}

// --- Programmatic control ---

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

	State.Speed = GetVelocity().Size() / 100.f; // cm/s to m/s
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
