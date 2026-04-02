#include "FSDSVehiclePawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Engine/World.h"

AFSDSVehiclePawn::AFSDSVehiclePawn()
{
	PrimaryActorTick.bCanEverTick = true;

	// Get the Chaos vehicle movement component (created by parent)
	VehicleMovement = CastChecked<UChaosWheeledVehicleMovementComponent>(GetVehicleMovementComponent());

	// Spring arm for chase camera
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(GetMesh());
	SpringArm->TargetArmLength = 500.f;
	SpringArm->SetRelativeLocation(FVector(0.f, 0.f, 150.f));
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
