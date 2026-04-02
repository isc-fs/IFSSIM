#include "FSDSVehiclePawn.h"
#include "Components/InputComponent.h"
#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "UObject/ConstructorHelpers.h"

AFSDSVehiclePawn::AFSDSVehiclePawn()
{
	PrimaryActorTick.bCanEverTick = true;

	// Root collision box
	UBoxComponent* BoxComp = CreateDefaultSubobject<UBoxComponent>(TEXT("BoxCollision"));
	BoxComp->SetBoxExtent(FVector(200.f, 100.f, 60.f));
	BoxComp->SetSimulatePhysics(false);
	BoxComp->SetCollisionProfileName(TEXT("Pawn"));
	SetRootComponent(BoxComp);

	// Visible car body mesh (cube placeholder)
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		CarBodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CarBodyMesh"));
		CarBodyMesh->SetupAttachment(RootComponent);
		CarBodyMesh->SetStaticMesh(CubeMesh.Object);
		CarBodyMesh->SetRelativeScale3D(FVector(4.0f, 2.0f, 1.2f));
		CarBodyMesh->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
		CarBodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}

	// Floating pawn movement — simple WASD driving
	Movement = CreateDefaultSubobject<UFloatingPawnMovement>(TEXT("Movement"));
	Movement->MaxSpeed = 2000.f;      // 20 m/s = ~72 km/h
	Movement->Acceleration = 4000.f;   // Fast acceleration
	Movement->Deceleration = 8000.f;   // Quick braking
	Movement->TurningBoost = 2.0f;

	// Spring arm for chase camera
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(RootComponent);
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
	CameraSensor->SetupAttachment(RootComponent);
	CameraSensor->SetRelativeLocation(FVector(160.f, 0.f, 50.f));

	LidarSensor = CreateDefaultSubobject<UFSDSLidarSensor>(TEXT("LidarSensor"));
	ImuSensor = CreateDefaultSubobject<UFSDSImuSensor>(TEXT("ImuSensor"));
	GpsSensor = CreateDefaultSubobject<UFSDSGpsSensor>(TEXT("GpsSensor"));
	GssSensor = CreateDefaultSubobject<UFSDSGssSensor>(TEXT("GssSensor"));
}

void AFSDSVehiclePawn::BeginPlay()
{
	Super::BeginPlay();
	PreviousPosition = GetActorLocation();
	UE_LOG(LogTemp, Log, TEXT("FSDS: Vehicle pawn spawned at %s"), *GetActorLocation().ToString());
}

void AFSDSVehiclePawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Calculate velocity and acceleration
	FVector CurrentPosition = GetActorLocation();
	FVector CurrentVelocity = (DeltaTime > 0.f) ? (CurrentPosition - PreviousPosition) / DeltaTime : FVector::ZeroVector;

	if (DeltaTime > 0.f)
	{
		CurrentAcceleration = (CurrentVelocity - PreviousVelocity) / DeltaTime;
	}
	PreviousVelocity = CurrentVelocity;
	PreviousPosition = CurrentPosition;
}

void AFSDSVehiclePawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis("MoveForward", this, &AFSDSVehiclePawn::OnMoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &AFSDSVehiclePawn::OnMoveRight);
}

void AFSDSVehiclePawn::OnMoveForward(float Value)
{
	if (Value != 0.f)
	{
		FVector Forward = GetActorForwardVector();
		AddMovementInput(Forward, Value);
	}
}

void AFSDSVehiclePawn::OnMoveRight(float Value)
{
	if (Value != 0.f)
	{
		// Rotate the pawn for steering
		FRotator NewRotation = GetActorRotation();
		NewRotation.Yaw += Value * 2.0f; // Steering sensitivity
		SetActorRotation(NewRotation);
	}
}

// --- Programmatic control ---

void AFSDSVehiclePawn::SetCarControls(const FCarControls& Controls)
{
	CurrentControls = Controls;

	// Apply controls as movement input
	if (FMath::Abs(Controls.Throttle) > 0.01f)
	{
		AddMovementInput(GetActorForwardVector(), Controls.Throttle);
	}
	if (FMath::Abs(Controls.Steering) > 0.01f)
	{
		FRotator NewRotation = GetActorRotation();
		NewRotation.Yaw += Controls.Steering * 2.0f;
		SetActorRotation(NewRotation);
	}
}

AFSDSVehiclePawn::FCarControls AFSDSVehiclePawn::GetCarControls() const
{
	return CurrentControls;
}

AFSDSVehiclePawn::FCarState AFSDSVehiclePawn::GetCarState() const
{
	FCarState State;

	State.Speed = PreviousVelocity.Size() / 100.f; // cm/s to m/s
	State.Position = GetActorLocation();
	State.Orientation = GetActorQuat();
	State.LinearVelocity = PreviousVelocity;
	State.AngularVelocity = FVector::ZeroVector;
	State.LinearAcceleration = CurrentAcceleration;
	State.bHandbrake = CurrentControls.bHandbrake;
	State.Gear = 1;
	State.RPM = 3000.f;
	State.MaxRPM = 5700.f;
	State.Timestamp = FPlatformTime::Cycles64();

	return State;
}
