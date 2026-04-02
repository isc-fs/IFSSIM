#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "FSDSVehiclePawn.generated.h"

/**
 * FSDS Vehicle Pawn — Chaos Physics wheeled vehicle.
 * Supports both keyboard input (for manual testing) and
 * programmatic control via SetCarControls (for RPC/autonomous driving).
 */
UCLASS()
class FSDSPLUGIN_API AFSDSVehiclePawn : public AWheeledVehiclePawn
{
	GENERATED_BODY()

public:
	AFSDSVehiclePawn();

	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual void BeginPlay() override;

	// --- Programmatic control (called from RPC) ---

	struct FCarControls
	{
		float Throttle = 0.f;
		float Steering = 0.f;
		float Brake = 0.f;
		bool bHandbrake = false;
		bool bIsManualGear = false;
		int32 ManualGear = 0;
		bool bGearImmediate = true;
	};

	struct FCarState
	{
		float Speed = 0.f;
		int32 Gear = 0;
		float RPM = 0.f;
		float MaxRPM = 0.f;
		bool bHandbrake = false;
		FVector Position = FVector::ZeroVector;
		FQuat Orientation = FQuat::Identity;
		FVector LinearVelocity = FVector::ZeroVector;
		FVector AngularVelocity = FVector::ZeroVector;
		FVector LinearAcceleration = FVector::ZeroVector;
		uint64 Timestamp = 0;
	};

	void SetCarControls(const FCarControls& Controls);
	FCarControls GetCarControls() const;
	FCarState GetCarState() const;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UChaosWheeledVehicleMovementComponent* VehicleMovement;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	USpringArmComponent* SpringArm;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UCameraComponent* FollowCamera;

private:
	// Keyboard input handlers
	void OnThrottleInput(float Value);
	void OnSteeringInput(float Value);
	void OnBrakeInput(float Value);
	void OnHandbrakePressed();
	void OnHandbrakeReleased();

	// Current control state
	FCarControls CurrentControls;
	bool bApiControlEnabled = false;

	// Previous frame velocity for acceleration calculation
	FVector PreviousVelocity = FVector::ZeroVector;
	FVector CurrentAcceleration = FVector::ZeroVector;
};
