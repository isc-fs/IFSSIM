#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Sensors/FSDSCameraSensor.h"
#include "Sensors/FSDSLidarSensor.h"
#include "Sensors/FSDSImuSensor.h"
#include "Sensors/FSDSGpsSensor.h"
#include "Sensors/FSDSGssSensor.h"
#include "FSDSVehiclePawn.generated.h"

/**
 * FSDS Vehicle Pawn — simple movement for immediate testing.
 * Uses FloatingPawnMovement for WASD driving.
 * Will be upgraded to full Chaos vehicle when skeletal mesh is available.
 */
UCLASS()
class FSDSPLUGIN_API AFSDSVehiclePawn : public APawn
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

	// --- Components ---

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UStaticMeshComponent* CarBodyMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UFloatingPawnMovement* Movement;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	USpringArmComponent* SpringArm;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UCameraComponent* FollowCamera;

	// --- Sensors ---

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSCameraSensor* CameraSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSLidarSensor* LidarSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSImuSensor* ImuSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSGpsSensor* GpsSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSGssSensor* GssSensor;

private:
	// Keyboard input handlers
	void OnMoveForward(float Value);
	void OnMoveRight(float Value);

	// Current control state
	FCarControls CurrentControls;
	bool bApiControlEnabled = false;

	// Previous frame velocity for acceleration calculation
	FVector PreviousVelocity = FVector::ZeroVector;
	FVector CurrentAcceleration = FVector::ZeroVector;
	FVector PreviousPosition = FVector::ZeroVector;
};
