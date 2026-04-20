#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "Camera/CameraComponent.h"
#include "Sensors/FSDSCameraSensor.h"
#include "Sensors/FSDSLidarSensor.h"
#include "Sensors/FSDSImuSensor.h"
#include "Sensors/FSDSGpsSensor.h"
#include "Sensors/FSDSGssSensor.h"
#include "Sensors/FSDSDistanceSensor.h"
#include "Sensors/FSDSBarometerSensor.h"
#include "Sensors/FSDSMagnetometerSensor.h"
#include "Vehicles/FSDSWheelFront.h"
#include "Vehicles/FSDSWheelRear.h"
#include "FSDSVehiclePawn.generated.h"

/**
 * FSDS Vehicle Pawn — Chaos Physics wheeled vehicle.
 * Uses AWheeledVehiclePawn with FormulaMesh skeletal mesh.
 * Falls back to simple movement if skeletal mesh fails to load.
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

	// --- Programmatic control ---

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
	void SetApiControlEnabled(bool bEnabled) { bApiControlEnabled = bEnabled; }

	// EBS analog: clamps the handbrake and locks all input channels (API +
	// keyboard) until explicitly released. The real IFS-08 EBS is a
	// pneumatic rear-axle brake that can only be reset manually, not by
	// the autonomy stack — while latched, no throttle/brake/steering
	// input takes effect and the handbrake stays engaged.
	void ActivateEbs();
	void ReleaseEbs();
	bool IsEbsLatched() const { return bEbsLatched; }

	// --- Components ---

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UChaosWheeledVehicleMovementComponent* VehicleMovement;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	USpringArmComponent* SpringArm;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Vehicle")
	UCameraComponent* FollowCamera;

	// --- Sensors ---

	/** Multiple cameras from settings.json */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	TMap<FString, UFSDSCameraSensor*> Cameras;

	/** Get a camera by name (returns nullptr if not found) */
	UFSDSCameraSensor* GetCamera(const FString& Name) const;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSLidarSensor* LidarSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSImuSensor* ImuSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSGpsSensor* GpsSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSGssSensor* GssSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSDistanceSensor* DistanceSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSBarometerSensor* BarometerSensor;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Sensors")
	UFSDSMagnetometerSensor* MagnetometerSensor;

	/** Create sensors from settings.json config (call in BeginPlay) */
	void SetupSensorsFromSettings();

	UPROPERTY()
	UFloatingPawnMovement* FallbackMovement = nullptr;

	UPROPERTY()
	class UBoxComponent* PhysicsBox = nullptr;

private:
	void SetupVehicleMovement();
	void OnThrottleInput(float Value);
	void OnSteeringInput(float Value);
	void OnBrakeInput(float Value);
	void OnHandbrakePressed();
	void OnHandbrakeReleased();

	FCarControls CurrentControls;
	bool bApiControlEnabled = false;
	bool bChaosVehicleActive = false;
	bool bEbsLatched = false;

	FVector PreviousVelocity = FVector::ZeroVector;
	FVector CurrentAcceleration = FVector::ZeroVector;

	// IFS-08 Aerodynamics
	float CdA = 0.95f;               // Drag coefficient * frontal area (m²)
	float ClA = 3.0f;                // Downforce coefficient * plan area (m²)
	float AeroBalanceFront = 0.45f;   // 45% downforce on front axle
	void ApplyAeroForces();
};
