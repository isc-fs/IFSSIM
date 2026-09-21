#include "Vehicles/FSDSWheeledVehicleMovementComponent.h"

#include "ChaosVehicleWheel.h"
#include "VehicleUtility.h"   // Chaos::MToCm
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Physics/PhysicsInterfaceCore.h"

// Field-by-field notes on what the engine's public setters reach, verified in
// ChaosWheeledVehicleMovementComponent.cpp (UE 5.7):
//
//   SetWheelRadius            -> VehicleWheel.SetWheelRadius(Radius)   [cm]
//   SetWheelFrictionMultiplier-> VehicleWheel.FrictionMultiplier
//   SetWheelMaxBrakeTorque    -> VehicleWheel.MaxBrakeTorque           [Nm]
//   SetWheelMaxSteerAngle     -> VehicleWheel.MaxSteeringAngle         [deg]
//   SetWheelSlipGraphMultiplier-> VehicleWheel.LateralSlipGraphMultiplier
//   SetSuspensionParams       -> Suspension[i].AccessSetup().SpringRate etc.
//
// Each wraps FPhysicsCommand::ExecuteWrite on the chassis actor and writes to
// VehicleSimulationPT->PVehicle, i.e. genuine solver state — not the
// game-thread UChaosVehicleWheel that Chaos ignores after CreateVehicle().

bool UFSDSWheeledVehicleMovementComponent::ApplyWheelConfigToPhysics(int32 WheelIndex)
{
	if (!Wheels.IsValidIndex(WheelIndex) || Wheels[WheelIndex] == nullptr)
	{
		return false;
	}
	if (GetBodyInstance() == nullptr)
	{
		// Physics state not created yet. Expected when called before
		// BeginPlay; the caller should retry once the vehicle exists.
		return false;
	}

	const UChaosVehicleWheel* W = Wheels[WheelIndex];

	// Radius and width are authored in cm on UChaosVehicleWheel, which is the
	// same unit the solver stores, so no conversion here.
	SetWheelRadius(WheelIndex, W->WheelRadius);
	SetWheelFrictionMultiplier(WheelIndex, W->FrictionForceMultiplier);
	SetWheelMaxBrakeTorque(WheelIndex, W->MaxBrakeTorque);
	SetWheelHandbrakeTorque(WheelIndex, W->MaxHandBrakeTorque);
	SetWheelMaxSteerAngle(WheelIndex, W->MaxSteerAngle);

	// SUSPENSION IS DELIBERATELY NOT PUSHED HERE.
	//
	// Re-writing suspension parameters on a LIVE vehicle resets solver-side
	// suspension state (spring length / contact), and the correction impulse
	// launched the car into the air on the first test. The wheel classes never
	// set SpringRate anyway, so it is the engine default 250 and there is
	// nothing from settings.json to deliver — pushing it is all risk, no gain.
	//
	// If suspension ever needs to come from settings.json, do it by setting the
	// values in the WHEEL CLASS CONSTRUCTORS (FSDSWheelFront/Rear), which lands
	// them in the class default object BEFORE the vehicle is created — the same
	// path Chaos already uses, with no live re-initialisation.
	//
	// Retained for reference, note the unit trap if this is ever revisited:
	//   ChaosVehicleWheel.h:392 (CDO path)  SpringRate = Chaos::MToCm(SpringRate)
	//   ChaosWheeledVehicleMovementComponent.cpp:2728 (setter) writes it RAW.

	return true;
}

bool UFSDSWheeledVehicleMovementComponent::PushFullWheelConfigToPhysics(int32 WheelIndex)
{
	if (!Wheels.IsValidIndex(WheelIndex) || Wheels[WheelIndex] == nullptr)
	{
		return false;
	}

	FBodyInstance* TargetInstance = GetBodyInstance();
	if (UpdatedPrimitive == nullptr || TargetInstance == nullptr)
	{
		return false;
	}

	bool bPushed = false;

	// Mirrors UChaosWheeledVehicleMovementComponent::SetWheelClass
	// (ChaosWheeledVehicleMovementComponent.cpp:2330-2355) but keeps the
	// existing wheel object: we want its CURRENT configuration pushed, not a
	// fresh CDO-derived one, which is the whole point.
	FPhysicsCommand::ExecuteWrite(TargetInstance->GetPhysicsActor(),
		[&](const FPhysicsActorHandle& Chassis)
		{
			if (VehicleSimulationPT)
			{
				UChaosVehicleWheel* Wheel = Wheels[WheelIndex];
				VehicleSimulationPT->InitializeWheel(WheelIndex, &Wheel->GetPhysicsWheelConfig());
				VehicleSimulationPT->InitializeSuspension(WheelIndex, &Wheel->GetPhysicsSuspensionConfig());
				bPushed = true;
			}
		});

	return bPushed;
}

int32 UFSDSWheeledVehicleMovementComponent::ApplyAllWheelConfigsToPhysics(bool bFullReinit)
{
	int32 Applied = 0;
	for (int32 i = 0; i < Wheels.Num(); ++i)
	{
		// Full re-init first (it rebuilds from GetPhysicsWheelConfig, which
		// carries the baked slip curve), then the per-field setters on top so
		// that anything the re-init leaves stale is overwritten with the
		// values we actually care about.
		if (bFullReinit)
		{
			PushFullWheelConfigToPhysics(i);
		}
		if (ApplyWheelConfigToPhysics(i))
		{
			++Applied;
		}
	}

	UE_LOG(LogTemp, Log,
		TEXT("FSDS: pushed wheel config to physics for %d/%d wheels (full re-init: %s)"),
		Applied, Wheels.Num(), bFullReinit ? TEXT("yes") : TEXT("no"));

	return Applied;
}

bool UFSDSWheeledVehicleMovementComponent::VerifyWheelConfigApplied(int32 WheelIndex, float Tolerance) const
{
	if (!Wheels.IsValidIndex(WheelIndex) || Wheels[WheelIndex] == nullptr)
	{
		return false;
	}
	if (!VehicleSimulationPT || !VehicleSimulationPT->PVehicle
		|| !VehicleSimulationPT->PVehicle->Wheels.IsValidIndex(WheelIndex))
	{
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS: wheel %d config unverifiable — no live simulation"), WheelIndex);
		return false;
	}

	const UChaosVehicleWheel* W = Wheels[WheelIndex];
	const Chaos::FSimpleWheelSim& S = VehicleSimulationPT->PVehicle->Wheels[WheelIndex];

	bool bOk = true;
	auto Check = [&](const TCHAR* Field, float Configured, float InSolver)
	{
		if (!FMath::IsNearlyEqual(Configured, InSolver, Tolerance))
		{
			UE_LOG(LogTemp, Error,
				TEXT("FSDS: wheel %d %s did NOT reach the solver — configured %.3f, solver %.3f"),
				WheelIndex, Field, Configured, InSolver);
			bOk = false;
		}
	};

	Check(TEXT("WheelRadius[cm]"),   W->WheelRadius,             S.GetEffectiveRadius());
	Check(TEXT("FrictionMultiplier"),W->FrictionForceMultiplier, S.FrictionMultiplier);
	Check(TEXT("MaxBrakeTorque[Nm]"),W->MaxBrakeTorque,          S.MaxBrakeTorque);
	Check(TEXT("MaxSteerAngle[deg]"),W->MaxSteerAngle,           S.MaxSteeringAngle);

	return bOk;
}

bool UFSDSWheeledVehicleMovementComponent::VerifyAllWheelConfigsApplied(float Tolerance) const
{
	bool bAllOk = true;
	int32 Checked = 0;

	for (int32 i = 0; i < Wheels.Num(); ++i)
	{
		if (Wheels[i] == nullptr) continue;
		bAllOk &= VerifyWheelConfigApplied(i, Tolerance);
		++Checked;
	}

	if (bAllOk)
	{
		UE_LOG(LogTemp, Log,
			TEXT("FSDS: wheel config verified against the solver on %d wheels — settings.json is live"),
			Checked);
	}
	else
	{
		// Deliberately Error, not Warning: a silent mismatch means the car
		// being simulated is not the car that was configured, and every
		// measurement taken from it is attributable to nothing.
		UE_LOG(LogTemp, Error,
			TEXT("FSDS: wheel config MISMATCH — the simulated car does not match settings.json"));
	}

	return bAllOk;
}

void UFSDSWheeledVehicleMovementComponent::LogInheritedWheelDefaults() const
{
	// Engine defaults, read from UChaosVehicleWheel::UChaosVehicleWheel
	// (ChaosVehicleWheel.cpp:20-53) in UE 5.7. Kept here so the log can say
	// "still at the engine default" rather than just printing a number.
	struct FDefault { const TCHAR* Name; float Value; };
	static const FDefault Defaults[] = {
		{ TEXT("CorneringStiffness"),   1000.f },
		{ TEXT("SideSlipModifier"),        1.f },
		{ TEXT("SlipThreshold"),          20.f },
		{ TEXT("SkidThreshold"),          20.f },
		{ TEXT("MaxWheelspinRotation"),   30.f },
		{ TEXT("MaxHandBrakeTorque"),   3000.f },
		{ TEXT("SpringPreload"),          50.f },
		{ TEXT("RollbarScaling"),        0.15f },
	};

	UE_LOG(LogTemp, Warning,
		TEXT("FSDS: ---- wheel parameters NOBODY CHOSE (Chaos engine defaults) ----"));

	for (int32 i = 0; i < Wheels.Num(); i++)
	{
		const UChaosVehicleWheel* W = Wheels[i];
		if (!W) continue;

		const float Actual[] = {
			W->CorneringStiffness, W->SideSlipModifier, W->SlipThreshold,
			W->SkidThreshold, static_cast<float>(W->MaxWheelspinRotation),
			W->MaxHandBrakeTorque, W->SpringPreload, W->RollbarScaling
		};

		FString Line;
		for (int32 f = 0; f < UE_ARRAY_COUNT(Defaults); f++)
		{
			const bool bInherited = FMath::IsNearlyEqual(Actual[f], Defaults[f].Value, 1e-3f);
			Line += FString::Printf(TEXT("%s=%.3g%s  "),
				Defaults[f].Name, Actual[f], bInherited ? TEXT("(default)") : TEXT("(set)"));
		}

		UE_LOG(LogTemp, Warning, TEXT("FSDS:   wheel %d  %s"), i, *Line);
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS:   wheel %d  bAffectedByBrake=%s bAffectedByHandbrake=%s ")
			TEXT("SuspensionSmoothing=%d SuspensionAxis=(%.1f,%.1f,%.1f) ")
			TEXT("SuspensionForceOffset=(%.1f,%.1f,%.1f)"),
			i,
			W->bAffectedByBrake ? TEXT("true") : TEXT("false"),
			W->bAffectedByHandbrake ? TEXT("true") : TEXT("false"),
			W->SuspensionSmoothing,
			W->SuspensionAxis.X, W->SuspensionAxis.Y, W->SuspensionAxis.Z,
			W->SuspensionForceOffset.X, W->SuspensionForceOffset.Y, W->SuspensionForceOffset.Z);
	}

	// Called out separately because it is the one with a safety consequence.
	// The handbrake channel is this car's EBS (both wheel classes set
	// bAffectedByHandbrake=true so it acts on all four corners, matching the
	// pneumatic system). At 3000 N.m per wheel against a grip limit of roughly
	// mu * Fz * r ~ 1.4 * 675 N * 0.202 m ~ 190 N.m, the commanded torque
	// exceeds what the tyre can transmit by more than an order of magnitude, so
	// every EBS application is a guaranteed four-wheel lock-up rather than a
	// modelled deceleration. Any stopping distance measured from it is a
	// property of the tyre model alone.
	UE_LOG(LogTemp, Warning,
		TEXT("FSDS: EBS runs on the Chaos handbrake channel at the engine-default ")
		TEXT("3000 N.m/wheel — roughly 15x the ~190 N.m the tyre can transmit, so ")
		TEXT("EBS is modelled as instant lock-up. Needs a real IFS-08 number ")
		TEXT("before any stopping-distance claim. See docs/fmu_plant_migration.md."));
	UE_LOG(LogTemp, Warning, TEXT("FSDS: ---- end unchosen wheel parameters ----"));
}
