#include "Vehicles/FSDSWheeledVehicleMovementComponent.h"

#include "ChaosVehicleWheel.h"
#include "VehicleUtility.h"   // Chaos::MToCm
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "FSDSSettings.h"

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
	// pneumatic system). The pawn now sizes it from settings.json
	// (EbsGripMultiple x TireMu x Fz_static x r, ~286 N.m); this line shouts if
	// it is still at the engine default, i.e. instant four-wheel lock-up.
	if (Wheels.Num() > 0 && Wheels[0])
	{
		const float Ebs = Wheels[0]->MaxHandBrakeTorque;
		if (FMath::IsNearlyEqual(Ebs, 3000.f, 1e-3f))
		{
			UE_LOG(LogTemp, Warning,
				TEXT("FSDS: EBS is still at the engine-default 3000 N.m/wheel, ")
				TEXT("~15x the tyre's grip limit: every EBS stop is an instant lock-up."));
		}
		else
		{
			UE_LOG(LogTemp, Log, TEXT("FSDS: EBS torque %.0f N.m/wheel (from settings.json)"), Ebs);
		}
	}
	// Chaos engine braking acts on the driven wheels whenever the Chaos
	// throttle is zero, which for this car is always (the EMRAX drives the
	// wheels through SetDriveTorque). Anything but 0 is a hidden drag.
	if (EngineSetup.EngineBrakeEffect != 0.f)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS: Chaos EngineBrakeEffect is %.3f, not 0: the driven wheels get ")
			TEXT("EngineRPM x %.3f N.m of brake torque at all times (hidden drag)."),
			EngineSetup.EngineBrakeEffect, EngineSetup.EngineBrakeEffect);
	}
	else
	{
		UE_LOG(LogTemp, Log, TEXT("FSDS: Chaos engine braking off (EngineBrakeEffect 0)"));
	}
	UE_LOG(LogTemp, Warning, TEXT("FSDS: ---- end unchosen wheel parameters ----"));
}

// ---------------------------------------------------------------------------
// Wheel placement
// ---------------------------------------------------------------------------

namespace
{
	// Where settings.json puts each wheel, in the root body's frame (cm, UE
	// axes: +X forward, +Y right). Order matches WheelSetups: FL, FR, RL, RR.
	// The centre of mass is the origin, so the front axle sits a = L*(1-Wf)
	// ahead of it and the rear axle b = L*Wf behind it (Wf = b/L).
	bool TargetWheelXY(int32 WheelIndex, FVector2D& Out)
	{
		const FFSDSVehicleSettings* V = FFSDSSettings::Get().GetDefaultVehicle();
		if (!V || WheelIndex < 0 || WheelIndex > 3) return false;
		const FFSDSVehiclePhysics& P = V->Physics;
		if (P.Wheelbase <= 0.f || P.WeightDistFront <= 0.f || P.WeightDistFront >= 1.f
			|| P.TrackFront <= 0.f || P.TrackRear <= 0.f)
		{
			return false;
		}
		const bool bFront = WheelIndex < 2;
		const bool bLeft = (WheelIndex % 2) == 0;
		const double X = bFront ? P.Wheelbase * (1.0 - P.WeightDistFront) * 100.0
		                        : -P.Wheelbase * P.WeightDistFront * 100.0;
		const double HalfTrack = (bFront ? P.TrackFront : P.TrackRear) * 50.0;
		Out = FVector2D(X, bLeft ? -HalfTrack : HalfTrack);
		return true;
	}
}

void UFSDSWheeledVehicleMovementComponent::OnCreatePhysicsState()
{
	PlaceWheelsFromSettings();
	Super::OnCreatePhysicsState();
	VerifyWheelPlacement();
}

void UFSDSWheeledVehicleMovementComponent::PlaceWheelsFromSettings()
{
	TArray<FVector> Offsets;
	FString Report;
	for (int32 i = 0; i < WheelSetups.Num(); i++)
	{
		FChaosWheelSetup& Setup = WheelSetups[i];
		const UChaosVehicleWheel* Cdo = Setup.WheelClass ? Setup.WheelClass.GetDefaultObject() : nullptr;
		FVector2D Target;
		if (!Cdo || Setup.BoneName == NAME_None || !TargetWheelXY(i, Target))
		{
			UE_LOG(LogTemp, Error,
				TEXT("FSDS: wheel %d not placed from settings.json (no wheel class, bone or ")
				TEXT("valid Wheelbase/WeightDistFront/Track); it stays at its bone"), i);
			Offsets.Add(Setup.AdditionalOffset);
			continue;
		}

		// Same lookup Chaos does in GetWheelRestingPosition, without our offset.
		const FVector AtBone = LocateBoneOffset(Setup.BoneName, Cdo->Offset);
		Setup.AdditionalOffset.X = Target.X - AtBone.X;
		Setup.AdditionalOffset.Y = Target.Y - AtBone.Y;
		Offsets.Add(Setup.AdditionalOffset);
		Report += FString::Printf(TEXT("  %s bone (%.1f, %.1f) -> (%.1f, %.1f)"),
			*Setup.BoneName.ToString(), AtBone.X, AtBone.Y, Target.X, Target.Y);
	}

	if (Offsets != LastPlacedOffsets)
	{
		UE_LOG(LogTemp, Log, TEXT("FSDS: wheels placed from settings.json, cm in the body frame:%s"), *Report);
		LastPlacedOffsets = MoveTemp(Offsets);
	}
}

void UFSDSWheeledVehicleMovementComponent::VerifyWheelPlacement()
{
	if (WheelSetups.Num() < 4) return;

	bool bOk = true;
	FString Report;
	FVector Rest[4];
	for (int32 i = 0; i < 4; i++)
	{
		Rest[i] = GetWheelRestingPosition(WheelSetups[i]);
		FVector2D Target;
		if (!TargetWheelXY(i, Target)) return;   // already reported by PlaceWheelsFromSettings
		const double Err = FVector2D::Distance(FVector2D(Rest[i].X, Rest[i].Y), Target);
		if (Err > 1.0) bOk = false;
		Report += FString::Printf(TEXT(" w%d=(%.1f, %.1f, %.1f)"), i, Rest[i].X, Rest[i].Y, Rest[i].Z);
	}

	const double WheelbaseCm = 0.5 * ((Rest[0].X + Rest[1].X) - (Rest[2].X + Rest[3].X));
	const double FrontShare = -0.5 * (Rest[2].X + Rest[3].X) / WheelbaseCm;
	if (bOk)
	{
		UE_LOG(LogTemp, Log,
			TEXT("FSDS: physics wheelbase %.1f cm, %.1f%% of the weight on the front axle;%s"),
			WheelbaseCm, FrontShare * 100.0, *Report);
	}
	else
	{
		UE_LOG(LogTemp, Error,
			TEXT("FSDS: physics wheels are NOT where settings.json puts them (more than 1 cm off): ")
			TEXT("wheelbase %.1f cm;%s"), WheelbaseCm, *Report);
	}
}
