#include "Plant/FSDSChaosPlant.h"
#include "FSDSVehiclePawn.h"
#include "Vehicles/FSDSWheeledVehicleMovementComponent.h"
#include "Components/SkeletalMeshComponent.h"

FFSDSChaosPlant::FFSDSChaosPlant(AFSDSVehiclePawn* InPawn)
	: Pawn(InPawn)
{
}

bool FFSDSChaosPlant::Initialise()
{
	if (!Pawn.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS Plant: Chaos plant has no pawn"));
		return false;
	}
	return true;
}

void FFSDSChaosPlant::UeToVec(const FVector& Ue, double Out[3], double Scale)
{
	// UE: X forward, Y RIGHT, Z up, left-handed, centimetres.
	// Contract: X forward, Y LEFT, Z up, right-handed, metres.
	// The handedness flip is the Y negation; getting it wrong mirrors the car
	// and is very hard to see from a lap time.
	Out[0] =  Ue.X * Scale;
	Out[1] = -Ue.Y * Scale;
	Out[2] =  Ue.Z * Scale;
}

void FFSDSChaosPlant::UeToAxial(const FVector& Ue, double Out[3], double Scale)
{
	// Axial (pseudo-)vector under an improper map: negate X and Z, keep Y.
	// See the header for why this differs from UeToVec.
	Out[0] = -Ue.X * Scale;
	Out[1] =  Ue.Y * Scale;
	Out[2] = -Ue.Z * Scale;
}

void FFSDSChaosPlant::UeToWorld(const FVector& Ue, double Out[3])
{
	UeToVec(Ue, Out, 0.01);   // cm -> m
}

void FFSDSChaosPlant::UeToQuat(const FQuat& Ue, double Out[4])
{
	// Flipping the Y axis flips the sign of the rotations about X and Z, and
	// leaves the rotation about Y. Same rule as the vector case, applied to the
	// quaternion's vector part.
	Out[0] =  Ue.W;
	Out[1] = -Ue.X;
	Out[2] =  Ue.Y;
	Out[3] = -Ue.Z;
}

void FFSDSChaosPlant::PreStep(const FFSDSPlantInput& In)
{
	// Chaos is driven by the pawn's existing Tick, not from here. Keeping the
	// input is what lets PostStep compute proper acceleration against the same
	// gravity the caller declared, rather than assuming 9.81.
	LastInput = In;
}

void FFSDSChaosPlant::PostStep(FFSDSPlantOutput& Out)
{
	Out = FFSDSPlantOutput();
	AFSDSVehiclePawn* P = Pawn.Get();
	if (!P)
	{
		Out.bPlantOk = false;
		Out.PlantStatus = 3;
		return;
	}

	USkeletalMeshComponent* Mesh = P->GetMesh();
	const bool bPhysicsLive = Mesh && Mesh->IsSimulatingPhysics();
	if (!bPhysicsLive)
	{
		// Report the failure rather than publishing a well-formed zero. This is
		// the inversion the design doc calls for: the current guards fall to a
		// zero branch, so a dead plant looks exactly like a stationary car.
		UE_LOG(LogTemp, Error,
			TEXT("FSDS Plant: Chaos physics is not simulating — refusing to report state"));
		Out.bPlantOk = false;
		Out.PlantStatus = 3;
		return;
	}

	const FTransform Xf = P->GetActorTransform();
	UeToWorld(Xf.GetLocation(), Out.Position);
	UeToQuat (Xf.GetRotation(), Out.Quat);

	const FVector VelUe = P->GetVelocity();                       // cm/s
	UeToVec(VelUe, Out.VelWorld, 0.01);

	const FVector VelBodyUe = Xf.InverseTransformVectorNoScale(VelUe);
	UeToVec(VelBodyUe, Out.VelBody, 0.01);

	const FVector OmegaUe = Mesh->GetPhysicsAngularVelocityInRadians();
	const FVector OmegaBodyUe = Xf.InverseTransformVectorNoScale(OmegaUe);
	UeToAxial(OmegaBodyUe, Out.OmegaBody, 1.0);   // AXIAL, not polar

	// PROPER ACCELERATION, by finite difference of BODY velocity with the
	// Coriolis term restored and gravity removed. Chaos exposes no accelerometer,
	// so a difference is the only route — but doing it in the body frame with
	// -omega x v included is what the current sim gets wrong, and it is the
	// signal the EKF consumes.
	const FVector VelBodyMs = FVector(Out.VelBody[0], Out.VelBody[1], Out.VelBody[2]);
	const double Dt = FMath::Max(LastInput.DeltaTime, KINDA_SMALL_NUMBER);
	if (bHavePrevVel)
	{
		const FVector Omega(Out.OmegaBody[0], Out.OmegaBody[1], Out.OmegaBody[2]);
		const FVector DvDt   = (VelBodyMs - PrevVelBodyMs) / Dt;
		const FVector Coriolis = FVector::CrossProduct(Omega, VelBodyMs);
		// a_body = dv/dt + omega x v ; proper acceleration removes gravity.
		const FVector ABody  = DvDt + Coriolis;
		const FQuat Q = Xf.GetRotation();
		const FVector GravWorld(0.0, 0.0, LastInput.GravityZ);
		const FVector GravBodyUe = Q.UnrotateVector(FVector(0, 0, LastInput.GravityZ));
		const FVector GravBody(GravBodyUe.X, -GravBodyUe.Y, GravBodyUe.Z);
		const FVector Proper = ABody - GravBody;
		Out.AccelProper[0] = Proper.X;
		Out.AccelProper[1] = Proper.Y;
		Out.AccelProper[2] = Proper.Z;
	}
	PrevVelBodyMs = VelBodyMs;
	bHavePrevVel = true;

	const FRotator Rot = Xf.Rotator();
	Out.Attitude[0] = FMath::DegreesToRadians(-Rot.Roll);   // handedness flip
	Out.Attitude[1] = FMath::DegreesToRadians(Rot.Pitch);
	Out.Attitude[2] = Out.Position[2];

	// --- per wheel -------------------------------------------------------
	if (UFSDSWheeledVehicleMovementComponent* VM = P->VehicleMovement)
	{
		const int32 N = FMath::Min((int32)FSDS_NUM_WHEELS, VM->Wheels.Num());
		for (int32 i = 0; i < N; i++)
		{
			const FWheelStatus S = VM->GetWheelState(i);
			// SpringForce is in UE force units (kg*cm/s^2); 0.01 converts to N.
			Out.WheelFz[i] = S.SpringForce * 0.01;
			Out.bWheelInContact[i] = S.bInContact;
			// SlipAngle's unit at the Chaos fill site is unverified — see the
			// design doc's open questions. Left unconverted and flagged rather
			// than silently assumed to be radians.
			Out.WheelSlipAngle[i] = S.SlipAngle;
			if (const UChaosVehicleWheel* W = VM->Wheels[i])
			{
				Out.WheelSteer[i] = FMath::DegreesToRadians(W->GetSteerAngle());
			}
		}
	}

	// NOT AVAILABLE FROM CHAOS, and left at zero deliberately rather than
	// filled with a plausible substitute:
	//   WheelOmega      Chaos snaps wheel speed to ground speed, so there is no
	//                   independent wheel state to read (WheelSystem.cpp:217).
	//   WheelFx/Fy      never exposed.
	//   WheelSlipRatio  structurally unrepresentable for the same reason.
	// An A/B against an FMU must therefore compare only what BOTH plants have.

	Out.bPlantOk = true;
	Out.PlantStatus = 0;
}

void FFSDSChaosPlant::Reset(const double Position[3], const double Quat[4])
{
	AFSDSVehiclePawn* P = Pawn.Get();
	if (!P) return;

	// Contract -> UE: undo the same handedness flip.
	const FVector Loc(Position[0] * 100.0, -Position[1] * 100.0, Position[2] * 100.0);
	const FQuat   Rot(-Quat[1], Quat[2], -Quat[3], Quat[0]);
	P->SetActorLocationAndRotation(Loc, Rot, false, nullptr, ETeleportType::TeleportPhysics);
	bHavePrevVel = false;   // the difference across a teleport is meaningless
}
