#include "Plant/FSDSFmuPlant.h"
#include "Misc/Paths.h"

FFSDSFmuPlant::FFSDSFmuPlant(const FString& InFmuPath)
	: FmuPath(InFmuPath)
{
}

FFSDSFmuPlant::~FFSDSFmuPlant()
{
	// Before Terminate: the state belongs to the instance that produced it.
	if (PristineState) Fmu.FreeState(PristineState);
	Fmu.Terminate();
	Fmu.FreeInstance();
	Fmu.Unload();
}

FString FFSDSFmuPlant::GetName() const
{
	return FString::Printf(TEXT("FMU:%s"), *FPaths::GetBaseFilename(FmuPath));
}

bool FFSDSFmuPlant::Initialise()
{
	bReady = false;

	if (!Package.Open(FmuPath))
	{
		LastError = Package.GetError();
		return false;
	}
	// Gate check is not decoration: an FMU without state save/restore, or with
	// an internal step that does not divide the communication step, will run
	// and produce plausible numbers that are not reproducible.
	if (!Package.LogGateResults(1.0 / 60.0))
	{
		LastError = TEXT("FMU failed a required gate — see the log");
		return false;
	}

	const FString Lib = Package.GetBinaryPathForHost();
	if (Lib.IsEmpty())
	{
		LastError = TEXT("no binary for this host");
		return false;
	}
	if (!Fmu.Load(Lib))
	{
		LastError = Fmu.GetLastError();
		return false;
	}

	NameToVR    = Package.GetInfo().VariableRefs;
	NameToCount = Package.GetInfo().VariableCounts;

	const FString Resources = FPaths::Combine(Package.GetExtractedDir(), TEXT("resources"));
	if (!Fmu.Instantiate(TEXT("ifssim_plant"), Package.GetInfo().InstantiationToken, Resources))
	{
		LastError = Fmu.GetLastError();
		return false;
	}
	if (!Fmu.EnterInitializationMode(0.0, 1.0e6) || !Fmu.ExitInitializationMode())
	{
		LastError = Fmu.GetLastError();
		return false;
	}

	CurrentTime = 0.0;
	bReady = true;

	// Snapshot the pristine state NOW, before a single DoStep. This is what
	// Reset() restores, and taking it here rather than lazily is deliberate:
	// the first caller to ask for a reset is usually asking mid-mission, and
	// a snapshot taken then would restore the car to wherever it had got to.
	if (Package.GetInfo().bCanGetAndSetState)
	{
		if (Fmu.GetState(PristineState))
		{
			PristineTime = CurrentTime;
		}
		else
		{
			PristineState = nullptr;
			UE_LOG(LogTemp, Warning,
				TEXT("FSDS Plant: %s declares canGetAndSetFMUState but the call "
				     "failed — Reset() will not work"), *GetName());
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS Plant: %s cannot save state, so it cannot be reset. "
			     "A second mission in the same session will run from wherever "
			     "the first one ended."), *GetName());
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS Plant: %s ready, %d variables resolved by name%s"),
		*GetName(), NameToVR.Num(),
		PristineState ? TEXT(", reset snapshot held") : TEXT(", NO reset snapshot"));
	return true;
}

int64 FFSDSFmuPlant::VR(const TCHAR* Name) const
{
	if (const uint32* Found = NameToVR.Find(Name)) return (int64)*Found;
	return INDEX_NONE;
}

bool FFSDSFmuPlant::SetScalar(const TCHAR* Name, double Value)
{
	const int64 R = VR(Name);
	if (R == INDEX_NONE) return false;
	return Fmu.SetFloat64((uint32)R, Value);
}

bool FFSDSFmuPlant::GetScalar(const TCHAR* Name, double& Out) const
{
	const int64 R = VR(Name);
	if (R == INDEX_NONE) return false;
	return Fmu.GetFloat64((uint32)R, Out);
}

bool FFSDSFmuPlant::SetArray(const TCHAR* Name, const double* In, int32 Count)
{
	const int64 R = VR(Name);
	if (R == INDEX_NONE) return false;
	return Fmu.SetFloat64Array((uint32)R, In, Count);
}

bool FFSDSFmuPlant::GetArray(const TCHAR* Name, double* Out, int32 Count) const
{
	const int64 R = VR(Name);
	if (R == INDEX_NONE) return false;
	return Fmu.GetFloat64Array((uint32)R, Out, Count);
}

void FFSDSFmuPlant::PreStep(const FFSDSPlantInput& In)
{
	if (!bReady) return;

	// Names match the Simulink bus elements exactly. If one is missing the set
	// silently no-ops, which is why Initialise() logs how many resolved — a
	// count far below expectation means the contract has drifted.
	SetScalar(TEXT("Cmd.throttle"),   In.Throttle);
	SetScalar(TEXT("Cmd.regen"),      In.Regen);
	SetScalar(TEXT("Cmd.steer_norm"), In.SteerNorm);
	SetScalar(TEXT("Cmd.ebs_latch"),  In.bEbsLatched ? 1.0 : 0.0);
	SetScalar(TEXT("Cmd.handbrake"),  In.bHandbrake  ? 1.0 : 0.0);

	double Valid[FSDS_NUM_WHEELS], Nx[FSDS_NUM_WHEELS], Ny[FSDS_NUM_WHEELS], Nz[FSDS_NUM_WHEELS];
	for (int32 i = 0; i < FSDS_NUM_WHEELS; i++)
	{
		Valid[i] = In.bRoadValid[i] ? 1.0 : 0.0;
		Nx[i] = In.RoadNormal[i][0];
		Ny[i] = In.RoadNormal[i][1];
		Nz[i] = In.RoadNormal[i][2];
	}
	SetArray(TEXT("Road.valid"),    Valid,            FSDS_NUM_WHEELS);
	SetArray(TEXT("Road.height"),   In.RoadHeight,    FSDS_NUM_WHEELS);
	SetArray(TEXT("Road.normal_x"), Nx,               FSDS_NUM_WHEELS);
	SetArray(TEXT("Road.normal_y"), Ny,               FSDS_NUM_WHEELS);
	SetArray(TEXT("Road.normal_z"), Nz,               FSDS_NUM_WHEELS);
	SetArray(TEXT("Road.residual"), In.RoadResidual,  FSDS_NUM_WHEELS);
	SetArray(TEXT("Road.mu"),       In.RoadMu,        FSDS_NUM_WHEELS);

	SetScalar(TEXT("Env.gravity_z"),        In.GravityZ);
	SetArray (TEXT("Env.ext_force"),        In.ExtForce,  3);
	SetArray (TEXT("Env.ext_torque"),       In.ExtTorque, 3);
	SetArray (TEXT("Env.ext_point"),        In.ExtPoint,  3);
	SetScalar(TEXT("Env.chassis_grounded"), In.bChassisGrounded ? 1.0 : 0.0);

	// State injection. Written every step, not only when enabled, so the
	// enable flag is the only thing that changes between a synced and an
	// unsynced step — leaving stale pose in the other fields would make a
	// later enable pick up whatever was last written.
	if (bPendingSync)
	{
		// A reset's pose wins over any caller-supplied sync this step. The two
		// only collide when a reset lands on the same tick as a parity sync,
		// and then the reset is the one that must survive.
		const double Zero3[3] = {0,0,0};
		SetScalar(TEXT("Sync.enable"), 1.0);
		SetArray (TEXT("Sync.pos"),        PendingPos,  3);
		SetArray (TEXT("Sync.quat"),       PendingQuat, 4);
		SetArray (TEXT("Sync.vel_body"),   Zero3,       3);
		SetArray (TEXT("Sync.omega_body"), Zero3,       3);
		bPendingSync = false;
	}
	else
	{
		SetScalar(TEXT("Sync.enable"), In.bSyncState ? 1.0 : 0.0);
		SetArray (TEXT("Sync.pos"),        In.SyncPosition,  3);
		SetArray (TEXT("Sync.quat"),       In.SyncQuat,      4);
		SetArray (TEXT("Sync.vel_body"),   In.SyncVelBody,   3);
		SetArray (TEXT("Sync.omega_body"), In.SyncOmegaBody, 3);
	}

	// The FMU integrates here. Synchronous, in-process, on the calling thread:
	// N calls of exactly DeltaTime, which is what makes the run reproducible.
	if (!Fmu.DoStep(CurrentTime, In.DeltaTime))
	{
		LastError = Fmu.GetLastError();
		bReady = false;   // a failed step must not be papered over
		return;
	}
	CurrentTime += In.DeltaTime;
}

void FFSDSFmuPlant::PostStep(FFSDSPlantOutput& Out)
{
	Out.bPlantOk = bReady;
	if (!bReady)
	{
		Out.PlantStatus = 3;
		return;
	}

	GetArray(TEXT("Pose.position"),     Out.Position,    3);
	GetArray(TEXT("Pose.quat"),         Out.Quat,        4);
	GetArray(TEXT("Pose.vel_world"),    Out.VelWorld,    3);
	GetArray(TEXT("Pose.vel_body"),     Out.VelBody,     3);
	GetArray(TEXT("Pose.omega_body"),   Out.OmegaBody,   3);
	GetArray(TEXT("Pose.alpha_body"),   Out.AlphaBody,   3);
	GetArray(TEXT("Pose.accel_proper"), Out.AccelProper, 3);
	GetArray(TEXT("Pose.attitude"),     Out.Attitude,    3);

	GetArray(TEXT("Wheels.omega"),       Out.WheelOmega,      FSDS_NUM_WHEELS);
	GetArray(TEXT("Wheels.steer"),       Out.WheelSteer,      FSDS_NUM_WHEELS);
	GetArray(TEXT("Wheels.fz"),          Out.WheelFz,         FSDS_NUM_WHEELS);
	GetArray(TEXT("Wheels.fx"),          Out.WheelFx,         FSDS_NUM_WHEELS);
	GetArray(TEXT("Wheels.fy"),          Out.WheelFy,         FSDS_NUM_WHEELS);
	GetArray(TEXT("Wheels.slip_ratio"),  Out.WheelSlipRatio,  FSDS_NUM_WHEELS);
	GetArray(TEXT("Wheels.slip_angle"),  Out.WheelSlipAngle,  FSDS_NUM_WHEELS);
	GetArray(TEXT("Wheels.susp_travel"), Out.WheelSuspTravel, FSDS_NUM_WHEELS);

	double Contact[FSDS_NUM_WHEELS] = {};
	if (GetArray(TEXT("Wheels.in_contact"), Contact, FSDS_NUM_WHEELS))
	{
		for (int32 i = 0; i < FSDS_NUM_WHEELS; i++) Out.bWheelInContact[i] = Contact[i] > 0.5;
	}

	GetScalar(TEXT("Powertrain.motor_rpm"),    Out.MotorRpm);
	GetScalar(TEXT("Powertrain.motor_torque"), Out.MotorTorque);
	GetScalar(TEXT("Powertrain.motor_power"),  Out.MotorPower);
	GetScalar(TEXT("Powertrain.batt_soc"),     Out.BattSoc);
	GetScalar(TEXT("Powertrain.batt_voltage"), Out.BattVoltage);

	Out.PlantStatus = 0;
}

void FFSDSFmuPlant::Reset(const double Position[3], const double Quat[4])
{
	if (!bReady) return;

	if (!PristineState)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS Plant: %s has no reset snapshot — ignoring the reset. "
			     "The plant keeps running from where it was."), *GetName());
		return;
	}

	if (!Fmu.SetState(PristineState))
	{
		LastError = Fmu.GetLastError();
		bReady = false;   // a failed restore leaves the FMU in an unknown state
		UE_LOG(LogTemp, Error,
			TEXT("FSDS Plant: %s failed to restore its reset snapshot (%s) — "
			     "dropping the plant rather than running from an unknown state"),
			*GetName(), *LastError);
		return;
	}

	// Restoring state restores the FMU's internal time with it, so the
	// communication point has to go back too. Leaving CurrentTime where it was
	// would hand DoStep a time the FMU has already passed, which is a spec
	// violation the FMU is entitled to reject — or worse, silently accept.
	CurrentTime = PristineTime;

	// The snapshot restores WHAT the car is (wheel speeds, filter memory,
	// integrator history); the injection then decides WHERE it is. Neither
	// alone is a reset: a snapshot puts the car back at its own start rather
	// than the requested gate, and an injection alone would place a car that
	// is still spinning its wheels from the previous mission.
	//
	// Queued rather than written now, because inputs are only read at the next
	// PreStep. Writing here would set values the FMU has already passed.
	for (int32 i = 0; i < 3; i++) PendingPos[i]  = Position[i];
	for (int32 i = 0; i < 4; i++) PendingQuat[i] = Quat[i];
	bPendingSync = true;

	UE_LOG(LogTemp, Log,
		TEXT("FSDS Plant: %s reset — snapshot restored (t=%.3f), pose (%.2f, %.2f, %.2f) "
		     "queued for the next step"),
		*GetName(), CurrentTime, Position[0], Position[1], Position[2]);
}

bool FFSDSFmuPlant::SupportsStateSaveRestore() const
{
	return bReady && Package.GetInfo().bCanGetAndSetState;
}

bool FFSDSFmuPlant::SaveState(void*& OutState)
{
	OutState = nullptr;
	if (!SupportsStateSaveRestore()) return false;
	return Fmu.GetState(OutState);
}

bool FFSDSFmuPlant::RestoreState(void* State)
{
	if (!SupportsStateSaveRestore() || !State) return false;
	return Fmu.SetState(State);
}

void FFSDSFmuPlant::FreeState(void*& State)
{
	if (State) Fmu.FreeState(State);
	State = nullptr;
}
