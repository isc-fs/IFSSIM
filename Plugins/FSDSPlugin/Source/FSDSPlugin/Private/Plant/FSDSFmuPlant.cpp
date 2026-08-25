#include "Plant/FSDSFmuPlant.h"
#include "Misc/Paths.h"

FFSDSFmuPlant::FFSDSFmuPlant(const FString& InFmuPath)
	: FmuPath(InFmuPath)
{
}

FFSDSFmuPlant::~FFSDSFmuPlant()
{
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
	UE_LOG(LogTemp, Log, TEXT("FSDS Plant: %s ready, %d variables resolved by name"),
		*GetName(), NameToVR.Num());
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

void FFSDSFmuPlant::Reset(const double /*Position*/[3], const double /*Quat*/[4])
{
	// An FMU's pose is internal state; there is no input to write it to. The
	// honest reset is a state restore from a snapshot taken at the start line,
	// or a re-instantiate. Teleporting by writing pose is not available and
	// pretending otherwise would silently do nothing.
	UE_LOG(LogTemp, Warning,
		TEXT("FSDS Plant: %s cannot be teleported — reset via SaveState/RestoreState "
		     "from a start-line snapshot, or re-Initialise()"), *GetName());
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
