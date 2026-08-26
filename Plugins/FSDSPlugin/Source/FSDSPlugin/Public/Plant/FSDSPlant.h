// The platform <-> plant boundary. One interface, several implementations.
#pragma once

#include "CoreMinimal.h"

/**
 * Wheel order is FL=0, FR=1, RL=2, RR=3 everywhere, on both sides of the
 * boundary. It is already load-bearing in the existing code and in the Simulink
 * model; changing it is a silent, symmetric, near-undetectable bug.
 */
enum { FSDS_FL = 0, FSDS_FR = 1, FSDS_RL = 2, FSDS_RR = 3, FSDS_NUM_WHEELS = 4 };

/**
 * What the platform gives the plant.
 *
 * Mirrors IFSSIM_CmdBus / IFSSIM_RoadBus / IFSSIM_EnvBus in
 * matlab/plant/ifssim_plant_buses.m field for field. The two must not drift:
 * the Simulink model is the reference and this is the C++ view of the same
 * contract.
 *
 * SI throughout. Body frame is ISO 8855 / REP-103 — x forward, y LEFT, z up.
 * World is ENU. Both differ from UE's left-handed centimetres, and the
 * conversion belongs in ONE adapter, not scattered through implementations.
 */
struct FSDSPLUGIN_API FFSDSPlantInput
{
	// --- commands, normalised exactly as they arrive on the wire ---
	double Throttle = 0.0;      // 0..1
	double Regen    = 0.0;      // 0..1 — the only service braking this car has
	double SteerNorm = 0.0;     // -1..1, kept NORMALISED (see the design doc)
	bool   bEbsLatched = false;
	bool   bHandbrake  = false;

	// --- road under each wheel; only the platform owns terrain ---
	bool   bRoadValid[FSDS_NUM_WHEELS]   = { false, false, false, false };
	double RoadHeight[FSDS_NUM_WHEELS]   = { 0, 0, 0, 0 };   // m, world Z
	double RoadNormal[FSDS_NUM_WHEELS][3]= {};               // unit
	double RoadResidual[FSDS_NUM_WHEELS] = { 0, 0, 0, 0 };   // m, plane-fit quality
	double RoadMu[FSDS_NUM_WHEELS]       = { 0, 0, 0, 0 };

	// --- environment and external forces ---
	double GravityZ = -9.81;        // m/s^2, told rather than assumed
	double ExtForce[3]  = {0,0,0};  // N,   world frame
	double ExtTorque[3] = {0,0,0};  // N*m, world frame
	double ExtPoint[3]  = {0,0,0};  // m,   world frame
	bool   bChassisGrounded = false;

	double DeltaTime = 1.0 / 60.0;  // s, the communication step
	double SimTime   = 0.0;         // s
};

/**
 * What the plant gives back. Mirrors IFSSIM_PoseBus / IFSSIM_WheelsBus /
 * IFSSIM_PowertrainBus / IFSSIM_StatusBus.
 */
struct FSDSPLUGIN_API FFSDSPlantOutput
{
	// --- pose and motion ---
	double Position[3] = {0,0,0};        // m, world ENU
	double Quat[4]     = {1,0,0,0};      // w,x,y,z — identity, NOT zeros
	double VelWorld[3] = {0,0,0};        // m/s
	double VelBody[3]  = {0,0,0};        // m/s
	double OmegaBody[3]= {0,0,0};        // rad/s
	double AlphaBody[3]= {0,0,0};        // rad/s^2 — needed for offset sensors
	/** Proper acceleration at the BODY ORIGIN. Excludes gravity: this is what
	 *  an accelerometer reads. Publishing it directly is what removes the
	 *  finite-difference-plus-g construction the EKF currently consumes. */
	double AccelProper[3] = {0,0,0};     // m/s^2, body frame
	double Attitude[3] = {0,0,0};        // roll rad, pitch rad, heave m

	// --- per wheel ---
	double WheelOmega[FSDS_NUM_WHEELS]  = {};   // rad/s, a REAL state
	double WheelSteer[FSDS_NUM_WHEELS]  = {};   // rad, actual not commanded
	double WheelFz[FSDS_NUM_WHEELS]     = {};   // N
	double WheelFx[FSDS_NUM_WHEELS]     = {};   // N
	double WheelFy[FSDS_NUM_WHEELS]     = {};   // N
	double WheelSlipRatio[FSDS_NUM_WHEELS] = {};
	double WheelSlipAngle[FSDS_NUM_WHEELS] = {}; // rad
	double WheelSuspTravel[FSDS_NUM_WHEELS] = {}; // m
	bool   bWheelInContact[FSDS_NUM_WHEELS] = {};

	// --- powertrain ---
	double MotorRpm    = 0.0;
	double MotorTorque = 0.0;   // N*m, signed; negative = regen
	double MotorPower  = 0.0;   // W,   signed
	double BattSoc     = 0.0;   // 0..1
	double BattVoltage = 0.0;   // V

	// --- health ---
	/** False means the step FAILED. The caller must log and must NOT publish a
	 *  well-formed zero, which is the failure mode the IsSimulatingPhysics()
	 *  guards currently have. */
	bool   bPlantOk = false;
	int32  PlantStatus = 0;     // 0 ok, 1 warning, 2 diverged, 3 failed
};

/**
 * A vehicle plant.
 *
 * PreStep/PostStep rather than a single Step, so the Chaos implementation can
 * be a genuine implementation rather than a wrapper around a lie: under Chaos
 * the engine integrates BETWEEN the two calls, and pretending otherwise would
 * make an A/B comparison meaningless. It is a leaky abstraction and that is
 * deliberate — see docs/fmu_plant_migration.md.
 */
class FSDSPLUGIN_API IFSDSPlant
{
public:
	virtual ~IFSDSPlant() = default;

	/** Human-readable, for logs and comparison reports. */
	virtual FString GetName() const = 0;

	/** One-time setup. Returns false if the plant cannot run. */
	virtual bool Initialise() = 0;

	/** Hand the plant this step's inputs. Called in TG_PrePhysics. */
	virtual void PreStep(const FFSDSPlantInput& In) = 0;

	/** Collect this step's outputs. Called in TG_PostPhysics. */
	virtual void PostStep(FFSDSPlantOutput& Out) = 0;

	/** Restore to a known pose with zero velocity. */
	virtual void Reset(const double Position[3], const double Quat[4]) = 0;

	/** Opaque state save/restore, where the plant supports it. Returns false
	 *  when it does not — the caller must not assume it silently worked. */
	virtual bool SaveState(void*& OutState) { OutState = nullptr; return false; }
	virtual bool RestoreState(void* /*State*/) { return false; }
	virtual void FreeState(void*& State) { State = nullptr; }
	virtual bool SupportsStateSaveRestore() const { return false; }
};
