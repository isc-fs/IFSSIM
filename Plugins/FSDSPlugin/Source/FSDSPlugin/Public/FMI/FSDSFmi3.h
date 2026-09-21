// FMI 3.0 Co-Simulation binding: load an FMU's shared library and step it.
#pragma once

#include "CoreMinimal.h"

/**
 * A direct binding to the FMI 3.0 Co-Simulation C API.
 *
 * WHY NOT MODELON FMI LIBRARY
 * ---------------------------
 * The migration doc proposed vendoring Modelon's FMI Library, and that was the
 * right call when the plan was written. It is not, now, for one reason: most of
 * that library's value is container handling and modelDescription parsing, and
 * FFSDSZipReader + FFSDSFmuPackage already do both. What is left is resolving
 * about fifteen C entry points out of a shared library — small enough that
 * vendoring a CMake C dependency into UE's build, on every target platform,
 * costs more than it saves.
 *
 * The trade is real and worth stating: this is FMI 3.0 ONLY. A team that ships
 * an FMI 2.0 FMU gets a clear error rather than silent misbehaviour, and adding
 * 2.0 later is a second symbol table, not a redesign.
 *
 * THE SIGNATURES ARE NOT FROM MEMORY. They were read out of the C sources that
 * Simulink R2025b shipped inside the exported FMU
 * (sources/<model>_fmu.c), which is the code these pointers will actually call.
 * Getting a C ABI subtly wrong does not fail cleanly — it corrupts the stack —
 * so "verified against the callee" is the only acceptable standard here.
 */
namespace FSDSFmi3
{
	// --- fmi3PlatformTypes.h ---
	using fmi3Instance            = void*;
	using fmi3InstanceEnvironment = void*;
	using fmi3FMUState            = void*;
	using fmi3ValueReference      = uint32;
	using fmi3Float64             = double;
	using fmi3String              = const char*;
	// fmi3Boolean is C's `bool`. One byte on every target here, and matching
	// that exactly matters: this crosses a C ABI boundary.
	using fmi3Boolean             = bool;

	enum fmi3Status : int32
	{
		fmi3OK      = 0,
		fmi3Warning = 1,
		fmi3Discard = 2,
		fmi3Error   = 3,
		fmi3Fatal   = 4
	};

	using fmi3LogMessageCallback = void (*)(fmi3InstanceEnvironment, fmi3Status,
	                                        fmi3String category, fmi3String message);
	using fmi3IntermediateUpdateCallback = void (*)(fmi3InstanceEnvironment, fmi3Float64,
	                                                fmi3Boolean, fmi3Boolean, fmi3Boolean,
	                                                fmi3Boolean, fmi3Boolean*, fmi3Float64*);

	using PFN_GetVersion = fmi3String (*)(void);

	using PFN_InstantiateCoSimulation = fmi3Instance (*)(
		fmi3String instanceName, fmi3String instantiationToken, fmi3String resourcePath,
		fmi3Boolean visible, fmi3Boolean loggingOn, fmi3Boolean eventModeUsed,
		fmi3Boolean earlyReturnAllowed,
		const fmi3ValueReference requiredIntermediateVariables[],
		size_t nRequiredIntermediateVariables,
		fmi3InstanceEnvironment instanceEnvironment,
		fmi3LogMessageCallback logMessage,
		fmi3IntermediateUpdateCallback intermediateUpdate);

	using PFN_EnterInitializationMode = fmi3Status (*)(
		fmi3Instance, fmi3Boolean toleranceDefined, fmi3Float64 tolerance,
		fmi3Float64 startTime, fmi3Boolean stopTimeDefined, fmi3Float64 stopTime);

	using PFN_ExitInitializationMode = fmi3Status (*)(fmi3Instance);

	using PFN_DoStep = fmi3Status (*)(
		fmi3Instance, fmi3Float64 currentCommunicationPoint,
		fmi3Float64 communicationStepSize, fmi3Boolean noSetFMUStatePriorToCurrentPoint,
		fmi3Boolean* eventHandlingNeeded, fmi3Boolean* terminateSimulation,
		fmi3Boolean* earlyReturn, fmi3Float64* lastSuccessfulTime);

	using PFN_GetFloat64 = fmi3Status (*)(fmi3Instance, const fmi3ValueReference[], size_t,
	                                      fmi3Float64[], size_t);
	using PFN_SetFloat64 = fmi3Status (*)(fmi3Instance, const fmi3ValueReference[], size_t,
	                                      const fmi3Float64[], size_t);

	using PFN_GetFMUState  = fmi3Status (*)(fmi3Instance, fmi3FMUState*);
	using PFN_SetFMUState  = fmi3Status (*)(fmi3Instance, fmi3FMUState);
	using PFN_FreeFMUState = fmi3Status (*)(fmi3Instance, fmi3FMUState*);

	using PFN_Terminate    = fmi3Status (*)(fmi3Instance);
	using PFN_FreeInstance = void (*)(fmi3Instance);

	FSDSPLUGIN_API const TCHAR* StatusToString(fmi3Status S);
}

/**
 * A loaded FMU: shared library plus one Co-Simulation instance.
 *
 * Owns the library handle and the instance, and tears both down in order on
 * destruction. Terminate-before-free and free-before-unload are not optional —
 * getting that order wrong is a crash inside someone else's generated code,
 * which is close to undebuggable from here.
 */
class FSDSPLUGIN_API FFSDSFmi3Instance
{
public:
	~FFSDSFmi3Instance();

	/** Load the library and resolve entry points. Does not instantiate. */
	bool Load(const FString& SharedLibraryPath);

	/** Create the CS instance. Requires Load(). */
	bool Instantiate(const FString& InstanceName, const FString& InstantiationToken,
	                 const FString& ResourcePath, bool bLoggingOn = true);

	bool EnterInitializationMode(double StartTime, double StopTime);
	bool ExitInitializationMode();

	/** One communication step. Returns false on Error/Fatal. */
	bool DoStep(double CurrentCommunicationPoint, double StepSize);

	bool SetFloat64(uint32 ValueReference, double Value);
	bool GetFloat64(uint32 ValueReference, double& OutValue);

	/** Array forms. FMI 3.0 requires nValues to equal the TOTAL element count
	 *  across the requested references — passing 1 for a 3-element variable is
	 *  an error, not a partial read. */
	bool SetFloat64Array(uint32 ValueReference, const double* Values, int32 Count);
	bool GetFloat64Array(uint32 ValueReference, double* OutValues, int32 Count);

	/** State save/restore — the deterministic-reset primitive. */
	bool GetState(void*& OutState);
	bool SetState(void* State);
	bool FreeState(void*& State);

	void Terminate();
	void FreeInstance();
	void Unload();

	bool IsLoaded() const { return Handle != nullptr; }
	bool IsInstantiated() const { return Instance != nullptr; }
	FString GetVersion() const;
	const FString& GetLastError() const { return LastError; }

private:
	void* Handle = nullptr;
	FSDSFmi3::fmi3Instance Instance = nullptr;
	FString LibPath;
	FString LastError;

	FSDSFmi3::PFN_GetVersion                GetVersionFn = nullptr;
	FSDSFmi3::PFN_InstantiateCoSimulation   InstantiateFn = nullptr;
	FSDSFmi3::PFN_EnterInitializationMode   EnterInitFn = nullptr;
	FSDSFmi3::PFN_ExitInitializationMode    ExitInitFn = nullptr;
	FSDSFmi3::PFN_DoStep                    DoStepFn = nullptr;
	FSDSFmi3::PFN_GetFloat64                GetFloat64Fn = nullptr;
	FSDSFmi3::PFN_SetFloat64                SetFloat64Fn = nullptr;
	FSDSFmi3::PFN_GetFMUState               GetStateFn = nullptr;
	FSDSFmi3::PFN_SetFMUState               SetStateFn = nullptr;
	FSDSFmi3::PFN_FreeFMUState              FreeStateFn = nullptr;
	FSDSFmi3::PFN_Terminate                 TerminateFn = nullptr;
	FSDSFmi3::PFN_FreeInstance              FreeInstanceFn = nullptr;
};
