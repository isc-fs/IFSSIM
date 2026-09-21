#include "FMI/FSDSFmi3.h"

#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"

namespace FSDSFmi3
{
	const TCHAR* StatusToString(fmi3Status S)
	{
		switch (S)
		{
		case fmi3OK:      return TEXT("OK");
		case fmi3Warning: return TEXT("Warning");
		case fmi3Discard: return TEXT("Discard");
		case fmi3Error:   return TEXT("Error");
		case fmi3Fatal:   return TEXT("Fatal");
		default:          return TEXT("<unknown>");
		}
	}

	// The FMU logs through this. Routed to UE_LOG at a matching verbosity so an
	// FMU complaining about its own inputs is visible rather than swallowed —
	// that message is usually the only clue about what the plant disliked.
	static void LogCallback(fmi3InstanceEnvironment /*Env*/, fmi3Status Status,
	                        fmi3String Category, fmi3String Message)
	{
		const FString Cat = Category ? UTF8_TO_TCHAR(Category) : TEXT("");
		const FString Msg = Message ? UTF8_TO_TCHAR(Message) : TEXT("");
		if (Status >= fmi3Error)
		{
			UE_LOG(LogTemp, Error, TEXT("FMU[%s] %s: %s"), StatusToString(Status), *Cat, *Msg);
		}
		else if (Status == fmi3Warning || Status == fmi3Discard)
		{
			UE_LOG(LogTemp, Warning, TEXT("FMU[%s] %s: %s"), StatusToString(Status), *Cat, *Msg);
		}
		else
		{
			UE_LOG(LogTemp, Log, TEXT("FMU %s: %s"), *Cat, *Msg);
		}
	}
}

using namespace FSDSFmi3;

FFSDSFmi3Instance::~FFSDSFmi3Instance()
{
	// Order is load-bearing: terminate the instance, free it, then unload the
	// library. Unloading with a live instance runs the FMU's static destructors
	// underneath its own live state, which crashes inside generated code.
	Terminate();
	FreeInstance();
	Unload();
}

bool FFSDSFmi3Instance::Load(const FString& SharedLibraryPath)
{
	Unload();
	LibPath = SharedLibraryPath;

	Handle = FPlatformProcess::GetDllHandle(*SharedLibraryPath);
	if (!Handle)
	{
		LastError = FString::Printf(TEXT("could not load '%s' — wrong architecture, or a "
		                                 "missing dependency the loader will not name"),
		                            *SharedLibraryPath);
		return false;
	}

	auto Resolve = [this](const TCHAR* Name) -> void*
	{
		void* P = FPlatformProcess::GetDllExport(Handle, Name);
		if (!P)
		{
			LastError = FString::Printf(TEXT("missing export '%s'"), Name);
		}
		return P;
	};

	GetVersionFn   = (PFN_GetVersion)              Resolve(TEXT("fmi3GetVersion"));
	InstantiateFn  = (PFN_InstantiateCoSimulation) Resolve(TEXT("fmi3InstantiateCoSimulation"));
	EnterInitFn    = (PFN_EnterInitializationMode) Resolve(TEXT("fmi3EnterInitializationMode"));
	ExitInitFn     = (PFN_ExitInitializationMode)  Resolve(TEXT("fmi3ExitInitializationMode"));
	DoStepFn       = (PFN_DoStep)                  Resolve(TEXT("fmi3DoStep"));
	GetFloat64Fn   = (PFN_GetFloat64)              Resolve(TEXT("fmi3GetFloat64"));
	SetFloat64Fn   = (PFN_SetFloat64)              Resolve(TEXT("fmi3SetFloat64"));
	TerminateFn    = (PFN_Terminate)               Resolve(TEXT("fmi3Terminate"));
	FreeInstanceFn = (PFN_FreeInstance)            Resolve(TEXT("fmi3FreeInstance"));

	// Optional: only present when the FMU advertises canGetAndSetFMUState.
	// Absence is not a load failure — it is a capability the caller must gate on.
	GetStateFn  = (PFN_GetFMUState)  FPlatformProcess::GetDllExport(Handle, TEXT("fmi3GetFMUState"));
	SetStateFn  = (PFN_SetFMUState)  FPlatformProcess::GetDllExport(Handle, TEXT("fmi3SetFMUState"));
	FreeStateFn = (PFN_FreeFMUState) FPlatformProcess::GetDllExport(Handle, TEXT("fmi3FreeFMUState"));

	const bool bCore = GetVersionFn && InstantiateFn && EnterInitFn && ExitInitFn &&
	                   DoStepFn && GetFloat64Fn && SetFloat64Fn && TerminateFn && FreeInstanceFn;
	if (!bCore)
	{
		// An FMI 2.0 FMU loads fine and then has none of these, which would
		// otherwise present as a confusing null-pointer crash.
		LastError += TEXT(" (is this an FMI 2.0 FMU? this binding is FMI 3.0 only)");
		Unload();
		return false;
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS FMI: loaded '%s', reports version %s"),
		*FPaths::GetCleanFilename(SharedLibraryPath), *GetVersion());
	return true;
}

FString FFSDSFmi3Instance::GetVersion() const
{
	if (!GetVersionFn) return FString();
	const char* V = GetVersionFn();
	return V ? UTF8_TO_TCHAR(V) : FString();
}

bool FFSDSFmi3Instance::Instantiate(const FString& InstanceName, const FString& InstantiationToken,
                                    const FString& ResourcePath, bool bLoggingOn)
{
	if (!InstantiateFn) { LastError = TEXT("not loaded"); return false; }
	FreeInstance();

	const FTCHARToUTF8 NameUtf8(*InstanceName);
	const FTCHARToUTF8 TokenUtf8(*InstantiationToken);
	const FTCHARToUTF8 ResUtf8(*ResourcePath);

	Instance = InstantiateFn(
		(fmi3String)NameUtf8.Get(),
		(fmi3String)TokenUtf8.Get(),
		(fmi3String)ResUtf8.Get(),
		/*visible=*/            false,
		/*loggingOn=*/          bLoggingOn,
		// eventModeUsed=false: the platform drives a plain fixed-step loop. The
		// FMU may advertise hasEventMode, but opting in changes the calling
		// protocol and is not something to enable by accident.
		/*eventModeUsed=*/      false,
		// earlyReturnAllowed=false: a step that returns early would make the
		// number of substeps per macro step data-dependent, which is exactly
		// the nondeterminism the fixed-timestep work exists to remove.
		/*earlyReturnAllowed=*/ false,
		/*requiredIntermediateVariables=*/ nullptr,
		/*nRequired=*/          0,
		/*instanceEnvironment=*/ this,
		/*logMessage=*/          &FSDSFmi3::LogCallback,
		/*intermediateUpdate=*/  nullptr);

	if (!Instance)
	{
		LastError = TEXT("fmi3InstantiateCoSimulation returned null — the instantiation "
		                 "token usually mismatches modelDescription.xml when this happens");
		return false;
	}
	return true;
}

bool FFSDSFmi3Instance::EnterInitializationMode(double StartTime, double StopTime)
{
	if (!Instance) { LastError = TEXT("not instantiated"); return false; }
	const fmi3Status S = EnterInitFn(Instance, false, 0.0, StartTime, true, StopTime);
	if (S >= fmi3Error)
	{
		LastError = FString::Printf(TEXT("EnterInitializationMode -> %s"), StatusToString(S));
		return false;
	}
	return true;
}

bool FFSDSFmi3Instance::ExitInitializationMode()
{
	if (!Instance) { LastError = TEXT("not instantiated"); return false; }
	const fmi3Status S = ExitInitFn(Instance);
	if (S >= fmi3Error)
	{
		LastError = FString::Printf(TEXT("ExitInitializationMode -> %s"), StatusToString(S));
		return false;
	}
	return true;
}

bool FFSDSFmi3Instance::DoStep(double CurrentCommunicationPoint, double StepSize)
{
	if (!Instance) { LastError = TEXT("not instantiated"); return false; }

	fmi3Boolean bEventNeeded = false, bTerminate = false, bEarlyReturn = false;
	fmi3Float64 LastTime = 0.0;

	const fmi3Status S = DoStepFn(Instance, CurrentCommunicationPoint, StepSize,
		/*noSetFMUStatePriorToCurrentPoint=*/ true,
		&bEventNeeded, &bTerminate, &bEarlyReturn, &LastTime);

	if (S >= fmi3Error)
	{
		LastError = FString::Printf(TEXT("DoStep(t=%.6f, h=%.6f) -> %s"),
			CurrentCommunicationPoint, StepSize, StatusToString(S));
		return false;
	}
	if (bTerminate)
	{
		LastError = TEXT("FMU requested termination");
		return false;
	}
	// earlyReturn was refused at instantiation, so seeing it means the FMU
	// ignored that. Say so rather than silently running on a short step.
	if (bEarlyReturn)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS FMI: FMU returned early to t=%.6f despite earlyReturnAllowed=false"),
			LastTime);
	}
	return true;
}

bool FFSDSFmi3Instance::SetFloat64(uint32 ValueReference, double Value)
{
	if (!Instance) return false;
	const fmi3ValueReference VR = ValueReference;
	const fmi3Float64 V = Value;
	return SetFloat64Fn(Instance, &VR, 1, &V, 1) < fmi3Error;
}

bool FFSDSFmi3Instance::GetFloat64(uint32 ValueReference, double& OutValue)
{
	if (!Instance) return false;
	const fmi3ValueReference VR = ValueReference;
	fmi3Float64 V = 0.0;
	const fmi3Status S = GetFloat64Fn(Instance, &VR, 1, &V, 1);
	if (S >= fmi3Error) return false;
	OutValue = V;
	return true;
}

bool FFSDSFmi3Instance::SetFloat64Array(uint32 ValueReference, const double* Values, int32 Count)
{
	if (!Instance || Count <= 0) return false;
	const fmi3ValueReference VR = ValueReference;
	return SetFloat64Fn(Instance, &VR, 1, Values, (size_t)Count) < fmi3Error;
}

bool FFSDSFmi3Instance::GetFloat64Array(uint32 ValueReference, double* OutValues, int32 Count)
{
	if (!Instance || Count <= 0) return false;
	const fmi3ValueReference VR = ValueReference;
	return GetFloat64Fn(Instance, &VR, 1, OutValues, (size_t)Count) < fmi3Error;
}

bool FFSDSFmi3Instance::GetState(void*& OutState)
{
	if (!Instance || !GetStateFn) return false;
	fmi3FMUState S = nullptr;
	if (GetStateFn(Instance, &S) >= fmi3Error) return false;
	OutState = S;
	return true;
}

bool FFSDSFmi3Instance::SetState(void* State)
{
	if (!Instance || !SetStateFn || !State) return false;
	return SetStateFn(Instance, State) < fmi3Error;
}

bool FFSDSFmi3Instance::FreeState(void*& State)
{
	if (!Instance || !FreeStateFn || !State) return false;
	fmi3FMUState S = State;
	const bool bOk = FreeStateFn(Instance, &S) < fmi3Error;
	State = nullptr;
	return bOk;
}

void FFSDSFmi3Instance::Terminate()
{
	if (Instance && TerminateFn) TerminateFn(Instance);
}

void FFSDSFmi3Instance::FreeInstance()
{
	if (Instance && FreeInstanceFn) FreeInstanceFn(Instance);
	Instance = nullptr;
}

void FFSDSFmi3Instance::Unload()
{
	if (Handle)
	{
		FPlatformProcess::FreeDllHandle(Handle);
		Handle = nullptr;
	}
	GetVersionFn = nullptr; InstantiateFn = nullptr; EnterInitFn = nullptr;
	ExitInitFn = nullptr; DoStepFn = nullptr; GetFloat64Fn = nullptr;
	SetFloat64Fn = nullptr; GetStateFn = nullptr; SetStateFn = nullptr;
	FreeStateFn = nullptr; TerminateFn = nullptr; FreeInstanceFn = nullptr;
}
