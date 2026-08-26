// FMU package inspection and extraction. No FMI runtime yet — this is the
// layer that decides whether an .fmu is one we can actually run.
#pragma once

#include "CoreMinimal.h"

/** Which FMI standard the package declares. */
enum class EFSDSFmiVersion : uint8
{
	Unknown,
	FMI2,
	FMI3
};

/**
 * Everything the platform needs to know about an .fmu BEFORE trying to run it.
 *
 * Every field here corresponds to a gate in docs/fmu_plant_migration.md, and
 * every one of them fails silently if unchecked — which is exactly why this
 * exists as an explicit, logged step rather than as assumptions scattered
 * through a loader.
 */
struct FSDSPLUGIN_API FFSDSFmuInfo
{
	EFSDSFmiVersion Version = EFSDSFmiVersion::Unknown;
	FString ModelName;
	FString ModelIdentifier;      // base name of the shared library
	FString InstantiationToken;   // FMI 3 "instantiationToken" / FMI 2 "guid"
	FString GenerationTool;

	bool bHasCoSimulation = false;

	/**
	 * State save/restore, on which the whole deterministic-reset design rests.
	 *
	 * SPELLING TRAP: FMI 2.0 writes `canGetAndSetFMUstate` (lowercase s) and
	 * FMI 3.0 writes `canGetAndSetFMUState` (capital S). A parser matching one
	 * silently reads false on the other and disables state restore without an
	 * error anywhere. Both spellings are accepted, and the one actually found
	 * is recorded so the log can prove which was read.
	 */
	bool bCanGetAndSetState = false;
	FString StateAttributeFound;

	bool bCanSerializeState = false;
	bool bCanHandleVariableStep = false;
	bool bOnlyOneInstancePerProcess = false;
	bool bHasEventMode = false;

	/** 0 if not declared. Must divide the communication step exactly. */
	double FixedInternalStepSize = 0.0;

	TArray<FString> BinaryPlatforms;   // directory names under binaries/
	bool bHasSourceCode = false;
	bool bHasResources = false;

	int32 NumInputs = 0;
	int32 NumOutputs = 0;
	int32 NumParameters = 0;

	/**
	 * Every scalar/array variable, by name, with its value reference and
	 * element count.
	 *
	 * Resolving by NAME is the point. Simulink renumbers value references
	 * freely on re-export, so hardcoding them works until somebody adds a
	 * signal — and then every port is still a double, so the mismatch
	 * type-checks perfectly and silently feeds the plant the wrong numbers.
	 */
	TMap<FString, uint32> VariableRefs;
	TMap<FString, int32>  VariableCounts;
};

/** One gate result, so failures can be reported together rather than one at a time. */
struct FSDSPLUGIN_API FFSDSFmuGate
{
	FString Name;
	bool bPassed = false;
	bool bRequired = true;
	FString Detail;
};

/**
 * Opens an .fmu, extracts it, reads modelDescription.xml, and checks it against
 * what this platform requires.
 *
 * Extraction to a REAL filesystem path is not an implementation convenience:
 * the FMI spec requires the resources/ directory to be available in extracted
 * form for the lifetime of the instance, and the shared library has to be
 * dlopen'd from a real path. Neither can be served from a .pak.
 */
class FSDSPLUGIN_API FFSDSFmuPackage
{
public:
	/** Extracts under <ProjectSaved>/FMU/<token-or-name>/. */
	bool Open(const FString& FmuPath);

	bool IsValid() const { return bValid; }
	const FString& GetError() const { return Error; }
	const FFSDSFmuInfo& GetInfo() const { return Info; }
	const FString& GetExtractedDir() const { return ExtractedDir; }

	/** Full path to the shared library for THIS host, or empty if none. */
	FString GetBinaryPathForHost() const;

	/**
	 * Run every gate. Does not log — the caller decides severity — but
	 * LogGateResults() is the intended companion.
	 */
	TArray<FFSDSFmuGate> CheckGates(double CommunicationStep) const;

	/** Log gate results; returns true if every REQUIRED gate passed. */
	bool LogGateResults(double CommunicationStep) const;

	/** Binary directory names acceptable on this host, best match first. */
	static TArray<FString> GetHostPlatformTuples();

private:
	bool ParseModelDescription(const FString& XmlPath);

	FFSDSFmuInfo Info;
	FString ExtractedDir;
	FString Error;
	bool bValid = false;
};
