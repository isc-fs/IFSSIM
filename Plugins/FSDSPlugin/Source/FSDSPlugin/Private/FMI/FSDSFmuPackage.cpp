#include "FMI/FSDSFmuPackage.h"

#include "FMI/FSDSZipReader.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Misc/Paths.h"
#include "XmlFile.h"

namespace
{
	bool AttrBool(const FXmlNode* Node, const TCHAR* Name, bool bDefault = false)
	{
		if (!Node) return bDefault;
		const FString V = Node->GetAttribute(Name);
		if (V.IsEmpty()) return bDefault;
		return V.Equals(TEXT("true"), ESearchCase::IgnoreCase) || V == TEXT("1");
	}
}

TArray<FString> FFSDSFmuPackage::GetHostPlatformTuples()
{
	// FMI 2.0 used coarse names (darwin64); FMI 3.0 moved to <arch>-<os>
	// (aarch64-darwin). Both are in the wild, and which one a tool emits is not
	// something to guess at — accept every spelling that would actually work on
	// this machine, most specific first.
	TArray<FString> Out;
#if PLATFORM_MAC
	#if PLATFORM_CPU_ARM_FAMILY
		Out.Add(TEXT("aarch64-darwin"));
	#else
		Out.Add(TEXT("x86_64-darwin"));
	#endif
	Out.Add(TEXT("darwin64"));
#elif PLATFORM_LINUX
	#if PLATFORM_CPU_ARM_FAMILY
		Out.Add(TEXT("aarch64-linux"));
	#else
		Out.Add(TEXT("x86_64-linux"));
	#endif
	Out.Add(TEXT("linux64"));
#elif PLATFORM_WINDOWS
	Out.Add(TEXT("x86_64-windows"));
	Out.Add(TEXT("win64"));
#endif
	return Out;
}

bool FFSDSFmuPackage::Open(const FString& FmuPath)
{
	Info = FFSDSFmuInfo();
	bValid = false;
	Error.Empty();

	if (!IFileManager::Get().FileExists(*FmuPath))
	{
		Error = FString::Printf(TEXT("no such file: %s"), *FmuPath);
		return false;
	}

	FFSDSZipReader Zip(FmuPath);
	if (!Zip.IsValid())
	{
		Error = FString::Printf(TEXT("cannot read .fmu container: %s"), *Zip.GetError());
		return false;
	}

	// Read the manifest before extracting, so a package we cannot run does not
	// leave a directory behind.
	TArray<uint8> Xml;
	if (!Zip.ExtractFile(TEXT("modelDescription.xml"), Xml))
	{
		// Report WHAT WAS in the archive. "no modelDescription.xml" on a file
		// another tool reads happily means our reader is wrong, not the FMU —
		// and the entry names are the evidence that distinguishes the two.
		FString Listing;
		int32 Shown = 0;
		for (const FFSDSZipReader::FEntry& E : Zip.GetEntries())
		{
			if (Shown++ >= 12) { Listing += TEXT(", ..."); break; }
			if (!Listing.IsEmpty()) Listing += TEXT(", ");
			Listing += FString::Printf(TEXT("'%s'"), *E.Name);
		}
		Error = FString::Printf(
			TEXT("no modelDescription.xml — not a valid FMU. Archive holds %d entr(ies): %s"),
			Zip.GetEntries().Num(), *Listing);
		return false;
	}

	// Extract under a token-derived directory. Two FMUs with the same model
	// name but different builds must not share a directory, or a stale binary
	// gets loaded and the mismatch surfaces as inexplicable physics.
	const FString Stem = FPaths::GetBaseFilename(FmuPath);
	// ABSOLUTE, deliberately. ProjectSavedDir() is relative to the engine
	// binary's working directory, so leaving it relative produced a path like
	// "../../../../../../Users/.../Saved/FMU/...". That still resolves today
	// only because the working directory happens to be right — and the very
	// next step is handing this path to dlopen for the FMU's shared library,
	// where a CWD-dependent path is a bug waiting for the first caller that
	// changes directory.
	ExtractedDir = FPaths::ConvertRelativePathToFull(
		FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("FMU"), Stem));

	// Wipe any previous extraction: a partial one from an interrupted run is
	// worse than none, because the missing file may be one nothing reads until
	// much later.
	IFileManager::Get().DeleteDirectory(*ExtractedDir, /*RequireExists=*/false, /*Tree=*/true);

	const int32 Count = Zip.ExtractAll(ExtractedDir);
	if (Count < 0)
	{
		Error = TEXT("extraction failed");
		return false;
	}

	const FString XmlPath = FPaths::Combine(ExtractedDir, TEXT("modelDescription.xml"));
	if (!ParseModelDescription(XmlPath))
	{
		return false;
	}

	// Packaging facts come from the extracted tree, not from guesses.
	IFileManager& FM = IFileManager::Get();
	TArray<FString> BinDirs;
	FM.IterateDirectory(*FPaths::Combine(ExtractedDir, TEXT("binaries")),
		[&BinDirs](const TCHAR* Name, bool bIsDir) -> bool
		{
			if (bIsDir) BinDirs.Add(FPaths::GetCleanFilename(Name));
			return true;
		});
	Info.BinaryPlatforms = MoveTemp(BinDirs);

	Info.bHasResources = FM.DirectoryExists(*FPaths::Combine(ExtractedDir, TEXT("resources")));
	Info.bHasSourceCode =
		FM.DirectoryExists(*FPaths::Combine(ExtractedDir, TEXT("sources"))) ||
		FM.DirectoryExists(*FPaths::Combine(ExtractedDir, TEXT("sourceCode")));

	bValid = true;
	UE_LOG(LogTemp, Log,
		TEXT("FSDS FMU: extracted %d file(s) from '%s' to '%s'"),
		Count, *FPaths::GetCleanFilename(FmuPath), *ExtractedDir);
	return true;
}

bool FFSDSFmuPackage::ParseModelDescription(const FString& XmlPath)
{
	FXmlFile Doc(XmlPath);
	if (!Doc.IsValid())
	{
		Error = FString::Printf(TEXT("modelDescription.xml is not valid XML: %s"),
			*Doc.GetLastError());
		return false;
	}

	const FXmlNode* Root = Doc.GetRootNode();
	if (!Root)
	{
		Error = TEXT("modelDescription.xml has no root node");
		return false;
	}

	const FString Ver = Root->GetAttribute(TEXT("fmiVersion"));
	if (Ver.StartsWith(TEXT("3"))) Info.Version = EFSDSFmiVersion::FMI3;
	else if (Ver.StartsWith(TEXT("2"))) Info.Version = EFSDSFmiVersion::FMI2;

	Info.ModelName      = Root->GetAttribute(TEXT("modelName"));
	Info.GenerationTool = Root->GetAttribute(TEXT("generationTool"));
	// FMI 2.0 "guid" was renamed "instantiationToken" in FMI 3.0.
	Info.InstantiationToken = Root->GetAttribute(TEXT("instantiationToken"));
	if (Info.InstantiationToken.IsEmpty())
	{
		Info.InstantiationToken = Root->GetAttribute(TEXT("guid"));
	}

	const FXmlNode* CS = Root->FindChildNode(TEXT("CoSimulation"));
	Info.bHasCoSimulation = (CS != nullptr);

	if (CS)
	{
		Info.ModelIdentifier = CS->GetAttribute(TEXT("modelIdentifier"));

		// The spelling trap, handled explicitly. Record which one was present
		// so the log can prove the right attribute was read rather than a
		// default being reported as a genuine "false".
		static const TCHAR* StateSpellings[] = {
			TEXT("canGetAndSetFMUState"),   // FMI 3.0
			TEXT("canGetAndSetFMUstate")    // FMI 2.0
		};
		for (const TCHAR* Spelling : StateSpellings)
		{
			if (!CS->GetAttribute(Spelling).IsEmpty())
			{
				Info.StateAttributeFound = Spelling;
				Info.bCanGetAndSetState = AttrBool(CS, Spelling);
				break;
			}
		}

		Info.bCanSerializeState =
			AttrBool(CS, TEXT("canSerializeFMUState")) || AttrBool(CS, TEXT("canSerializeFMUstate"));
		Info.bCanHandleVariableStep     = AttrBool(CS, TEXT("canHandleVariableCommunicationStepSize"));
		Info.bOnlyOneInstancePerProcess = AttrBool(CS, TEXT("canBeInstantiatedOnlyOncePerProcess"));
		Info.bHasEventMode              = AttrBool(CS, TEXT("hasEventMode"));

		const FString Step = CS->GetAttribute(TEXT("fixedInternalStepSize"));
		Info.FixedInternalStepSize = Step.IsEmpty() ? 0.0 : FCString::Atod(*Step);
	}

	// Variable counts. FMI 2.0 wraps everything in <ScalarVariable>; FMI 3.0
	// uses a per-type element (<Float64>, <Int32>, ...). Counting by causality
	// works for both without enumerating type names.
	if (const FXmlNode* MV = Root->FindChildNode(TEXT("ModelVariables")))
	{
		for (const FXmlNode* V : MV->GetChildrenNodes())
		{
			const FString C = V->GetAttribute(TEXT("causality"));
			if (C == TEXT("input"))          Info.NumInputs++;
			else if (C == TEXT("output"))    Info.NumOutputs++;
			else if (C == TEXT("parameter") || C == TEXT("calculatedParameter")) Info.NumParameters++;
		}
	}

	return true;
}

FString FFSDSFmuPackage::GetBinaryPathForHost() const
{
	if (!bValid || Info.ModelIdentifier.IsEmpty()) return FString();

#if PLATFORM_MAC
	const TCHAR* Ext = TEXT(".dylib");
#elif PLATFORM_WINDOWS
	const TCHAR* Ext = TEXT(".dll");
#else
	const TCHAR* Ext = TEXT(".so");
#endif

	for (const FString& Tuple : GetHostPlatformTuples())
	{
		if (!Info.BinaryPlatforms.Contains(Tuple)) continue;
		const FString Candidate = FPaths::Combine(
			ExtractedDir, TEXT("binaries"), Tuple, Info.ModelIdentifier + Ext);
		if (IFileManager::Get().FileExists(*Candidate)) return Candidate;
	}
	return FString();
}

TArray<FFSDSFmuGate> FFSDSFmuPackage::CheckGates(double CommunicationStep) const
{
	TArray<FFSDSFmuGate> Gates;
	auto Add = [&Gates](const TCHAR* Name, bool bOk, bool bRequired, const FString& Detail)
	{
		Gates.Add({ Name, bOk, bRequired, Detail });
	};

	Add(TEXT("Co-Simulation present"), Info.bHasCoSimulation, true,
		TEXT("a Model-Exchange-only FMU needs an external solver this platform does not have"));

	Add(TEXT("canGetAndSetFMUState"), Info.bCanGetAndSetState, true,
		Info.StateAttributeFound.IsEmpty()
			? FString(TEXT("attribute ABSENT — deterministic reset is impossible; in Simulink "
			               "this is the state-save export option"))
			: FString::Printf(TEXT("read from '%s'"), *Info.StateAttributeFound));

	if (Info.FixedInternalStepSize > 0.0)
	{
		const double Ratio = CommunicationStep / Info.FixedInternalStepSize;
		const bool bIntegral = FMath::Abs(Ratio - FMath::RoundToDouble(Ratio)) < 1e-9;
		Add(TEXT("internal step divides the communication step"), bIntegral, true,
			FString::Printf(TEXT("%.6f / %g = %.6f substeps%s"),
				CommunicationStep, Info.FixedInternalStepSize, Ratio,
				bIntegral ? TEXT("") : TEXT("  NOT an integer")));
	}
	else
	{
		Add(TEXT("internal step divides the communication step"), false, false,
			TEXT("fixedInternalStepSize not declared — a data-dependent substep count "
			     "would break the N-ticks-equals-N-doSteps invariant"));
	}

	Add(TEXT("multi-instance allowed"), !Info.bOnlyOneInstancePerProcess, true,
		TEXT("canBeInstantiatedOnlyOncePerProcess=true prevents editor reload without a "
		     "full process restart; Simulink's supportMultiInstance defaults OFF"));

	const FString HostBinary = GetBinaryPathForHost();
	Add(TEXT("binary for this host"), !HostBinary.IsEmpty() || Info.bHasSourceCode, true,
		FString::Printf(TEXT("present: [%s]; this host accepts [%s]%s"),
			*FString::Join(Info.BinaryPlatforms, TEXT(", ")),
			*FString::Join(GetHostPlatformTuples(), TEXT(", ")),
			Info.bHasSourceCode ? TEXT("; sourceCode present, so it can be compiled") : TEXT("")));

	return Gates;
}

bool FFSDSFmuPackage::LogGateResults(double CommunicationStep) const
{
	UE_LOG(LogTemp, Log, TEXT("FSDS FMU: ---- %s ----"), *Info.ModelName);
	UE_LOG(LogTemp, Log, TEXT("FSDS FMU:   FMI %s, id '%s', tool '%s'"),
		Info.Version == EFSDSFmiVersion::FMI3 ? TEXT("3.0")
			: Info.Version == EFSDSFmiVersion::FMI2 ? TEXT("2.0") : TEXT("?"),
		*Info.ModelIdentifier, *Info.GenerationTool);
	UE_LOG(LogTemp, Log, TEXT("FSDS FMU:   token %s"), *Info.InstantiationToken);
	UE_LOG(LogTemp, Log, TEXT("FSDS FMU:   binaries [%s]  sources %s  resources %s"),
		*FString::Join(Info.BinaryPlatforms, TEXT(", ")),
		Info.bHasSourceCode ? TEXT("yes") : TEXT("no"),
		Info.bHasResources ? TEXT("yes") : TEXT("no"));
	UE_LOG(LogTemp, Log, TEXT("FSDS FMU:   %d input(s), %d output(s), %d parameter(s)"),
		Info.NumInputs, Info.NumOutputs, Info.NumParameters);

	bool bAllRequiredPassed = true;
	for (const FFSDSFmuGate& G : CheckGates(CommunicationStep))
	{
		if (G.bPassed)
		{
			UE_LOG(LogTemp, Log, TEXT("FSDS FMU:   [PASS] %s — %s"), *G.Name, *G.Detail);
		}
		else if (G.bRequired)
		{
			bAllRequiredPassed = false;
			UE_LOG(LogTemp, Error, TEXT("FSDS FMU:   [FAIL] %s — %s"), *G.Name, *G.Detail);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("FSDS FMU:   [WARN] %s — %s"), *G.Name, *G.Detail);
		}
	}
	return bAllRequiredPassed;
}
