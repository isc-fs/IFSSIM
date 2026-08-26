// An FMU as a vehicle plant.
#pragma once

#include "CoreMinimal.h"
#include "Plant/FSDSPlant.h"
#include "FMI/FSDSFmuPackage.h"
#include "FMI/FSDSFmi3.h"

/**
 * Drives an FMI 3.0 Co-Simulation FMU through the plant interface.
 *
 * Value references are resolved BY NAME from modelDescription.xml, not
 * hardcoded. A Simulink re-export renumbers them freely — pinning the numbers
 * would work until the first time somebody added a signal, and would then be
 * wrong in a way that type-checks perfectly because every port is a double.
 */
class FSDSPLUGIN_API FFSDSFmuPlant : public IFSDSPlant
{
public:
	/** Path to the .fmu. Call Initialise() afterwards. */
	explicit FFSDSFmuPlant(const FString& InFmuPath);
	virtual ~FFSDSFmuPlant();

	virtual FString GetName() const override;
	virtual bool Initialise() override;
	virtual void PreStep(const FFSDSPlantInput& In) override;
	virtual void PostStep(FFSDSPlantOutput& Out) override;
	virtual void Reset(const double Position[3], const double Quat[4]) override;

	virtual bool SaveState(void*& OutState) override;
	virtual bool RestoreState(void* State) override;
	virtual void FreeState(void*& State) override;
	virtual bool SupportsStateSaveRestore() const override;

	const FString& GetLastError() const { return LastError; }

private:
	/** Look up a value reference by variable name; INDEX_NONE if absent. */
	int64 VR(const TCHAR* Name) const;
	bool  SetScalar(const TCHAR* Name, double Value);
	bool  GetScalar(const TCHAR* Name, double& Out) const;
	bool  GetArray(const TCHAR* Name, double* Out, int32 Count) const;
	bool  SetArray(const TCHAR* Name, const double* In, int32 Count);

	FString FmuPath;
	FString LastError;
	FFSDSFmuPackage Package;
	mutable FFSDSFmi3Instance Fmu;
	TMap<FString, uint32> NameToVR;
	TMap<FString, int32>  NameToCount;
	double CurrentTime = 0.0;
	bool bReady = false;

	/** The FMU's state immediately after initialisation, before any DoStep.
	 *  This is what Reset() restores — an FMU's pose is internal state with no
	 *  input to write, so a snapshot is the only honest reset there is. */
	void*  PristineState = nullptr;
	double PristineTime  = 0.0;

	/** A reset asks for a pose the snapshot alone cannot provide. The snapshot
	 *  clears the internal state (wheel speeds, filter memory); the injection
	 *  then puts the body where the platform asked. Applied on the NEXT
	 *  PreStep because that is when the FMU next reads its inputs. */
	bool   bPendingSync = false;
	double PendingPos[3]  = {0,0,0};
	double PendingQuat[4] = {1,0,0,0};
};
