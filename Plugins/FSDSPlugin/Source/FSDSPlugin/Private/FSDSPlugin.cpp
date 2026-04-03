#include "FSDSPlugin.h"
#include "Misc/Paths.h"

#define LOCTEXT_NAMESPACE "FFSDSPluginModule"

void FFSDSPluginModule::StartupModule()
{
	UE_LOG(LogTemp, Log, TEXT("FSDSPlugin: Module started"));

	// Register a content path alias so legacy /AirSim/ asset references resolve
	// The FSDS car assets have internal references to /AirSim/VehicleAdv/...
	// We mount our plugin content at /AirSim/ as well so they can find each other
	FString PluginContentDir = FPaths::Combine(
		FPaths::ProjectPluginsDir(), TEXT("FSDSPlugin"), TEXT("Content"));

	if (FPaths::DirectoryExists(PluginContentDir))
	{
		FPackageName::RegisterMountPoint(TEXT("/AirSim/"), PluginContentDir + TEXT("/"));
		UE_LOG(LogTemp, Log, TEXT("FSDSPlugin: Registered /AirSim/ mount point at %s"), *PluginContentDir);
	}
}

void FFSDSPluginModule::ShutdownModule()
{
	UE_LOG(LogTemp, Log, TEXT("FSDSPlugin: Module shutdown"));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FFSDSPluginModule, FSDSPlugin)
