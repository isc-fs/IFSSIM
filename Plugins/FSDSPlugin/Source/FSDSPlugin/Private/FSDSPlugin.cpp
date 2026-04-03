#include "FSDSPlugin.h"
#include "Misc/Paths.h"
#include "Misc/PackageName.h"

#define LOCTEXT_NAMESPACE "FFSDSPluginModule"

// Register /AirSim/ mount point as a static initializer — runs before CDO construction
// This is critical so that FormulaMesh.uasset can find its skeleton at /AirSim/...
struct FFSDSEarlyMountPoint
{
	FFSDSEarlyMountPoint()
	{
		FString PluginContentDir = FPaths::Combine(
			FPaths::ProjectPluginsDir(), TEXT("FSDSPlugin"), TEXT("Content"));

		if (FPaths::DirectoryExists(PluginContentDir))
		{
			FPackageName::RegisterMountPoint(TEXT("/AirSim/"), PluginContentDir + TEXT("/"));
		}
	}
};
static FFSDSEarlyMountPoint GEarlyMountPoint;

void FFSDSPluginModule::StartupModule()
{
	UE_LOG(LogTemp, Log, TEXT("FSDSPlugin: Module started (mount point already registered)"));
}

void FFSDSPluginModule::ShutdownModule()
{
	UE_LOG(LogTemp, Log, TEXT("FSDSPlugin: Module shutdown"));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FFSDSPluginModule, FSDSPlugin)
