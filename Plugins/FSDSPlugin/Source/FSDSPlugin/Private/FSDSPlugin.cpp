#include "FSDSPlugin.h"
#include "Misc/Paths.h"
#include "Misc/PackageName.h"

#define LOCTEXT_NAMESPACE "FFSDSPluginModule"

void FFSDSPluginModule::StartupModule()
{
	// Register /AirSim/ mount point so assets whose internal references still use
	// the old plugin name resolve correctly against FSDSPlugin/Content/.
	FString PluginContentDir = FPaths::Combine(
		FPaths::ProjectPluginsDir(), TEXT("FSDSPlugin"), TEXT("Content"));

	if (FPaths::DirectoryExists(PluginContentDir))
	{
		FPackageName::RegisterMountPoint(TEXT("/AirSim/"), PluginContentDir + TEXT("/"));
	}
}

void FFSDSPluginModule::ShutdownModule()
{
	FPackageName::UnRegisterMountPoint(TEXT("/AirSim/"), FPaths::Combine(
		FPaths::ProjectPluginsDir(), TEXT("FSDSPlugin"), TEXT("Content")) + TEXT("/"));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FFSDSPluginModule, FSDSPlugin)
