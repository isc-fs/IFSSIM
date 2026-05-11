#include "FSDSPlugin.h"

#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

#define LOCTEXT_NAMESPACE "FFSDSPluginModule"

void FFSDSPluginModule::StartupModule()
{
	// Register the plugin's Shaders/ directory under the virtual path
	// "/Plugin/FSDSPlugin" so .usf files can be referenced by
	// IMPLEMENT_GLOBAL_SHADER without absolute paths. Mirrors the
	// canonical UE5 plugin-shader bootstrap pattern; see #223 Phase 3
	// (FSDSLidarDecode.usf is the first shader to live here).
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("FSDSPlugin"));
	if (Plugin.IsValid())
	{
		const FString ShaderDir = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders"));
		AddShaderSourceDirectoryMapping(TEXT("/Plugin/FSDSPlugin"), ShaderDir);
	}
}

void FFSDSPluginModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FFSDSPluginModule, FSDSPlugin)
