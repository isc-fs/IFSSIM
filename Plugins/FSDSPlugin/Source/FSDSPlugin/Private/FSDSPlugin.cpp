#include "FSDSPlugin.h"

#define LOCTEXT_NAMESPACE "FFSDSPluginModule"

void FFSDSPluginModule::StartupModule()
{
	UE_LOG(LogTemp, Log, TEXT("FSDSPlugin: Module started"));
}

void FFSDSPluginModule::ShutdownModule()
{
	UE_LOG(LogTemp, Log, TEXT("FSDSPlugin: Module shutdown"));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FFSDSPluginModule, FSDSPlugin)
