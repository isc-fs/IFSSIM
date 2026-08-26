using UnrealBuildTool;
using System.IO;

public class FSDSPlugin : ModuleRules
{
	public FSDSPlugin(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bEnableExceptions = true;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"ChaosVehicles",
			"ChaosVehiclesCore",   // Chaos::MToCm, FSimpleWheelSim readback (Stage 0 config verification)
			"PhysicsCore",
			"RHI",
			"RenderCore",
			"Projects",        // IPluginManager — needed to register Shaders/ path (#223 Phase 3)
			"ImageWrapper",
			"Networking",
			"Sockets",
			"Json",
			"JsonUtilities"
		});

		PrivateDependencyModuleNames.AddRange(new string[] {
			"Slate",
			"SlateCore",
			// FMU (.fmu) packages are ZIP containers. We read them ourselves
			// on top of zlib rather than using the engine's FZipArchiveReader,
			// whose libzip backend is linked only under bBuildEditor — that
			// would make FMU loading work in the editor and fail silently in a
			// packaged build. See FSDSZipReader.h.
			"zlib",
			// modelDescription.xml parsing.
			"XmlParser"
		});

		// winmm.lib provides timeBeginPeriod / timeEndPeriod, which we
		// call in FFSDSPluginModule::StartupModule on Windows to drop
		// the system scheduler tick to 1 ms. Without it FPlatformProcess::
		// Sleep(2.5ms) returns after ~15.6 ms and the sensor stream
		// runs at 12 Hz instead of the intended 400 Hz. See FSDSPlugin.cpp
		// for the full rationale.
		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			PublicSystemLibraries.Add("winmm.lib");
		}
	}
}
