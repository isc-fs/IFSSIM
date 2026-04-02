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
			"PhysicsCore",
			"RHI",
			"RenderCore",
			"ImageWrapper"
		});

		PrivateDependencyModuleNames.AddRange(new string[] {
			"Slate",
			"SlateCore"
		});

		// rpclib will be integrated in a later phase
		// For now, the plugin compiles as a standalone UE5 module
	}
}
