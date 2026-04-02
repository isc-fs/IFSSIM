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
			"ImageWrapper",
			"Networking",
			"Sockets"
		});

		PrivateDependencyModuleNames.AddRange(new string[] {
			"Slate",
			"SlateCore"
		});
	}
}
