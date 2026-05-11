using UnrealBuildTool;

public class IFSSIMEditorTarget : TargetRules
{
	public IFSSIMEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("IFSSIM");
	}
}
