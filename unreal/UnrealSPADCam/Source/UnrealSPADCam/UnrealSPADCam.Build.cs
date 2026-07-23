using UnrealBuildTool;

public class UnrealSPADCam : ModuleRules
{
	public UnrealSPADCam(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"Sockets",
			"Networking",
			"RHI",
			"RenderCore"
		});
	}
}
