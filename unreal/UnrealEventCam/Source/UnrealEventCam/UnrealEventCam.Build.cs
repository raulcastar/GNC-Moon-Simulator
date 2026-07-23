using UnrealBuildTool;

public class UnrealEventCam : ModuleRules
{
	public UnrealEventCam(ReadOnlyTargetRules Target) : base(Target)
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
