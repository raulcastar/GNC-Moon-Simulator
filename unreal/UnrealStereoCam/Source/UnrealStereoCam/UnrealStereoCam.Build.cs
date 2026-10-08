using UnrealBuildTool;

public class UnrealStereoCam : ModuleRules
{
	public UnrealStereoCam(ReadOnlyTargetRules Target) : base(Target)
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
