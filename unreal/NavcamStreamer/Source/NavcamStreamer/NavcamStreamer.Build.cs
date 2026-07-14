using UnrealBuildTool;

public class NavcamStreamer : ModuleRules
{
	public NavcamStreamer(ReadOnlyTargetRules Target) : base(Target)
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
