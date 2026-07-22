using UnrealBuildTool;

public class StereoCamStreamer : ModuleRules
{
	public StereoCamStreamer(ReadOnlyTargetRules Target) : base(Target)
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
