using UnrealBuildTool;

public class UnrealLidarCam : ModuleRules
{
	public UnrealLidarCam(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });
		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"PhysicsCore", "Sockets", "Networking", "RHI", "RenderCore"
		});
	}
}
