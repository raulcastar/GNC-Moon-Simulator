// GncSimTimeLibrary: the simulation time (GncSimTime) for Blueprints.
//
// The rover Blueprint adds it to its telemetry, so odometry is stamped with
// the time Unreal measured the pose on (the clock the sensor plugins stamp
// with) instead of when the message reached ROS:
//   "t_ns": Get Sim Time Ns String   (exact nanoseconds since the Unix epoch, as
//                                     text: a JSON number cannot hold 19 digits)

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "GncSimTimeLibrary.generated.h"

UCLASS()
class UNREALSIMCLOCK_API UGncSimTimeLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// Simulation time in ns since the Unix epoch, as decimal text (exact).
	UFUNCTION(BlueprintPure, Category = "GNC|Sim Time", meta = (WorldContext = "WorldContextObject"))
	static FString GetSimTimeNsString(const UObject* WorldContextObject);

	// Simulation time in seconds since the Unix epoch (sub-microsecond resolution).
	UFUNCTION(BlueprintPure, Category = "GNC|Sim Time", meta = (WorldContext = "WorldContextObject"))
	static double GetSimTimeSeconds(const UObject* WorldContextObject);
};
