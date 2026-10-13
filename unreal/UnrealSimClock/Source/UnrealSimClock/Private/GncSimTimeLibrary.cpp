#include "GncSimTimeLibrary.h"

#include "GncSimTime.h"

FString UGncSimTimeLibrary::GetSimTimeNsString(const UObject* WorldContextObject)
{
	return FString::Printf(TEXT("%lld"), GncSimTime::NowNs(WorldContextObject));
}

double UGncSimTimeLibrary::GetSimTimeSeconds(const UObject* WorldContextObject)
{
	return GncSimTime::Now(WorldContextObject);
}
