// GncSimTime.h — the simulation time shared by the GNC sensor plugins, with no
// dependency between them. An IDENTICAL copy lives in each plugin's Private
// folder (UnrealSimClock, UnrealStereoCam, UnrealLidarCam, UnrealEventCam,
// UnrealSPADCam); change all of them together.
//
// Sim time (ns since the Unix epoch) = Anchor + World->GetTimeSeconds(): it
// follows Unreal's game time (stops when Play is paused, follows time dilation).
// The anchor is the wall clock minus the world time when the first plugin asks
// during a Play session. It is stored as a tag on the world's AWorldSettings, so
// every plugin in that world reads the same anchor and stamps on one clock. A new
// Play session is a new world, so it takes a new, later anchor: the clock does not
// run backwards across sessions (as long as a session's dilation stays <= 1).
// Outside Play (editor world) the wall clock is returned.
//
// Game thread only (all sensor plugins stamp on it).

#pragma once

#include "CoreMinimal.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "Misc/DateTime.h"

namespace GncSimTime
{
	inline int64 WallNs()
	{
		static const FDateTime Epoch(1970, 1, 1);
		return (FDateTime::UtcNow() - Epoch).GetTicks() * 100;     // 1 tick = 100 ns
	}

	// Simulation time of the world Context belongs to, in ns since the Unix epoch.
	inline int64 NowNs(const UObject* Context)
	{
		static const FString Prefix(TEXT("GncSimTimeAnchorNs="));   // no '_': FName would split a number off
		UWorld* World = Context ? Context->GetWorld() : nullptr;
		AWorldSettings* Settings = (World && World->IsGameWorld()) ? World->GetWorldSettings() : nullptr;
		if (!Settings)
		{
			return WallNs();
		}
		const int64 WorldNs = static_cast<int64>(World->GetTimeSeconds() * 1e9);
		for (const FName& Tag : Settings->Tags)
		{
			const FString Text = Tag.ToString();
			if (Text.StartsWith(Prefix))
			{
				return FCString::Atoi64(*Text.RightChop(Prefix.Len())) + WorldNs;
			}
		}
		const int64 Anchor = WallNs() - WorldNs;
		Settings->Tags.Add(FName(*FString::Printf(TEXT("%s%lld"), *Prefix, Anchor)));
		return Anchor + WorldNs;
	}

	// The same in seconds (sub-microsecond resolution at today's epoch values).
	inline double Now(const UObject* Context)
	{
		return static_cast<double>(NowNs(Context)) * 1e-9;
	}
}
