// UnrealSimClock: streams the simulation time to ROS as /clock.
//
// The simulation time itself is GncSimTime (Private/GncSimTime.h, an identical
// copy of which every GNC sensor plugin carries): Unreal's game time anchored to
// the wall clock once per Play session, in ns since the Unix epoch. The sensor
// plugins stamp with it without depending on this plugin, and this plugin only
// publishes it, so ROS nodes on use_sim_time run on the same clock.
//
// Every world tick (at most simclock.MaxRateHz times a second) the time goes out
// over TCP to component_sim_clock, which publishes /clock. The plugin dials it
// when Play begins and redials every 2 s while it is not there; connecting and
// sending never block the game thread. One 24-byte record per send:
//   offset size  field
//   0      4     magic 'SCLK'
//   4      1     version = 1
//   5      3     reserved
//   8      8     sim time, ns since the Unix epoch (u64, little-endian)
//   16     8     frame (u64): GFrameCounter
// Console variables: simclock.Host (127.0.0.1), simclock.Port (3008),
// simclock.MaxRateHz (200).

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "UnrealSimClock.generated.h"

class FSocket;
class FInternetAddr;

UCLASS()
class USimClockSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	virtual void Deinitialize() override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;

private:
	void Dial();                       // Start a non-blocking connect
	void CloseLink(const TCHAR* Why);

	enum class ELink : uint8 { Idle, Connecting, Connected };

	FSocket* Socket = nullptr;
	TSharedPtr<FInternetAddr> Target;
	ELink Link = ELink::Idle;
	double LinkSince = 0.0;            // Platform time of the last dial / state change
	double LastSendSeconds = -1.0;     // Platform time of the last record
	TArray<uint8> Unsent;              // Rest of a record the socket could not take yet
	bool bLoggedWaiting = false;
};
