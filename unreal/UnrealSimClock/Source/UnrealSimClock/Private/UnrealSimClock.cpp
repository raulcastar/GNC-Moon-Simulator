#include "UnrealSimClock.h"
#include "GncSimTime.h"

#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "IPAddress.h"
#include "Sockets.h"
#include "SocketSubsystem.h"

namespace
{
	TAutoConsoleVariable<FString> CVarHost(TEXT("simclock.Host"), TEXT("127.0.0.1"),
		TEXT("Where component_sim_clock listens for the simulation clock (read when Play begins)."));
	TAutoConsoleVariable<int32> CVarPort(TEXT("simclock.Port"), 3008,
		TEXT("TCP port of component_sim_clock (read when Play begins)."));
	TAutoConsoleVariable<float> CVarMaxRateHz(TEXT("simclock.MaxRateHz"), 200.0f,
		TEXT("Most clock records sent per second (one per world tick below that)."));

	constexpr double RedialSeconds = 2.0;      // Between connection attempts
	constexpr double ConnectTimeout = 3.0;     // Give up on a dial after this

#pragma pack(push, 1)
	struct FClockRecord
	{
		uint8  Magic[4];     // 'SCLK'
		uint8  Version;      // 1
		uint8  Reserved[3];
		uint64 SimNs;
		uint64 Frame;
	};
#pragma pack(pop)
	static_assert(sizeof(FClockRecord) == 24, "clock record must be 24 bytes");
}

bool USimClockSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
	const UWorld* World = Cast<UWorld>(Outer);
	return World && (World->WorldType == EWorldType::Game || World->WorldType == EWorldType::PIE);
}

void USimClockSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	ISocketSubsystem* Sockets = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	FIPv4Address Ip;
	if (!Sockets || !FIPv4Address::Parse(CVarHost.GetValueOnGameThread(), Ip))
	{
		UE_LOG(LogTemp, Error, TEXT("UnrealSimClock: bad simclock.Host '%s'; /clock not streamed."),
		       *CVarHost.GetValueOnGameThread());
		return;
	}
	Target = Sockets->CreateInternetAddr();
	Target->SetIp(Ip.Value);
	Target->SetPort(CVarPort.GetValueOnGameThread());
	UE_LOG(LogTemp, Log, TEXT("UnrealSimClock: sim time %.3f s (Unix) at Play start; streaming to %s."),
	       GncSimTime::Now(this), *Target->ToString(true));
	Dial();
}

void USimClockSubsystem::Dial()
{
	ISocketSubsystem* Sockets = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	LinkSince = FPlatformTime::Seconds();
	Socket = Sockets ? Sockets->CreateSocket(NAME_Stream, TEXT("UnrealSimClock"), false) : nullptr;
	if (!Socket)
	{
		Link = ELink::Idle;
		return;
	}
	Socket->SetNonBlocking(true);              // Connect returns at once; Tick polls it
	Socket->SetNoDelay(true);                  // Small records: no Nagle batching
	Socket->Connect(*Target);
	Link = ELink::Connecting;
	Unsent.Reset();
}

void USimClockSubsystem::CloseLink(const TCHAR* Why)
{
	if (Socket)
	{
		Socket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
		Socket = nullptr;
	}
	if (Link == ELink::Connected)
	{
		UE_LOG(LogTemp, Warning, TEXT("UnrealSimClock: link to %s dropped (%s); redialling."),
		       *Target->ToString(true), Why);
	}
	Link = ELink::Idle;
	LinkSince = FPlatformTime::Seconds();
	Unsent.Reset();
}

void USimClockSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if (!Target.IsValid())
	{
		return;
	}
	const double Platform = FPlatformTime::Seconds();

	// ---- link: dial, wait for the non-blocking connect, redial after a drop ----
	if (Link == ELink::Idle)
	{
		if (Platform - LinkSince >= RedialSeconds)
		{
			Dial();
		}
		return;
	}
	if (Link == ELink::Connecting)
	{
		const ESocketConnectionState State = Socket->GetConnectionState();
		if (State == SCS_Connected)
		{
			Link = ELink::Connected;
			bLoggedWaiting = false;
			UE_LOG(LogTemp, Log, TEXT("UnrealSimClock: connected to %s."), *Target->ToString(true));
		}
		else if (State == SCS_ConnectionError || Platform - LinkSince > ConnectTimeout)
		{
			if (!bLoggedWaiting)
			{
				bLoggedWaiting = true;         // Once: component_sim_clock may simply not be up yet
				UE_LOG(LogTemp, Log, TEXT("UnrealSimClock: %s not answering; retrying every %.0f s."),
				       *Target->ToString(true), RedialSeconds);
			}
			CloseLink(TEXT("no answer"));
			return;
		}
		else
		{
			return;
		}
	}

	// ---- send: one 24-byte record per tick (at most MaxRateHz) ----
	const float MaxRate = CVarMaxRateHz.GetValueOnGameThread();
	if (Unsent.Num() == 0)
	{
		if (MaxRate > 0.0f && LastSendSeconds >= 0.0 && Platform - LastSendSeconds < 1.0 / MaxRate)
		{
			return;
		}
		FClockRecord R;
		R.Magic[0] = 'S'; R.Magic[1] = 'C'; R.Magic[2] = 'L'; R.Magic[3] = 'K';
		R.Version = 1;
		R.Reserved[0] = R.Reserved[1] = R.Reserved[2] = 0;
		R.SimNs = static_cast<uint64>(GncSimTime::NowNs(this));
		R.Frame = GFrameCounter;
		Unsent.Append(reinterpret_cast<const uint8*>(&R), sizeof(R));
		LastSendSeconds = Platform;
	}
	// A record the socket only partly took is finished next tick, before any new
	// one, so records never interleave. Would-block just waits for the next tick.
	int32 Sent = 0;
	if (Socket->Send(Unsent.GetData(), Unsent.Num(), Sent))
	{
		Unsent.RemoveAt(0, Sent, EAllowShrinking::No);
	}
	else
	{
		const ESocketErrors Error = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode();
		if (Error != SE_EWOULDBLOCK && Error != SE_NO_ERROR)
		{
			CloseLink(ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetSocketError(Error));
		}
	}
}

void USimClockSubsystem::Deinitialize()
{
	if (Socket)
	{
		Socket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
		Socket = nullptr;
	}
	Link = ELink::Idle;
	Super::Deinitialize();
}

TStatId USimClockSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(USimClockSubsystem, STATGROUP_Tickables);
}
