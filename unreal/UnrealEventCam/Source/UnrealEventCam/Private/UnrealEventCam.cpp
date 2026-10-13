#include "UnrealEventCam.h"
#include "GncSimTime.h"

#include "Sockets.h"
#include "SocketSubsystem.h"
#include "UObject/UObjectIterator.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/SCS_Node.h"
#include "Engine/World.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "RenderingThread.h"
#include "TextureResource.h"

namespace
{
#pragma pack(push, 1)
	struct FEventPacketHeader
	{
		uint8  Magic[4];      // 'EVT0'
		uint8  Version;       // 1
		uint8  CamId;
		uint16 Width;
		uint16 Height;
		uint64 T0Ns;          // base timestamp; event t = T0Ns + t_rel
		uint32 Count;
		uint16 Reserved;
	};
	struct FEventWire
	{
		uint16 X;
		uint16 Y;
		uint32 TRelNs;
		uint8  Pol;
	};
#pragma pack(pop)
	static_assert(sizeof(FEventPacketHeader) == 24, "event header must be 24 bytes");
	static_assert(sizeof(FEventWire) == 9, "event record must be 9 bytes");
}

UUnrealEventCam::UUnrealEventCam()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UUnrealEventCam::BeginPlay()
{
	Super::BeginPlay();

	ResolveCapture();

	Stream = MakeShared<FEventStreamState, ESPMode::ThreadSafe>();

	// The worker runs the event model, owns the socket, and (re)dials the node
	// every few seconds, so the start order of Unreal and ROS is irrelevant.
	bStopWorker = false;
	WorkerThread = FRunnableThread::Create(this, TEXT("EventCamWorker"));
}

void UUnrealEventCam::EndPlay(const EEndPlayReason::Type Reason)
{
	bStopWorker = true;
	if (WorkerThread)
	{
		WorkerThread->Kill(true);
		delete WorkerThread;
		WorkerThread = nullptr;
	}
	bConnected = false;

	// Drop our reference. Any render command still in flight holds its own and
	// frees the state — and its readbacks — on the render thread when it finishes.
	Stream.Reset();
	Super::EndPlay(Reason);
}

TArray<FString> UUnrealEventCam::GetSceneCaptureNames() const
{
	// Same discovery as UnrealStereoCam: the owner's SceneCapture2Ds and every
	// instance in the world when placed in a level, plus the blueprint's own
	// components when editing the blueprint (this component is then a template
	// with no owner).
	TSet<FString> Names;

	if (const AActor* Owner = GetOwner())
	{
		TArray<USceneCaptureComponent2D*> Captures;
		Owner->GetComponents<USceneCaptureComponent2D>(Captures);
		for (const USceneCaptureComponent2D* C : Captures)
		{
			Names.Add(C->GetName());
		}
		for (TObjectIterator<USceneCaptureComponent2D> It; It; ++It)
		{
			if (It->GetWorld() == GetWorld() && !It->IsTemplate())
			{
				Names.Add(It->GetName());
			}
		}
	}

	for (const UObject* Outer = GetOuter(); Outer; Outer = Outer->GetOuter())
	{
		const UBlueprintGeneratedClass* BPGC = Cast<UBlueprintGeneratedClass>(Outer);
		if (!BPGC)
		{
			BPGC = Cast<UBlueprintGeneratedClass>(Outer->GetClass());
		}
		for (; BPGC; BPGC = Cast<UBlueprintGeneratedClass>(BPGC->GetSuperClass()))
		{
			if (!BPGC->SimpleConstructionScript)
			{
				continue;
			}
			for (const USCS_Node* Node : BPGC->SimpleConstructionScript->GetAllNodes())
			{
				if (Node && Node->ComponentTemplate &&
				    Node->ComponentTemplate->IsA<USceneCaptureComponent2D>())
				{
					Names.Add(Node->GetVariableName().ToString());
				}
			}
		}
	}

	TArray<FString> Out = Names.Array();
	Out.Sort();
	if (Out.Num() == 0)
	{
		Out.Add(TEXT("(no SceneCapture2D components found)"));
	}
	return Out;
}

namespace
{
	FString NormalizeName(const FString& S)
	{
		return S.Replace(TEXT("_"), TEXT("")).Replace(TEXT(" "), TEXT("")).ToLower();
	}
}

void UUnrealEventCam::ResolveCapture()
{
	Capture = nullptr;
	if (CaptureName.IsNone() || !GetOwner())
	{
		UE_LOG(LogTemp, Error,
		       TEXT("UnrealEventCam: no CaptureName set (or no owner); no frames "
		            "will be produced."));
		return;
	}

	TArray<USceneCaptureComponent2D*> Candidates;
	GetOwner()->GetComponents<USceneCaptureComponent2D>(Candidates);
	for (TObjectIterator<USceneCaptureComponent2D> It; It; ++It)
	{
		if (It->GetWorld() == GetWorld() && !It->IsTemplate())
		{
			Candidates.AddUnique(*It);
		}
	}

	for (USceneCaptureComponent2D* C : Candidates)          // exact name
	{
		if (C->GetFName() == CaptureName)
		{
			Capture = C;
			break;
		}
	}
	if (!Capture)                                           // tolerant name
	{
		const FString Wanted = NormalizeName(CaptureName.ToString());
		for (USceneCaptureComponent2D* C : Candidates)
		{
			if (NormalizeName(C->GetName()) == Wanted)
			{
				Capture = C;
				break;
			}
		}
	}
	if (!Capture)
	{
		TArray<FString> Names;
		for (USceneCaptureComponent2D* C : Candidates)
		{
			Names.Add(C->GetName());
		}
		UE_LOG(LogTemp, Error,
		       TEXT("UnrealEventCam: no SceneCapture2D named '%s'. Available: [%s]"),
		       *CaptureName.ToString(), *FString::Join(Names, TEXT(", ")));
		return;
	}

	if (!Capture->TextureTarget)
	{
		UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(Capture);
		RT->RenderTargetFormat = RTF_RGBA8;
		RT->ClearColor = FLinearColor::Black;
		RT->InitAutoFormat(CaptureWidth, CaptureHeight);
		RT->UpdateResourceImmediate(true);
		Capture->TextureTarget = RT;
		UE_LOG(LogTemp, Log,
		       TEXT("UnrealEventCam: created a %dx%d RGBA8 render target for '%s'."),
		       CaptureWidth, CaptureHeight, *CaptureName.ToString());
	}

	// Render every frame on the render thread; read back asynchronously.
	Capture->bCaptureEveryFrame = true;
	Capture->bCaptureOnMovement = false;
	Capture->CaptureSource = SCS_FinalColorLDR;

	UE_LOG(LogTemp, Log,
	       TEXT("UnrealEventCam: capturing '%s' (%dx%d) -> cam id %d, streaming "
	            "events to %s:%d."),
	       *CaptureName.ToString(), Capture->TextureTarget->SizeX,
	       Capture->TextureTarget->SizeY, CamId, *ServerIp, ServerPort);
}

bool UUnrealEventCam::TryConnect()
{
	// Worker thread only. Non-blocking connect with a short bounded wait, so the
	// thread — and EndPlay's Kill(true) join of it — never stalls for long.
	ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	FIPv4Address Ip;
	if (!FIPv4Address::Parse(ServerIp, Ip))
	{
		UE_LOG(LogTemp, Error, TEXT("UnrealEventCam: bad ServerIp '%s'."), *ServerIp);
		return false;
	}
	TSharedRef<FInternetAddr> Addr = Subsystem->CreateInternetAddr();
	Addr->SetIp(Ip.Value);
	Addr->SetPort(ServerPort);

	FSocket* NewSocket = Subsystem->CreateSocket(NAME_Stream, TEXT("EventCamStream"), false);
	NewSocket->SetNonBlocking(true);
	NewSocket->SetNoDelay(true);
	int32 Unused;
	NewSocket->SetSendBufferSize(4 * 1024 * 1024, Unused);

	NewSocket->Connect(*Addr);
	const bool bWritable =
		NewSocket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromSeconds(2.0));
	const bool bOk = bWritable && NewSocket->GetConnectionState() == SCS_Connected;

	if (!bOk)
	{
		if (!bLoggedWaiting)
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("UnrealEventCam: cannot reach %s:%d yet — retrying every "
			            "3 s until the ROS event node is up."),
			       *ServerIp, ServerPort);
			bLoggedWaiting = true;
		}
		NewSocket->Close();
		Subsystem->DestroySocket(NewSocket);
		return false;
	}

	bLoggedWaiting = false;
	Socket = NewSocket;
	bConnected = true;
	UE_LOG(LogTemp, Log, TEXT("UnrealEventCam: connected to %s:%d."), *ServerIp, ServerPort);
	return true;
}

void UUnrealEventCam::TickComponent(float DeltaTime, ELevelTick TickType,
                                    FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bConnected || !Stream.IsValid() || !Capture || !Capture->TextureTarget)
	{
		return;
	}

	// Game time, not wall time: MaxSampleHz is samples per simulated second,
	// so the rate holds relative to physics and /clock when the editor runs
	// slower than real time (fixed frame rate). A new Play session restarts
	// game time at 0.
	const double Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
	if (Now < LastSampleTime)
	{
		LastSampleTime = -1.0e9;
	}
	if (MaxSampleHz > 0.0f && Now - LastSampleTime < 1.0 / MaxSampleHz)
	{
		return;
	}
	LastSampleTime = Now;
	PumpCapture();
}

void UUnrealEventCam::PumpCapture()
{
	FTextureRenderTargetResource* Resource =
		Capture->TextureTarget->GameThread_GetRenderTargetResource();
	if (!Resource)
	{
		return;
	}
	const int32 W = Capture->TextureTarget->SizeX;
	const int32 H = Capture->TextureTarget->SizeY;

	// Timestamp the frame on the game thread: simulation time (GncSimTime.h), ns.
	const int64 FrameTimeNs = GncSimTime::NowNs(this);

	TSharedPtr<FEventStreamState, ESPMode::ThreadSafe> S = Stream;

	ENQUEUE_RENDER_COMMAND(EventCamPump)(
		[S, Resource, W, H, FrameTimeNs](FRHICommandListImmediate& RHICmdList)
		{
			// 1. Drain readbacks started on a previous pump, in order, into Ready.
			for (int32 i = 0; i < S->Pending.Num(); )
			{
				FEventPendingReadback& Job = S->Pending[i];
				if (!Job.Readback->IsReady())
				{
					break;      // keep frames in render order; wait for this one
				}

				const int32 JW = Job.Width, JH = Job.Height;
				FEventReadyFrame Frame;
				Frame.Width = JW;
				Frame.Height = JH;
				Frame.TimeNs = Job.TimeNs;
				Frame.Bgra.SetNumUninitialized(JW * JH * 4);

				int32 RowPitchPixels = 0;
				void* Src = Job.Readback->Lock(RowPitchPixels);
				if (Src)
				{
					const uint8* SrcBytes = static_cast<const uint8*>(Src);
					for (int32 Row = 0; Row < JH; ++Row)
					{
						FMemory::Memcpy(Frame.Bgra.GetData() + Row * JW * 4,
						                SrcBytes + Row * RowPitchPixels * 4, JW * 4);
					}
					Job.Readback->Unlock();
					S->Ready.Enqueue(MoveTemp(Frame));
				}
				S->Pending.RemoveAt(i);
			}

			// Never let readbacks pile up unbounded (consumer stalled).
			while (S->Pending.Num() > 8)
			{
				S->Pending.RemoveAt(0);
			}

			// 2. Start this frame's readback (async GPU copy; nothing waits).
			FEventPendingReadback Job;
			Job.Width = W;
			Job.Height = H;
			Job.TimeNs = FrameTimeNs;
			Job.Readback = MakeUnique<FRHIGPUTextureReadback>(TEXT("EventCamReadback"));
			Job.Readback->EnqueueCopy(RHICmdList, Resource->GetRenderTargetTexture());
			S->Pending.Add(MoveTemp(Job));
		});
}

namespace
{
	void CloseSocket(FSocket*& Socket)
	{
		if (Socket)
		{
			Socket->Close();
			ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
			Socket = nullptr;
		}
	}

	// Blocking send of a whole buffer; false on socket error.
	bool SendAll(FSocket* Socket, const uint8* Data, int32 Num, const FThreadSafeBool& bStop)
	{
		int32 Offset = 0;
		while (Offset < Num && !bStop)
		{
			int32 Sent = 0;
			if (!Socket->Send(Data + Offset, Num - Offset, Sent))
			{
				// The socket is non-blocking: a full send buffer (a big packet,
				// e.g. the burst of events of the first frames) answers "would
				// block", which is not a broken link. Wait for room and try again;
				// only a real error drops the link.
				const ESocketErrors Error =
					ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode();
				if (Error == SE_EWOULDBLOCK || Error == SE_NO_ERROR)
				{
					Socket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromMilliseconds(100));
					continue;
				}
				return false;
			}
			Offset += Sent;
		}
		return true;
	}
}

uint32 UUnrealEventCam::Run()
{
	TSharedPtr<FEventStreamState, ESPMode::ThreadSafe> S = Stream;

	// The event model lives entirely on this thread: single-threaded, so its
	// per-pixel state needs no locking. Snapshot the params once at start.
	FEventSimulator Sim;
	FEventSimulator::FConfig Cfg;
	Cfg.Cp = ContrastThresholdPos;
	Cfg.Cm = ContrastThresholdNeg;
	Cfg.SigmaCp = SigmaThresholdPos;
	Cfg.SigmaCm = SigmaThresholdNeg;
	Cfg.RefractoryPeriodNs = static_cast<int64>(RefractoryPeriodMs * 1e6);
	Cfg.bUseLogImage = bUseLogImage;
	Sim.Configure(Cfg);

	TArray<float> Intensity;
	TArray<FEventCamEvent> Events;
	TArray<uint8> Packet;

	while (!bStopWorker)
	{
		if (!bConnected)
		{
			const double Now = FPlatformTime::Seconds();
			if (Now - LastConnectAttempt >= 3.0)
			{
				LastConnectAttempt = Now;
				TryConnect();
			}
			else
			{
				FPlatformProcess::Sleep(0.1f);
			}
			continue;
		}

		FEventReadyFrame Frame;
		if (!S.IsValid() || !S->Ready.Dequeue(Frame))
		{
			FPlatformProcess::Sleep(0.001f);
			continue;
		}

		const int32 W = Frame.Width, H = Frame.Height, N = W * H;
		if (N <= 0 || Frame.Bgra.Num() < N * 4)
		{
			continue;
		}

		// BGRA8 -> grayscale intensity in [0, 1] (Rec. 601 luma).
		Intensity.SetNumUninitialized(N);
		const uint8* P = Frame.Bgra.GetData();
		for (int32 i = 0; i < N; ++i)
		{
			const float B = P[i * 4 + 0];
			const float G = P[i * 4 + 1];
			const float R = P[i * 4 + 2];
			Intensity[i] = (0.114f * B + 0.587f * G + 0.299f * R) / 255.0f;
		}

		Events.Reset();
		Sim.ProcessFrame(Intensity.GetData(), W, H, Frame.TimeNs, Events);
		if (Events.Num() == 0)
		{
			continue;
		}

		// Pack: header + N * 9 bytes, timestamps relative to the earliest event.
		const int64 T0 = Events[0].TimeNs;    // events are sorted by time
		const int32 Bytes = sizeof(FEventPacketHeader) + Events.Num() * sizeof(FEventWire);
		Packet.SetNumUninitialized(Bytes);

		FEventPacketHeader* Hd = reinterpret_cast<FEventPacketHeader*>(Packet.GetData());
		Hd->Magic[0] = 'E'; Hd->Magic[1] = 'V'; Hd->Magic[2] = 'T'; Hd->Magic[3] = '0';
		Hd->Version = 1;
		Hd->CamId = static_cast<uint8>(CamId);
		Hd->Width = static_cast<uint16>(W);
		Hd->Height = static_cast<uint16>(H);
		Hd->T0Ns = static_cast<uint64>(T0);
		Hd->Count = static_cast<uint32>(Events.Num());
		Hd->Reserved = 0;

		FEventWire* Wire = reinterpret_cast<FEventWire*>(
			Packet.GetData() + sizeof(FEventPacketHeader));
		for (int32 i = 0; i < Events.Num(); ++i)
		{
			const FEventCamEvent& E = Events[i];
			Wire[i].X = E.X;
			Wire[i].Y = E.Y;
			Wire[i].TRelNs = static_cast<uint32>(FMath::Clamp<int64>(
				E.TimeNs - T0, 0, static_cast<int64>(MAX_uint32)));
			Wire[i].Pol = E.bPositive ? 1 : 0;
		}

		if (!SendAll(Socket, Packet.GetData(), Packet.Num(), bStopWorker))
		{
			CloseSocket(Socket);
			bConnected = false;
			if (S.IsValid())
			{
				FEventReadyFrame Discard;
				while (S->Ready.Dequeue(Discard)) {}   // drop stale frames
			}
			Sim.Reset();                               // resync on reconnect
			UE_LOG(LogTemp, Warning,
			       TEXT("UnrealEventCam: link lost; redialling every 3 s."));
		}
	}

	CloseSocket(Socket);
	return 0;
}

void UUnrealEventCam::Stop()
{
	bStopWorker = true;
}
