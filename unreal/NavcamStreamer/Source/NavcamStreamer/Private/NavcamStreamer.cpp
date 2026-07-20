#include "NavcamStreamer.h"

#include "Sockets.h"
#include "SocketSubsystem.h"
#include "UObject/UObjectIterator.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/SCS_Node.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "RenderingThread.h"
#include "TextureResource.h"

namespace
{
#pragma pack(push, 1)
	struct FNavFrameHeader
	{
		uint8  Magic[4];      // 'NAVF'
		uint8  Version;       // 1
		uint8  CamId;
		uint8  Format;        // 1 = BGRA8
		uint8  Reserved;
		uint16 Width;
		uint16 Height;
		uint64 FrameId;
		uint32 PayloadSize;
	};
#pragma pack(pop)
	static_assert(sizeof(FNavFrameHeader) == 24, "header must be 24 bytes");
}

UNavcamStreamer::UNavcamStreamer()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UNavcamStreamer::BeginPlay()
{
	Super::BeginPlay();

	Active.Empty();
	ResolveCapture(LeftCaptureName, LeftCamId);
	ResolveCapture(RightCaptureName, RightCamId);
	ResolveCapture(CenterCaptureName, CenterCamId);

	// The sender thread runs regardless; the tick loop (re)dials the node
	// every few seconds, so the start order of Unreal and ROS is irrelevant.
	bStopSender = false;
	SenderThread = FRunnableThread::Create(this, TEXT("NavcamSender"));
}

void UNavcamStreamer::EndPlay(const EEndPlayReason::Type Reason)
{
	bStopSender = true;
	if (SenderThread)
	{
		// Bounded: Run() checks bStopSender between sends and its connect
		// attempts are capped at the 2 s Wait() in TryConnect, so this join
		// never hangs the way a blocking Connect() on the game thread would.
		SenderThread->Kill(true);
		delete SenderThread;
		SenderThread = nullptr;
	}
	bConnected = false;         // Run() already closed Socket on its way out.
	Pending.Empty();
	Active.Empty();
	Super::EndPlay(Reason);
}

TArray<FString> UNavcamStreamer::GetSceneCaptureNames() const
{
	TSet<FString> Names;

	// On an instance (level or PIE): the owner's components, then the world.
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

	// In the blueprint editor this component is a template with no owner:
	// walk the generated class's construction script (and its parents) for
	// SceneCapture2D nodes — those names are what the instances get at spawn.
	for (const UObject* Outer = GetOuter(); Outer; Outer = Outer->GetOuter())
	{
		const UBlueprintGeneratedClass* BPGC =
			Cast<UBlueprintGeneratedClass>(Outer);
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
			for (const USCS_Node* Node :
			     BPGC->SimpleConstructionScript->GetAllNodes())
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
	// "Navcam_Left", "navcam left" and "NavcamLeft" should all match: names
	// are compared with underscores/spaces stripped, case-insensitively.
	FString NormalizeName(const FString& S)
	{
		return S.Replace(TEXT("_"), TEXT(""))
		        .Replace(TEXT(" "), TEXT("")).ToLower();
	}
}

void UNavcamStreamer::ResolveCapture(const FName& Name, int32 CamId)
{
	if (Name.IsNone())
	{
		return;
	}

	// Candidates: this component's actor first, then the whole world — the
	// captures do not have to live on the pawn carrying the streamer.
	TArray<USceneCaptureComponent2D*> Candidates;
	GetOwner()->GetComponents<USceneCaptureComponent2D>(Candidates);
	for (TObjectIterator<USceneCaptureComponent2D> It; It; ++It)
	{
		if (It->GetWorld() == GetWorld() && !It->IsTemplate())
		{
			Candidates.AddUnique(*It);
		}
	}

	USceneCaptureComponent2D* Found = nullptr;
	for (USceneCaptureComponent2D* C : Candidates)      // exact name
	{
		if (C->GetFName() == Name)
		{
			Found = C;
			break;
		}
	}
	if (!Found)                                         // tolerant name
	{
		const FString Wanted = NormalizeName(Name.ToString());
		for (USceneCaptureComponent2D* C : Candidates)
		{
			if (NormalizeName(C->GetName()) == Wanted)
			{
				Found = C;
				UE_LOG(LogTemp, Log,
				       TEXT("NavcamStreamer: '%s' matched component '%s' on "
				            "'%s' (underscores/case ignored)."),
				       *Name.ToString(), *C->GetName(),
				       *GetNameSafe(C->GetOwner()));
				break;
			}
		}
	}
	if (!Found)
	{
		TArray<FString> Names;
		for (USceneCaptureComponent2D* C : Candidates)
		{
			Names.Add(FString::Printf(TEXT("%s (on %s)"), *C->GetName(),
			                          *GetNameSafe(C->GetOwner())));
		}
		UE_LOG(LogTemp, Error,
		       TEXT("NavcamStreamer: no SceneCapture2D named '%s' anywhere in "
		            "the world. Available: [%s]"),
		       *Name.ToString(), *FString::Join(Names, TEXT(", ")));
		return;
	}
	if (!Found->TextureTarget)
	{
		// No render target assigned: make one. Saves the Content Browser
		// round-trip of creating and wiring three assets by hand.
		UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(Found);
		RT->RenderTargetFormat = RTF_RGBA8;
		RT->ClearColor = FLinearColor::Black;
		RT->InitAutoFormat(CaptureWidth, CaptureHeight);
		RT->UpdateResourceImmediate(true);
		Found->TextureTarget = RT;
		UE_LOG(LogTemp, Log,
		       TEXT("NavcamStreamer: created a %dx%d RGBA8 render target for '%s'."),
		       CaptureWidth, CaptureHeight, *Name.ToString());
	}

	// Renders every frame on the render thread — no game-thread stall — and
	// is read back at SendHz. An on-demand variant (CaptureSceneDeferred,
	// triggered only at SendHz) was tried to cut the GPU cost further, but
	// its readback one tick later captured a stale buffer regardless of the
	// scene: a GPU-pipeline-depth timing bug, not yet solved. Revisit only
	// with a proper GPU fence instead of a fixed one-tick delay.
	Found->bCaptureEveryFrame = true;
	Found->bCaptureOnMovement = false;
	Found->CaptureSource = SCS_FinalColorLDR;

	if (bLockExposure)
	{
		// AEM_Manual disables scene-luminance metering entirely, so there is
		// no auto-exposure feedback loop to fight — unlike pinning
		// AutoExposureMin/MaxBrightness, which fixes the METERED luminance,
		// not the output: forcing that to a value far from the scene's real
		// luminance (a dim lunar surface) drives the exposure curve to
		// wildly overcompensate, which is what blew the image to white.
		FPostProcessSettings& PP = Found->PostProcessSettings;
		PP.bOverride_AutoExposureMethod = true;
		PP.AutoExposureMethod = AEM_Manual;
		PP.bOverride_AutoExposureBias = true;
		PP.AutoExposureBias = ExposureCompensationEV;
	}

	FNavcamResolved Entry;
	Entry.Capture = Found;
	Entry.CamId = CamId;
	Active.Add(Entry);
	UE_LOG(LogTemp, Log, TEXT("NavcamStreamer: '%s' -> cam id %d (%dx%d)."),
	       *Name.ToString(), CamId,
	       Found->TextureTarget->SizeX, Found->TextureTarget->SizeY);
}

bool UNavcamStreamer::TryConnect()
{
	// Called only from Run() (background thread). Non-blocking connect with a
	// short bounded wait: FSocket::Connect() on a BLOCKING socket can take the
	// full OS TCP timeout (many seconds) to fail, and an earlier version of
	// this file called the equivalent of this function from TickComponent —
	// which froze the entire engine (render, input, everything) for that
	// duration, every 3-second retry, whenever the ROS node wasn't already up
	// when Play was pressed. Never do socket I/O on the game thread.
	ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	FIPv4Address Ip;
	if (!FIPv4Address::Parse(ServerIp, Ip))
	{
		UE_LOG(LogTemp, Error, TEXT("NavcamStreamer: bad ServerIp '%s'."), *ServerIp);
		return false;
	}
	TSharedRef<FInternetAddr> Addr = Subsystem->CreateInternetAddr();
	Addr->SetIp(Ip.Value);
	Addr->SetPort(ServerPort);

	FSocket* NewSocket = Subsystem->CreateSocket(NAME_Stream, TEXT("NavcamStream"), false);
	NewSocket->SetNonBlocking(true);
	NewSocket->SetNoDelay(true);
	int32 Unused;
	NewSocket->SetSendBufferSize(4 * 1024 * 1024, Unused);

	// A non-blocking connect returns immediately; success is confirmed by
	// waiting for the socket to become writable, bounded so this thread —
	// and EndPlay's Kill(true) join of it — can never stall for long.
	NewSocket->Connect(*Addr);
	const bool bWritable =
		NewSocket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromSeconds(2.0));
	const bool bOk = bWritable && NewSocket->GetConnectionState() == SCS_Connected;

	if (!bOk)
	{
		if (!bLoggedWaiting)
		{
			UE_LOG(LogTemp, Warning,
			       TEXT("NavcamStreamer: cannot reach %s:%d yet — retrying "
			            "every 3 s until the ROS stereo node is up."),
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
	UE_LOG(LogTemp, Log, TEXT("NavcamStreamer: connected to %s:%d."), *ServerIp, ServerPort);
	return true;
}

void UNavcamStreamer::TickComponent(float DeltaTime, ELevelTick TickType,
                                    FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// The game thread never touches the socket — only this flag, which Run()
	// (background thread) owns. No blocking calls here, ever.
	if (!bConnected)
	{
		return;
	}

	PollReadbacks();

	const double Now = FPlatformTime::Seconds();
	if (Now - LastSendTime >= 1.0 / FMath::Max(SendHz, 0.5f))
	{
		LastSendTime = Now;
		EnqueueCaptures();
	}
}

void UNavcamStreamer::EnqueueCaptures()
{
	// One tick, one frame id: this is what makes the left/right pair a rig.
	// bCaptureEveryFrame means the capture already rendered this frame before
	// component tick order guarantees matter here — the readback reads
	// whatever the capture last rendered, which at every-frame is current.
	const uint64 FrameId = GFrameCounter;

	for (FNavcamResolved& Cam : Active)
	{
		if (!Cam.Capture || !Cam.Capture->TextureTarget)
		{
			continue;
		}
		FTextureRenderTargetResource* Resource =
			Cam.Capture->TextureTarget->GameThread_GetRenderTargetResource();
		if (!Resource)
		{
			continue;
		}

		FPendingReadback Job;
		Job.CamId = Cam.CamId;
		Job.Width = Cam.Capture->TextureTarget->SizeX;
		Job.Height = Cam.Capture->TextureTarget->SizeY;
		Job.FrameId = FrameId;
		Job.Readback = MakeUnique<FRHIGPUTextureReadback>(TEXT("NavcamReadback"));

		FRHIGPUTextureReadback* ReadbackPtr = Job.Readback.Get();
		ENQUEUE_RENDER_COMMAND(NavcamEnqueueCopy)(
			[ReadbackPtr, Resource](FRHICommandListImmediate& RHICmdList)
			{
				ReadbackPtr->EnqueueCopy(RHICmdList,
				                         Resource->GetRenderTargetTexture());
			});
		Pending.Add(MoveTemp(Job));
	}
}

void UNavcamStreamer::PollReadbacks()
{
	for (int32 i = Pending.Num() - 1; i >= 0; --i)
	{
		FPendingReadback& Job = Pending[i];
		if (!Job.Readback->IsReady())
		{
			continue;
		}

		const int32 W = Job.Width, H = Job.Height;
		TArray<uint8> Packet;
		Packet.SetNumUninitialized(sizeof(FNavFrameHeader) + W * H * 4);

		FNavFrameHeader* Header = reinterpret_cast<FNavFrameHeader*>(Packet.GetData());
		Header->Magic[0] = 'N'; Header->Magic[1] = 'A';
		Header->Magic[2] = 'V'; Header->Magic[3] = 'F';
		Header->Version = 1;
		Header->CamId = static_cast<uint8>(Job.CamId);
		Header->Format = 1;                 // BGRA8
		Header->Reserved = 0;
		Header->Width = static_cast<uint16>(W);
		Header->Height = static_cast<uint16>(H);
		Header->FrameId = Job.FrameId;
		Header->PayloadSize = static_cast<uint32>(W * H * 4);

		// Lock/copy on the render thread; the GPU work already completed, so
		// this is a memcpy, not a stall. Rows may be padded (pitch >= width).
		uint8* Dest = Packet.GetData() + sizeof(FNavFrameHeader);
		FRHIGPUTextureReadback* ReadbackPtr = Job.Readback.Get();
		ENQUEUE_RENDER_COMMAND(NavcamLockCopy)(
			[ReadbackPtr, Dest, W, H](FRHICommandListImmediate&)
			{
				int32 RowPitchPixels = 0;
				void* Src = ReadbackPtr->Lock(RowPitchPixels);
				if (Src)
				{
					const uint8* SrcBytes = static_cast<const uint8*>(Src);
					for (int32 Row = 0; Row < H; ++Row)
					{
						FMemory::Memcpy(Dest + Row * W * 4,
						                SrcBytes + Row * RowPitchPixels * 4,
						                W * 4);
					}
					ReadbackPtr->Unlock();
				}
			});
		// The lock-copy must complete before the packet is queued; flushing
		// here waits only for the memcpy above, not for scene rendering.
		FlushRenderingCommands();

		SendQueue.Enqueue(MoveTemp(Packet));
		Pending.RemoveAt(i);
	}
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
}

uint32 UNavcamStreamer::Run()
{
	TArray<uint8> Packet;
	while (!bStopSender)
	{
		if (!bConnected)
		{
			const double Now = FPlatformTime::Seconds();
			if (Now - LastConnectAttempt >= 3.0)
			{
				LastConnectAttempt = Now;
				TryConnect();          // blocks this thread only, briefly and boundedly
			}
			else
			{
				FPlatformProcess::Sleep(0.1f);
			}
			continue;
		}

		if (!SendQueue.Dequeue(Packet))
		{
			FPlatformProcess::Sleep(0.005f);
			continue;
		}
		int32 Offset = 0;
		bool bFailed = false;
		while (Offset < Packet.Num() && !bStopSender)
		{
			int32 Sent = 0;
			if (!Socket->Send(Packet.GetData() + Offset, Packet.Num() - Offset, Sent))
			{
				bFailed = true;
				break;
			}
			Offset += Sent;
		}
		if (bFailed)
		{
			CloseSocket(Socket);
			bConnected = false;
			TArray<uint8> Discard;
			while (SendQueue.Dequeue(Discard)) {}    // stale frames, drop them
			UE_LOG(LogTemp, Warning,
			       TEXT("NavcamStreamer: link lost; redialling every 3 s."));
		}
	}
	CloseSocket(Socket);
	return 0;
}

void UNavcamStreamer::Stop()
{
	bStopSender = true;
}
