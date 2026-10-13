#include "UnrealSPADCam.h"
#include "GncSimTime.h"

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
	struct FSpadPacketHeader
	{
		uint8  Magic[4];      // 'SPD0'
		uint8  Version;       // 1
		uint8  CamId;
		uint16 Width;
		uint16 Height;
		uint64 TimeNs;
		uint32 PayloadSize;   // width*height (mono8)
		uint16 Reserved;
	};
#pragma pack(pop)
	static_assert(sizeof(FSpadPacketHeader) == 24, "SPAD header must be 24 bytes");
}

UUnrealSPADCam::UUnrealSPADCam()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UUnrealSPADCam::BeginPlay()
{
	Super::BeginPlay();

	ResolveCapture();

	Stream = MakeShared<FSpadStreamState, ESPMode::ThreadSafe>();

	bStopWorker = false;
	WorkerThread = FRunnableThread::Create(this, TEXT("SPADCamWorker"));
}

void UUnrealSPADCam::EndPlay(const EEndPlayReason::Type Reason)
{
	bStopWorker = true;
	if (WorkerThread)
	{
		WorkerThread->Kill(true);
		delete WorkerThread;
		WorkerThread = nullptr;
	}
	bConnected = false;

	Stream.Reset();
	Super::EndPlay(Reason);
}

TArray<FString> UUnrealSPADCam::GetSceneCaptureNames() const
{
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

void UUnrealSPADCam::ResolveCapture()
{
	Capture = nullptr;
	if (CaptureName.IsNone() || !GetOwner())
	{
		UE_LOG(LogTemp, Error,
		       TEXT("UnrealSPADCam: no CaptureName set (or no owner); no frames "
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
		       TEXT("UnrealSPADCam: no SceneCapture2D named '%s'. Available: [%s]"),
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
		       TEXT("UnrealSPADCam: created a %dx%d RGBA8 render target for '%s'."),
		       CaptureWidth, CaptureHeight, *CaptureName.ToString());
	}

	Capture->bCaptureEveryFrame = true;
	Capture->bCaptureOnMovement = false;
	Capture->CaptureSource = SCS_FinalColorLDR;

	UE_LOG(LogTemp, Log,
	       TEXT("UnrealSPADCam: capturing '%s' (%dx%d) -> cam id %d, streaming "
	            "intensity to %s:%d."),
	       *CaptureName.ToString(), Capture->TextureTarget->SizeX,
	       Capture->TextureTarget->SizeY, CamId, *ServerIp, ServerPort);
}

bool UUnrealSPADCam::TryConnect()
{
	ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	FIPv4Address Ip;
	if (!FIPv4Address::Parse(ServerIp, Ip))
	{
		UE_LOG(LogTemp, Error, TEXT("UnrealSPADCam: bad ServerIp '%s'."), *ServerIp);
		return false;
	}
	TSharedRef<FInternetAddr> Addr = Subsystem->CreateInternetAddr();
	Addr->SetIp(Ip.Value);
	Addr->SetPort(ServerPort);

	FSocket* NewSocket = Subsystem->CreateSocket(NAME_Stream, TEXT("SPADCamStream"), false);
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
			       TEXT("UnrealSPADCam: cannot reach %s:%d yet — retrying every "
			            "3 s until the ROS SPAD node is up."),
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
	UE_LOG(LogTemp, Log, TEXT("UnrealSPADCam: connected to %s:%d."), *ServerIp, ServerPort);
	return true;
}

void UUnrealSPADCam::TickComponent(float DeltaTime, ELevelTick TickType,
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

void UUnrealSPADCam::PumpCapture()
{
	FTextureRenderTargetResource* Resource =
		Capture->TextureTarget->GameThread_GetRenderTargetResource();
	if (!Resource)
	{
		return;
	}
	const int32 W = Capture->TextureTarget->SizeX;
	const int32 H = Capture->TextureTarget->SizeY;
	const int64 FrameTimeNs = GncSimTime::NowNs(this);   // Simulation time (GncSimTime.h)

	TSharedPtr<FSpadStreamState, ESPMode::ThreadSafe> S = Stream;

	ENQUEUE_RENDER_COMMAND(SPADCamPump)(
		[S, Resource, W, H, FrameTimeNs](FRHICommandListImmediate& RHICmdList)
		{
			// 1. Drain readbacks started on a previous pump, in order, into Ready.
			for (int32 i = 0; i < S->Pending.Num(); )
			{
				FSpadPendingReadback& Job = S->Pending[i];
				if (!Job.Readback->IsReady())
				{
					break;      // keep frames in render order; wait for this one
				}

				const int32 JW = Job.Width, JH = Job.Height;
				FSpadReadyFrame Frame;
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

			while (S->Pending.Num() > 8)
			{
				S->Pending.RemoveAt(0);
			}

			// 2. Start this frame's readback (async GPU copy; nothing waits).
			FSpadPendingReadback Job;
			Job.Width = W;
			Job.Height = H;
			Job.TimeNs = FrameTimeNs;
			Job.Readback = MakeUnique<FRHIGPUTextureReadback>(TEXT("SPADCamReadback"));
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

uint32 UUnrealSPADCam::Run()
{
	TSharedPtr<FSpadStreamState, ESPMode::ThreadSafe> S = Stream;

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

		FSpadReadyFrame Frame;
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

		// BGRA8 -> mono8 intensity (Rec. 601 luma). The SPAD model on the ROS
		// side expects an 8-bit intensity image.
		const int32 Bytes = sizeof(FSpadPacketHeader) + N;
		Packet.SetNumUninitialized(Bytes);

		FSpadPacketHeader* Hd = reinterpret_cast<FSpadPacketHeader*>(Packet.GetData());
		Hd->Magic[0] = 'S'; Hd->Magic[1] = 'P'; Hd->Magic[2] = 'D'; Hd->Magic[3] = '0';
		Hd->Version = 1;
		Hd->CamId = static_cast<uint8>(CamId);
		Hd->Width = static_cast<uint16>(W);
		Hd->Height = static_cast<uint16>(H);
		Hd->TimeNs = static_cast<uint64>(Frame.TimeNs);
		Hd->PayloadSize = static_cast<uint32>(N);
		Hd->Reserved = 0;

		uint8* Dest = Packet.GetData() + sizeof(FSpadPacketHeader);
		const uint8* P = Frame.Bgra.GetData();
		for (int32 i = 0; i < N; ++i)
		{
			const float B = P[i * 4 + 0];
			const float G = P[i * 4 + 1];
			const float R = P[i * 4 + 2];
			Dest[i] = static_cast<uint8>(0.114f * B + 0.587f * G + 0.299f * R);
		}

		if (!SendAll(Socket, Packet.GetData(), Packet.Num(), bStopWorker))
		{
			CloseSocket(Socket);
			bConnected = false;
			if (S.IsValid())
			{
				FSpadReadyFrame Discard;
				while (S->Ready.Dequeue(Discard)) {}   // drop stale frames
			}
			UE_LOG(LogTemp, Warning,
			       TEXT("UnrealSPADCam: link lost; redialling every 3 s."));
		}
	}

	CloseSocket(Socket);
	return 0;
}

void UUnrealSPADCam::Stop()
{
	bStopWorker = true;
}
