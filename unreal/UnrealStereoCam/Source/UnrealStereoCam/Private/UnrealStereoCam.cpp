#include "UnrealStereoCam.h"
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
	struct FNavFrameHeader
	{
		uint8  Magic[4];      // 'NAVF'
		uint8  Version;       // 2
		uint8  CamId;
		uint8  Format;        // 1 = BGRA8, 2 = R32F depth (cm)
		uint8  Reserved;
		uint16 Width;
		uint16 Height;
		uint64 FrameId;
		uint32 PayloadSize;
		uint64 StampNs;       // Capture time, simulation clock (ns since the Unix epoch)
	};
#pragma pack(pop)
	static_assert(sizeof(FNavFrameHeader) == 32, "header must be 32 bytes");
}

UUnrealStereoCam::UUnrealStereoCam()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UUnrealStereoCam::BeginPlay()
{
	Super::BeginPlay();

	Active.Empty();
	ResolveCapture(LeftCaptureName, LeftCamId);
	ResolveCapture(RightCaptureName, RightCamId);
	ResolveCapture(CenterCaptureName, CenterCamId);
	ResolveCapture(MaskCaptureName, MaskCamId);   // optional; skipped if blank
	if (bStreamCenterDepth)
	{
		AddCenterDepthCapture();
	}

	Stream = MakeShared<FStereoCamStreamState, ESPMode::ThreadSafe>();

	// The sender thread runs regardless; it (re)dials the node every few
	// seconds, so the start order of Unreal and ROS is irrelevant.
	bStopSender = false;
	SenderThread = FRunnableThread::Create(this, TEXT("StereoCamSender"));
}

void UUnrealStereoCam::EndPlay(const EEndPlayReason::Type Reason)
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

	// Drop the component's reference. Any render command still in flight holds
	// its own reference and will free the state — and its readbacks — on the
	// render thread when it finishes. Nothing to block on here: this is what
	// makes stopping instant instead of the multi-second freeze that a
	// FlushRenderingCommands / fence Wait caused.
	Stream.Reset();
	Active.Empty();
	Super::EndPlay(Reason);
}

TArray<FString> UUnrealStereoCam::GetSceneCaptureNames() const
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

void UUnrealStereoCam::ResolveCapture(const FName& Name, int32 CamId)
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
				       TEXT("UnrealStereoCam: '%s' matched component '%s' on "
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
		       TEXT("UnrealStereoCam: no SceneCapture2D named '%s' anywhere in "
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
		       TEXT("UnrealStereoCam: created a %dx%d RGBA8 render target for '%s'."),
		       CaptureWidth, CaptureHeight, *Name.ToString());
	}

	// With bCaptureOnDemand, rendered only when a frame is sent: TickComponent
	// calls CaptureScene(), which queues the render on the render thread, and
	// the readback command is queued right after it, so the render thread runs
	// them in order and the copy reads this render (as UnrealLidarCam does). An
	// earlier variant (CaptureSceneDeferred, read back one tick later) read a
	// stale buffer; the difference is the readback queued behind the render.
	// The rendering state persists between renders, so exposure, temporal AA
	// and Lumen keep their history. Otherwise: rendered every engine frame.
	Found->bCaptureEveryFrame = !bCaptureOnDemand;
	Found->bAlwaysPersistRenderingState = true;
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
	else if (bLimitAutoExposure)
	{
		// Keep auto-exposure, but clamp the EV100 range it may adapt into.
		// AutoExposureMin/MaxBrightness carry EV100 under UE5's extended
		// luminance range (the editor's "Min/Max EV100"). Setting the two
		// equal pins a fixed exposure identical on both stereo eyes.
		FPostProcessSettings& PP = Found->PostProcessSettings;
		const float MinEv = FMath::Min(MinEV100, MaxEV100);
		const float MaxEv = FMath::Max(MinEV100, MaxEV100);
		PP.bOverride_AutoExposureMinBrightness = true;
		PP.AutoExposureMinBrightness = MinEv;
		PP.bOverride_AutoExposureMaxBrightness = true;
		PP.AutoExposureMaxBrightness = MaxEv;
		UE_LOG(LogTemp, Log,
		       TEXT("UnrealStereoCam: '%s' auto-exposure clamped to EV100 "
		            "[%.2f, %.2f]."),
		       *Name.ToString(), MinEv, MaxEv);
	}

	FStereoCamResolved Entry;
	Entry.Capture = Found;
	Entry.CamId = CamId;
	Active.Add(Entry);
	UE_LOG(LogTemp, Log, TEXT("UnrealStereoCam: '%s' -> cam id %d (%dx%d)."),
	       *Name.ToString(), CamId,
	       Found->TextureTarget->SizeX, Found->TextureTarget->SizeY);
}

void UUnrealStereoCam::AddCenterDepthCapture()
{
	const FStereoCamResolved* Center = Active.FindByPredicate(
		[this](const FStereoCamResolved& R) { return R.CamId == CenterCamId && R.Capture; });
	if (!Center || !Center->Capture->TextureTarget)
	{
		UE_LOG(LogTemp, Warning,
		       TEXT("UnrealStereoCam: no centre capture, so no ground-truth depth."));
		return;
	}
	USceneCaptureComponent2D* Source = Center->Capture;
	USceneCaptureComponent2D* Depth = NewObject<USceneCaptureComponent2D>(
		GetOwner(), MakeUniqueObjectName(GetOwner(), USceneCaptureComponent2D::StaticClass(),
		                                 TEXT("StereoCamCenterDepth")));
	Depth->RegisterComponent();
	Depth->AttachToComponent(Source, FAttachmentTransformRules::SnapToTargetNotIncludingScale);

	// The same view as the centre camera: projection, FOV, what it shows.
	Depth->ProjectionType = Source->ProjectionType;
	Depth->FOVAngle = Source->FOVAngle;
	Depth->OrthoWidth = Source->OrthoWidth;
	Depth->PrimitiveRenderMode = Source->PrimitiveRenderMode;
	Depth->HiddenActors = Source->HiddenActors;
	Depth->ShowOnlyActors = Source->ShowOnlyActors;

	UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(Depth);
	RT->RenderTargetFormat = RTF_R32f;
	RT->ClearColor = FLinearColor(1.0e9f, 0.0f, 0.0f, 0.0f);    // Nothing there: far
	RT->InitAutoFormat(Source->TextureTarget->SizeX, Source->TextureTarget->SizeY);
	RT->UpdateResourceImmediate(true);
	Depth->TextureTarget = RT;
	Depth->CaptureSource = SCS_SceneDepth;
	Depth->bCaptureEveryFrame = !bCaptureOnDemand;
	Depth->bAlwaysPersistRenderingState = true;
	Depth->bCaptureOnMovement = false;

	// Depth only: skip all the shading that does not change it (lighting,
	// shadows, global illumination, reflections, fog, decals, translucency,
	// post-processing). Decals and translucent particles write no depth.
	FEngineShowFlags& F = Depth->ShowFlags;
	F.SetLighting(false);
	F.SetDynamicShadows(false);
	F.SetGlobalIllumination(false);
	F.SetLumenGlobalIllumination(false);
	F.SetLumenReflections(false);
	F.SetReflectionEnvironment(false);
	F.SetScreenSpaceReflections(false);
	F.SetAmbientOcclusion(false);
	F.SetDistanceFieldAO(false);
	F.SetSkyLighting(false);
	F.SetFog(false);
	F.SetVolumetricFog(false);
	F.SetAtmosphere(false);
	F.SetDecals(false);
	F.SetTranslucency(false);
	F.SetParticles(false);
	F.SetBloom(false);
	F.SetEyeAdaptation(false);
	F.SetMotionBlur(false);
	F.SetPostProcessing(false);

	FStereoCamResolved Entry;
	Entry.Capture = Depth;
	Entry.CamId = DepthCamId;
	Entry.Format = 2;
	Active.Add(Entry);
	UE_LOG(LogTemp, Log,
	       TEXT("UnrealStereoCam: ground-truth depth of '%s' -> cam id %d (%dx%d, R32F cm)."),
	       *Source->GetName(), DepthCamId, RT->SizeX, RT->SizeY);
}

bool UUnrealStereoCam::TryConnect()
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
		UE_LOG(LogTemp, Error, TEXT("UnrealStereoCam: bad ServerIp '%s'."), *ServerIp);
		return false;
	}
	TSharedRef<FInternetAddr> Addr = Subsystem->CreateInternetAddr();
	Addr->SetIp(Ip.Value);
	Addr->SetPort(ServerPort);

	FSocket* NewSocket = Subsystem->CreateSocket(NAME_Stream, TEXT("StereoCamStream"), false);
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
			       TEXT("UnrealStereoCam: cannot reach %s:%d yet — retrying "
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
	UE_LOG(LogTemp, Log, TEXT("UnrealStereoCam: connected to %s:%d."), *ServerIp, ServerPort);
	return true;
}

void UUnrealStereoCam::TickComponent(float DeltaTime, ELevelTick TickType,
                                    FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// The game thread never touches the socket or a readback — only these two
	// flags, which the sender/render threads own. No blocking calls here, ever.
	if (!bConnected || !Stream.IsValid())
	{
		return;
	}

	// Game time, not wall time: SendHz is frames per simulated second, so the
	// cameras keep their rate relative to physics and /clock when the editor
	// runs slower than real time (fixed frame rate). A new Play session
	// restarts game time at 0.
	const double Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
	if (Now < LastSendTime)
	{
		LastSendTime = -1.0e9;
	}
	if (Now - LastSendTime >= 1.0 / FMath::Max(SendHz, 0.5f))
	{
		LastSendTime = Now;
		if (bCaptureOnDemand)
		{
			for (FStereoCamResolved& Cam : Active)
			{
				if (Cam.Capture && Cam.Capture->TextureTarget)
				{
					Cam.Capture->CaptureScene();     // Queued before PumpCaptures' readback
				}
			}
		}
		PumpCaptures();
	}
}

namespace
{
	// One camera's render target, resolved on the game thread and handed to
	// the render command. File scope so the ENQUEUE_RENDER_COMMAND lambda can
	// name the type.
	struct FStereoCamCap
	{
		FTextureRenderTargetResource* Resource;
		int32 CamId;
		uint8 Format;
		int32 W;
		int32 H;
	};
}

void UUnrealStereoCam::PumpCaptures()
{
	// Gather render-target pointers on the game thread (the only step that
	// must be here), then post ONE render command that owns the whole readback
	// lifecycle. bCaptureEveryFrame means each target already holds this
	// frame's render; the readback of it completes asynchronously on the GPU.
	TArray<FStereoCamCap> Caps;
	for (FStereoCamResolved& Cam : Active)
	{
		if (!Cam.Capture || !Cam.Capture->TextureTarget)
		{
			continue;
		}
		FTextureRenderTargetResource* Resource =
			Cam.Capture->TextureTarget->GameThread_GetRenderTargetResource();
		if (Resource)
		{
			Caps.Add({ Resource, Cam.CamId, Cam.Format,
			           Cam.Capture->TextureTarget->SizeX,
			           Cam.Capture->TextureTarget->SizeY });
		}
	}
	if (Caps.Num() == 0)
	{
		return;
	}

	// One tick, one frame id: this is what pairs left/right as a rig. One capture
	// time too, from the shared simulation clock.
	const uint64 FrameId = GFrameCounter;
	const int64 StampNs = GncSimTime::NowNs(this);
	TSharedPtr<FStereoCamStreamState, ESPMode::ThreadSafe> S = Stream;

	ENQUEUE_RENDER_COMMAND(StereoCamPump)(
		[S, Caps, FrameId, StampNs](FRHICommandListImmediate& RHICmdList)
		{
			// 1. Drain the readbacks started on a previous pump. At any real
			//    SendHz they finished on the GPU long ago; IsReady() gates it
			//    regardless. Lock/copy/Unlock all here on the render thread.
			for (int32 i = S->Pending.Num() - 1; i >= 0; --i)
			{
				FPendingReadback& Job = S->Pending[i];
				if (!Job.Readback->IsReady())
				{
					continue;
				}

				const int32 W = Job.Width, H = Job.Height;
				TArray<uint8> Packet;
				Packet.SetNumUninitialized(sizeof(FNavFrameHeader) + W * H * 4);

				FNavFrameHeader* Hd = reinterpret_cast<FNavFrameHeader*>(Packet.GetData());
				Hd->Magic[0] = 'N'; Hd->Magic[1] = 'A';
				Hd->Magic[2] = 'V'; Hd->Magic[3] = 'F';
				Hd->Version = 2;
				Hd->CamId = static_cast<uint8>(Job.CamId);
				Hd->Format = Job.Format;        // 1 BGRA8, 2 R32F: 4 bytes per pixel either way
				Hd->Reserved = 0;
				Hd->Width = static_cast<uint16>(W);
				Hd->Height = static_cast<uint16>(H);
				Hd->FrameId = Job.FrameId;
				Hd->PayloadSize = static_cast<uint32>(W * H * 4);
				Hd->StampNs = static_cast<uint64>(Job.StampNs);

				int32 RowPitchPixels = 0;
				void* Src = Job.Readback->Lock(RowPitchPixels);
				if (Src)
				{
					uint8* Dest = Packet.GetData() + sizeof(FNavFrameHeader);
					const uint8* SrcBytes = static_cast<const uint8*>(Src);
					for (int32 Row = 0; Row < H; ++Row)
					{
						FMemory::Memcpy(Dest + Row * W * 4,
						                SrcBytes + Row * RowPitchPixels * 4, W * 4);
					}
					Job.Readback->Unlock();
					S->Ready.Enqueue(MoveTemp(Packet));
				}
				S->Pending.RemoveAt(i);
			}

			// Safety net: never let readbacks pile up unbounded and exhaust
			// VRAM if the consumer somehow stalls. Up to 5 cameras x a few frames.
			while (S->Pending.Num() > 16)
			{
				S->Pending.RemoveAt(0);
			}

			// 2. Start this frame's readbacks (async GPU copy; nothing waits).
			for (const FStereoCamCap& C : Caps)
			{
				FPendingReadback Job;
				Job.CamId = C.CamId;
				Job.Format = C.Format;
				Job.Width = C.W;
				Job.Height = C.H;
				Job.FrameId = FrameId;
				Job.StampNs = StampNs;
				Job.Readback = MakeUnique<FRHIGPUTextureReadback>(TEXT("StereoCamReadback"));
				Job.Readback->EnqueueCopy(RHICmdList, C.Resource->GetRenderTargetTexture());
				S->Pending.Add(MoveTemp(Job));
			}
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
}

uint32 UUnrealStereoCam::Run()
{
	// Hold our own reference for the thread's lifetime, so the ready-frame
	// queue stays valid even as the component drops its reference at EndPlay.
	TSharedPtr<FStereoCamStreamState, ESPMode::ThreadSafe> S = Stream;

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

		if (!S.IsValid() || !S->Ready.Dequeue(Packet))
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
				// The socket is non-blocking: a full send buffer (several frames
				// in one pump) answers "would block", which is not a broken link.
				// Wait for room and try again; only a real error drops the link.
				const ESocketErrors Error =
					ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode();
				if (Error == SE_EWOULDBLOCK || Error == SE_NO_ERROR)
				{
					Socket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromMilliseconds(100));
					continue;
				}
				bFailed = true;
				break;
			}
			Offset += Sent;
		}
		if (bFailed)
		{
			CloseSocket(Socket);
			bConnected = false;
			if (S.IsValid())
			{
				TArray<uint8> Discard;
				while (S->Ready.Dequeue(Discard)) {}    // stale frames, drop them
			}
			UE_LOG(LogTemp, Warning,
			       TEXT("UnrealStereoCam: link lost; redialling every 3 s."));
		}
	}
	CloseSocket(Socket);
	return 0;
}

void UUnrealStereoCam::Stop()
{
	bStopSender = true;
}
