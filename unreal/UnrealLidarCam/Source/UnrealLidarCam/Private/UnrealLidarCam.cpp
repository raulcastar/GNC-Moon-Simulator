#include "UnrealLidarCam.h"

#include "Async/ParallelFor.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "DrawDebugHelpers.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Math/RandomStream.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "RenderingThread.h"
#include "RHIGPUReadback.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "TextureResource.h"

#include <limits>

namespace
{
#pragma pack(push, 1)
	struct FLidarHeader
	{
		uint8  Magic[4];       // 'LIDR'
		uint8  Version;        // 1
		uint8  SensorId;
		uint16 Flags;          // bit 0: organized
		uint64 FrameId;
		double Stamp;
		float  HorizontalFov;
		float  VerticalFov;
		uint32 Columns;
		uint32 Rows;
		uint32 PointCount;
		uint32 PointStep;      // 20
	};

	struct FLidarPoint
	{
		float  X, Y, Z;        // Metres, sensor frame, ROS axes (x forward, y left, z up)
		float  Intensity;      // 0-255
		uint16 Ring;
		uint16 Column;
	};
#pragma pack(pop)
	static_assert(sizeof(FLidarHeader) == 48, "header must be 48 bytes");
	static_assert(sizeof(FLidarPoint) == 20, "point must be 20 bytes");

	constexpr float DefaultReflectivity = 20.0f;   // As MetaLidar, without a PM_Reflectivity_ material
	constexpr int32 MaxQueuedScans = 2;           // Older scans are dropped if the link lags
	constexpr int32 MaxGpuScansInFlight = 3;      // GPU readbacks not yet collected
	constexpr float GpuFarCm = 1.0e7f;            // Depth beyond this: nothing there (sky)
	constexpr float GpuMaxVerticalFov = 160.0f;   // Beyond it a perspective view cannot hold the aperture
	constexpr float GpuMaxSectorFov = 90.0f;      // Horizontal span of one depth capture
	constexpr int32 GpuMaxTextureSize = 4096;

	FString NormalizeName(const FString& S)
	{
		return S.Replace(TEXT("_"), TEXT("")).Replace(TEXT(" "), TEXT("")).ToLower();
	}

	// Reflectivity 0-255 from a physical material named "PM_Reflectivity_<n>" (MetaLidar's convention).
	float Reflectivity(const UPhysicalMaterial* Material)
	{
		if (Material)
		{
			const FString Name = Material->GetName();
			if (Name.StartsWith(TEXT("PM_Reflectivity_")))
			{
				return FMath::Clamp((float)FCString::Atoi(*Name.RightChop(16)), 0.0f, 255.0f);
			}
		}
		return DefaultReflectivity;
	}

	float GaussianNoise(FRandomStream& Random, float StdDev)
	{
		const float U1 = FMath::Max(Random.GetFraction(), 1e-7f);      // Box-Muller
		const float U2 = Random.GetFraction();
		return StdDev * FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * UE_PI * U2);
	}

	// Splits [0, Count) into one chunk per worker thread and runs Body(Begin, End, ChunkIndex).
	template <typename TBody>
	void ForChunks(int32 Count, TBody&& Body)
	{
		const int32 Chunks = FMath::Max(1, FMath::Min(Count, FPlatformMisc::NumberOfWorkerThreadsToSpawn() * 4));
		ParallelFor(Chunks, [&](int32 Chunk)
		{
			const int32 Begin = static_cast<int32>((int64)Count * Chunk / Chunks);
			const int32 End = static_cast<int32>((int64)Count * (Chunk + 1) / Chunks);
			Body(Begin, End, Chunk);
		});
	}
}

// -----------------------------------------------------------------------------
// GPU mode: depth captures read back asynchronously. Like UnrealStereoCam,
// readbacks are created, polled, locked and freed only on the render thread;
// finished scans reach the game thread through a lock-free queue.
// -----------------------------------------------------------------------------

struct FGpuScanJob
{
	double Stamp = 0.0;
	uint64 FrameId = 0;
	TArray<TUniquePtr<FRHIGPUTextureReadback>> Readbacks;
	TArray<FIntPoint> Sizes;
};

struct FGpuScanResult
{
	double Stamp = 0.0;
	uint64 FrameId = 0;
	TArray<TArray<float>> Depth;            // One per sector, row-major, centimetres
};

struct FUnrealLidarCamGpuState
{
	TArray<FGpuScanJob> Pending;            // Render thread only
	TQueue<FGpuScanResult, EQueueMode::Mpsc> Ready;
	FThreadSafeCounter InFlight;

	// Render thread: move every scan whose readbacks have all landed to Ready.
	void Drain()
	{
		for (int32 J = 0; J < Pending.Num();)
		{
			FGpuScanJob& Job = Pending[J];
			bool bReady = true;
			for (const TUniquePtr<FRHIGPUTextureReadback>& Readback : Job.Readbacks)
			{
				bReady &= Readback->IsReady();
			}
			if (!bReady)
			{
				++J;
				continue;
			}
			FGpuScanResult Result;
			Result.Stamp = Job.Stamp;
			Result.FrameId = Job.FrameId;
			for (int32 K = 0; K < Job.Readbacks.Num(); ++K)
			{
				const FIntPoint Size = Job.Sizes[K];
				TArray<float> Depth;
				Depth.SetNumUninitialized(Size.X * Size.Y);
				int32 PitchPixels = 0;
				const float* Src = static_cast<const float*>(Job.Readbacks[K]->Lock(PitchPixels));
				if (Src)
				{
					for (int32 Row = 0; Row < Size.Y; ++Row)
					{
						FMemory::Memcpy(Depth.GetData() + Row * Size.X, Src + Row * PitchPixels,
						                Size.X * sizeof(float));
					}
					Job.Readbacks[K]->Unlock();
				}
				else
				{
					for (float& D : Depth)
					{
						D = GpuFarCm * 10.0f;
					}
				}
				Result.Depth.Add(MoveTemp(Depth));
			}
			Ready.Enqueue(MoveTemp(Result));
			Pending.RemoveAt(J);
		}
		while (Pending.Num() > MaxGpuScansInFlight + 1)   // Never pile up readbacks
		{
			Pending.RemoveAt(0);
			InFlight.Decrement();
		}
	}
};

UUnrealLidarCam::UUnrealLidarCam()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;   // Scan from this frame's poses
}

TArray<FString> UUnrealLidarCam::GetMountNames() const
{
	TSet<FString> Names;
	if (const AActor* Owner = GetOwner())
	{
		TArray<USceneComponent*> Components;
		Owner->GetComponents<USceneComponent>(Components);
		for (const USceneComponent* C : Components)
		{
			if (C->IsA<USceneCaptureComponent2D>() || C->IsA<UCameraComponent>())
			{
				Names.Add(C->GetName());
			}
		}
	}
	// In the blueprint editor this component is a template: list the
	// construction script's cameras and captures (their names at spawn).
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
				    (Node->ComponentTemplate->IsA<USceneCaptureComponent2D>() ||
				     Node->ComponentTemplate->IsA<UCameraComponent>()))
				{
					Names.Add(Node->GetVariableName().ToString());
				}
			}
		}
	}
	TArray<FString> Out = Names.Array();
	Out.Sort();
	Out.Insert(TEXT("None"), 0);
	return Out;
}

USceneComponent* UUnrealLidarCam::ResolveMount() const
{
	if (FollowComponentName.IsNone() || !GetOwner())
	{
		return nullptr;
	}
	TArray<USceneComponent*> Components;
	GetOwner()->GetComponents<USceneComponent>(Components);
	const FString Wanted = NormalizeName(FollowComponentName.ToString());
	for (USceneComponent* C : Components)
	{
		if (C->GetFName() == FollowComponentName || NormalizeName(C->GetName()) == Wanted)
		{
			return C;
		}
	}
	return nullptr;
}

void UUnrealLidarCam::BuildBeams()
{
	Columns = FMath::Max(1, HorizontalSamples);
	Rows = FMath::Max(1, Channels);
	const bool bFullCircle = HorizontalFOV >= 359.99f;

	// Azimuth positive to the left (ROS), column 0 leftmost; elevation row 0 top.
	const float AzStep = bFullCircle ? 360.0f / Columns
	                                 : (Columns > 1 ? HorizontalFOV / (Columns - 1) : 0.0f);
	const float Az0 = bFullCircle ? 180.0f : (Columns > 1 ? HorizontalFOV * 0.5f : 0.0f);
	const float ElStep = Rows > 1 ? VerticalFOV / (Rows - 1) : 0.0f;
	const float El0 = Rows > 1 ? VerticalFOV * 0.5f : 0.0f;

	Beams.SetNumUninitialized(Rows * Columns);
	for (int32 R = 0; R < Rows; ++R)
	{
		const float El = FMath::DegreesToRadians(El0 - R * ElStep);
		for (int32 C = 0; C < Columns; ++C)
		{
			const float Az = FMath::DegreesToRadians(Az0 - C * AzStep);
			// Unreal axes: +X forward, +Y right, +Z up.
			Beams[R * Columns + C] = FVector(FMath::Cos(El) * FMath::Cos(Az),
			                                 -FMath::Cos(El) * FMath::Sin(Az),
			                                 FMath::Sin(El));
		}
	}
}

bool UUnrealLidarCam::SetUpGpu(USceneComponent* Pose)
{
	if (VerticalFOV > GpuMaxVerticalFov)
	{
		UE_LOG(LogTemp, Warning, TEXT("UnrealLidarCam: a %.0f deg vertical aperture is more than GPU mode "
		                              "covers (%.0f); using CPU mode."), VerticalFOV, GpuMaxVerticalFov);
		return false;
	}
	const bool bFullCircle = HorizontalFOV >= 359.99f;
	const float Span = bFullCircle ? 360.0f : HorizontalFOV;
	const int32 Sectors = FMath::Max(1, FMath::CeilToInt(Span / GpuMaxSectorFov - 1e-3f));
	const float SectorSpan = Span / Sectors;
	const float RenderFov = FMath::Min(SectorSpan + 2.0f, 170.0f);      // A margin for edge beams
	const float LeftEdge = bFullCircle ? 180.0f : Span * 0.5f;

	// Focal length from the finest beam spacing, oversampled, so neighbouring
	// beams never read the same pixel.
	float StepDeg = bFullCircle ? 360.0f / Columns : (Columns > 1 ? Span / (Columns - 1) : Span);
	if (Rows > 1)
	{
		StepDeg = FMath::Min(StepDeg, VerticalFOV / (Rows - 1));
	}
	float Focal = GpuOversampling / FMath::DegreesToRadians(FMath::Max(StepDeg, 0.01f));
	const float HalfW = FMath::Tan(FMath::DegreesToRadians(RenderFov * 0.5f));
	// At a sector's edge the vertical aperture spreads by 1 / cos(half width).
	const float HalfH = FMath::Tan(FMath::DegreesToRadians(VerticalFOV * 0.5f + 1.0f)) /
	                    FMath::Cos(FMath::DegreesToRadians(RenderFov * 0.5f));
	float Width = 2.0f * Focal * HalfW;
	float Height = 2.0f * Focal * HalfH;
	if (Width > GpuMaxTextureSize || Height > GpuMaxTextureSize)
	{
		const float Scale = FMath::Min(GpuMaxTextureSize / Width, GpuMaxTextureSize / Height);
		Focal *= Scale;
		Width *= Scale;
		Height *= Scale;
		UE_LOG(LogTemp, Warning, TEXT("UnrealLidarCam: depth maps capped at %d px; some beams will share pixels."),
		       GpuMaxTextureSize);
	}
	const int32 W = FMath::Max(16, FMath::CeilToInt(Width));
	const int32 H = FMath::Max(16, FMath::CeilToInt(Height));
	Focal = (W * 0.5f) / HalfW;                                          // Exactly as Unreal will render it

	SectorCaptures.Reset();
	SectorRotation.Reset();
	SectorSize.Reset();
	SectorFocal.Reset();
	for (int32 K = 0; K < Sectors; ++K)
	{
		const float CentreAz = LeftEdge - (K + 0.5f) * SectorSpan;      // ROS: positive left
		const FRotator Relative(0.0f, -CentreAz, 0.0f);                  // Unreal yaw: positive right

		USceneCaptureComponent2D* Capture = NewObject<USceneCaptureComponent2D>(
			GetOwner(), MakeUniqueObjectName(GetOwner(), USceneCaptureComponent2D::StaticClass(),
			                                 TEXT("UnrealLidarCamDepth")));
		Capture->RegisterComponent();
		Capture->AttachToComponent(Pose, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
		Capture->SetRelativeRotation(Relative);
		Capture->ProjectionType = ECameraProjectionMode::Perspective;
		Capture->FOVAngle = RenderFov;
		Capture->CaptureSource = SCS_SceneDepth;
		Capture->bCaptureEveryFrame = false;                             // Rendered only when a scan is due
		Capture->bCaptureOnMovement = false;
		if (bIgnoreOwner && GetOwner())
		{
			Capture->HiddenActors.Add(GetOwner());
		}
		// Depth only: skip what does not change it.
		Capture->ShowFlags.SetAtmosphere(false);
		Capture->ShowFlags.SetFog(false);
		Capture->ShowFlags.SetDynamicShadows(false);
		Capture->ShowFlags.SetMotionBlur(false);

		UTextureRenderTarget2D* Target = NewObject<UTextureRenderTarget2D>(Capture);
		Target->RenderTargetFormat = RTF_R32f;
		Target->ClearColor = FLinearColor(GpuFarCm * 10.0f, 0.0f, 0.0f, 0.0f);
		Target->InitAutoFormat(W, H);
		Target->UpdateResourceImmediate(true);
		Capture->TextureTarget = Target;

		SectorCaptures.Add(Capture);
		SectorRotation.Add(Relative.Quaternion());
		SectorSize.Add(FIntPoint(W, H));
		SectorFocal.Add(Focal);
	}

	// Where each beam lands: its sector, and its pixel in that sector's image.
	const int32 Count = Beams.Num();
	BeamSector.SetNumUninitialized(Count);
	BeamPixel.SetNumUninitialized(Count);
	BeamInSector.SetNumUninitialized(Count);
	for (int32 I = 0; I < Count; ++I)
	{
		const FVector& D = Beams[I];
		const float Az = static_cast<float>(FMath::RadiansToDegrees(FMath::Atan2(-D.Y, D.X)));   // ROS: positive left
		const int32 K = FMath::Clamp(FMath::FloorToInt((LeftEdge - Az) / SectorSpan), 0, Sectors - 1);
		const FVector Dc = SectorRotation[K].UnrotateVector(D);              // Sector camera frame
		BeamSector[I] = K;
		BeamInSector[I] = FVector3f(Dc);
		BeamPixel[I] = FVector2f(static_cast<float>(W * 0.5f + Focal * Dc.Y / Dc.X),
		                         static_cast<float>(H * 0.5f - Focal * Dc.Z / Dc.X));
	}
	Gpu = MakeShared<FUnrealLidarCamGpuState, ESPMode::ThreadSafe>();
	UE_LOG(LogTemp, Log, TEXT("UnrealLidarCam: GPU mode, %d depth capture(s) of %.1f deg, %d x %d px."),
	       Sectors, RenderFov, W, H);
	return true;
}

void UUnrealLidarCam::BeginPlay()
{
	Super::BeginPlay();

	BuildBeams();
	Mount = ResolveMount();
	if (!FollowComponentName.IsNone() && !Mount.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("UnrealLidarCam: no component '%s' on '%s'; scanning from the lidar's own pose."),
		       *FollowComponentName.ToString(), *GetNameSafe(GetOwner()));
	}
	USceneComponent* Pose = Mount.IsValid() ? Mount.Get() : this;
	ActiveMode = Mode;
	if (ActiveMode == EUnrealLidarCamMode::GPU && !SetUpGpu(Pose))
	{
		ActiveMode = EUnrealLidarCamMode::CPU;
	}
	UE_LOG(LogTemp, Log,
	       TEXT("UnrealLidarCam: %s, %d x %d beams over %.1f x %.1f deg, %.2f-%.1f m, %.1f Hz, from '%s' -> %s:%d."),
	       ActiveMode == EUnrealLidarCamMode::GPU ? TEXT("GPU") : TEXT("CPU"), Columns, Rows,
	       HorizontalFOV, VerticalFOV, MinRange, MaxRange, ScanRateHz, *Pose->GetName(), *ServerIp, ServerPort);

	Outbox = MakeShared<FUnrealLidarCamOutbox, ESPMode::ThreadSafe>();
	NextScanTime = 0.0;
	bStopSender = false;
	SenderThread = FRunnableThread::Create(this, TEXT("UnrealLidarCamSender"));
}

void UUnrealLidarCam::EndPlay(const EEndPlayReason::Type Reason)
{
	bStopSender = true;
	if (SenderThread)
	{
		SenderThread->Kill(true);           // Bounded: the sender waits at most ~2 s per step
		delete SenderThread;
		SenderThread = nullptr;
	}
	bConnected = false;
	Outbox.Reset();
	Gpu.Reset();                            // In-flight render commands keep their own reference
	for (USceneCaptureComponent2D* Capture : SectorCaptures)
	{
		if (IsValid(Capture))
		{
			Capture->DestroyComponent();
		}
	}
	SectorCaptures.Reset();
	Super::EndPlay(Reason);
}

void UUnrealLidarCam::TickComponent(float DeltaTime, ELevelTick TickType,
                                    FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UWorld* World = GetWorld();
	if (!World || !Outbox.IsValid())
	{
		return;
	}
	if (ActiveMode == EUnrealLidarCamMode::GPU)
	{
		PumpGpu();                          // Scans whose depth maps have arrived
	}
	if (!bConnected && !bDrawPoints)
	{
		return;                             // Nobody listening: do not spend the work
	}
	const double Now = World->GetTimeSeconds();
	if (Now < NextScanTime)
	{
		return;
	}
	const double Period = 1.0 / FMath::Max(ScanRateHz, 0.1f);
	NextScanTime = (Now - NextScanTime > Period) ? Now + Period : NextScanTime + Period;
	if (ActiveMode == EUnrealLidarCamMode::GPU)
	{
		ScanGpu(Now);
	}
	else
	{
		ScanCpu(Now);
	}
}

void UUnrealLidarCam::ScanCpu(double Stamp)
{
	UWorld* World = GetWorld();
	const USceneComponent* Pose = Mount.IsValid() ? Mount.Get() : this;
	const FTransform Sensor = Pose->GetComponentTransform();
	const FVector Origin = Sensor.GetLocation();
	const FQuat Rotation = Sensor.GetRotation();

	FCollisionQueryParams Params(TEXT("UnrealLidarCam"), bTraceComplex);
	Params.bReturnPhysicalMaterial = true;
	if (bIgnoreOwner && GetOwner())
	{
		Params.AddIgnoredActor(GetOwner());
	}

	const int32 Count = Beams.Num();
	const float MinCm = MinRange * 100.0f;
	const float MaxCm = FMath::Max(MaxRange * 100.0f, MinCm + 1.0f);
	const float Noise = RangeNoiseStdDev;
	const ECollisionChannel Channel = TraceChannel;
	const float NaN = std::numeric_limits<float>::quiet_NaN();
	const int32 Seed = static_cast<int32>(GFrameCounter & 0x7fffffff);
	TArray<float> Range, Intensity;
	Range.SetNumUninitialized(Count);
	Intensity.SetNumUninitialized(Count);

	ForChunks(Count, [&](int32 Begin, int32 End, int32 Chunk)
	{
		FRandomStream Random(Seed + Chunk * 7919);
		for (int32 I = Begin; I < End; ++I)
		{
			const FVector Dir = Rotation.RotateVector(Beams[I]);
			FHitResult Hit;
			if (World->LineTraceSingleByChannel(Hit, Origin + Dir * MinCm, Origin + Dir * MaxCm, Channel, Params))
			{
				float RangeM = (Hit.Distance + MinCm) / 100.0f;
				if (Noise > 0.0f)
				{
					RangeM += GaussianNoise(Random, Noise);
				}
				Range[I] = RangeM;
				const float Incidence = static_cast<float>(FMath::Abs(FVector::DotProduct(Hit.ImpactNormal, -Dir)));
				Intensity[I] = Reflectivity(Hit.PhysMaterial.Get()) * Incidence;
			}
			else
			{
				Range[I] = NaN;
				Intensity[I] = 0.0f;
			}
		}
	});
	Publish(Stamp, GFrameCounter, Range, Intensity);
}

void UUnrealLidarCam::ScanGpu(double Stamp)
{
	if (!Gpu.IsValid())
	{
		return;
	}
	if (Gpu->InFlight.GetValue() >= MaxGpuScansInFlight)
	{
		return;                             // The GPU is behind: skip this scan
	}
	TArray<FTextureRenderTargetResource*> Targets;
	for (USceneCaptureComponent2D* Capture : SectorCaptures)
	{
		if (!IsValid(Capture) || !Capture->TextureTarget)
		{
			return;
		}
		Capture->CaptureScene();            // Queued on the render thread before the copy below
		Targets.Add(Capture->TextureTarget->GameThread_GetRenderTargetResource());
	}
	TSharedPtr<FUnrealLidarCamGpuState, ESPMode::ThreadSafe> S = Gpu;
	TArray<FIntPoint> Sizes = SectorSize;
	const uint64 FrameId = GFrameCounter;
	S->InFlight.Increment();
	ENQUEUE_RENDER_COMMAND(UnrealLidarCamReadback)(
		[S, Targets, Sizes, Stamp, FrameId](FRHICommandListImmediate& RHICmdList)
		{
			S->Drain();
			FGpuScanJob Job;
			Job.Stamp = Stamp;
			Job.FrameId = FrameId;
			for (int32 K = 0; K < Targets.Num(); ++K)
			{
				TUniquePtr<FRHIGPUTextureReadback> Readback =
					MakeUnique<FRHIGPUTextureReadback>(TEXT("UnrealLidarCamReadback"));
				Readback->EnqueueCopy(RHICmdList, Targets[K]->GetRenderTargetTexture());
				Job.Readbacks.Add(MoveTemp(Readback));
				Job.Sizes.Add(Sizes[K]);
			}
			S->Pending.Add(MoveTemp(Job));
		});
}

void UUnrealLidarCam::PumpGpu()
{
	if (!Gpu.IsValid())
	{
		return;
	}
	if (Gpu->InFlight.GetValue() > 0)
	{
		// Poll on the render thread; finished scans come back through Ready.
		TSharedPtr<FUnrealLidarCamGpuState, ESPMode::ThreadSafe> S = Gpu;
		ENQUEUE_RENDER_COMMAND(UnrealLidarCamDrain)([S](FRHICommandListImmediate&) { S->Drain(); });
	}

	FGpuScanResult Result;
	while (Gpu->Ready.Dequeue(Result))
	{
		Gpu->InFlight.Decrement();
		const int32 Count = Beams.Num();
		const float MinCm = MinRange * 100.0f;
		const float MaxCm = FMath::Max(MaxRange * 100.0f, MinCm + 1.0f);
		const float Noise = RangeNoiseStdDev;
		const float NaN = std::numeric_limits<float>::quiet_NaN();
		const int32 Seed = static_cast<int32>(Result.FrameId & 0x7fffffff);
		TArray<float> Range, Intensity;
		Range.SetNumUninitialized(Count);
		Intensity.SetNumUninitialized(Count);

		ForChunks(Count, [&](int32 Begin, int32 End, int32 Chunk)
		{
			FRandomStream Random(Seed + Chunk * 7919);
			for (int32 I = Begin; I < End; ++I)
			{
				const int32 K = BeamSector[I];
				const FIntPoint Size = SectorSize[K];
				const float F = SectorFocal[K];
				const TArray<float>& Depth = Result.Depth[K];
				const FVector2f Px = BeamPixel[I];
				const int32 U = FMath::Clamp(FMath::FloorToInt(Px.X), 0, Size.X - 1);
				const int32 V = FMath::Clamp(FMath::FloorToInt(Px.Y), 0, Size.Y - 1);
				const float Z = Depth[V * Size.X + U];            // Planar depth along the sector's axis, cm
				const FVector3f Dc = BeamInSector[I];
				const float RangeCm = Z / Dc.X;
				if (!(Z < GpuFarCm) || RangeCm < MinCm || RangeCm > MaxCm)
				{
					Range[I] = NaN;
					Intensity[I] = 0.0f;
					continue;
				}
				Range[I] = RangeCm / 100.0f + (Noise > 0.0f ? GaussianNoise(Random, Noise) : 0.0f);

				// Surface normal from the neighbouring depth pixels -> incidence.
				auto Back = [&](int32 Uq, int32 Vq, float Zq)
				{
					return FVector3f(Zq, Zq * ((Uq + 0.5f) - Size.X * 0.5f) / F,
					                 -Zq * ((Vq + 0.5f) - Size.Y * 0.5f) / F);
				};
				const int32 U2 = U + 1 < Size.X ? U + 1 : U - 1;
				const int32 V2 = V + 1 < Size.Y ? V + 1 : V - 1;
				const float Zu = Depth[V * Size.X + U2];
				const float Zv = Depth[V2 * Size.X + U];
				float Incidence = 1.0f;
				if (U2 >= 0 && V2 >= 0 && Zu < GpuFarCm && Zv < GpuFarCm)
				{
					const FVector3f P = Back(U, V, Z);
					const FVector3f N = FVector3f::CrossProduct(Back(U2, V, Zu) - P, Back(U, V2, Zv) - P).GetSafeNormal();
					if (!N.IsZero())
					{
						Incidence = FMath::Abs(FVector3f::DotProduct(N, Dc));
					}
				}
				Intensity[I] = DefaultReflectivity * Incidence;
			}
		});
		Publish(Result.Stamp, Result.FrameId, Range, Intensity);
	}
}

void UUnrealLidarCam::Publish(double Stamp, uint64 FrameId, const TArray<float>& Range,
                              const TArray<float>& Intensity)
{
	const int32 Count = Beams.Num();
	TArray<uint8> Packet;
	Packet.SetNumUninitialized(sizeof(FLidarHeader) + Count * sizeof(FLidarPoint));
	FLidarHeader* Header = reinterpret_cast<FLidarHeader*>(Packet.GetData());
	FLidarPoint* Points = reinterpret_cast<FLidarPoint*>(Packet.GetData() + sizeof(FLidarHeader));

	Header->Magic[0] = 'L'; Header->Magic[1] = 'I'; Header->Magic[2] = 'D'; Header->Magic[3] = 'R';
	Header->Version = 1;
	Header->SensorId = static_cast<uint8>(SensorId);
	Header->Flags = 1;                      // Organized
	Header->FrameId = FrameId;
	Header->Stamp = Stamp;
	Header->HorizontalFov = HorizontalFOV;
	Header->VerticalFov = VerticalFOV;
	Header->Columns = Columns;
	Header->Rows = Rows;
	Header->PointCount = Count;
	Header->PointStep = sizeof(FLidarPoint);

	const float NaN = std::numeric_limits<float>::quiet_NaN();
	int32 Returns = 0;
	for (int32 I = 0; I < Count; ++I)
	{
		FLidarPoint& P = Points[I];
		P.Ring = static_cast<uint16>(I / Columns);
		P.Column = static_cast<uint16>(I % Columns);
		P.Intensity = Intensity[I];
		if (FMath::IsNaN(Range[I]))
		{
			P.X = P.Y = P.Z = NaN;
			continue;
		}
		const FVector Local = Beams[I] * Range[I];            // Sensor frame, Unreal axes
		P.X = static_cast<float>(Local.X);
		P.Y = static_cast<float>(-Local.Y);                  // ROS: y left
		P.Z = static_cast<float>(Local.Z);
		++Returns;
	}
	LastReturnCount = Returns;

	if (bDrawPoints)
	{
		const USceneComponent* Pose = Mount.IsValid() ? Mount.Get() : this;
		const FTransform Sensor = Pose->GetComponentTransform();
		const int32 Stride = FMath::Max(1, Count / 20000);  // Keep the overlay affordable
		for (int32 I = 0; I < Count; I += Stride)
		{
			const FLidarPoint& P = Points[I];
			if (!FMath::IsNaN(P.X))
			{
				const FVector World3 = Sensor.TransformPositionNoScale(FVector(P.X, -P.Y, P.Z) * 100.0f);
				DrawDebugPoint(GetWorld(), World3, 3.0f, FColor::Green, false, 1.0f / FMath::Max(ScanRateHz, 0.1f));
			}
		}
	}

	if (!bConnected)
	{
		return;
	}
	if (Outbox->Queued.GetValue() >= MaxQueuedScans)
	{
		if (!bLoggedDrop)
		{
			UE_LOG(LogTemp, Warning, TEXT("UnrealLidarCam: the link is slower than the scans; dropping scans."));
			bLoggedDrop = true;
		}
		return;
	}
	Outbox->Packets.Enqueue(MoveTemp(Packet));
	Outbox->Queued.Increment();
}

bool UUnrealLidarCam::TryConnect()
{
	// Sender thread only: a non-blocking connect with a bounded wait, so neither
	// this thread nor EndPlay's join can stall for long (as in UnrealStereoCam).
	ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	FIPv4Address Ip;
	if (!FIPv4Address::Parse(ServerIp, Ip))
	{
		UE_LOG(LogTemp, Error, TEXT("UnrealLidarCam: bad ServerIp '%s'."), *ServerIp);
		return false;
	}
	TSharedRef<FInternetAddr> Addr = Subsystem->CreateInternetAddr();
	Addr->SetIp(Ip.Value);
	Addr->SetPort(ServerPort);

	FSocket* NewSocket = Subsystem->CreateSocket(NAME_Stream, TEXT("UnrealLidarCam"), false);
	NewSocket->SetNonBlocking(true);
	NewSocket->SetNoDelay(true);
	int32 Unused;
	NewSocket->SetSendBufferSize(4 * 1024 * 1024, Unused);
	NewSocket->Connect(*Addr);
	const bool bOk = NewSocket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromSeconds(2.0)) &&
	                 NewSocket->GetConnectionState() == SCS_Connected;
	if (!bOk)
	{
		if (!bLoggedWaiting)
		{
			UE_LOG(LogTemp, Warning, TEXT("UnrealLidarCam: cannot reach %s:%d yet; retrying every 3 s."),
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
	UE_LOG(LogTemp, Log, TEXT("UnrealLidarCam: connected to %s:%d."), *ServerIp, ServerPort);
	return true;
}

uint32 UUnrealLidarCam::Run()
{
	TSharedPtr<FUnrealLidarCamOutbox, ESPMode::ThreadSafe> Box = Outbox;
	TArray<uint8> Packet;
	auto CloseSocket = [this]()
	{
		if (Socket)
		{
			Socket->Close();
			ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
			Socket = nullptr;
		}
	};

	while (!bStopSender)
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
		if (!Box.IsValid() || !Box->Packets.Dequeue(Packet))
		{
			FPlatformProcess::Sleep(0.005f);
			continue;
		}
		Box->Queued.Decrement();

		int32 Offset = 0;
		bool bFailed = false;
		while (Offset < Packet.Num() && !bStopSender)
		{
			int32 Sent = 0;
			if (!Socket->Send(Packet.GetData() + Offset, Packet.Num() - Offset, Sent))
			{
				// Non-blocking socket: a full buffer is "would block", not a broken link.
				const ESocketErrors Error = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->GetLastErrorCode();
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
			CloseSocket();
			bConnected = false;
			while (Box->Packets.Dequeue(Packet))
			{
				Box->Queued.Decrement();
			}
			UE_LOG(LogTemp, Warning, TEXT("UnrealLidarCam: link lost; redialling every 3 s."));
		}
	}
	CloseSocket();
	return 0;
}

void UUnrealLidarCam::Stop()
{
	bStopSender = true;
}
