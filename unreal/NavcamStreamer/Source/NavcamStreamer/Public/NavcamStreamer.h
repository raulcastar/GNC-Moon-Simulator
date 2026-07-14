// NavcamStreamer: pushes navcam frames to ROS without stalling the game.
//
// UnrealCV renders on demand: every vget blocks the game thread for a full
// render + GPU readback (~300 ms per camera, measured). This component
// replaces that path for RGB. Each SceneCapture renders as part of the normal
// frame on the render thread, the readback is asynchronous (FRHIGPUTextureReadback,
// polled until ready, never flushed), and completed frames go out over TCP
// from a worker thread. The game thread only enqueues copies and memcpys.
//
// All captures enqueued in one tick share a frame id (GFrameCounter), which is
// what lets the ROS side pair left/right images from the same world state —
// the same guarantee the batched UnrealCV request used to provide.
//
// Wire format, little-endian, one packet per frame:
//   offset size  field
//   0      4     magic 'NAVF'
//   4      1     version = 1
//   5      1     cam_id            (matches ids in component_stereo_depth params)
//   6      1     format = 1        (BGRA8, payload = width*height*4)
//   7      1     reserved
//   8      2     width  (u16)
//   10     2     height (u16)
//   12     8     frame_id (u64)    (GFrameCounter at capture)
//   20     4     payload size (u32)
//   24     ...   pixels

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/EngineTypes.h"
#include "Engine/TextureRenderTarget2D.h"
#include "RHIGPUReadback.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Containers/Queue.h"
#include "NavcamStreamer.generated.h"

class FSocket;

// A capture resolved and validated at BeginPlay, ready to stream.
USTRUCT()
struct NAVCAMSTREAMER_API FNavcamResolved
{
	GENERATED_BODY()

	UPROPERTY(Transient)
	USceneCaptureComponent2D* Capture = nullptr;

	int32 CamId = 0;
};

// One in-flight asynchronous readback.
struct FPendingReadback
{
	TUniquePtr<FRHIGPUTextureReadback> Readback;
	int32 CamId = 0;
	int32 Width = 0;
	int32 Height = 0;
	uint64 FrameId = 0;
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class NAVCAMSTREAMER_API UNavcamStreamer : public UActorComponent, public FRunnable
{
	GENERATED_BODY()

public:
	UNavcamStreamer();

	// Where the ROS stereo node listens (see stream_port in its params.yaml).
	UPROPERTY(EditAnywhere, Category = "Navcam|Network")
	FString ServerIp = TEXT("127.0.0.1");

	UPROPERTY(EditAnywhere, Category = "Navcam|Network")
	int32 ServerPort = 3004;

	// Frames per second pushed to ROS. Captures render every frame regardless;
	// this only limits readback + network. 5 Hz is ample for a 1 Hz pipeline.
	UPROPERTY(EditAnywhere, Category = "Navcam", meta = (ClampMin = "0.5", ClampMax = "30"))
	float SendHz = 5.0f;

	// The three navcam SceneCapture2Ds, selected by name from a dropdown
	// listing every SceneCapture2D in reach (this blueprint's components tree,
	// or the world when placed in a level). Resolved at BeginPlay with a log
	// line per camera saying what was found and where.
	//
	// Direct object pointers cannot work here: the details panel cannot point
	// a plain component property at a per-instance component, which is why a
	// pointer field only ever offered loaded templates (UnrealCV's internal
	// "lit"/"normal" sensors).
	UPROPERTY(EditAnywhere, Category = "Navcam|Cameras",
	          meta = (GetOptions = "GetSceneCaptureNames"))
	FName LeftCaptureName = TEXT("NavcamLeft");

	UPROPERTY(EditAnywhere, Category = "Navcam|Cameras",
	          meta = (GetOptions = "GetSceneCaptureNames"))
	FName RightCaptureName = TEXT("NavcamRight");

	UPROPERTY(EditAnywhere, Category = "Navcam|Cameras",
	          meta = (GetOptions = "GetSceneCaptureNames"))
	FName CenterCaptureName = TEXT("NavcamCenter");

	// Feeds the dropdowns above. Lists the blueprint's own SceneCapture2D
	// components when editing the blueprint, plus every instance in the world
	// when placed in a level.
	UFUNCTION()
	TArray<FString> GetSceneCaptureNames() const;

	// Captures with no TextureRenderTarget2D assigned get one created at this
	// size automatically (RGBA8). Match the UnrealCV camera resolution: the
	// ROS node solved the rig at that size and refuses mismatched frames.
	UPROPERTY(EditAnywhere, Category = "Navcam|Cameras")
	int32 CaptureWidth = 640;

	UPROPERTY(EditAnywhere, Category = "Navcam|Cameras")
	int32 CaptureHeight = 480;

	// Ids the ROS side knows the cameras by (left/right/center_camera_id in
	// component_stereo_depth/config/params.yaml). Defaults match its defaults.
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Navcam|Cameras")
	int32 LeftCamId = 1;

	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Navcam|Cameras")
	int32 RightCamId = 2;

	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "Navcam|Cameras")
	int32 CenterCamId = 0;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	// FRunnable — the sender thread drains the packet queue into the socket.
	virtual uint32 Run() override;
	virtual void Stop() override;

private:
	void ResolveCapture(const FName& Name, int32 CamId);
	void EnqueueCaptures();
	void PollReadbacks();
	bool ConnectSocket();

	// Captures found by name at BeginPlay.
	UPROPERTY(Transient)
	TArray<FNavcamResolved> Active;

	FSocket* Socket = nullptr;
	FRunnableThread* SenderThread = nullptr;
	FThreadSafeBool bStopSender = false;
	// Set by the sender thread on a failed send; the game thread then closes
	// the socket and the tick loop redials.
	FThreadSafeBool bSocketDead = false;
	double LastConnectAttempt = 0.0;
	bool bLoggedWaiting = false;

	// Packets ready to send: header already prepended.
	TQueue<TArray<uint8>, EQueueMode::Mpsc> SendQueue;

	TArray<FPendingReadback> Pending;
	double LastSendTime = 0.0;
};
