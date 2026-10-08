// UnrealStereoCam: pushes stereo/navcam frames to ROS without stalling the game.
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
//   6      1     format            1 = BGRA8 colour, 2 = R32F depth: centimetres along
//                                  the camera axis (Unreal scene depth); payload =
//                                  width*height*4 either way
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
#include "Engine/Scene.h"
#include "Engine/TextureRenderTarget2D.h"
#include "RHIGPUReadback.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Containers/Queue.h"
#include "UnrealStereoCam.generated.h"

class FSocket;

// A capture resolved and validated at BeginPlay, ready to stream.
USTRUCT()
struct UNREALSTEREOCAM_API FStereoCamResolved
{
	GENERATED_BODY()

	UPROPERTY(Transient)
	USceneCaptureComponent2D* Capture = nullptr;

	int32 CamId = 0;
	uint8 Format = 1;               // Wire format: 1 BGRA8, 2 R32F depth
};

// One in-flight asynchronous readback. Touched only on the render thread.
struct FPendingReadback
{
	TUniquePtr<FRHIGPUTextureReadback> Readback;
	int32 CamId = 0;
	uint8 Format = 1;
	int32 Width = 0;
	int32 Height = 0;
	uint64 FrameId = 0;
};

// Shared between three threads through a ref-counted pointer, so it outlives
// whichever drops its reference last and teardown never has to block:
//   - Pending: created, polled, locked and freed ONLY on the render thread.
//     Every earlier crash/freeze came from touching readbacks off the render
//     thread; here the game thread never does.
//   - Ready: lock-free MPSC handoff. The render thread pushes finished frames,
//     the sender thread pops them. No lock, no wait.
// The component, each in-flight render command, and the sender thread each
// hold a shared reference. The last one to finish frees this — and with it
// the FRHIGPUTextureReadback objects — on whatever thread that happens to be,
// which for in-flight render commands is the render thread. Correct by
// construction, with no FlushRenderingCommands / fence / join to stall on.
struct FStereoCamStreamState
{
	TArray<FPendingReadback> Pending;                      // render thread only
	TQueue<TArray<uint8>, EQueueMode::Mpsc> Ready;         // render -> sender
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class UNREALSTEREOCAM_API UUnrealStereoCam : public UActorComponent, public FRunnable
{
	GENERATED_BODY()

public:
	UUnrealStereoCam();

	// Where the ROS stereo node listens (see stream_port in its params.yaml).
	UPROPERTY(EditAnywhere, Category = "StereoCam|Network")
	FString ServerIp = TEXT("127.0.0.1");

	UPROPERTY(EditAnywhere, Category = "StereoCam|Network")
	int32 ServerPort = 3004;

	// Frames per second pushed to ROS. The stereo node's default
	// image_reading_hz_rate is 1 Hz; every frame beyond what it actually
	// consumes is still fully parsed and BGRA->RGB converted by the receiver
	// for nothing — at 640x480x4 bytes x3 cameras, each extra Hz here is
	// ~3.7 MB/s of wasted work. Keep this close to (a small margin above)
	// the consumer's real rate, not high "for safety".
	UPROPERTY(EditAnywhere, Category = "StereoCam", meta = (ClampMin = "0.5", ClampMax = "30"))
	float SendHz = 2.0f;

	// The three navcam SceneCapture2Ds, selected by name from a dropdown
	// listing every SceneCapture2D in reach (this blueprint's components tree,
	// or the world when placed in a level). Resolved at BeginPlay with a log
	// line per camera saying what was found and where.
	//
	// Direct object pointers cannot work here: the details panel cannot point
	// a plain component property at a per-instance component, which is why a
	// pointer field only ever offered loaded templates (UnrealCV's internal
	// "lit"/"normal" sensors).
	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras",
	          meta = (GetOptions = "GetSceneCaptureNames"))
	FName LeftCaptureName = TEXT("NavcamLeft");

	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras",
	          meta = (GetOptions = "GetSceneCaptureNames"))
	FName RightCaptureName = TEXT("NavcamRight");

	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras",
	          meta = (GetOptions = "GetSceneCaptureNames"))
	FName CenterCaptureName = TEXT("NavcamCenter");

	// Optional object-segmentation capture. Point this at a SceneCapture whose
	// post-process material colours each object from the Custom Depth-Stencil
	// buffer (the UnrealCV object_mask, reproduced without UnrealCV). Leave
	// blank to stream no mask. It rides the same path as the RGB cameras: same
	// frame id, so the ROS side overlays it on the central image. Nothing about
	// the segmentation lives in this plugin — the colours come entirely from the
	// material on the capture; here it is just another SceneCapture to read back.
	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras",
	          meta = (GetOptions = "GetSceneCaptureNames"))
	FName MaskCaptureName;

	// Also stream the ground-truth depth of the central camera: a depth-only
	// capture created at BeginPlay, attached to the centre capture with its
	// FOV and resolution, so it sees exactly the same view. Each pixel is
	// Unreal's scene depth (centimetres along the camera axis), sent as float
	// (format 2) on DepthCamId. Nothing to add to the blueprint.
	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras")
	bool bStreamCenterDepth = true;

	// Feeds the dropdowns above. Lists the blueprint's own SceneCapture2D
	// components when editing the blueprint, plus every instance in the world
	// when placed in a level.
	UFUNCTION()
	TArray<FString> GetSceneCaptureNames() const;

	// Captures with no TextureRenderTarget2D assigned get one created at this
	// size automatically (RGBA8). Match the UnrealCV camera resolution: the
	// ROS node solved the rig at that size and refuses mismatched frames.
	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras")
	int32 CaptureWidth = 640;

	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras")
	int32 CaptureHeight = 480;

	// Lock the captures to manual exposure instead of auto-exposing. Useful
	// for the glare study (auto-exposure would hide the saturation being
	// measured, and could let the two eyes expose differently from each
	// other, poisoning stereo matching) — but NOT the fix for a white image
	// from sporadic captures; that turned out to be a capture-timing bug, not
	// exposure. Off by default: verify the base image is correct with this
	// off before turning it on.
	//
	// Manual mode (AEM_Manual) disables scene metering, so there is no
	// scene-dependent feedback loop to fight. ExposureCompensationEV is a
	// plain brightness dial: 0 = neutral, positive = brighter, negative =
	// darker.
	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras")
	bool bLockExposure = false;

	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras",
	          meta = (EditCondition = "bLockExposure"))
	float ExposureCompensationEV = 0.0f;

	// Constrain the AUTO-exposure range. Ignored when bLockExposure is on (that
	// switches to manual metering, which has no adapting range). Unreal's auto-
	// exposure adapts brightness between a min and max EV100; clamping the range
	// stops the dim lunar surface from being driven to white (max too high) or
	// crushed to black (min too low). Setting MinEV100 == MaxEV100 pins a fixed
	// exposure — the same on both stereo eyes, so their brightness matches
	// without disabling metering entirely. These map onto the capture's
	// PostProcessSettings AutoExposure Min/Max Brightness, which carry EV100
	// values under UE5's extended luminance range (the editor labels them
	// "Min EV100" / "Max EV100").
	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras")
	bool bLimitAutoExposure = false;

	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras",
	          meta = (EditCondition = "bLimitAutoExposure", DisplayName = "Min EV100"))
	float MinEV100 = -2.0f;

	UPROPERTY(EditAnywhere, Category = "StereoCam|Cameras",
	          meta = (EditCondition = "bLimitAutoExposure", DisplayName = "Max EV100"))
	float MaxEV100 = 4.0f;

	// Ids the ROS side knows the cameras by (left/right/center_camera_id in
	// component_stereo_depth/config/params.yaml). Defaults match its defaults.
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "StereoCam|Cameras")
	int32 LeftCamId = 1;

	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "StereoCam|Cameras")
	int32 RightCamId = 2;

	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "StereoCam|Cameras")
	int32 CenterCamId = 0;

	// Must match mask_camera_id in component_stereo_depth/config/params.yaml.
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "StereoCam|Cameras")
	int32 MaskCamId = 3;

	// Must match depth_ground_truth_camera_id in that file.
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "StereoCam|Cameras")
	int32 DepthCamId = 4;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	// FRunnable — the sender thread drains the packet queue into the socket.
	virtual uint32 Run() override;
	virtual void Stop() override;

private:
	void ResolveCapture(const FName& Name, int32 CamId);

	// Creates the centre camera's depth capture (bStreamCenterDepth).
	void AddCenterDepthCapture();

	// Enqueues ONE render command per send tick that does everything on the
	// render thread: drain the readbacks started last tick (now finished on
	// the GPU) into Ready, then start this tick's readbacks. The game thread
	// only gathers render-target pointers and posts the command — it never
	// locks, waits, or frees a readback.
	void PumpCaptures();

	// Socket ownership lives ENTIRELY on the background thread (Run()): create,
	// connect, send, close. FSocket::Connect() is synchronous and can block for
	// the OS TCP timeout; calling it from TickComponent — as an earlier version
	// of this file did — freezes the whole engine (render, input, everything)
	// for that duration, every retry. TryConnect uses a non-blocking socket
	// with a short bounded Wait() so even this thread never stalls for long,
	// keeping EndPlay's Kill(true) join bounded too.
	bool TryConnect();

	// Captures found by name at BeginPlay.
	UPROPERTY(Transient)
	TArray<FStereoCamResolved> Active;

	FSocket* Socket = nullptr;                  // Run()'s thread only.
	FRunnableThread* SenderThread = nullptr;
	FThreadSafeBool bStopSender = false;
	// The only socket state the game thread may read. Set/cleared by Run().
	FThreadSafeBool bConnected = false;

	double LastConnectAttempt = 0.0;             // Run()'s thread only.
	bool bLoggedWaiting = false;                 // Run()'s thread only.

	// Readback + ready-frame state, shared with render commands and the sender
	// thread. Created in BeginPlay, released (not blocked-on) in EndPlay.
	TSharedPtr<FStereoCamStreamState, ESPMode::ThreadSafe> Stream;

	double LastSendTime = 0.0;
};
