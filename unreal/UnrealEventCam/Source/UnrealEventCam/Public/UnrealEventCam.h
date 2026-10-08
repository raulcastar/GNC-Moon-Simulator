// UnrealEventCam: a simulated event (DVS) camera for Unreal.
//
// A SceneCapture renders an intensity image with each game frame; its readback
// is asynchronous (never stalls the game thread, same design as the sibling
// UnrealStereoCam plugin). A worker thread converts each frame to grayscale,
// runs the ESIM event-generation model (see EventSimulator.h — ported from
// rpg_esim, MIT (c) 2018 Robotics and Perception Group), and streams the
// resulting events to a ROS node over TCP.
//
// Accuracy note: ESIM assumes the intensity signal is sampled far faster than
// the scene changes. Here the sampling rate is the render frame rate, so fast
// motion between frames is linearly interpolated and can alias. Run the editor
// at a high, steady FPS (and keep contrast thresholds from being tiny) for the
// cleanest event stream.
//
// Wire format, little-endian, one packet per frame that produced events:
//   offset size  field
//   0      4     magic 'EVT0'
//   4      1     version = 1
//   5      1     cam_id
//   6      2     width  (u16)
//   8      2     height (u16)
//   10     8     t0_ns  (u64)   base timestamp; each event t = t0 + t_rel
//   18     4     count  (u32)   number of events in this packet
//   22     2     reserved
//   24     ...   count * 9 bytes, each: x u16 | y u16 | t_rel_ns u32 | pol u8
//                pol: 1 = brightness increase, 0 = decrease

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "RHIGPUReadback.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/ThreadSafeBool.h"
#include "Containers/Queue.h"
#include "EventSimulator.h"
#include "UnrealEventCam.generated.h"

class FSocket;

// One in-flight asynchronous readback. Touched only on the render thread.
struct FEventPendingReadback
{
	TUniquePtr<FRHIGPUTextureReadback> Readback;
	int32 Width = 0;
	int32 Height = 0;
	int64 TimeNs = 0;      // world time when this frame was rendered
};

// One intensity frame handed from the render thread to the worker: raw BGRA as
// read back, plus its size and timestamp. The worker converts and runs ESIM.
struct FEventReadyFrame
{
	TArray<uint8> Bgra;   // Width*Height*4
	int32 Width = 0;
	int32 Height = 0;
	int64 TimeNs = 0;
};

// Shared between the render commands and the worker through a ref-counted
// pointer, so teardown never has to block (same pattern as UnrealStereoCam):
//   Pending : created, polled, locked and freed ONLY on the render thread.
//   Ready   : lock-free MPSC handoff, render thread -> worker thread.
struct FEventStreamState
{
	TArray<FEventPendingReadback> Pending;                 // render thread only
	TQueue<FEventReadyFrame, EQueueMode::Mpsc> Ready;      // render -> worker
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class UNREALEVENTCAM_API UUnrealEventCam : public UActorComponent, public FRunnable
{
	GENERATED_BODY()

public:
	UUnrealEventCam();

	// Where the ROS event node listens.
	UPROPERTY(EditAnywhere, Category = "EventCam|Network")
	FString ServerIp = TEXT("127.0.0.1");

	UPROPERTY(EditAnywhere, Category = "EventCam|Network")
	int32 ServerPort = 3005;

	// Id sent in every packet, so several event cameras can share one consumer.
	UPROPERTY(EditAnywhere, Category = "EventCam|Network")
	int32 CamId = 0;

	// The SceneCapture2D this event camera renders from, chosen by name from a
	// dropdown of every SceneCapture2D in reach (this blueprint's components, or
	// the world when placed in a level). Same selection scheme as
	// UnrealStereoCam — no object pickers, which cannot target per-instance
	// components from the details panel.
	UPROPERTY(EditAnywhere, Category = "EventCam|Camera",
	          meta = (GetOptions = "GetSceneCaptureNames"))
	FName CaptureName = TEXT("EventCam");

	UFUNCTION()
	TArray<FString> GetSceneCaptureNames() const;

	// If the capture has no render target, one is created at this size (RGBA8).
	// DVS sensors are low-resolution; keep this modest — the event model is O(W*H)
	// per frame on the CPU.
	UPROPERTY(EditAnywhere, Category = "EventCam|Camera")
	int32 CaptureWidth = 346;

	UPROPERTY(EditAnywhere, Category = "EventCam|Camera")
	int32 CaptureHeight = 260;

	// Upper bound on how often frames are sampled for event generation (Hz).
	// The capture still renders every frame; this only caps the readback/event
	// rate. 0 = every rendered frame (most accurate, most CPU). Higher is better
	// for event fidelity — unlike an RGB stream, undersampling here loses events.
	UPROPERTY(EditAnywhere, Category = "EventCam|Camera", meta = (ClampMin = "0", ClampMax = "1000"))
	float MaxSampleHz = 0.0f;

	// ── ESIM model parameters (see EventSimulator.h) ───────────────────────────
	// Contrast thresholds C+ / C- in log-intensity units. Real DVS pixels sit
	// around 0.1-0.5; larger fires fewer events.
	UPROPERTY(EditAnywhere, Category = "EventCam|Model", meta = (ClampMin = "0.01"))
	float ContrastThresholdPos = 0.5f;

	UPROPERTY(EditAnywhere, Category = "EventCam|Model", meta = (ClampMin = "0.01"))
	float ContrastThresholdNeg = 0.5f;

	// Per-event Gaussian noise on the thresholds (pixel mismatch). 0 disables.
	UPROPERTY(EditAnywhere, Category = "EventCam|Model", AdvancedDisplay, meta = (ClampMin = "0.0"))
	float SigmaThresholdPos = 0.0f;

	UPROPERTY(EditAnywhere, Category = "EventCam|Model", AdvancedDisplay, meta = (ClampMin = "0.0"))
	float SigmaThresholdNeg = 0.0f;

	// Per-pixel dead time after firing (milliseconds). 0 disables.
	UPROPERTY(EditAnywhere, Category = "EventCam|Model", AdvancedDisplay, meta = (ClampMin = "0.0"))
	float RefractoryPeriodMs = 0.0f;

	// Operate on log intensity (the physical DVS model). Leave on outside debugging.
	UPROPERTY(EditAnywhere, Category = "EventCam|Model", AdvancedDisplay)
	bool bUseLogImage = true;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	// FRunnable — the worker thread runs the event model and drains to the socket.
	virtual uint32 Run() override;
	virtual void Stop() override;

private:
	void ResolveCapture();
	void PumpCapture();          // one render command: drain + start a readback
	bool TryConnect();           // socket I/O lives entirely on the worker thread

	UPROPERTY(Transient)
	USceneCaptureComponent2D* Capture = nullptr;

	FSocket* Socket = nullptr;                  // Run()'s thread only
	FRunnableThread* WorkerThread = nullptr;
	FThreadSafeBool bStopWorker = false;
	FThreadSafeBool bConnected = false;

	double LastConnectAttempt = 0.0;            // Run()'s thread only
	bool bLoggedWaiting = false;                // Run()'s thread only

	TSharedPtr<FEventStreamState, ESPMode::ThreadSafe> Stream;

	double LastSampleTime = 0.0;

	// Monotonic, pause-aware simulated time in seconds (accumulated game
	// DeltaTime). Kept as a double so event timestamps stay microsecond-precise
	// however long the run — a float second count would quantise them after a
	// few minutes. Freezes with the game, so a pause does not fabricate a huge
	// inter-frame delta.
	double AccumulatedSeconds = 0.0;
};
