// UnrealSPADCam: a simulated single-photon (SPAD) camera front-end for Unreal.
//
// A SceneCapture renders an intensity image with each game frame; its readback
// is asynchronous (never stalls the game thread, same design as the sibling
// UnrealEventCam / UnrealStereoCam plugins). A worker thread converts each
// frame to 8-bit grayscale and streams it to a ROS node over TCP.
//
// The SPAD sensor MODEL (photon-flux estimation, Poisson photon arrivals, binary
// thresholding) is NOT in this plugin: it lives in the ROS component
// component_spad_camera, which ports spadSim (David Rodriguez-Martinez, MIT). The
// plugin only delivers the intensity frames the model consumes — the same split
// the stereo pipeline uses (plugin streams pixels, ROS does the processing).
//
// Wire format, little-endian, one packet per sampled frame:
//   offset size  field
//   0      4     magic 'SPD0'
//   4      1     version = 1
//   5      1     cam_id
//   6      2     width  (u16)
//   8      2     height (u16)
//   10     8     t_ns   (u64)   frame timestamp (simulated time)
//   18     4     payload size (u32) = width*height (mono8)
//   22     2     reserved
//   24     ...   width*height bytes, 8-bit grayscale intensity, row-major

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
#include "UnrealSPADCam.generated.h"

class FSocket;

// One in-flight asynchronous readback. Touched only on the render thread.
struct FSpadPendingReadback
{
	TUniquePtr<FRHIGPUTextureReadback> Readback;
	int32 Width = 0;
	int32 Height = 0;
	int64 TimeNs = 0;
};

// One frame handed from the render thread to the worker: raw BGRA as read back,
// plus its size and timestamp. The worker converts to mono8 and sends.
struct FSpadReadyFrame
{
	TArray<uint8> Bgra;   // Width*Height*4
	int32 Width = 0;
	int32 Height = 0;
	int64 TimeNs = 0;
};

// Shared between the render commands and the worker through a ref-counted
// pointer so teardown never blocks (same pattern as UnrealEventCam):
//   Pending : created, polled, locked and freed ONLY on the render thread.
//   Ready   : lock-free MPSC handoff, render thread -> worker thread.
struct FSpadStreamState
{
	TArray<FSpadPendingReadback> Pending;                  // render thread only
	TQueue<FSpadReadyFrame, EQueueMode::Mpsc> Ready;       // render -> worker
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class UNREALSPADCAM_API UUnrealSPADCam : public UActorComponent, public FRunnable
{
	GENERATED_BODY()

public:
	UUnrealSPADCam();

	// Where the ROS SPAD node listens (component_spad_camera).
	UPROPERTY(EditAnywhere, Category = "SPADCam|Network")
	FString ServerIp = TEXT("127.0.0.1");

	UPROPERTY(EditAnywhere, Category = "SPADCam|Network")
	int32 ServerPort = 3006;

	// Id stamped in every packet, so several SPAD cameras can share one consumer.
	UPROPERTY(EditAnywhere, Category = "SPADCam|Network")
	int32 CamId = 0;

	// The SceneCapture2D this SPAD camera renders from, chosen by name from a
	// dropdown of every SceneCapture2D in reach (same scheme as the other
	// plugins — the details panel cannot point at a per-instance component).
	UPROPERTY(EditAnywhere, Category = "SPADCam|Camera",
	          meta = (GetOptions = "GetSceneCaptureNames"))
	FName CaptureName = TEXT("SPADCam");

	UFUNCTION()
	TArray<FString> GetSceneCaptureNames() const;

	// If the capture has no render target, one is created at this size (RGBA8).
	// SPAD sensors are low-resolution — keep this modest.
	UPROPERTY(EditAnywhere, Category = "SPADCam|Camera")
	int32 CaptureWidth = 256;

	UPROPERTY(EditAnywhere, Category = "SPADCam|Camera")
	int32 CaptureHeight = 256;

	// Upper bound on how often frames are sampled and sent (Hz). The capture
	// still renders every frame; this only caps the send rate. 0 = every
	// rendered frame. The SPAD model's effective exposure is set on the ROS side.
	UPROPERTY(EditAnywhere, Category = "SPADCam|Camera", meta = (ClampMin = "0", ClampMax = "1000"))
	float MaxSampleHz = 0.0f;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	// FRunnable — the worker thread converts frames to mono8 and drains to the socket.
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

	TSharedPtr<FSpadStreamState, ESPMode::ThreadSafe> Stream;

	double LastSampleTime = 0.0;

	// Monotonic, pause-aware simulated time in seconds (accumulated game
	// DeltaTime), kept as a double so frame timestamps stay precise however long
	// the run and freeze when the game pauses.
	double AccumulatedSeconds = 0.0;
};
