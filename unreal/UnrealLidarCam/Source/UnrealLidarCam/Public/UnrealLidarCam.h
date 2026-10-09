// UnrealLidarCam: a lidar with a configurable aperture, streamed to ROS over TCP.
//
// The beam pattern is free: HorizontalFOV x VerticalFOV degrees, covered by
// HorizontalSamples columns x Channels rows, evenly spaced. HorizontalFOV = 360
// makes it a spinning lidar; less makes it a solid-state / flash lidar that can
// share a camera's view.
//
// Two ways to measure the beams (Mode):
//   GPU (default)  depth-only scene captures of the aperture (sectors of at most
//                  90 degrees horizontally) are rendered when a scan is due and
//                  read back asynchronously; each beam reads its range from the
//                  depth map. Cheap for dense patterns. Intensity: the default
//                  reflectivity times the incidence cosine (normals from the
//                  depth map).
//   CPU            one line trace per beam, spread over worker threads (as in
//                  MetaLidar); reflectivity from physical materials named
//                  "PM_Reflectivity_<0-255>", times the incidence cosine.
//
// Mounting: it is a scene component, so attach it under a camera (or anything)
// in the component tree; or name a camera / scene capture in FollowComponentName
// and it scans from that component's pose instead (e.g. the navcam it should
// match). Its forward (+X) is the scan's centre. The carrying actor is ignored.
//
// Each scan is one TCP packet to component_lidar (ROS), little-endian:
//   offset size  field
//   0      4     magic 'LIDR'
//   4      1     version = 1
//   5      1     sensor_id
//   6      2     flags            bit 0: organized (rows x columns, NaN where no return)
//   8      8     frame_id (u64)   GFrameCounter at the scan
//   16     8     stamp (f64)      simulation time (GncSimTime.h), s since the Unix epoch
//   24     4     horizontal_fov (f32, degrees)
//   28     4     vertical_fov   (f32, degrees)
//   32     4     columns (u32)
//   36     4     rows    (u32)
//   40     4     point_count (u32) = rows * columns
//   44     4     point_step  (u32) = 20
//   48     ...   points: x, y, z (f32, metres), intensity (f32, 0-255), ring (u16), column (u16)
// Points are in the sensor frame with the ROS convention: x forward, y left,
// z up. Row 0 is the top channel, column 0 the leftmost beam.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "Engine/EngineTypes.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/ThreadSafeBool.h"
#include "HAL/ThreadSafeCounter.h"
#include "Containers/Queue.h"
#include "UnrealLidarCam.generated.h"

class FSocket;
class USceneCaptureComponent2D;
struct FUnrealLidarCamGpuState;

UENUM(BlueprintType)
enum class EUnrealLidarCamMode : uint8
{
	GPU UMETA(DisplayName = "GPU (depth captures)"),
	CPU UMETA(DisplayName = "CPU (line traces)"),
};

// Finished scans, game thread -> sender thread. Ref-counted so it outlives the
// component if the sender is still draining it.
struct FUnrealLidarCamOutbox
{
	TQueue<TArray<uint8>, EQueueMode::Spsc> Packets;
	FThreadSafeCounter Queued;
};

UCLASS(ClassGroup = (Sensors), meta = (BlueprintSpawnableComponent), DisplayName = "Unreal Lidar Cam")
class UNREALLIDARCAM_API UUnrealLidarCam : public USceneComponent, public FRunnable
{
	GENERATED_BODY()

public:
	UUnrealLidarCam();

	// ---------------------------------------------------------------- Network
	// Where component_lidar listens (port in its params.yaml).
	UPROPERTY(EditAnywhere, Category = "Lidar|Network")
	FString ServerIp = TEXT("127.0.0.1");

	UPROPERTY(EditAnywhere, Category = "Lidar|Network")
	int32 ServerPort = 3007;

	// Tags every packet, for several lidars on one link.
	UPROPERTY(EditAnywhere, Category = "Lidar|Network", meta = (ClampMin = "0", ClampMax = "255"))
	int32 SensorId = 0;

	// --------------------------------------------------------------- Mounting
	// Optional: scan from this camera / scene capture's pose (dropdown of the
	// actor's cameras and captures) instead of this component's own. Leave
	// blank to use where this component is attached.
	UPROPERTY(EditAnywhere, Category = "Lidar|Mounting", meta = (GetOptions = "GetMountNames"))
	FName FollowComponentName;

	// ------------------------------------------------------------------- Scan
	// GPU: depth captures, read back asynchronously (default). CPU: one line
	// trace per beam (physical-material reflectivity).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan")
	EUnrealLidarCamMode Mode = EUnrealLidarCamMode::GPU;

	// Horizontal aperture, degrees, centred on forward. 360 = spinning lidar.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan",
	          meta = (ClampMin = "0.1", ClampMax = "360", Units = "Degrees"))
	float HorizontalFOV = 90.0f;

	// Vertical aperture, degrees, centred on forward (GPU mode: up to 160).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan",
	          meta = (ClampMin = "0.1", ClampMax = "180", Units = "Degrees"))
	float VerticalFOV = 30.0f;

	// Beams across the horizontal aperture (columns).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "1", ClampMax = "4096"))
	int32 HorizontalSamples = 512;

	// Beams across the vertical aperture (rows / channels).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "1", ClampMax = "512"))
	int32 Channels = 32;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "0", Units = "Meters"))
	float MinRange = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "0.1", Units = "Meters"))
	float MaxRange = 100.0f;

	// Scans per second (each scan is a full rows x columns frame).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "0.1", ClampMax = "50"))
	float ScanRateHz = 10.0f;

	// Gaussian range noise, standard deviation in metres (0 = exact).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "0", Units = "Meters"))
	float RangeNoiseStdDev = 0.0f;

	// GPU mode: depth-map pixels per beam spacing (above 1, so neighbouring
	// beams never share a pixel).
	UPROPERTY(EditAnywhere, Category = "Lidar|Scan", AdvancedDisplay, meta = (ClampMin = "1", ClampMax = "4"))
	float GpuOversampling = 1.5f;

	// CPU mode: collision channel of the traces.
	UPROPERTY(EditAnywhere, Category = "Lidar|Scan", AdvancedDisplay)
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

	// CPU mode: trace against the meshes' triangles rather than their simple collision.
	UPROPERTY(EditAnywhere, Category = "Lidar|Scan", AdvancedDisplay)
	bool bTraceComplex = true;

	// Do not see the actor carrying the lidar (the rover itself).
	UPROPERTY(EditAnywhere, Category = "Lidar|Scan", AdvancedDisplay)
	bool bIgnoreOwner = true;

	// Draws the returns in the viewport (debugging; costs frame time).
	UPROPERTY(EditAnywhere, Category = "Lidar|Debug")
	bool bDrawPoints = false;

	UFUNCTION()
	TArray<FString> GetMountNames() const;

	// Returns of the last scan (beams that hit something).
	UFUNCTION(BlueprintCallable, Category = "Lidar")
	int32 GetLastReturnCount() const { return LastReturnCount; }

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	// FRunnable: the sender thread owns the socket.
	virtual uint32 Run() override;
	virtual void Stop() override;

private:
	void BuildBeams();
	USceneComponent* ResolveMount() const;
	bool SetUpGpu(USceneComponent* Pose);
	void ScanCpu(double Stamp);
	void ScanGpu(double Stamp);
	void PumpGpu();                                     // Collect finished GPU scans
	// Ranges (metres, NaN = no return) and intensities per beam -> packet + outbox.
	void Publish(double Stamp, uint64 FrameId, const TArray<float>& Range, const TArray<float>& Intensity);
	bool TryConnect();

	TArray<FVector> Beams;                  // Unit directions, sensor frame (Unreal axes)
	int32 Rows = 0;
	int32 Columns = 0;
	EUnrealLidarCamMode ActiveMode = EUnrealLidarCamMode::GPU;

	// GPU mode: one depth capture per horizontal sector, and where each beam
	// falls in them.
	UPROPERTY(Transient)
	TArray<TObjectPtr<USceneCaptureComponent2D>> SectorCaptures;
	TArray<FQuat> SectorRotation;           // Sector camera in the sensor frame
	TArray<FIntPoint> SectorSize;
	TArray<float> SectorFocal;              // Pixels
	TArray<int32> BeamSector;
	TArray<FVector2f> BeamPixel;            // Sub-pixel position in its sector's image
	TArray<FVector3f> BeamInSector;         // Direction in its sector camera's frame
	TSharedPtr<FUnrealLidarCamGpuState, ESPMode::ThreadSafe> Gpu;

	TWeakObjectPtr<USceneComponent> Mount;
	double NextScanTime = 0.0;
	int32 LastReturnCount = 0;
	bool bLoggedDrop = false;

	TSharedPtr<FUnrealLidarCamOutbox, ESPMode::ThreadSafe> Outbox;
	FSocket* Socket = nullptr;              // Sender thread only
	FRunnableThread* SenderThread = nullptr;
	FThreadSafeBool bStopSender = false;
	FThreadSafeBool bConnected = false;
	double LastConnectAttempt = 0.0;        // Sender thread only
	bool bLoggedWaiting = false;            // Sender thread only
};
