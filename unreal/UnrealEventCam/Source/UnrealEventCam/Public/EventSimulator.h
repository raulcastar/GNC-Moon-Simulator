// EventSimulator: the ESIM event-generation model, ported to Unreal.
//
// ── Credit ──────────────────────────────────────────────────────────────────
// This file is a port of the event-generation core of ESIM (rpg_esim) by the
// Robotics and Perception Group, University of Zurich:
//
//     https://github.com/uzh-rpg/rpg_esim   (MIT License, (c) 2018 RPG)
//     event_camera_simulator/esim/src/event_simulator.cpp
//
// If you use this in academic work, please cite:
//
//     H. Rebecq, D. Gehrig, D. Scaramuzza,
//     "ESIM: an Open Event Camera Simulator",
//     Conference on Robot Learning (CoRL), 2018.
//
// The algorithm below (per-pixel log-intensity threshold crossings with linear
// timestamp interpolation, per-polarity contrast thresholds, optional Gaussian
// threshold noise and a per-pixel refractory period) follows ESIM's
// EventSimulator::imageCallback. Only the surrounding types are changed: OpenCV
// / glog / ze are replaced by plain arrays and Unreal primitives so the model
// carries no external dependencies. The MIT notice ships in the plugin LICENSE.
// ─────────────────────────────────────────────────────────────────────────────

#pragma once

#include "CoreMinimal.h"
#include <random>

// One DVS event: pixel (X, Y), timestamp in nanoseconds, polarity (brightness
// increase = positive). Matches the fields of ESIM's Event.
struct FEventCamEvent
{
	uint16 X = 0;
	uint16 Y = 0;
	int64  TimeNs = 0;
	bool   bPositive = false;
};

// The event-camera model. Feed it consecutive intensity frames (grayscale in
// [0, 1]) with monotonically increasing nanosecond timestamps; it returns the
// events fired in the interval since the previous frame, sorted by time.
//
// Not a UObject and not thread-affine: create it on and drive it from a single
// thread (here, the plugin's sender/worker thread), which keeps its per-pixel
// state (reference levels, last-event times) consistent without locking.
class FEventSimulator
{
public:
	struct FConfig
	{
		// Contrast thresholds C+ / C- in log-intensity units (ESIM Cp / Cm). An
		// event fires each time the log intensity moves by this much. Real DVS
		// pixels sit around 0.1-0.5; larger = fewer events.
		float Cp = 0.5f;
		float Cm = 0.5f;

		// Per-event Gaussian noise on the threshold (ESIM sigma_Cp / sigma_Cm),
		// modelling pixel-to-pixel mismatch. 0 disables it.
		float SigmaCp = 0.0f;
		float SigmaCm = 0.0f;

		// Dead time per pixel after it fires (ESIM refractory_period_ns). Events
		// closer together than this at the same pixel are dropped. 0 disables.
		int64 RefractoryPeriodNs = 0;

		// Operate on log intensity (ESIM use_log_image). This is the physical DVS
		// model and should stay on; the raw-intensity path exists for debugging.
		bool bUseLogImage = true;

		// Offset inside the log to keep dark pixels finite: log(LogEps + I),
		// I in [0, 1] (ESIM log_eps).
		float LogEps = 0.001f;
	};

	FEventSimulator();

	void Configure(const FConfig& InConfig);

	// Forget all per-pixel state; the next frame re-initialises the reference.
	void Reset();

	// Process one frame. Intensity is row-major, W*H, each value in [0, 1].
	// Appends the events fired since the previous frame to OutEvents (sorted by
	// time). The first frame after a reset only primes the state and fires none.
	void ProcessFrame(const float* Intensity, int32 W, int32 H, int64 TimeNs,
	                  TArray<FEventCamEvent>& OutEvents);

private:
	void Init(const TArray<float>& LogImg, int32 W, int32 H, int64 TimeNs);

	FConfig Config;
	bool  bInitialized = false;
	int64 CurrentTimeNs = 0;
	int32 Width = 0;
	int32 Height = 0;

	TArray<float> RefValues;       // ref_values_       : last fired log level
	TArray<float> LastImg;         // last_img_         : previous log frame
	TArray<int64> LastEventTsNs;   // last_event_timestamp_ (0 = never fired)
	TArray<float> Preprocessed;    // scratch log frame, reused per call

	std::mt19937 Rng;
	std::normal_distribution<float> UnitNormal;   // N(0,1), scaled by sigma
};
