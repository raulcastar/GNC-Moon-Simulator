// Port of ESIM's EventSimulator::imageCallback — see EventSimulator.h for the
// full credit and citation (rpg_esim, MIT (c) 2018 Robotics and Perception
// Group, University of Zurich).

#include "EventSimulator.h"

FEventSimulator::FEventSimulator()
	: Rng(0x9E3779B9u)      // fixed seed: reproducible event noise across runs
	, UnitNormal(0.0f, 1.0f)
{
}

void FEventSimulator::Configure(const FConfig& InConfig)
{
	Config = InConfig;
	Reset();
}

void FEventSimulator::Reset()
{
	bInitialized = false;
	CurrentTimeNs = 0;
	Width = Height = 0;
	RefValues.Reset();
	LastImg.Reset();
	LastEventTsNs.Reset();
}

void FEventSimulator::Init(const TArray<float>& LogImg, int32 W, int32 H,
                           int64 TimeNs)
{
	// Mirrors EventSimulator::init: prime the per-pixel state from the first
	// frame so no events fire until there is a delta to measure.
	Width = W;
	Height = H;
	LastImg = LogImg;
	RefValues = LogImg;
	LastEventTsNs.Init(0, W * H);
	CurrentTimeNs = TimeNs;
	bInitialized = true;
}

void FEventSimulator::ProcessFrame(const float* Intensity, int32 W, int32 H,
                                   int64 TimeNs,
                                   TArray<FEventCamEvent>& OutEvents)
{
	if (!Intensity || W <= 0 || H <= 0)
	{
		return;
	}

	// Preprocess into log space: preprocessed = log(eps + I), I in [0, 1].
	const int32 N = W * H;
	Preprocessed.SetNumUninitialized(N);
	if (Config.bUseLogImage)
	{
		for (int32 i = 0; i < N; ++i)
		{
			Preprocessed[i] = FMath::Loge(Config.LogEps + Intensity[i]);
		}
	}
	else
	{
		FMemory::Memcpy(Preprocessed.GetData(), Intensity, N * sizeof(float));
	}

	// First frame (or after a size change / reset): prime and fire nothing.
	if (!bInitialized || W != Width || H != Height)
	{
		Init(Preprocessed, W, H, TimeNs);
		return;
	}

	const int64 DeltaTNs = TimeNs - CurrentTimeNs;
	if (DeltaTNs <= 0)
	{
		// Non-increasing time (paused / duplicate frame): nothing to integrate.
		LastImg = Preprocessed;
		return;
	}

	constexpr float Tolerance = 1e-6f;
	constexpr float MinContrast = 0.01f;

	for (int32 y = 0; y < H; ++y)
	{
		for (int32 x = 0; x < W; ++x)
		{
			const int32 i = y * W + x;
			const float itdt = Preprocessed[i];   // current  log intensity
			const float it   = LastImg[i];         // previous log intensity
			float prev_cross = RefValues[i];

			if (FMath::Abs(it - itdt) <= Tolerance)
			{
				continue;
			}

			const float pol = (itdt >= it) ? +1.0f : -1.0f;
			float C = (pol > 0.0f) ? Config.Cp : Config.Cm;
			const float sigma_C = (pol > 0.0f) ? Config.SigmaCp : Config.SigmaCm;
			if (sigma_C > 0.0f)
			{
				C += UnitNormal(Rng) * sigma_C;
				C = FMath::Max(MinContrast, C);
			}

			float curr_cross = prev_cross;
			bool all_crossings = false;
			do
			{
				curr_cross += pol * C;

				if ((pol > 0.0f && curr_cross > it && curr_cross <= itdt) ||
				    (pol < 0.0f && curr_cross < it && curr_cross >= itdt))
				{
					// Linearly interpolate the crossing time within [t0, t].
					const int64 edt =
						static_cast<int64>((curr_cross - it) * DeltaTNs / (itdt - it));
					const int64 t = CurrentTimeNs + edt;

					const int64 last = LastEventTsNs[i];
					if (last == 0 || (t - last) >= Config.RefractoryPeriodNs)
					{
						FEventCamEvent Ev;
						Ev.X = static_cast<uint16>(x);
						Ev.Y = static_cast<uint16>(y);
						Ev.TimeNs = t;
						Ev.bPositive = (pol > 0.0f);
						OutEvents.Add(Ev);
						LastEventTsNs[i] = t;
					}
					// else: dropped by the refractory period.
					RefValues[i] = curr_cross;
				}
				else
				{
					all_crossings = true;
				}
			} while (!all_crossings);
		}
	}

	CurrentTimeNs = TimeNs;
	LastImg = Preprocessed;

	// ESIM returns events sorted by timestamp; downstream consumers rely on it.
	OutEvents.Sort([](const FEventCamEvent& A, const FEventCamEvent& B)
	{
		return A.TimeNs < B.TimeNs;
	});
}
