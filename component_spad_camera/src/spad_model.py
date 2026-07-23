"""
SPAD sensor model — ported from spadSim.

── Credit ────────────────────────────────────────────────────────────────────
This is a port of the photon-detection core of spadSim by David Rodriguez-
Martinez, used under its MIT License:

    https://github.com/drodriguezSRL/spadSim   (MIT, (c) 2025 David Rodriguez-Martinez)
    scripts/spad_emulator.py

Related work (from the spadSim repository):
    "Fast Vision in the Dark: A Case for Single-Photon Imaging in Planetary
    Navigation", arXiv:2510.10597.

The pipeline follows spadSim's `generate_spad_sequence_from_pair`: normalise the
intensity to [0, 1], convert to an expected photon count (accounting for quantum
efficiency and the SPAD-vs-RGB exposure ratio), optionally add a dark-count
term, draw the actual photon count from a Poisson distribution, and threshold to
a binary single-photon frame. Only the surrounding plumbing differs: this runs on
a live intensity stream from the UnrealSPADCam plugin instead of pre-recorded
video, so spadSim's optical-flow frame interpolation (a trick for boosting the
frame rate of recorded footage) is not needed — Unreal renders real frames.
────────────────────────────────────────────────────────────────────────────────
"""

import numpy as np


class SPADModel:
    """
    Turns an 8-bit intensity image into a binary single-photon frame.

    Stateless apart from its RNG: call sample() once per intensity frame. The
    default parameters match spadSim (rgb_photons=1000, QE=0.5, dcr=100 c/s,
    dark counts off, detection threshold 1 photon).
    """

    def __init__(self, rgb_photons=1000.0, quantum_efficiency=0.5,
                 spad_fps=10000.0, rgb_fps=30.0, dcr=100.0, include_dcr=False,
                 detection_threshold=1, seed=None):
        self.rgb_photons = float(rgb_photons)
        self.qe = float(quantum_efficiency)
        self.spad_fps = float(spad_fps)
        self.rgb_fps = float(rgb_fps)
        self.dcr = float(dcr)
        self.include_dcr = bool(include_dcr)
        self.detection_threshold = int(detection_threshold)
        self.rng = np.random.default_rng(seed)

    @property
    def t_spad(self) -> float:
        """SPAD exposure time (s)."""
        return 1.0 / max(self.spad_fps, 1e-9)

    @property
    def t_rgb(self) -> float:
        """Reference RGB exposure time (s), the basis for rgb_photons."""
        return 1.0 / max(self.rgb_fps, 1e-9)

    def sample(self, intensity_u8: np.ndarray) -> np.ndarray:
        """
        intensity_u8: HxW uint8 intensity image.
        Returns a HxW uint8 binary frame (0/1) of single-photon detections.
        """
        # spadSim stage 1: normalise to [0, 1].
        i_norm = np.clip(intensity_u8.astype(np.float32) / 255.0, 0.0, 1.0)

        # spadSim stage 1 cont'd: expected signal photons for one SPAD exposure.
        #   lambda_signal = rgb_photons * i_norm * QE * (t_spad / t_rgb)
        exposure_ratio = self.t_spad / self.t_rgb
        lambda_signal = self.rgb_photons * i_norm * self.qe * exposure_ratio

        # spadSim stage 2: optional dark counts.
        lambda_total = lambda_signal
        if self.include_dcr:
            lambda_total = lambda_total + self.dcr * self.t_spad

        # spadSim stage 3: Poisson photon arrivals.
        n = self.rng.poisson(lam=lambda_total)

        # spadSim stage 4: binary thresholding.
        return (n >= self.detection_threshold).astype(np.uint8)

    def sample_sum(self, intensity_u8: np.ndarray, k: int):
        """
        Integrate `k` independent single-photon exposures of the same intensity
        (a SPAD sums many fast frames between readouts). Returns
        (sum HxW int32, last_binary HxW uint8) — the same per-exposure model as
        sample(), drawn k-at-once for efficiency.
        """
        k = max(1, int(k))
        i_norm = np.clip(intensity_u8.astype(np.float32) / 255.0, 0.0, 1.0)
        lambda_total = (self.rgb_photons * i_norm * self.qe
                        * (self.t_spad / self.t_rgb))
        if self.include_dcr:
            lambda_total = lambda_total + self.dcr * self.t_spad
        n = self.rng.poisson(lam=lambda_total, size=(k,) + i_norm.shape)
        bits = (n >= self.detection_threshold).astype(np.uint8)
        return bits.sum(axis=0, dtype=np.int32), bits[-1]
