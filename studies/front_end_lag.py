import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from physics import clip

SLOPES_KV_PER_US = (1.0, 3.0, 6.0, 10.0)
PEAKS_KV = (3.0, 8.0, 16.0)
TRIALS = 200
AIR_GAP = 0.2e-3
PRE_SECONDS = 2e-6
POST_SECONDS = 1e-6


def bench_pulse(slope, peak, rng):
    rise = peak / slope * 1e-6
    fine = np.arange(int((PRE_SECONDS + rise + POST_SECONDS) * clip.FINE_RATE)) / clip.FINE_RATE - PRE_SECONDS
    offset = rng.uniform(0, 1 / clip.FINE_RATE)
    lead = np.clip((fine + offset) * slope * 1e9, 0, None)
    lead[fine + offset >= rise] = 0.0
    return lead


def shortfall_samples(slope, peak, rng):
    counts_per_sample = slope * 1e9 / clip.SAMPLE_RATE * clip.divider_gain(AIR_GAP) / clip.volts_per_count()
    true_counts = peak * 1e3 * clip.divider_gain(AIR_GAP) / clip.volts_per_count()
    lags = []
    for _ in range(TRIALS):
        filtered = clip.front_end(bench_pulse(slope, peak, rng), AIR_GAP)
        phase = int(rng.integers(0, int(clip.FINE_RATE / clip.SAMPLE_RATE)))
        sampled = clip.decimate_phase(filtered, phase) / clip.volts_per_count()
        lags.append((true_counts - sampled.max()) / counts_per_sample)
    return np.mean(lags), np.std(lags)


if __name__ == "__main__":
    rng = np.random.default_rng(1)
    everything = []
    print(f"{'kV/us':>6} {'peak kV':>8} {'lag samples':>12} {'spread':>7}")
    for slope in SLOPES_KV_PER_US:
        for peak in PEAKS_KV:
            mean, spread = shortfall_samples(slope, peak, rng)
            everything.append(mean)
            print(f"{slope:6.1f} {peak:8.1f} {mean:12.3f} {spread:7.3f}")
    print(f"median lag: {np.median(everything):.2f} samples")