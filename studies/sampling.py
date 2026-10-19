import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, lfilter

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from physics import ignition
from physics.catalog import load_bikes

FINE_RATE = 1e9
RATES_MSPS = (5, 20, 100)
FILTER_SHARE = 0.25
TRIALS = 300
CASES = (("cub110", 22.0, 3.3), ("cub110", 22.0, 16.0), ("fi125", 26.0, 3.3), ("fi125", 26.0, 19.0))


def peak_error(coil, open_kv, breakdown_kv, rate, rng):
    times = np.arange(int((coil.rise_us * 1e-6 + 4e-6) * FINE_RATE)) / FINE_RATE - 1e-6
    volts, _ = ignition.spark_waveform(coil, open_kv, breakdown_kv, times, rng)
    b, a = butter(2, FILTER_SHARE * rate * 1e6 / (FINE_RATE / 2))
    filtered = lfilter(b, a, volts)
    step = int(FINE_RATE / (rate * 1e6))
    errors = []
    for _ in range(TRIALS):
        phase = int(rng.integers(0, step))
        errors.append(1 - filtered[phase::step].max() / (breakdown_kv * 1000))
    return 100 * np.mean(errors), 100 * np.percentile(errors, 95)


def main():
    bikes = load_bikes()
    rng = np.random.default_rng(2)
    print(f"{'ignition':>8} {'spark kV':>9} " + " ".join(f"{str(r) + ' MSa/s':>17}" for r in RATES_MSPS))
    for name, open_kv, breakdown_kv in CASES:
        coil = bikes[name].ignition
        cells = []
        for rate in RATES_MSPS:
            mean, worst = peak_error(coil, open_kv, breakdown_kv, rate, rng)
            cells.append(f"{mean:6.1f}% (p95 {worst:4.1f})")
        print(f"{coil.kind:>8} {breakdown_kv:9.1f} " + " ".join(f"{c:>17}" for c in cells))


if __name__ == "__main__":
    main()