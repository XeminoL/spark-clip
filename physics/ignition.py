import numpy as np

COLLAPSE_SECONDS = 5e-9
RING_FREQUENCY = 30e6
RING_SECONDS = 60e-9
RING_SHARE = 0.15
MISFIRE_DECAY_RISES = 3.0
TAIL_SECONDS = 20e-6
TAIL_FREQUENCY = 50e3
TAIL_SHARE = 0.5
BURN_ENERGY_SHARE = 0.6
BURN_RIPPLE = 0.05


def rise_shape(fraction):
    return 0.5 * (1 - np.cos(np.pi * np.clip(fraction, 0.0, 1.0)))


def open_circuit_kv(ignition, rpm, battery_volts):
    if ignition.kind == "ac_cdi":
        return ignition.peak_kv * (1 - np.exp(-rpm / ignition.speed_scale_rpm))
    if ignition.kind == "dc_cdi":
        share = (battery_volts - ignition.cutoff_v) / (ignition.full_v - ignition.cutoff_v)
        return ignition.peak_kv * float(np.clip(share, 0.0, 1.0))
    return ignition.peak_kv * min(1.0, battery_volts / ignition.full_v)


def breakdown_time(ignition, open_kv, breakdown_kv):
    if breakdown_kv >= open_kv:
        return None
    return ignition.rise_us * 1e-6 * np.arccos(1 - 2 * breakdown_kv / open_kv) / np.pi


def burn_seconds(ignition, reserve_used):
    return ignition.burn_us * 1e-6 * (1 - BURN_ENERGY_SHARE * reserve_used**2)


def spark_waveform(ignition, open_kv, breakdown_kv, times, rng):
    rise = ignition.rise_us * 1e-6
    volts = open_kv * rise_shape(times / rise)
    volts[times < 0] = 0.0
    fire_time = breakdown_time(ignition, open_kv, breakdown_kv)
    if fire_time is None:
        after = times > rise
        elapsed = times[after] - rise
        volts[after] = open_kv * np.exp(-elapsed / (MISFIRE_DECAY_RISES * rise)) * np.cos(np.pi * elapsed / (2 * rise))
        return volts * 1000, False
    burn = burn_seconds(ignition, breakdown_kv / open_kv)
    after = times >= fire_time
    elapsed = times[after] - fire_time
    collapse = breakdown_kv * np.exp(-elapsed / COLLAPSE_SECONDS)
    ring = RING_SHARE * breakdown_kv * np.exp(-elapsed / RING_SECONDS) * np.sin(2 * np.pi * RING_FREQUENCY * elapsed)
    ripple = 1 + BURN_RIPPLE * np.sin(2 * np.pi * elapsed / (burn / 7) + rng.uniform(0, 2 * np.pi))
    plateau = np.where(elapsed < burn, ignition.burn_kv * ripple * (1 - np.exp(-elapsed / (20 * COLLAPSE_SECONDS))), 0.0)
    past = elapsed - burn
    tail = np.where(past >= 0, TAIL_SHARE * ignition.burn_kv * np.exp(-past / TAIL_SECONDS) * np.cos(2 * np.pi * TAIL_FREQUENCY * past), 0.0)
    volts[after] = collapse + ring + plateau + tail
    return volts * 1000, True