import numpy as np
from scipy.signal import bilinear, butter, lfilter

VACUUM_PERMITTIVITY = 8.854e-12
CORE_RADIUS = 0.6e-3
LEAD_RADIUS = 3.5e-3
INSULATION_PERMITTIVITY = 2.8
ELECTRODE_LENGTH = 25e-3
LOAD_CAPACITANCE = 68e-9
LOAD_RESISTANCE = 100e3
FILTER_CUTOFF = 25e6
FINE_RATE = 500e6
SAMPLE_RATE = 100e6
ADC_BITS = 12
ADC_FULL_SCALE = 1.0
ADC_NOISE_LSB = 1.2


def coupling_capacitance(air_gap):
    insulation = np.log(LEAD_RADIUS / CORE_RADIUS) / INSULATION_PERMITTIVITY
    air = np.log((LEAD_RADIUS + air_gap) / LEAD_RADIUS)
    return 2 * np.pi * VACUUM_PERMITTIVITY * ELECTRODE_LENGTH / (insulation + air)


def divider_gain(air_gap):
    coupling = coupling_capacitance(air_gap)
    return coupling / (coupling + LOAD_CAPACITANCE)


def volts_per_count():
    return 2 * ADC_FULL_SCALE / 2**ADC_BITS


def front_end(lead_volts, air_gap):
    time_constant = LOAD_RESISTANCE * (coupling_capacitance(air_gap) + LOAD_CAPACITANCE)
    highpass_b, highpass_a = bilinear([time_constant, 0.0], [time_constant, 1.0], FINE_RATE)
    lowpass_b, lowpass_a = butter(2, FILTER_CUTOFF / (FINE_RATE / 2))
    coupled = divider_gain(air_gap) * lfilter(highpass_b, highpass_a, lead_volts)
    return lfilter(lowpass_b, lowpass_a, coupled)


def digitise(clip_volts, offset_counts, rng):
    counts = np.round(clip_volts / volts_per_count() + offset_counts + ADC_NOISE_LSB * rng.standard_normal(clip_volts.size))
    limit = 2 ** (ADC_BITS - 1)
    return np.clip(counts, -limit, limit - 1).astype(np.int16)


def decimate_phase(fine, phase):
    step = int(round(FINE_RATE / SAMPLE_RATE))
    return fine[phase::step]