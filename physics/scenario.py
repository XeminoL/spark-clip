from dataclasses import dataclass, replace

import numpy as np

from physics import clip, ignition
from physics.cranking import Battery, CrankingModel, Cylinder, Firing, simulate
from physics.inertia import inertia_budget
from physics.paschen import load_air, plug_kv

CALIBRATION_SAMPLES = 8192
PRE_TRIGGER_SECONDS = 10e-6
POST_BURN_SECONDS = 120e-6
BREAKDOWN_SCATTER = 0.04
CLIP_AIR_GAP_RANGE = (0.05e-3, 0.4e-3)
ADC_OFFSET_RANGE = (-20, 20)
EXHAUST_TEMPERATURE_SPREAD = 5.0
ADVANCE_SPREAD_DEG = 0.7
INERTIA_SPREAD = 0.04
FRICTION_SPREAD = 0.10
MOTOR_SPREAD = 0.03
ARMATURE_SPREAD = 0.05
COIL_SPREAD = 0.05
HEAT_TRANSFER_SPREAD = 0.15


@dataclass(frozen=True)
class Scenario:
    bike: str
    battery: Battery
    leak_area: float
    plug_gap: float
    heat_transfer: float = 60.0
    firing: Firing = None
    start_angle_deg: float = 100.0
    seed: int = 1


@dataclass
class Window:
    start: int
    samples: np.ndarray


@dataclass
class Capture:
    scenario: Scenario
    truth_bike: object
    record: object
    calibration: np.ndarray
    windows: list
    breakdown_kv: list
    open_kv: list
    fired: list
    clip_gain: float
    heat_transfer: float


def spread(rng, size):
    return 1 + size * rng.uniform(-1, 1)


def as_built(bike, rng):
    budget_total = inertia_budget(bike).total
    rotating = replace(bike.rotating, other_inertia=bike.rotating.other_inertia + budget_total * INERTIA_SPREAD * rng.uniform(-1, 1))
    starter = replace(
        bike.starter,
        motor_constant=bike.starter.motor_constant * spread(rng, MOTOR_SPREAD),
        armature_resistance=bike.starter.armature_resistance * spread(rng, ARMATURE_SPREAD),
    )
    friction = replace(
        bike.friction,
        static_torque=bike.friction.static_torque * spread(rng, FRICTION_SPREAD),
        viscous=bike.friction.viscous * spread(rng, FRICTION_SPREAD),
    )
    coil = replace(bike.ignition, peak_kv=bike.ignition.peak_kv * spread(rng, COIL_SPREAD), rise_us=bike.ignition.rise_us * spread(rng, COIL_SPREAD))
    engine = replace(bike.engine, spark_advance_deg=bike.engine.spark_advance_deg + ADVANCE_SPREAD_DEG * rng.uniform(-1, 1))
    return replace(bike, rotating=rotating, starter=starter, friction=friction, ignition=coil, engine=engine)


def lead_window(coil, open_kv, breakdown_kv, rng):
    length = PRE_TRIGGER_SECONDS + coil.rise_us * 1e-6 + coil.burn_us * 1e-6 + POST_BURN_SECONDS
    times = np.arange(int(length * clip.FINE_RATE)) / clip.FINE_RATE - PRE_TRIGGER_SECONDS
    volts, fired = ignition.spark_waveform(coil, open_kv, breakdown_kv, times, rng)
    return volts, fired


def build_capture(scenario, bikes):
    rng = np.random.default_rng(scenario.seed)
    bike = as_built(bikes[scenario.bike], rng)
    heat = scenario.heat_transfer * spread(rng, HEAT_TRANSFER_SPREAD)
    model = CrankingModel(bike, scenario.battery, Cylinder(scenario.leak_area, heat), scenario.firing)
    record = simulate(
        model,
        np.radians(scenario.start_angle_deg),
        rng,
        exhaust_temperature_error=EXHAUST_TEMPERATURE_SPREAD * rng.standard_normal(),
    )
    air = load_air()
    air_gap = rng.uniform(*CLIP_AIR_GAP_RANGE)
    offset = int(rng.integers(*ADC_OFFSET_RANGE, endpoint=True))
    quiet = np.zeros(CALIBRATION_SAMPLES * 5)
    calibration = clip.digitise(clip.decimate_phase(clip.front_end(quiet, air_gap), 0), offset, rng)
    windows, breakdowns, opens, fired_list = [], [], [], []
    previous_time = None
    for spark in record.sparks:
        rpm = spark.speed * 60 / (2 * np.pi) if previous_time is None else 60 / (spark.time - previous_time)
        previous_time = spark.time
        open_kv = ignition.open_circuit_kv(bike.ignition, rpm, model.battery_voltage(spark.speed))
        breakdown = plug_kv(air, spark.density_ratio, scenario.plug_gap) * (1 + BREAKDOWN_SCATTER * rng.standard_normal())
        lead_volts, fired = lead_window(bike.ignition, open_kv, breakdown, rng)
        phase = int(rng.integers(0, int(clip.FINE_RATE / clip.SAMPLE_RATE)))
        samples = clip.digitise(clip.decimate_phase(clip.front_end(lead_volts, air_gap), phase), offset, rng)
        start = CALIBRATION_SAMPLES + int(round((spark.time - PRE_TRIGGER_SECONDS) * clip.SAMPLE_RATE))
        windows.append(Window(start, samples))
        breakdowns.append(breakdown)
        opens.append(open_kv)
        fired_list.append(fired)
    return Capture(scenario, bike, record, calibration, windows, breakdowns, opens, fired_list, clip.divider_gain(air_gap), heat)