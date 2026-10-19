from dataclasses import dataclass, field

import numpy as np
from scipy.integrate import solve_ivp

from physics import geometry
from physics.inertia import inertia_budget
from physics.paschen import REFERENCE_DENSITY

GAS_CONSTANT = 287.05
HEAT_RATIO = 1.4
CV = GAS_CONSTANT / (HEAT_RATIO - 1)
CP = CV + GAS_CONSTANT
AMBIENT_PRESSURE = 1.013e5
AMBIENT_TEMPERATURE = 303.0
FRICTION_SMOOTHING = 0.5
COMBUSTION_SPAN = np.radians(40)
CRANKING_SECONDS = 2.5
START_SPEED = 0.05
RELATIVE_TOLERANCE = 1e-8
ABSOLUTE_TOLERANCE = (1e-9, 1e-7, 1e-12, 1e-6)
TRACE_STEP = 5e-4


@dataclass(frozen=True)
class Battery:
    emf: float
    internal_resistance: float


@dataclass(frozen=True)
class Cylinder:
    leak_area: float
    heat_transfer: float
    wall_temperature: float = AMBIENT_TEMPERATURE


@dataclass(frozen=True)
class Firing:
    first_compression: int
    heat_release: float


@dataclass(frozen=True)
class Spark:
    time: float
    compression: bool
    density_ratio: float
    gas_temperature: float
    speed: float
    jitter: float


@dataclass
class CrankingRecord:
    sparks: list = field(default_factory=list)
    trace_time: list = field(default_factory=list)
    trace_speed: list = field(default_factory=list)
    trace_pressure: list = field(default_factory=list)
    trace_battery: list = field(default_factory=list)
    polytropic: list = field(default_factory=list)
    trapped: list = field(default_factory=list)
    peak_pressure: list = field(default_factory=list)


def orifice_flow(upstream_pressure, upstream_temperature, downstream_pressure, area):
    ratio = downstream_pressure / upstream_pressure
    critical = (2 / (HEAT_RATIO + 1)) ** (HEAT_RATIO / (HEAT_RATIO - 1))
    if ratio <= critical:
        shape = np.sqrt(HEAT_RATIO) * (2 / (HEAT_RATIO + 1)) ** ((HEAT_RATIO + 1) / (2 * (HEAT_RATIO - 1)))
    else:
        shape = np.sqrt(2 * HEAT_RATIO / (HEAT_RATIO - 1) * (ratio ** (2 / HEAT_RATIO) - ratio ** ((HEAT_RATIO + 1) / HEAT_RATIO)))
    return area * upstream_pressure * shape / np.sqrt(GAS_CONSTANT * upstream_temperature)


def leak_rate(pressure, temperature, area):
    if pressure >= AMBIENT_PRESSURE:
        return orifice_flow(pressure, temperature, AMBIENT_PRESSURE, area)
    return -orifice_flow(AMBIENT_PRESSURE, AMBIENT_TEMPERATURE, pressure, area)


def starter_current(bike, battery, speed):
    starter = bike.starter
    back_emf = starter.motor_constant * starter.gear_ratio * speed
    resistance = battery.internal_resistance + starter.armature_resistance + starter.cable_resistance
    return max(0.0, (battery.emf - starter.brush_drop - back_emf) / resistance)


def starter_torque(bike, current):
    starter = bike.starter
    return starter.gear_efficiency * starter.motor_constant * starter.gear_ratio * current


def friction_torque(bike, speed):
    return bike.friction.static_torque * np.tanh(speed / FRICTION_SMOOTHING) + bike.friction.viscous * speed


class CrankingModel:
    def __init__(self, bike, battery, cylinder, firing=None):
        self.bike = bike
        self.engine = bike.engine
        self.battery = battery
        self.cylinder = cylinder
        self.firing = firing
        self.inertia = inertia_budget(bike).total

    def gas_pressure(self, theta, mass, temperature, sealed):
        if not sealed:
            return AMBIENT_PRESSURE
        return mass * GAS_CONSTANT * temperature / geometry.volume(self.engine, theta)

    def rates(self, theta, speed, mass, temperature, sealed, burn_start):
        pressure = self.gas_pressure(theta, mass, temperature, sealed)
        current = starter_current(self.bike, self.battery, speed)
        torque = starter_torque(self.bike, current) + (pressure - AMBIENT_PRESSURE) * geometry.volume_rate(self.engine, theta) - friction_torque(self.bike, speed)
        acceleration = torque / self.inertia
        if not sealed:
            return [speed, acceleration, 0.0, 0.0]
        volume_change = geometry.volume_rate(self.engine, theta) * speed
        heat_loss = self.cylinder.heat_transfer * geometry.wall_area(self.engine, theta) * (temperature - self.cylinder.wall_temperature)
        if burn_start is not None and 0.0 <= theta - burn_start < COMBUSTION_SPAN:
            heat_loss -= self.firing.heat_release * speed / COMBUSTION_SPAN
        outflow = leak_rate(pressure, temperature, self.cylinder.leak_area)
        flow_temperature = temperature if outflow > 0 else AMBIENT_TEMPERATURE
        mass_change = -outflow
        temperature_change = (-pressure * volume_change - heat_loss + (CP * flow_temperature - CV * temperature) * mass_change) / (mass * CV)
        return [speed, acceleration, mass_change, temperature_change]

    def battery_voltage(self, speed):
        return self.battery.emf - starter_current(self.bike, self.battery, speed) * self.battery.internal_resistance


def cycle_events(engine):
    return sorted([
        (geometry.intake_close_angle(engine), "intake_close"),
        (geometry.compression_spark_angle(engine), "compression_spark"),
        (geometry.exhaust_open_angle(engine), "exhaust_open"),
        (geometry.exhaust_spark_angle(engine), "exhaust_spark"),
    ])


def event_after(engine, theta):
    cycle_start = np.floor(theta / geometry.CYCLE) * geometry.CYCLE
    events = cycle_events(engine)
    for offset in (0.0, geometry.CYCLE):
        for index, (angle, _) in enumerate(events):
            if cycle_start + offset + angle > theta:
                return cycle_start + offset, index
    raise RuntimeError("no event ahead")


def event_target(engine, cycle_start, index, jitter):
    angle, name = cycle_events(engine)[index]
    return cycle_start + angle + (jitter if name.endswith("spark") else 0.0), name


def simulate(model, start_angle, rng, jitter_deg=0.3, exhaust_temperature_error=0.0, duration=CRANKING_SECONDS):
    engine = model.engine
    theta = start_angle
    sealed = geometry.is_sealed(engine, theta % geometry.CYCLE)
    mass = AMBIENT_PRESSURE * geometry.volume(engine, theta) / (GAS_CONSTANT * AMBIENT_TEMPERATURE)
    state = np.array([theta, START_SPEED, mass, AMBIENT_TEMPERATURE])
    time = 0.0
    record = CrankingRecord()
    compression_count = 0
    burn_start = None
    intake_state = None
    peak_pressure = 0.0
    exhaust_temperature = AMBIENT_TEMPERATURE
    jitter = np.radians(jitter_deg) * rng.standard_normal()
    next_trace = 0.0
    cycle_start, event_index = event_after(engine, theta)
    while time < duration:
        target, name = event_target(engine, cycle_start, event_index, jitter)
        reached = lambda t, y: y[0] - target
        reached.terminal = True
        reached.direction = 1
        solution = solve_ivp(
            lambda t, y: model.rates(y[0], y[1], y[2], y[3], sealed, burn_start),
            (time, duration),
            state,
            events=reached,
            rtol=RELATIVE_TOLERANCE,
            atol=ABSOLUTE_TOLERANCE,
            dense_output=True,
        )
        end_time = solution.t_events[0][0] if solution.t_events[0].size else solution.t[-1]
        while next_trace <= end_time:
            point = solution.sol(next_trace)
            record.trace_time.append(next_trace)
            record.trace_speed.append(point[1])
            record.trace_pressure.append(model.gas_pressure(point[0], point[2], point[3], sealed))
            record.trace_battery.append(model.battery_voltage(point[1]))
            if sealed:
                peak_pressure = max(peak_pressure, record.trace_pressure[-1])
            next_trace += TRACE_STEP
        if not solution.t_events[0].size:
            break
        time = end_time
        state = solution.y_events[0][0].copy()
        event_index += 1
        if event_index == len(cycle_events(engine)):
            event_index = 0
            cycle_start += geometry.CYCLE
        if name == "intake_close":
            sealed = True
            state[2] = AMBIENT_PRESSURE * geometry.volume(engine, state[0]) / (GAS_CONSTANT * AMBIENT_TEMPERATURE)
            state[3] = AMBIENT_TEMPERATURE
            intake_state = (state[2] / geometry.volume(engine, state[0]), state[3])
            burn_start = None
            peak_pressure = 0.0
        elif name == "exhaust_open":
            if burn_start is not None:
                pressure = model.gas_pressure(state[0], state[2], state[3], True)
                exhaust_temperature = state[3] * (AMBIENT_PRESSURE / pressure) ** ((HEAT_RATIO - 1) / HEAT_RATIO)
            else:
                exhaust_temperature = AMBIENT_TEMPERATURE
            sealed = False
            burn_start = None
        elif name == "compression_spark":
            density = state[2] / geometry.volume(engine, state[0])
            record.sparks.append(Spark(time, True, density / REFERENCE_DENSITY, state[3], state[1], jitter))
            if intake_state is not None:
                intake_density, intake_temperature = intake_state
                record.polytropic.append(1 + np.log(state[3] / intake_temperature) / np.log(density / intake_density))
                record.trapped.append(density / (intake_density * geometry.volume(engine, geometry.intake_close_angle(engine)) / geometry.volume(engine, geometry.compression_spark_angle(engine))))
            compression_count += 1
            if model.firing is not None and model.bike.fuel == "carburettor" and compression_count >= model.firing.first_compression:
                burn_start = state[0]
            jitter = np.radians(jitter_deg) * rng.standard_normal()
        elif name == "exhaust_spark":
            if record.trapped:
                record.peak_pressure.append(peak_pressure)
            gas_temperature = exhaust_temperature + exhaust_temperature_error
            density = AMBIENT_PRESSURE / (GAS_CONSTANT * gas_temperature)
            record.sparks.append(Spark(time, False, density / REFERENCE_DENSITY, gas_temperature, state[1], jitter))
            jitter = np.radians(jitter_deg) * rng.standard_normal()
    return record