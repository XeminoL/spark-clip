import numpy as np

CYCLE = 4 * np.pi
HEAD_AREA_FACTOR = 1.2


def piston_area(engine):
    return np.pi / 4 * engine.bore**2


def swept_volume(engine):
    return piston_area(engine) * engine.stroke


def clearance_volume(engine):
    return swept_volume(engine) / (engine.compression_ratio - 1)


def piston_travel(engine, theta):
    crank = engine.stroke / 2
    rod = engine.rod
    return crank * (1 - np.cos(theta)) + rod - np.sqrt(rod**2 - (crank * np.sin(theta)) ** 2)


def volume(engine, theta):
    return clearance_volume(engine) + piston_area(engine) * piston_travel(engine, theta)


def volume_rate(engine, theta):
    crank = engine.stroke / 2
    rod = engine.rod
    sine = np.sin(theta)
    lever = crank * sine + crank**2 * sine * np.cos(theta) / np.sqrt(rod**2 - (crank * sine) ** 2)
    return piston_area(engine) * lever


def wall_area(engine, theta):
    clearance_height = clearance_volume(engine) / piston_area(engine)
    return (1 + HEAD_AREA_FACTOR) * piston_area(engine) + np.pi * engine.bore * (clearance_height + piston_travel(engine, theta))


def intake_close_angle(engine):
    return 3 * np.pi + np.radians(engine.intake_close_abdc_deg)


def exhaust_open_angle(engine):
    return np.pi - np.radians(engine.exhaust_open_bbdc_deg)


def compression_spark_angle(engine):
    return CYCLE - np.radians(engine.spark_advance_deg)


def exhaust_spark_angle(engine):
    return 2 * np.pi - np.radians(engine.spark_advance_deg)


def is_sealed(engine, cycle_angle):
    return cycle_angle >= intake_close_angle(engine) or cycle_angle < exhaust_open_angle(engine)


def sealed_density_ratio(engine):
    return volume(engine, intake_close_angle(engine)) / volume(engine, compression_spark_angle(engine))