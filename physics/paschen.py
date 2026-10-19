from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

FIT_PATH = Path(__file__).resolve().parent.parent / "data" / "air.txt"
TORR_PER_BAR = 750.06
REFERENCE_DENSITY = 1.0e5 / (287.05 * 293.15)
DAKIN_LINEAR_KV = 24.36
DAKIN_ROOT_KV = 6.72
FIT_RANGE_BAR_CM = (0.03, 1.5)
FIT_POINTS = 40
START_GUESS = (15.0 / np.log(1 + 1 / 0.01), 365.0)
GAUSS_NEWTON_STEPS = 60
TIP_RADIUS = 1.0e-3
QUADRATURE_ORDER = 64


@dataclass(frozen=True)
class AirConstants:
    ionisation_over_k: float
    field_constant: float


def dakin_kv(bar_cm):
    return DAKIN_LINEAR_KV * bar_cm + DAKIN_ROOT_KV * np.sqrt(bar_cm)


def uniform_kv(constants, bar_cm):
    torr_cm = bar_cm * TORR_PER_BAR
    return constants.field_constant * torr_cm / np.log(constants.ionisation_over_k * torr_cm) / 1000


def full_paschen_kv(ionisation, field_constant, secondary_yield, bar_cm):
    torr_cm = bar_cm * TORR_PER_BAR
    return field_constant * torr_cm / (np.log(ionisation * torr_cm) - np.log(np.log(1 + 1 / secondary_yield))) / 1000


def relative_residuals(parameters, bar_cm, measured_kv):
    return uniform_kv(AirConstants(*parameters), bar_cm) / measured_kv - 1


def numeric_jacobian(function, parameters, step=1e-6):
    base = function(parameters)
    columns = []
    for index in range(len(parameters)):
        shifted = np.array(parameters, dtype=float)
        delta = step * max(abs(shifted[index]), 1.0)
        shifted[index] += delta
        columns.append((function(shifted) - base) / delta)
    return np.column_stack(columns)


def fit_air(bar_cm=None):
    if bar_cm is None:
        bar_cm = np.geomspace(*FIT_RANGE_BAR_CM, FIT_POINTS)
    measured = dakin_kv(bar_cm)
    parameters = np.array(START_GUESS)
    residual = lambda p: relative_residuals(p, bar_cm, measured)
    for _ in range(GAUSS_NEWTON_STEPS):
        jacobian = numeric_jacobian(residual, parameters)
        step, *_ = np.linalg.lstsq(jacobian, -residual(parameters), rcond=None)
        parameters = parameters + step
        if np.max(np.abs(step / parameters)) < 1e-12:
            break
    return AirConstants(*parameters), residual(parameters)


def three_parameter_singular_values(bar_cm=None):
    if bar_cm is None:
        bar_cm = np.geomspace(*FIT_RANGE_BAR_CM, FIT_POINTS)
    measured = dakin_kv(bar_cm)
    residual = lambda p: full_paschen_kv(p[0], p[1], p[2], bar_cm) / measured - 1
    jacobian = numeric_jacobian(residual, np.array([15.0, 365.0, 0.01]), step=1e-7)
    scaled = jacobian * np.array([15.0, 365.0, 0.01])
    return np.linalg.svd(scaled, compute_uv=False)


def save_air(constants, path=FIT_PATH):
    path.write_text(f"ionisation_over_k = {constants.ionisation_over_k:.9g}\nfield_constant = {constants.field_constant:.9g}\ntip_radius_mm = {TIP_RADIUS * 1e3:.6g}", encoding="utf-8")


def load_air(path=FIT_PATH):
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, value = line.split("=")
        values[key.strip()] = float(value)
    return AirConstants(values["ionisation_over_k"], values["field_constant"])


def axial_field_shape(gap, tip_radius):
    focal = np.sqrt(gap * (gap + tip_radius))
    nodes, weights = np.polynomial.legendre.leggauss(QUADRATURE_ORDER)
    heights = 0.5 * gap * (nodes + 1)
    field_per_volt = 1 / (focal * np.arctanh(gap / focal) * (1 - (heights / focal) ** 2))
    return field_per_volt, 0.5 * gap * weights


def townsend_sum(constants, volts, density_ratio, gap, tip_radius):
    field_per_volt, weights = axial_field_shape(gap, tip_radius)
    torr = density_ratio * TORR_PER_BAR
    field_v_per_cm = volts * field_per_volt / 100
    alpha_per_cm = constants.ionisation_over_k * torr * np.exp(-constants.field_constant * torr / field_v_per_cm)
    return np.sum(alpha_per_cm * weights * 100)


def plug_kv(constants, density_ratio, gap, tip_radius=TIP_RADIUS):
    upper = uniform_kv(constants, density_ratio * gap * 100) * 1000 * 3
    volts = brentq(lambda v: townsend_sum(constants, v, density_ratio, gap, tip_radius) - 1, 10.0, upper, xtol=1e-6)
    return volts / 1000


def density_from_kv(constants, kv, gap, tip_radius=TIP_RADIUS):
    return brentq(lambda rho: plug_kv(constants, rho, gap, tip_radius) - kv, 0.05, 60.0, xtol=1e-10)