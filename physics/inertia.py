from dataclasses import dataclass

import numpy as np
from scipy.integrate import tplquad

STEEL_DENSITY = 7850.0
FERRITE_DENSITY = 4900.0


@dataclass(frozen=True)
class InertiaBudget:
    rotor: float
    webs: float
    crank_pin: float
    rod_big_end: float
    reciprocating: float
    other: float
    starter: float

    @property
    def total(self):
        return self.rotor + self.webs + self.crank_pin + self.rod_big_end + self.reciprocating + self.other + self.starter


def shell_by_integral(density, inner, outer, height, start_angle=0.0, end_angle=2 * np.pi):
    moment, _ = tplquad(lambda z, phi, r: density * r**3, inner, outer, start_angle, end_angle, 0.0, height)
    return moment


def shell_closed_form(density, inner, outer, height, arc=2 * np.pi):
    mass = density * arc / 2 * (outer**2 - inner**2) * height
    return 0.5 * mass * (inner**2 + outer**2)


def rotor_pieces(rotating):
    rim_inner = rotating.rotor_radius - rotating.rotor_wall
    magnet_inner = rim_inner - rotating.magnet_thickness
    return [
        (STEEL_DENSITY, rim_inner, rotating.rotor_radius, rotating.rotor_height, 2 * np.pi),
        (STEEL_DENSITY, rotating.hub_radius, rim_inner, rotating.rotor_base, 2 * np.pi),
        (STEEL_DENSITY, 0.0, rotating.hub_radius, rotating.hub_height, 2 * np.pi),
        (FERRITE_DENSITY, magnet_inner, rim_inner, rotating.magnet_height, 2 * np.pi * rotating.magnet_coverage),
    ]


def rotor_by_integral(rotating):
    total = 0.0
    for density, inner, outer, height, arc in rotor_pieces(rotating)[:3]:
        total += shell_by_integral(density, inner, outer, height)
    density, inner, outer, height, arc = rotor_pieces(rotating)[3]
    pitch = 2 * np.pi / rotating.magnet_count
    width = pitch * rotating.magnet_coverage
    for index in range(rotating.magnet_count):
        start = index * pitch
        total += shell_by_integral(density, inner, outer, height, start, start + width)
    return total


def rotor_closed_form(rotating):
    return sum(shell_closed_form(*piece) for piece in rotor_pieces(rotating))


def inertia_budget(bike, rotor_method=rotor_closed_form):
    rotating = bike.rotating
    throw = bike.engine.stroke / 2
    web = shell_closed_form(STEEL_DENSITY, 0.0, rotating.web_radius, rotating.web_thickness)
    pin_mass = STEEL_DENSITY * np.pi * rotating.pin_radius**2 * rotating.pin_length
    return InertiaBudget(
        rotor=rotor_method(rotating),
        webs=2 * web,
        crank_pin=pin_mass * (0.5 * rotating.pin_radius**2 + throw**2),
        rod_big_end=rotating.rod_rotating_mass * throw**2,
        reciprocating=0.5 * rotating.reciprocating_mass * throw**2,
        other=rotating.other_inertia,
        starter=bike.starter.gear_ratio**2 * bike.starter.armature_inertia,
    )