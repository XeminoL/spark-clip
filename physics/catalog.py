import configparser
from dataclasses import dataclass, replace
from pathlib import Path

CATALOG_PATH = Path(__file__).resolve().parent.parent / "data" / "bikes.txt"
MM = 1e-3
GRAM = 1e-3
GRAM_M2 = 1e-3


@dataclass(frozen=True)
class Engine:
    bore: float
    stroke: float
    rod: float
    compression_ratio: float
    intake_close_abdc_deg: float
    exhaust_open_bbdc_deg: float
    spark_advance_deg: float


@dataclass(frozen=True)
class Ignition:
    kind: str
    peak_kv: float
    rise_us: float
    speed_scale_rpm: float
    cutoff_v: float
    full_v: float
    burn_kv: float
    burn_us: float
    plug_gap: float


@dataclass(frozen=True)
class Rotating:
    rotor_radius: float
    rotor_wall: float
    rotor_height: float
    rotor_base: float
    hub_radius: float
    hub_height: float
    magnet_count: int
    magnet_thickness: float
    magnet_height: float
    magnet_coverage: float
    web_radius: float
    web_thickness: float
    pin_radius: float
    pin_length: float
    rod_rotating_mass: float
    reciprocating_mass: float
    other_inertia: float


@dataclass(frozen=True)
class Starter:
    motor_constant: float
    armature_resistance: float
    brush_drop: float
    cable_resistance: float
    gear_ratio: float
    gear_efficiency: float
    armature_inertia: float


@dataclass(frozen=True)
class Friction:
    static_torque: float
    viscous: float


@dataclass(frozen=True)
class Bike:
    name: str
    label: str
    fuel: str
    engine: Engine
    ignition: Ignition
    rotating: Rotating
    starter: Starter
    friction: Friction

    def with_changes(self, **parts):
        return replace(self, **parts)


def read_bike(section):
    number = section.getfloat
    engine = Engine(
        bore=number("bore_mm") * MM,
        stroke=number("stroke_mm") * MM,
        rod=number("rod_mm") * MM,
        compression_ratio=number("compression_ratio"),
        intake_close_abdc_deg=number("intake_close_abdc_deg"),
        exhaust_open_bbdc_deg=number("exhaust_open_bbdc_deg"),
        spark_advance_deg=number("spark_advance_deg"),
    )
    ignition = Ignition(
        kind=section["ignition"],
        peak_kv=number("ignition_peak_kv"),
        rise_us=number("ignition_rise_us"),
        speed_scale_rpm=number("ignition_speed_scale_rpm", fallback=0.0),
        cutoff_v=number("ignition_cutoff_v", fallback=0.0),
        full_v=number("ignition_full_v", fallback=0.0),
        burn_kv=number("burn_kv"),
        burn_us=number("burn_us"),
        plug_gap=number("plug_gap_mm") * MM,
    )
    rotating = Rotating(
        rotor_radius=number("rotor_radius_mm") * MM,
        rotor_wall=number("rotor_wall_mm") * MM,
        rotor_height=number("rotor_height_mm") * MM,
        rotor_base=number("rotor_base_mm") * MM,
        hub_radius=number("hub_radius_mm") * MM,
        hub_height=number("hub_height_mm") * MM,
        magnet_count=section.getint("magnet_count"),
        magnet_thickness=number("magnet_thickness_mm") * MM,
        magnet_height=number("magnet_height_mm") * MM,
        magnet_coverage=number("magnet_coverage"),
        web_radius=number("web_radius_mm") * MM,
        web_thickness=number("web_thickness_mm") * MM,
        pin_radius=number("pin_radius_mm") * MM,
        pin_length=number("pin_length_mm") * MM,
        rod_rotating_mass=number("rod_rotating_g") * GRAM,
        reciprocating_mass=number("reciprocating_g") * GRAM,
        other_inertia=number("other_inertia_gm2") * GRAM_M2,
    )
    starter = Starter(
        motor_constant=number("motor_constant"),
        armature_resistance=number("armature_ohm"),
        brush_drop=number("brush_drop_v"),
        cable_resistance=number("cable_ohm"),
        gear_ratio=number("gear_ratio"),
        gear_efficiency=number("gear_efficiency"),
        armature_inertia=number("armature_inertia_gm2") * GRAM_M2,
    )
    friction = Friction(static_torque=number("friction_nm"), viscous=number("viscous_nms"))
    return Bike(section.name, section["label"], section["fuel"], engine, ignition, rotating, starter, friction)


def load_bikes(path=CATALOG_PATH):
    parser = configparser.ConfigParser()
    parser.read(path, encoding="utf-8")
    return {name: read_bike(parser[name]) for name in parser.sections()}