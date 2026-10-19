#pragma once

#include <map>
#include <string>

struct EngineGeometry {
    double bore;
    double stroke;
    double rod;
    double compression_ratio;
    double intake_close_abdc_deg;
    double exhaust_open_bbdc_deg;
    double spark_advance_deg;
};

struct IgnitionSpec {
    std::string kind;
    double peak_kv;
    double rise_us;
    double speed_scale_rpm;
    double cutoff_v;
    double full_v;
    double plug_gap;
};

struct RotatingParts {
    double rotor_radius;
    double rotor_wall;
    double rotor_height;
    double rotor_base;
    double hub_radius;
    double hub_height;
    int magnet_count;
    double magnet_thickness;
    double magnet_height;
    double magnet_coverage;
    double web_radius;
    double web_thickness;
    double pin_radius;
    double pin_length;
    double rod_rotating_mass;
    double reciprocating_mass;
    double other_inertia;
};

struct StarterSpec {
    double motor_constant;
    double armature_resistance;
    double brush_drop;
    double cable_resistance;
    double gear_ratio;
    double gear_efficiency;
    double armature_inertia;
};

struct Bike {
    std::string name;
    std::string label;
    std::string fuel;
    EngineGeometry engine;
    IgnitionSpec ignition;
    RotatingParts rotating;
    StarterSpec starter;
    double static_friction;
    double viscous_friction;
    double stock_battery_resistance;
};

struct AirConstants {
    double ionisation_over_k;
    double field_constant;
    double tip_radius;
};

std::map<std::string, Bike> load_bikes(const std::string& path);
AirConstants load_air(const std::string& path);