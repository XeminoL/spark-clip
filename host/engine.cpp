#include "engine.hpp"

#include <cmath>

namespace {

const double kHeadAreaFactor = 1.2;
const double kSteelDensity = 7850.0;
const double kFerriteDensity = 4900.0;

double travel(const EngineGeometry& engine, double theta) {
    double crank = engine.stroke / 2;
    double sine = std::sin(theta);
    return crank * (1 - std::cos(theta)) + engine.rod - std::sqrt(engine.rod * engine.rod - crank * crank * sine * sine);
}

double shell(double density, double inner, double outer, double height, double arc) {
    double mass = density * arc / 2 * (outer * outer - inner * inner) * height;
    return 0.5 * mass * (inner * inner + outer * outer);
}

}

double degrees_to_radians(double degrees) { return degrees * kPi / 180; }

double piston_area(const EngineGeometry& engine) { return kPi / 4 * engine.bore * engine.bore; }

double clearance_volume(const EngineGeometry& engine) {
    return piston_area(engine) * engine.stroke / (engine.compression_ratio - 1);
}

double cylinder_volume(const EngineGeometry& engine, double theta) {
    return clearance_volume(engine) + piston_area(engine) * travel(engine, theta);
}

double volume_rate(const EngineGeometry& engine, double theta) {
    double crank = engine.stroke / 2;
    double sine = std::sin(theta);
    double lever = crank * sine + crank * crank * sine * std::cos(theta) / std::sqrt(engine.rod * engine.rod - crank * crank * sine * sine);
    return piston_area(engine) * lever;
}

double wall_area(const EngineGeometry& engine, double theta) {
    double clearance_height = clearance_volume(engine) / piston_area(engine);
    return (1 + kHeadAreaFactor) * piston_area(engine) + kPi * engine.bore * (clearance_height + travel(engine, theta));
}

double intake_close_angle(const EngineGeometry& engine) { return 3 * kPi + degrees_to_radians(engine.intake_close_abdc_deg); }

double exhaust_open_angle(const EngineGeometry& engine) { return kPi - degrees_to_radians(engine.exhaust_open_bbdc_deg); }

double compression_spark_angle(const EngineGeometry& engine) { return kCycle - degrees_to_radians(engine.spark_advance_deg); }

double exhaust_spark_angle(const EngineGeometry& engine) { return 2 * kPi - degrees_to_radians(engine.spark_advance_deg); }

double sealed_density_ratio(const EngineGeometry& engine) {
    return cylinder_volume(engine, intake_close_angle(engine)) / cylinder_volume(engine, compression_spark_angle(engine));
}

double total_inertia(const Bike& bike) {
    const RotatingParts& r = bike.rotating;
    double rim_inner = r.rotor_radius - r.rotor_wall;
    double rotor = shell(kSteelDensity, rim_inner, r.rotor_radius, r.rotor_height, 2 * kPi) +
                   shell(kSteelDensity, r.hub_radius, rim_inner, r.rotor_base, 2 * kPi) +
                   shell(kSteelDensity, 0.0, r.hub_radius, r.hub_height, 2 * kPi) +
                   shell(kFerriteDensity, rim_inner - r.magnet_thickness, rim_inner, r.magnet_height, 2 * kPi * r.magnet_coverage);
    double throw_radius = bike.engine.stroke / 2;
    double webs = 2 * shell(kSteelDensity, 0.0, r.web_radius, r.web_thickness, 2 * kPi);
    double pin_mass = kSteelDensity * kPi * r.pin_radius * r.pin_radius * r.pin_length;
    double pin = pin_mass * (0.5 * r.pin_radius * r.pin_radius + throw_radius * throw_radius);
    double rod = r.rod_rotating_mass * throw_radius * throw_radius;
    double reciprocating = 0.5 * r.reciprocating_mass * throw_radius * throw_radius;
    double starter = bike.starter.gear_ratio * bike.starter.gear_ratio * bike.starter.armature_inertia;
    return rotor + webs + pin + rod + reciprocating + r.other_inertia + starter;
}