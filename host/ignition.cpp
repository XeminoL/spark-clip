#include "ignition.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
const double kPiValue = 3.14159265358979323846;
}

double Ignition::rise_shape(double fraction) const {
    double clamped = std::min(1.0, std::max(0.0, fraction));
    return 0.5 * (1 - std::cos(kPiValue * clamped));
}

double Ignition::rise_fraction(double share) const {
    double clamped = std::min(1.0, std::max(0.0, share));
    return std::acos(1 - 2 * clamped) / kPiValue;
}

double AcCdi::open_circuit_kv(double rpm, double) const {
    return spec_.peak_kv * (1 - std::exp(-rpm / spec_.speed_scale_rpm));
}

double DcCdi::open_circuit_kv(double, double battery_volts) const {
    double share = (battery_volts - spec_.cutoff_v) / (spec_.full_v - spec_.cutoff_v);
    return spec_.peak_kv * std::min(1.0, std::max(0.0, share));
}

double Tci::open_circuit_kv(double, double battery_volts) const {
    return spec_.peak_kv * std::min(1.0, battery_volts / spec_.full_v);
}

std::unique_ptr<Ignition> make_ignition(const IgnitionSpec& spec) {
    if (spec.kind == "ac_cdi") return std::unique_ptr<Ignition>(new AcCdi(spec));
    if (spec.kind == "dc_cdi") return std::unique_ptr<Ignition>(new DcCdi(spec));
    if (spec.kind == "tci") return std::unique_ptr<Ignition>(new Tci(spec));
    if (spec.kind == "coil_on_plug") return std::unique_ptr<Ignition>(new CoilOnPlug(spec));
    throw std::runtime_error("unknown ignition " + spec.kind);
}