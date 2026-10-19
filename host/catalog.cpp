#include "catalog.hpp"

#include <fstream>
#include <stdexcept>

namespace {

const double kMillimetre = 1e-3;
const double kGram = 1e-3;

std::string trim(const std::string& text) {
    std::size_t first = text.find_first_not_of(" \t\r");
    std::size_t last = text.find_last_not_of(" \t\r");
    return first == std::string::npos ? "" : text.substr(first, last - first + 1);
}

using Section = std::map<std::string, std::string>;

std::map<std::string, Section> read_sections(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open " + path);
    std::map<std::string, Section> sections;
    std::string current;
    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty()) continue;
        if (line.front() == '[') {
            current = line.substr(1, line.size() - 2);
            sections[current];
            continue;
        }
        std::size_t equals = line.find('=');
        if (equals == std::string::npos) throw std::runtime_error("bad line: " + line);
        sections[current][trim(line.substr(0, equals))] = trim(line.substr(equals + 1));
    }
    return sections;
}

double number(const Section& section, const std::string& key, double fallback) {
    auto found = section.find(key);
    return found == section.end() ? fallback : std::stod(found->second);
}

double number(const Section& section, const std::string& key) {
    auto found = section.find(key);
    if (found == section.end()) throw std::runtime_error("missing " + key);
    return std::stod(found->second);
}

Bike read_bike(const std::string& name, const Section& s) {
    Bike bike;
    bike.name = name;
    bike.label = s.at("label");
    bike.fuel = s.at("fuel");
    bike.engine = {number(s, "bore_mm") * kMillimetre, number(s, "stroke_mm") * kMillimetre, number(s, "rod_mm") * kMillimetre,
                   number(s, "compression_ratio"), number(s, "intake_close_abdc_deg"), number(s, "exhaust_open_bbdc_deg"),
                   number(s, "spark_advance_deg")};
    bike.ignition = {s.at("ignition"), number(s, "ignition_peak_kv"), number(s, "ignition_rise_us"),
                     number(s, "ignition_speed_scale_rpm", 0.0), number(s, "ignition_cutoff_v", 0.0), number(s, "ignition_full_v", 0.0),
                     number(s, "plug_gap_mm") * kMillimetre};
    bike.rotating = {number(s, "rotor_radius_mm") * kMillimetre, number(s, "rotor_wall_mm") * kMillimetre,
                     number(s, "rotor_height_mm") * kMillimetre, number(s, "rotor_base_mm") * kMillimetre,
                     number(s, "hub_radius_mm") * kMillimetre, number(s, "hub_height_mm") * kMillimetre,
                     static_cast<int>(number(s, "magnet_count")), number(s, "magnet_thickness_mm") * kMillimetre,
                     number(s, "magnet_height_mm") * kMillimetre, number(s, "magnet_coverage"),
                     number(s, "web_radius_mm") * kMillimetre, number(s, "web_thickness_mm") * kMillimetre,
                     number(s, "pin_radius_mm") * kMillimetre, number(s, "pin_length_mm") * kMillimetre,
                     number(s, "rod_rotating_g") * kGram, number(s, "reciprocating_g") * kGram, number(s, "other_inertia_gm2") * 1e-3};
    bike.starter = {number(s, "motor_constant"), number(s, "armature_ohm"), number(s, "brush_drop_v"), number(s, "cable_ohm"),
                    number(s, "gear_ratio"), number(s, "gear_efficiency"), number(s, "armature_inertia_gm2") * 1e-3};
    bike.static_friction = number(s, "friction_nm");
    bike.viscous_friction = number(s, "viscous_nms");
    bike.stock_battery_resistance = number(s, "stock_battery_ohm");
    return bike;
}

}

std::map<std::string, Bike> load_bikes(const std::string& path) {
    std::map<std::string, Bike> bikes;
    for (const auto& entry : read_sections(path)) bikes[entry.first] = read_bike(entry.first, entry.second);
    return bikes;
}

AirConstants load_air(const std::string& path) {
    Section values = read_sections(path)[""];
    return {number(values, "ionisation_over_k"), number(values, "field_constant"), number(values, "tip_radius_mm") * kMillimetre};
}