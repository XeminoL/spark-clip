#include "cranking.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "engine.hpp"

namespace {

const double kHeatRatio = 1.4;
const double kCv = kGasConstant / (kHeatRatio - 1);
const double kCp = kCv + kGasConstant;
const double kFrictionSmoothing = 0.5;
const double kStep = 50e-6;
const double kStallSpeed = 0.5;
const double kStallSeconds = 0.3;

using State = std::array<double, 4>;

enum class Event { IntakeClose, CompressionSpark, ExhaustOpen, ExhaustSpark };

struct CycleEvent {
    double angle;
    Event event;
};

std::array<CycleEvent, 4> cycle_events(const EngineGeometry& engine) {
    std::array<CycleEvent, 4> events = {{{intake_close_angle(engine), Event::IntakeClose},
                                         {compression_spark_angle(engine), Event::CompressionSpark},
                                         {exhaust_open_angle(engine), Event::ExhaustOpen},
                                         {exhaust_spark_angle(engine), Event::ExhaustSpark}}};
    std::sort(events.begin(), events.end(), [](const CycleEvent& a, const CycleEvent& b) { return a.angle < b.angle; });
    return events;
}

double orifice_flow(double upstream_pressure, double upstream_temperature, double downstream_pressure, double area) {
    double ratio = downstream_pressure / upstream_pressure;
    double critical = std::pow(2 / (kHeatRatio + 1), kHeatRatio / (kHeatRatio - 1));
    double shape;
    if (ratio <= critical)
        shape = std::sqrt(kHeatRatio) * std::pow(2 / (kHeatRatio + 1), (kHeatRatio + 1) / (2 * (kHeatRatio - 1)));
    else
        shape = std::sqrt(2 * kHeatRatio / (kHeatRatio - 1) * (std::pow(ratio, 2 / kHeatRatio) - std::pow(ratio, (kHeatRatio + 1) / kHeatRatio)));
    return area * upstream_pressure * shape / std::sqrt(kGasConstant * upstream_temperature);
}

class Cylinder {
public:
    explicit Cylinder(const CrankingSetup& setup) : setup_(setup), engine_(setup.bike.engine) {}

    double pressure(const State& s, bool sealed) const {
        if (!sealed) return kAmbientPressure;
        return s[2] * kGasConstant * s[3] / cylinder_volume(engine_, s[0]);
    }

    State rates(const State& s, bool sealed) const {
        double theta = s[0];
        double speed = s[1];
        double p = pressure(s, sealed);
        double current = starter_current(setup_, speed);
        const StarterSpec& starter = setup_.bike.starter;
        double motor = starter.gear_efficiency * starter.motor_constant * starter.gear_ratio * current;
        double friction = setup_.bike.static_friction * std::tanh(speed / kFrictionSmoothing) + setup_.bike.viscous_friction * speed;
        double acceleration = (motor + (p - kAmbientPressure) * volume_rate(engine_, theta) - friction) / setup_.inertia;
        if (!sealed) return {speed, acceleration, 0.0, 0.0};
        double temperature = s[3];
        double heat_loss = setup_.seal.heat_transfer * wall_area(engine_, theta) * (temperature - setup_.ambient_temperature);
        double outflow = p >= kAmbientPressure ? orifice_flow(p, temperature, kAmbientPressure, setup_.seal.leak_area)
                                               : -orifice_flow(kAmbientPressure, setup_.ambient_temperature, p, setup_.seal.leak_area);
        double flow_temperature = outflow > 0 ? temperature : setup_.ambient_temperature;
        double mass_change = -outflow;
        double temperature_change = (-p * volume_rate(engine_, theta) * speed - heat_loss + (kCp * flow_temperature - kCv * temperature) * mass_change) / (s[2] * kCv);
        return {speed, acceleration, mass_change, temperature_change};
    }

    State step(const State& s, bool sealed, double h) const {
        State k1 = rates(s, sealed);
        State k2 = rates(shifted(s, k1, h / 2), sealed);
        State k3 = rates(shifted(s, k2, h / 2), sealed);
        State k4 = rates(shifted(s, k3, h), sealed);
        State next;
        for (int i = 0; i < 4; ++i) next[i] = s[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        return next;
    }

    double fresh_mass(double theta) const {
        return kAmbientPressure * cylinder_volume(engine_, theta) / (kGasConstant * setup_.ambient_temperature);
    }

private:
    static State shifted(const State& s, const State& slope, double h) {
        State moved;
        for (int i = 0; i < 4; ++i) moved[i] = s[i] + h * slope[i];
        return moved;
    }

    const CrankingSetup& setup_;
    const EngineGeometry& engine_;
};

}

double starter_current(const CrankingSetup& setup, double speed) {
    const StarterSpec& starter = setup.bike.starter;
    double back_emf = starter.motor_constant * starter.gear_ratio * speed;
    double resistance = setup.battery.internal_resistance + starter.armature_resistance + starter.cable_resistance;
    return std::max(0.0, (setup.battery.emf - starter.brush_drop - back_emf) / resistance);
}

CrankingTrace run_cranking(const CrankingSetup& setup, double start_theta, double start_speed, double duration) {
    const EngineGeometry& engine = setup.bike.engine;
    Cylinder cylinder(setup);
    std::array<CycleEvent, 4> events = cycle_events(engine);
    CrankingTrace trace;
    double cycle_start = std::floor(start_theta / kCycle) * kCycle;
    std::size_t next = 0;
    while (next < events.size() && cycle_start + events[next].angle <= start_theta) ++next;
    if (next == events.size()) {
        next = 0;
        cycle_start += kCycle;
    }
    double phase = start_theta - std::floor(start_theta / kCycle) * kCycle;
    bool sealed = phase >= intake_close_angle(engine) || phase < exhaust_open_angle(engine);
    State state = {start_theta, start_speed, cylinder.fresh_mass(start_theta), setup.ambient_temperature};
    double intake_density = 0.0;
    double intake_temperature = setup.ambient_temperature;
    double peak = 0.0;
    double time = 0.0;
    double slow_time = 0.0;
    double volts_sum = 0.0;
    double current_sum = 0.0;
    double volts_min = setup.battery.emf;
    long steps = 0;
    while (time < duration) {
        double target = cycle_start + events[next].angle;
        State moved = cylinder.step(state, sealed, kStep);
        double h = kStep;
        bool hit = moved[0] >= target;
        if (hit) {
            h = kStep * (target - state[0]) / (moved[0] - state[0]);
            moved = cylinder.step(state, sealed, h);
        }
        time += h;
        state = moved;
        if (sealed) peak = std::max(peak, cylinder.pressure(state, true));
        double current = starter_current(setup, state[1]);
        double volts = setup.battery.emf - current * setup.battery.internal_resistance;
        volts_sum += volts;
        current_sum += current;
        volts_min = std::min(volts_min, volts);
        ++steps;
        slow_time = state[1] < kStallSpeed ? slow_time + h : 0.0;
        if (slow_time > kStallSeconds) {
            trace.stalled = true;
            break;
        }
        if (!hit) continue;
        Event event = events[next].event;
        if (event == Event::IntakeClose) {
            sealed = true;
            state[2] = cylinder.fresh_mass(state[0]);
            state[3] = setup.ambient_temperature;
            intake_density = state[2] / cylinder_volume(engine, state[0]);
            intake_temperature = state[3];
            peak = 0.0;
        } else if (event == Event::ExhaustOpen) {
            if (intake_density > 0) trace.peak_pressures.push_back(peak);
            sealed = false;
        } else if (event == Event::CompressionSpark) {
            double density = state[2] / cylinder_volume(engine, state[0]);
            trace.sparks.push_back({time, true, density / kReferenceDensity});
            if (intake_density > 0) {
                trace.polytropic.push_back(1 + std::log(state[3] / intake_temperature) / std::log(density / intake_density));
                trace.trapped.push_back(density / (intake_density * sealed_density_ratio(engine)));
            }
        } else {
            double density = kAmbientPressure / (kGasConstant * setup.ambient_temperature);
            trace.sparks.push_back({time, false, density / kReferenceDensity});
        }
        ++next;
        if (next == events.size()) {
            next = 0;
            cycle_start += kCycle;
        }
    }
    trace.mean_battery_volts = steps ? volts_sum / steps : setup.battery.emf;
    trace.mean_current = steps ? current_sum / steps : 0.0;
    trace.min_battery_volts = volts_min;
    return trace;
}