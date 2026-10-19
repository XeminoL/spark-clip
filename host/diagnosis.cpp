#include "diagnosis.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>

#include "cranking.hpp"
#include "engine.hpp"
#include "fit.hpp"
#include "ignition.hpp"
#include "paschen.hpp"
#include "sparks.hpp"

namespace {

const double kPulserJitterDeg = 0.3;
const double kPerSparkDensityScatter = 0.06;
const double kMissingSparkPenalty = 1e5;
const double kScanLowVolts = 8.0;
const double kScanHighVolts = 18.0;
const double kScanStepVolts = 0.25;
const double kFineScanStepVolts = 0.02;
const double kRunMargin = 0.05;
const int kMinimumCompressionSparks = 3;
const int kFitIterations = 40;
const int kNuisanceIterations = 12;
const double kReferenceSeconds = 2.5;
const double kReferenceStartDeg = 100.0;
const Battery kReferenceBattery = {12.8, 0.015};
const CylinderSeal kNewEngine = {0.02e-6, 60.0};
const double kBatteryResistanceSpread = 0.01;
const double kNuisanceProbe = 0.25;
const double kBatteryGood = 10.5;
const double kBatteryWeak = 9.6;
const double kCompressionGood = 85.0;
const double kCompressionWorn = 70.0;
const double kReserveMarginal = 1.15;
const double kMisfireShare = 0.3;
const double kWornGapExtra = 0.25e-3;
const double kCoilSpread = 0.05;
const double kCoilVoltsSpread = 0.5;
const double kChiSquareLimit = 25.0;
const double kConfidence = 2.0;
const double kOutOfRange = 1e7;
const double kDeltaStepCap = 0.3;
const double kHeatUnit = 100.0;
const double kSpeedUnit = 50.0;

const std::size_t kParameterCount = 4;
using Parameters = Vec<kParameterCount>;

struct Outputs {
    double cranking_volts;
    double cranking_amps;
    double gauge_bar;
    double percent_of_new;
    double polytropic;
    double leak_mm2;
};

const int kOutputCount = 6;

std::array<double, kOutputCount> as_array(const Outputs& o) {
    return {o.cranking_volts, o.cranking_amps, o.gauge_bar, o.percent_of_new, o.polytropic, o.leak_mm2};
}

struct Measurement {
    std::vector<double> times;
    std::vector<bool> compression;
    std::vector<double> density;
    std::vector<double> density_sigma;
};

struct Nuisance {
    double sigma;
    std::function<void(Bike&, double&, double)> apply;
};

double uniform_sigma(double half_width) { return half_width / std::sqrt(3.0); }

std::vector<Nuisance> nuisances() {
    std::vector<Nuisance> list;
    list.push_back({uniform_sigma(0.04), [](Bike& b, double&, double s) { b.rotating.other_inertia += s * total_inertia(b); }});
    list.push_back({uniform_sigma(0.10), [](Bike& b, double&, double s) { b.static_friction *= 1 + s; }});
    list.push_back({uniform_sigma(0.10), [](Bike& b, double&, double s) { b.viscous_friction *= 1 + s; }});
    list.push_back({uniform_sigma(0.03), [](Bike& b, double&, double s) { b.starter.motor_constant *= 1 + s; }});
    list.push_back({uniform_sigma(0.05), [](Bike& b, double&, double s) { b.starter.armature_resistance *= 1 + s; }});
    list.push_back({uniform_sigma(0.7), [](Bike& b, double&, double s) { b.engine.spark_advance_deg += s; }});
    list.push_back({kBatteryResistanceSpread, [](Bike& b, double&, double s) { b.stock_battery_resistance += s; }});
    list.push_back({5.0, [](Bike&, double& ambient, double s) { ambient += s; }});
    return list;
}

CrankingSetup make_setup(const Bike& bike, const Parameters& p, double ambient) {
    return {bike, total_inertia(bike), {p[0], bike.stock_battery_resistance}, {p[1] * 1e-6, p[2] * kHeatUnit}, ambient};
}

bool plausible(const Parameters& p) {
    return p[0] > 6 && p[0] < 20 && p[1] >= 0 && p[1] < 5 && p[2] > 0.02 && p[2] < 10 && p[3] > 0.01;
}

std::size_t residual_count(const Measurement& m) {
    std::size_t count = m.times.size() - 1;
    for (std::size_t k = 1; k < m.density.size(); ++k)
        if (m.density[k] > 0) ++count;
    return count;
}

std::vector<double> residuals_for(const Bike& bike, const Parameters& p, double ambient, const Measurement& m) {
    if (!plausible(p)) return std::vector<double>(residual_count(m), kOutOfRange);
    CrankingSetup setup = make_setup(bike, p, ambient);
    double duration = m.times.back() + kRunMargin;
    CrankingTrace trace = run_cranking(setup, exhaust_spark_angle(bike.engine), p[3] * kSpeedUnit, duration);
    std::vector<double> values;
    double jitter = degrees_to_radians(kPulserJitterDeg);
    for (std::size_t k = 1; k < m.times.size(); ++k) {
        double interval = m.times[k] - m.times[k - 1];
        double sigma = jitter * interval / (2 * kPi);
        if (k - 1 >= trace.sparks.size() || trace.sparks[k - 1].compression != m.compression[k]) {
            values.push_back(kMissingSparkPenalty);
            if (m.density[k] > 0) values.push_back(kMissingSparkPenalty);
            continue;
        }
        values.push_back((trace.sparks[k - 1].time - m.times[k]) / sigma);
        if (m.density[k] > 0) values.push_back((trace.sparks[k - 1].density_ratio - m.density[k]) / m.density_sigma[k]);
    }
    return values;
}

FitResult<kParameterCount> fit_cranking(const Bike& bike, double ambient, const Measurement& m, const Parameters& start, int iterations) {
    Parameters steps;
    steps[0] = 0.005;
    steps[1] = 0.002;
    steps[2] = 0.005;
    steps[3] = 0.002;
    auto residuals = [&](const Parameters& p) { return residuals_for(bike, p, ambient, m); };
    return levenberg_marquardt<kParameterCount>(residuals, start, steps, iterations);
}

double scan_volts(const Bike& bike, double ambient, const Measurement& m, Parameters start, double low, double high, double step) {
    double best_volts = high;
    double best_cost = INFINITY;
    for (double volts = low; volts <= high + 1e-9; volts += step) {
        start[0] = volts;
        double cost = sum_of_squares(residuals_for(bike, start, ambient, m));
        if (cost < best_cost) {
            best_cost = cost;
            best_volts = volts;
        }
    }
    return best_volts;
}

double best_start_volts(const Bike& bike, double ambient, const Measurement& m, const Parameters& start) {
    double coarse = scan_volts(bike, ambient, m, start, kScanLowVolts, kScanHighVolts, kScanStepVolts);
    return scan_volts(bike, ambient, m, start, coarse - kScanStepVolts, coarse + kScanStepVolts, kFineScanStepVolts);
}

double mean_tail(const std::vector<double>& values, std::size_t count) {
    if (values.empty()) return 0.0;
    std::size_t used = std::min(count, values.size());
    double sum = 0.0;
    for (std::size_t i = values.size() - used; i < values.size(); ++i) sum += values[i];
    return sum / used;
}

double reference_gauge(const Bike& bike, const CylinderSeal& seal, double ambient, double* polytropic) {
    CrankingSetup setup = {bike, total_inertia(bike), kReferenceBattery, seal, ambient};
    CrankingTrace trace = run_cranking(setup, degrees_to_radians(kReferenceStartDeg), 0.05, kReferenceSeconds);
    if (polytropic) *polytropic = mean_tail(trace.polytropic, 2);
    return (mean_tail(trace.peak_pressures, 2) - kAmbientPressure) / 1e5;
}

Outputs evaluate(const Bike& bike, const Parameters& p, double ambient, const Measurement& m, double new_gauge) {
    CrankingSetup setup = make_setup(bike, p, ambient);
    CrankingTrace trace = run_cranking(setup, exhaust_spark_angle(bike.engine), p[3] * kSpeedUnit, m.times.back() + kRunMargin);
    Outputs o;
    o.cranking_volts = trace.mean_battery_volts;
    o.cranking_amps = trace.mean_current;
    o.gauge_bar = reference_gauge(bike, setup.seal, ambient, &o.polytropic);
    o.percent_of_new = 100 * o.gauge_bar / new_gauge;
    o.leak_mm2 = p[1];
    return o;
}

std::array<double, kOutputCount> statistical_sigma(const Bike& bike, const FitResult<kParameterCount>& fit, double ambient, const Measurement& m,
                                                   double new_gauge) {
    std::array<std::array<double, kParameterCount>, kOutputCount> slopes{};
    for (std::size_t i = 0; i < kParameterCount; ++i) {
        double step = std::min(std::max(fit.sigma(i), 1e-3), kDeltaStepCap);
        Parameters up = fit.parameters;
        Parameters down = fit.parameters;
        up[i] += step;
        down[i] -= step;
        std::array<double, kOutputCount> high = as_array(evaluate(bike, up, ambient, m, new_gauge));
        std::array<double, kOutputCount> low = as_array(evaluate(bike, down, ambient, m, new_gauge));
        for (int k = 0; k < kOutputCount; ++k) slopes[k][i] = (high[k] - low[k]) / (2 * step);
    }
    std::array<double, kOutputCount> sigma{};
    for (int k = 0; k < kOutputCount; ++k) {
        double variance = 0.0;
        for (std::size_t i = 0; i < kParameterCount; ++i)
            for (std::size_t j = 0; j < kParameterCount; ++j) variance += slopes[k][i] * fit.covariance(i, j) * slopes[k][j];
        sigma[k] = std::sqrt(std::max(0.0, variance));
    }
    return sigma;
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

double spread_of(const std::vector<double>& values) {
    if (values.size() < 2) return 0.0;
    double mean = 0.0;
    for (double v : values) mean += v;
    mean /= values.size();
    double sum = 0.0;
    for (double v : values) sum += (v - mean) * (v - mean);
    return std::sqrt(sum / (values.size() - 1));
}

double exhaust_density(double ambient) { return kAmbientPressure / (kGasConstant * ambient) / kReferenceDensity; }

Measurement measure(const SparkTrain& train, const AirConstants& air, double gap, double ambient, bool use_ratio) {
    Measurement m;
    double origin = train.events[train.first_exhaust].record.seconds();
    for (std::size_t i = train.first_exhaust; i < train.usable_end; ++i) {
        const SparkEvent& event = train.events[i];
        m.times.push_back(event.record.seconds() - origin);
        m.compression.push_back(event.compression);
        double density = 0.0;
        double sigma = 0.0;
        if (use_ratio && event.compression && event.record.sparked) {
            std::vector<double> neighbours;
            if (i > train.first_exhaust) neighbours.push_back(train.events[i - 1].peak);
            if (i + 1 < train.usable_end) neighbours.push_back(train.events[i + 1].peak);
            double exhaust_peak = 0.0;
            for (double value : neighbours) exhaust_peak += value / neighbours.size();
            double ratio = event.peak / exhaust_peak;
            density = density_from_ratio(air, ratio, exhaust_density(ambient), gap);
            if (!std::isfinite(density)) density = 0.0;
            sigma = kPerSparkDensityScatter * density;
        }
        m.density.push_back(density);
        m.density_sigma.push_back(sigma);
    }
    return m;
}

Estimate known(double value, double sigma) { return {value, sigma, true}; }

Verdict grade(const Estimate& e, double good, double weak, const std::string& unit, const std::string& good_word,
              const std::string& middle_word, const std::string& bad_word) {
    std::ostringstream reason;
    reason.precision(3);
    reason << e.value << " " << unit << " +/- " << kConfidence * e.sigma;
    double low = e.value - kConfidence * e.sigma;
    double high = e.value + kConfidence * e.sigma;
    auto band = [&](double v) { return v >= good ? 2 : (v >= weak ? 1 : 0); };
    if (band(low) != band(high)) return {"unsure", reason.str()};
    const std::string words[3] = {bad_word, middle_word, good_word};
    return {words[band(e.value)], reason.str()};
}

std::string summarise(const Diagnosis& d) {
    if (d.misfires == d.compression_sparks && d.misfires > 0) return "no spark at compression, so compression cannot be read; charge the battery and test again";
    std::vector<std::string> causes;
    if (d.battery_verdict.state == "weak" || d.battery_verdict.state == "flat") causes.push_back("battery");
    if (d.ignition_verdict.state == "no spark" || d.ignition_verdict.state == "marginal") causes.push_back("ignition");
    if (d.compression_verdict.state == "worn" || d.compression_verdict.state == "low") causes.push_back("compression");
    bool unsure = d.battery_verdict.state == "unsure" || d.compression_verdict.state == "unsure" || d.ignition_verdict.state == "unsure";
    if (causes.empty()) return unsure ? "no clear fault, some readings unsure" : "no fault found";
    std::string text;
    for (std::size_t i = 0; i < causes.size(); ++i) text += (i ? " + " : "") + causes[i];
    return text + (unsure ? ", some readings unsure" : "");
}

void fill_sparks(Diagnosis& d, const SparkTrain& train) {
    double origin = train.events.empty() ? 0.0 : train.events[train.first_exhaust].record.seconds();
    for (const SparkEvent& event : train.events) {
        d.spark_times.push_back(event.record.seconds() - origin);
        d.spark_peaks.push_back(event.record.peak_height);
        d.spark_compression.push_back(event.compression);
        d.spark_fired.push_back(event.record.sparked);
        d.rise_amplitude.push_back(event.rise.valid ? event.rise.amplitude : 0.0);
        d.rise_start.push_back(event.rise.valid ? event.rise.start : 0.0);
        d.rise_length.push_back(event.rise.valid ? event.rise.rise : 0.0);
    }
}

}

Diagnosis diagnose(const Bike& bike, const AirConstants& air, const std::vector<SparkRecord>& records, double ambient_temperature, bool use_ratio) {
    Diagnosis d;
    d.bike = bike.label;
    std::unique_ptr<Ignition> ignition = make_ignition(bike.ignition);
    d.ignition = ignition->name();
    d.fast_sampling = ignition->needs_fast_sampling();
    if (!ignition->clippable()) {
        d.refused = true;
        d.refusal = "coil sits on the plug, no lead to clip";
        return d;
    }
    SparkTrain train = read_train(records, *ignition);
    fill_sparks(d, train);
    d.fired_at = train.fired_at;
    for (std::size_t i = train.first_exhaust; i < train.usable_end; ++i) {
        if (!train.events[i].compression) continue;
        ++d.compression_sparks;
        if (!train.events[i].record.sparked) ++d.misfires;
    }
    if (d.compression_sparks < kMinimumCompressionSparks) {
        d.refused = true;
        d.refusal = train.fired_at >= 0 ? "engine fired, close the fuel tap or hold the throttle open and crank again" : "too few revolutions, crank for 2 s or more";
        return d;
    }

    std::vector<double> reserves;
    std::vector<double> exhaust_peaks;
    for (std::size_t i = train.first_exhaust; i < train.usable_end; ++i) {
        const SparkEvent& event = train.events[i];
        if (!event.compression) exhaust_peaks.push_back(event.peak);
        if (event.compression && event.rise.valid) reserves.push_back(event.rise.reserve);
    }
    if (!reserves.empty()) d.reserve = known(median(reserves), std::max(spread_of(reserves) / std::sqrt(reserves.size()), 0.01));

    double gap = bike.ignition.plug_gap;
    Measurement m = measure(train, air, gap, ambient_temperature, use_ratio);
    Parameters start;
    start[1] = 0.1;
    start[2] = 0.6;
    start[3] = 2 * kPi / m.times[1] / kSpeedUnit;
    start[0] = best_start_volts(bike, ambient_temperature, m, start);
    FitResult<kParameterCount> fit = fit_cranking(bike, ambient_temperature, m, start, kFitIterations);

    bool coil_visible = false;
    for (std::size_t i = train.first_exhaust; i < train.usable_end; ++i)
        if (train.events[i].compression && (train.events[i].rise.valid || !train.events[i].record.sparked)) coil_visible = true;
    if (coil_visible) {
        CrankingTrace first = run_cranking(make_setup(bike, fit.parameters, ambient_temperature), exhaust_spark_angle(bike.engine), fit.parameters[3] * kSpeedUnit, m.times.back() + kRunMargin);
        auto gap_at = [&](double volts, double coil_scale) {
            std::vector<double> counts_per_kv;
            for (std::size_t i = train.first_exhaust; i < train.usable_end; ++i) {
                const SparkEvent& event = train.events[i];
                if (!event.compression) continue;
                double rpm = 60.0 / (event.record.seconds() - train.events[i - 1].record.seconds());
                double open_kv = coil_scale * ignition->open_circuit_kv(rpm, volts);
                if (event.rise.valid) counts_per_kv.push_back(event.rise.amplitude / open_kv);
                else if (!event.record.sparked) counts_per_kv.push_back(event.peak / open_kv);
            }
            return gap_from_breakdown(air, median(exhaust_peaks) / median(counts_per_kv), exhaust_density(ambient_temperature));
        };
        double found = gap_at(first.min_battery_volts, 1.0);
        double coil_shift = gap_at(first.min_battery_volts, 1 - uniform_sigma(kCoilSpread)) - found;
        double volts_shift = 0.5 * (gap_at(first.min_battery_volts - kCoilVoltsSpread, 1.0) - gap_at(first.min_battery_volts + kCoilVoltsSpread, 1.0));
        if (std::isfinite(found) && std::isfinite(coil_shift) && std::isfinite(volts_shift)) {
            gap = found;
            d.plug_gap_mm = known(gap * 1e3, std::hypot(coil_shift, volts_shift) * 1e3);
            m = measure(train, air, gap, ambient_temperature, use_ratio);
            fit = fit_cranking(bike, ambient_temperature, m, fit.parameters, kFitIterations);
        }
    }

    std::vector<double> trapped;
    for (std::size_t k = 0; k < m.density.size(); ++k)
        if (m.density[k] > 0) trapped.push_back(m.density[k] / (exhaust_density(ambient_temperature) * sealed_density_ratio(bike.engine)));
    if (!trapped.empty()) d.trapped_now = known(100 * median(trapped), 100 * std::max(spread_of(trapped) / std::sqrt(trapped.size()), 0.005));

    d.chi_square_per_point = fit.chi_square / std::max(1, fit.residual_count);
    if (d.chi_square_per_point > kChiSquareLimit) {
        d.refused = true;
        d.refusal = "cranking does not fit the model, result withheld";
        return d;
    }

    d.new_gauge_bar = reference_gauge(bike, kNewEngine, ambient_temperature, nullptr);
    Outputs base = evaluate(bike, fit.parameters, ambient_temperature, m, d.new_gauge_bar);
    std::array<double, kOutputCount> sigma = statistical_sigma(bike, fit, ambient_temperature, m, d.new_gauge_bar);
    std::array<double, kOutputCount> centre = as_array(base);
    for (const Nuisance& nuisance : nuisances()) {
        Bike shifted = bike;
        double ambient = ambient_temperature;
        nuisance.apply(shifted, ambient, nuisance.sigma * kNuisanceProbe);
        Measurement moved_measurement = measure(train, air, gap, ambient, use_ratio);
        FitResult<kParameterCount> moved_fit = fit_cranking(shifted, ambient, moved_measurement, fit.parameters, kNuisanceIterations);
        std::array<double, kOutputCount> moved = as_array(evaluate(shifted, moved_fit.parameters, ambient, moved_measurement, d.new_gauge_bar));
        for (int k = 0; k < kOutputCount; ++k) sigma[k] = std::hypot(sigma[k], (moved[k] - centre[k]) / kNuisanceProbe);
    }
    d.cranking_volts = known(base.cranking_volts, sigma[0]);
    d.cranking_amps = known(base.cranking_amps, sigma[1]);
    d.gauge_bar = known(base.gauge_bar, sigma[2]);
    d.percent_of_new = known(base.percent_of_new, sigma[3]);
    d.polytropic = known(base.polytropic, sigma[4]);
    d.leak_mm2 = known(base.leak_mm2, sigma[5]);
    d.cranking_rpm = known(60.0 * (m.times.size() - 1) / m.times.back(), 0.0);

    CrankingTrace fitted = run_cranking(make_setup(bike, fit.parameters, ambient_temperature), exhaust_spark_angle(bike.engine),
                                        fit.parameters[3] * kSpeedUnit, m.times.back() + kRunMargin);
    d.predicted_times.push_back(0.0);
    for (const PredictedSpark& spark : fitted.sparks) d.predicted_times.push_back(spark.time);

    d.battery_verdict = grade(d.cranking_volts, kBatteryGood, kBatteryWeak, "V while cranking", "good", "weak", "flat");
    d.compression_verdict = grade(d.percent_of_new, kCompressionGood, kCompressionWorn, "% of new", "good", "worn", "low");
    if (use_ratio && !d.trapped_now.known) d.compression_verdict = {"unsure", "no compression spark to read the kV ratio from"};
    std::ostringstream reason;
    reason.precision(3);
    if (d.misfires > 0) {
        reason << d.misfires << " of " << d.compression_sparks << " compression sparks missing";
        if (d.plug_gap_mm.known && (d.plug_gap_mm.value - kConfidence * d.plug_gap_mm.sigma) * 1e-3 > bike.ignition.plug_gap + kWornGapExtra)
            reason << ", plug gap about " << d.plug_gap_mm.value << " mm";
        else if (d.battery_verdict.state != "good")
            reason << ", " << ignition->sag_reason();
        bool mostly = d.misfires >= kMisfireShare * d.compression_sparks;
        d.ignition_verdict = {mostly ? "no spark" : "marginal", reason.str()};
    } else if (!d.reserve.known) {
        d.ignition_verdict = {"unsure", "rise too short to read the coil reserve"};
    } else {
        reason << "reserve " << d.reserve.value << " +/- " << kConfidence * d.reserve.sigma;
        if (d.plug_gap_mm.known) reason << ", plug gap about " << d.plug_gap_mm.value << " mm";
        bool marginal = d.reserve.value + kConfidence * d.reserve.sigma < kReserveMarginal;
        bool clear = d.reserve.value - kConfidence * d.reserve.sigma >= kReserveMarginal;
        d.ignition_verdict = {marginal ? "marginal" : (clear ? "good" : "unsure"), reason.str()};
    }
    d.summary = summarise(d);
    return d;
}