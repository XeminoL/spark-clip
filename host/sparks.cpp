#include "sparks.hpp"

#include <algorithm>
#include <cmath>

#include "fit.hpp"

namespace {

const double kLevelNoise = 1.5;
const std::size_t kMinimumRungs = 4;
const double kHotExhaustShare = 0.6;
const int kExhaustBaseline = 3;
const int kRiseIterations = 60;
const double kRiseSpreadLimit = 2.0;
const double kPeakLagSamples = 1.4;
const double kReserveSigmaLimit = 0.3;

double peak_threshold(const std::vector<SparkRecord>& records) {
    std::vector<double> peaks;
    for (const SparkRecord& record : records) peaks.push_back(record.peak_height);
    std::sort(peaks.begin(), peaks.end());
    double low = peaks[peaks.size() / 10];
    double high = peaks[peaks.size() * 9 / 10];
    return std::sqrt(low * high);
}

double lag_corrected_peak(const SparkRecord& record) {
    std::size_t rungs = record.ladder.size();
    if (!record.sparked || rungs < 3) return record.peak_height;
    double slope = 2.0 * kLadderStep / std::max(1, record.ladder[rungs - 1] - record.ladder[rungs - 3]);
    return record.peak_height + slope * kPeakLagSamples;
}

RiseFit fit_rise(const SparkRecord& record, double peak, const Ignition& ignition) {
    RiseFit fit;
    if (!record.sparked || record.ladder.size() < kMinimumRungs) return fit;
    double nominal_rise = ignition.rise_seconds() * kSampleRate;
    auto residuals = [&](const Vec<3>& p) {
        std::vector<double> values;
        double amplitude = p[0];
        double start = p[1];
        double rise = std::max(p[2], 1.0);
        for (std::size_t rung = 1; rung < record.ladder.size(); ++rung) {
            double level = kLadderStep * (rung + 1.0);
            values.push_back((amplitude * ignition.rise_shape((record.ladder[rung] - start) / rise) - level) / kLevelNoise);
        }
        return values;
    };
    Vec<3> start;
    start[0] = peak / 0.8;
    start[2] = nominal_rise;
    start[1] = -nominal_rise * ignition.rise_fraction(kTriggerLevel / start[0]);
    Vec<3> steps;
    steps[0] = 0.5;
    steps[1] = 0.2;
    steps[2] = 0.5;
    FitResult<3> result = levenberg_marquardt<3>(residuals, start, steps, kRiseIterations);
    fit.amplitude = result.parameters[0];
    fit.start = result.parameters[1];
    fit.rise = result.parameters[2];
    fit.reserve = fit.amplitude / peak;
    fit.reserve_sigma = result.sigma(0) / peak;
    double spread = fit.rise / nominal_rise;
    fit.valid = result.converged && spread < kRiseSpreadLimit && spread > 1 / kRiseSpreadLimit && fit.reserve_sigma < kReserveSigmaLimit;
    return fit;
}

}

SparkTrain read_train(const std::vector<SparkRecord>& records, const Ignition& ignition) {
    SparkTrain train;
    if (records.size() < 3) return train;
    double threshold = peak_threshold(records);
    for (const SparkRecord& record : records) {
        SparkEvent event;
        event.record = record;
        event.peak = lag_corrected_peak(record);
        event.compression = record.peak_height > threshold;
        if (event.compression) event.rise = fit_rise(record, event.peak, ignition);
        train.events.push_back(event);
    }
    std::size_t first = 0;
    while (first + 1 < train.events.size() && !(!train.events[first].compression && train.events[first + 1].compression)) ++first;
    train.first_exhaust = first;
    std::size_t end = first + 1;
    while (end < train.events.size() && train.events[end].compression != train.events[end - 1].compression) ++end;
    std::vector<double> exhaust_peaks;
    for (std::size_t i = first; i < end; ++i) {
        if (train.events[i].compression) continue;
        if (static_cast<int>(exhaust_peaks.size()) == kExhaustBaseline) {
            std::vector<double> sorted = exhaust_peaks;
            std::sort(sorted.begin(), sorted.end());
            if (train.events[i].peak < kHotExhaustShare * sorted[sorted.size() / 2]) {
                train.fired_at = static_cast<int>(i) - 1;
                end = i;
                break;
            }
            continue;
        }
        exhaust_peaks.push_back(train.events[i].peak);
    }
    train.usable_end = end;
    return train;
}