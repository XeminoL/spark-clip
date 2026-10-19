#pragma once

#include <string>
#include <vector>

#include "catalog.hpp"
#include "records.hpp"

struct Estimate {
    double value = 0.0;
    double sigma = 0.0;
    bool known = false;
};

struct Verdict {
    std::string state;
    std::string reason;
};

struct Diagnosis {
    std::string bike;
    std::string ignition;
    bool refused = false;
    std::string refusal;
    bool fast_sampling = false;
    int compression_sparks = 0;
    int misfires = 0;
    int fired_at = -1;
    Estimate reserve;
    Estimate plug_gap_mm;
    Estimate trapped_now;
    Estimate cranking_volts;
    Estimate cranking_amps;
    Estimate cranking_rpm;
    Estimate leak_mm2;
    Estimate gauge_bar;
    Estimate percent_of_new;
    Estimate polytropic;
    double new_gauge_bar = 0.0;
    double chi_square_per_point = 0.0;
    std::vector<double> spark_times;
    std::vector<int> spark_peaks;
    std::vector<bool> spark_compression;
    std::vector<bool> spark_fired;
    std::vector<double> rise_amplitude;
    std::vector<double> rise_start;
    std::vector<double> rise_length;
    std::vector<double> predicted_times;
    Verdict battery_verdict;
    Verdict ignition_verdict;
    Verdict compression_verdict;
    std::string summary;
};

Diagnosis diagnose(const Bike& bike, const AirConstants& air, const std::vector<SparkRecord>& records, double ambient_temperature, bool use_ratio);