#pragma once

#include <vector>

#include "ignition.hpp"
#include "records.hpp"

struct RiseFit {
    bool valid = false;
    double amplitude = 0.0;
    double start = 0.0;
    double rise = 0.0;
    double reserve = 0.0;
    double reserve_sigma = 0.0;
};

struct SparkEvent {
    SparkRecord record;
    double peak = 0.0;
    bool compression = false;
    RiseFit rise;
};

struct SparkTrain {
    std::vector<SparkEvent> events;
    std::size_t first_exhaust = 0;
    std::size_t usable_end = 0;
    int fired_at = -1;
};

SparkTrain read_train(const std::vector<SparkRecord>& records, const Ignition& ignition);