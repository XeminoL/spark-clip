#pragma once

#include <vector>

#include "catalog.hpp"

const double kGasConstant = 287.05;
const double kAmbientPressure = 1.013e5;
const double kReferenceDensity = 1.0e5 / (287.05 * 293.15);

struct Battery {
    double emf;
    double internal_resistance;
};

struct CylinderSeal {
    double leak_area;
    double heat_transfer;
};

struct CrankingSetup {
    Bike bike;
    double inertia;
    Battery battery;
    CylinderSeal seal;
    double ambient_temperature;
};

struct PredictedSpark {
    double time;
    bool compression;
    double density_ratio;
};

struct CrankingTrace {
    std::vector<PredictedSpark> sparks;
    std::vector<double> peak_pressures;
    std::vector<double> polytropic;
    std::vector<double> trapped;
    double mean_battery_volts = 0.0;
    double mean_current = 0.0;
    double min_battery_volts = 0.0;
    bool stalled = false;
};

double starter_current(const CrankingSetup& setup, double speed);
CrankingTrace run_cranking(const CrankingSetup& setup, double start_theta, double start_speed, double duration);