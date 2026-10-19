#pragma once

#include "catalog.hpp"

const double kPi = 3.14159265358979323846;
const double kCycle = 4 * kPi;

double degrees_to_radians(double degrees);
double piston_area(const EngineGeometry& engine);
double clearance_volume(const EngineGeometry& engine);
double cylinder_volume(const EngineGeometry& engine, double theta);
double volume_rate(const EngineGeometry& engine, double theta);
double wall_area(const EngineGeometry& engine, double theta);
double intake_close_angle(const EngineGeometry& engine);
double exhaust_open_angle(const EngineGeometry& engine);
double compression_spark_angle(const EngineGeometry& engine);
double exhaust_spark_angle(const EngineGeometry& engine);
double sealed_density_ratio(const EngineGeometry& engine);
double total_inertia(const Bike& bike);