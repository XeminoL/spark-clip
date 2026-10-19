#pragma once

#include "catalog.hpp"

double plug_breakdown_kv(const AirConstants& air, double density_ratio, double gap);
double density_from_breakdown(const AirConstants& air, double kv, double gap);
double density_from_ratio(const AirConstants& air, double ratio, double exhaust_density, double gap);
double gap_from_breakdown(const AirConstants& air, double kv, double density_ratio);