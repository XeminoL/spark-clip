#include "paschen.hpp"

#include <cmath>
#include <functional>
#include <vector>

namespace {

const double kTorrPerBar = 750.06;
const int kQuadratureOrder = 64;
const int kBisectionSteps = 80;
const double kPiValue = 3.14159265358979323846;

struct Quadrature {
    std::vector<double> nodes;
    std::vector<double> weights;
};

Quadrature gauss_legendre(int order) {
    Quadrature rule;
    for (int i = 1; i <= order; ++i) {
        double x = std::cos(kPiValue * (i - 0.25) / (order + 0.5));
        double derivative = 0.0;
        for (int step = 0; step < 100; ++step) {
            double previous = 1.0;
            double current = x;
            for (int k = 2; k <= order; ++k) {
                double next = ((2 * k - 1) * x * current - (k - 1) * previous) / k;
                previous = current;
                current = next;
            }
            derivative = order * (x * current - previous) / (x * x - 1);
            double shift = current / derivative;
            x -= shift;
            if (std::fabs(shift) < 1e-15) break;
        }
        rule.nodes.push_back(x);
        rule.weights.push_back(2 / ((1 - x * x) * derivative * derivative));
    }
    return rule;
}

const Quadrature& rule() {
    static const Quadrature cached = gauss_legendre(kQuadratureOrder);
    return cached;
}

double townsend_sum(const AirConstants& air, double volts, double density_ratio, double gap) {
    double focal = std::sqrt(gap * (gap + air.tip_radius));
    double geometry = focal * std::atanh(gap / focal);
    double torr = density_ratio * kTorrPerBar;
    double sum = 0.0;
    for (std::size_t i = 0; i < rule().nodes.size(); ++i) {
        double height = 0.5 * gap * (rule().nodes[i] + 1);
        double field_v_per_cm = volts / (geometry * (1 - height * height / (focal * focal))) / 100;
        double alpha_per_m = 100 * air.ionisation_over_k * torr * std::exp(-air.field_constant * torr / field_v_per_cm);
        sum += alpha_per_m * 0.5 * gap * rule().weights[i];
    }
    return sum;
}

double bisect(const std::function<double(double)>& rising, double low, double high) {
    if (rising(low) > 0 || rising(high) < 0) return NAN;
    for (int step = 0; step < kBisectionSteps; ++step) {
        double middle = 0.5 * (low + high);
        if (rising(middle) > 0) high = middle;
        else low = middle;
    }
    return 0.5 * (low + high);
}

}

double plug_breakdown_kv(const AirConstants& air, double density_ratio, double gap) {
    auto excess = [&](double volts) { return townsend_sum(air, volts, density_ratio, gap) - 1; };
    return bisect(excess, 10.0, 200000.0) / 1000;
}

double density_from_breakdown(const AirConstants& air, double kv, double gap) {
    auto excess = [&](double density) { return plug_breakdown_kv(air, density, gap) - kv; };
    return bisect(excess, 0.05, 60.0);
}

double density_from_ratio(const AirConstants& air, double ratio, double exhaust_density, double gap) {
    double exhaust_kv = plug_breakdown_kv(air, exhaust_density, gap);
    return density_from_breakdown(air, ratio * exhaust_kv, gap);
}

double gap_from_breakdown(const AirConstants& air, double kv, double density_ratio) {
    auto excess = [&](double gap) { return plug_breakdown_kv(air, density_ratio, gap) - kv; };
    return bisect(excess, 0.1e-3, 3.0e-3);
}