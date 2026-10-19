#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "catalog.hpp"
#include "cranking.hpp"
#include "engine.hpp"
#include "fit.hpp"
#include "ignition.hpp"
#include "paschen.hpp"
#include "records.hpp"

namespace {

int failures = 0;
int checks = 0;

void expect(bool condition, const std::string& what) {
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

bool close(double a, double b, double tolerance) { return std::fabs(a - b) <= tolerance * std::max(1.0, std::fabs(b)); }

void test_linear_algebra() {
    Mat<3> matrix;
    double values[3][3] = {{4, 1, 0}, {1, 3, 1}, {0, 1, 2}};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) matrix(r, c) = values[r][c];
    Mat<3> product;
    Mat<3> inverted = inverse(matrix);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            for (int k = 0; k < 3; ++k) product(r, c) += matrix(r, k) * inverted(k, c);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) expect(close(product(r, c), r == c ? 1.0 : 0.0, 1e-12), "inverse");
}

void test_levenberg_marquardt() {
    std::vector<double> xs = {0, 1, 2, 3, 4, 5};
    auto residuals = [&](const Vec<2>& p) {
        std::vector<double> out;
        for (double x : xs) out.push_back(p[0] * std::exp(-p[1] * x) - 2.5 * std::exp(-0.4 * x));
        return out;
    };
    Vec<2> start;
    start[0] = 1.0;
    start[1] = 1.0;
    Vec<2> steps;
    steps[0] = 1e-6;
    steps[1] = 1e-6;
    FitResult<2> fit = levenberg_marquardt<2>(residuals, start, steps, 100);
    expect(close(fit.parameters[0], 2.5, 1e-6) && close(fit.parameters[1], 0.4, 1e-6), "exponential fit");
}

void test_paschen(const AirConstants& air) {
    AirConstants flat = air;
    flat.tip_radius = 1e3;
    double torr_cm = 0.97 * 0.07 * 750.06;
    double uniform = air.field_constant * torr_cm / std::log(air.ionisation_over_k * torr_cm) / 1000;
    expect(close(plug_breakdown_kv(flat, 0.97, 0.7e-3), uniform, 1e-4), "flat electrode gives Paschen");
    double kv = plug_breakdown_kv(air, 6.5, 0.8e-3);
    expect(close(density_from_breakdown(air, kv, 0.8e-3), 6.5, 1e-6), "density inversion");
    expect(close(gap_from_breakdown(air, plug_breakdown_kv(air, 1.0, 0.9e-3), 1.0), 0.9e-3, 1e-6), "gap inversion");
    expect(plug_breakdown_kv(air, 1.0, 0.7e-3) < uniform * 1.0001, "tip lowers breakdown");
}

void test_records() {
    std::vector<std::uint8_t> bytes;
    auto put = [&](std::uint32_t word) {
        for (int i = 0; i < 4; ++i) bytes.push_back((word >> (8 * i)) & 0xFF);
    };
    put(0xA5000000u | (1u << 23) | (3u << 16) | 900u);
    put(123456789u);
    put((410u << 18) | 15000u);
    put(0u | (40u << 16));
    put(90u);
    std::vector<SparkRecord> records = parse_bytes(bytes);
    expect(records.size() == 1, "one record");
    expect(records[0].peak_height == 900 && records[0].sparked && records[0].start_time == 123456789u, "header fields");
    expect(records[0].peak_offset == 410 && records[0].burn_length == 15000, "timing fields");
    expect(records[0].ladder.size() == 3 && records[0].ladder[1] == 40 && records[0].ladder[2] == 90, "ladder");
}

void test_ignitions(const std::map<std::string, Bike>& bikes) {
    expect(make_ignition(bikes.at("cub110").ignition)->name() == "AC-CDI", "ac cdi");
    expect(make_ignition(bikes.at("fi125").ignition)->name() == "TCI", "tci");
    expect(!make_ignition(bikes.at("cop150").ignition)->clippable(), "coil on plug refused");
    expect(make_ignition(bikes.at("cub110").ignition)->needs_fast_sampling(), "cdi needs fast sampling");
    expect(!make_ignition(bikes.at("fi125").ignition)->needs_fast_sampling(), "tci does not");
    std::unique_ptr<Ignition> cdi = make_ignition(bikes.at("cub110").ignition);
    expect(cdi->open_circuit_kv(200, 12) < cdi->open_circuit_kv(500, 12), "ac cdi grows with speed");
    expect(close(cdi->rise_shape(cdi->rise_fraction(0.3)), 0.3, 1e-12), "rise shape inverse");
}

void test_inertia(const std::map<std::string, Bike>& bikes) {
    expect(close(total_inertia(bikes.at("cub110")), 4.377e-3, 1e-3), "cub110 inertia matches python budget");
}

void print_simulation(int argc, char** argv) {
    std::string data = argv[2];
    Bike bike = load_bikes(data + "/bikes.txt").at(argv[3]);
    CrankingSetup setup = {bike, total_inertia(bike), {std::stod(argv[4]), std::stod(argv[5])}, {std::stod(argv[6]) * 1e-6, std::stod(argv[7])}, 303.0};
    double start = degrees_to_radians(std::stod(argv[8]));
    CrankingTrace trace = run_cranking(setup, start, 0.05, argc > 9 ? std::stod(argv[9]) : 2.5);
    for (const PredictedSpark& spark : trace.sparks) std::printf("%.9f %d %.9f\n", spark.time, spark.compression ? 1 : 0, spark.density_ratio);
}

}

int main(int argc, char** argv) {
    if (argc >= 9 && std::string(argv[1]) == "sim") {
        print_simulation(argc, argv);
        return 0;
    }
    std::string root = argc > 1 ? argv[1] : "..";
    std::map<std::string, Bike> bikes = load_bikes(root + "/data/bikes.txt");
    AirConstants air = load_air(root + "/data/air.txt");
    test_linear_algebra();
    test_levenberg_marquardt();
    test_paschen(air);
    test_records();
    test_ignitions(bikes);
    test_inertia(bikes);
    std::printf("%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}