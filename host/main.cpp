#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "catalog.hpp"
#include "diagnosis.hpp"
#include "records.hpp"

namespace {

const double kDefaultAmbient = 303.0;
const char* kUsage = "usage: spark_clip <bike> <records.hex> [--json] [--no-ratio] [--ambient kelvin] [--data dir]";

std::string estimate_json(const Estimate& e) {
    if (!e.known) return "null";
    std::ostringstream text;
    text.precision(6);
    text << "{\"value\":" << e.value << ",\"sigma\":" << e.sigma << "}";
    return text.str();
}

std::string quoted(const std::string& text) {
    std::string out = "\"";
    for (char c : text) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out + "\"";
}

std::string verdict_json(const Verdict& v) { return "{\"state\":" + quoted(v.state) + ",\"reason\":" + quoted(v.reason) + "}"; }

template <class T>
std::string list_json(const std::vector<T>& values) {
    std::ostringstream text;
    text.precision(9);
    text << "[";
    for (std::size_t i = 0; i < values.size(); ++i) text << (i ? "," : "") << values[i];
    return text.str() + "]";
}

std::string flags_json(const std::vector<bool>& values) {
    std::string text = "[";
    for (std::size_t i = 0; i < values.size(); ++i) text += std::string(i ? "," : "") + (values[i] ? "1" : "0");
    return text + "]";
}

void print_json(const Diagnosis& d) {
    std::ostringstream o;
    o.precision(6);
    o << "{\"bike\":" << quoted(d.bike) << ",\"ignition\":" << quoted(d.ignition) << ",\"refused\":" << (d.refused ? "true" : "false")
      << ",\"refusal\":" << quoted(d.refusal) << ",\"fast_sampling\":" << (d.fast_sampling ? "true" : "false")
      << ",\"compression_sparks\":" << d.compression_sparks << ",\"misfires\":" << d.misfires << ",\"fired_at\":" << d.fired_at
      << ",\"reserve\":" << estimate_json(d.reserve) << ",\"plug_gap_mm\":" << estimate_json(d.plug_gap_mm)
      << ",\"trapped_now\":" << estimate_json(d.trapped_now) << ",\"cranking_amps\":" << estimate_json(d.cranking_amps)
      << ",\"cranking_volts\":" << estimate_json(d.cranking_volts) << ",\"cranking_rpm\":" << estimate_json(d.cranking_rpm)
      << ",\"leak_mm2\":" << estimate_json(d.leak_mm2) << ",\"gauge_bar\":" << estimate_json(d.gauge_bar)
      << ",\"percent_of_new\":" << estimate_json(d.percent_of_new) << ",\"polytropic\":" << estimate_json(d.polytropic)
      << ",\"new_gauge_bar\":" << d.new_gauge_bar << ",\"chi_square_per_point\":" << d.chi_square_per_point
      << ",\"spark_times\":" << list_json(d.spark_times)
      << ",\"spark_peaks\":" << list_json(d.spark_peaks) << ",\"spark_compression\":" << flags_json(d.spark_compression)
      << ",\"spark_fired\":" << flags_json(d.spark_fired) << ",\"predicted_times\":" << list_json(d.predicted_times)
      << ",\"rise_amplitude\":" << list_json(d.rise_amplitude) << ",\"rise_start\":" << list_json(d.rise_start)
      << ",\"rise_length\":" << list_json(d.rise_length)
      << ",\"battery\":" << verdict_json(d.battery_verdict) << ",\"ignition_verdict\":" << verdict_json(d.ignition_verdict)
      << ",\"compression\":" << verdict_json(d.compression_verdict) << ",\"summary\":" << quoted(d.summary) << "}";
    std::cout << o.str() << "\n";
}

void print_line(const char* label, const Estimate& e, const char* unit) {
    if (e.known) std::printf("  %-18s %8.3g +/- %-7.2g %s\n", label, e.value, 2 * e.sigma, unit);
    else std::printf("  %-18s %8s\n", label, "-");
}

void print_text(const Diagnosis& d) {
    std::printf("%s\n", d.bike.c_str());
    if (d.refused) {
        std::printf("no answer: %s\n", d.refusal.c_str());
        return;
    }
    std::printf("battery      %-9s %s\n", d.battery_verdict.state.c_str(), d.battery_verdict.reason.c_str());
    std::printf("ignition     %-9s %s\n", d.ignition_verdict.state.c_str(), d.ignition_verdict.reason.c_str());
    std::printf("compression  %-9s %s\n", d.compression_verdict.state.c_str(), d.compression_verdict.reason.c_str());
    std::printf("cause        %s\n", d.summary.c_str());
    print_line("cranking volts", d.cranking_volts, "V");
    print_line("cranking current", d.cranking_amps, "A");
    print_line("cranking speed", d.cranking_rpm, "rpm");
    print_line("trapped now", d.trapped_now, "%");
    print_line("leak area", d.leak_mm2, "mm2");
    print_line("gauge, good batt", d.gauge_bar, "bar");
    print_line("polytropic n", d.polytropic, "");
    print_line("plug gap", d.plug_gap_mm, "mm");
    std::printf("  %-18s %8.3g\n", "chi2 per point", d.chi_square_per_point);
}

}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << kUsage << "\n";
        return 2;
    }
    std::string bike_name = argv[1];
    std::string record_path = argv[2];
    bool json = false;
    bool use_ratio = true;
    double ambient = kDefaultAmbient;
    std::string data_dir = "data";
    for (int i = 3; i < argc; ++i) {
        std::string option = argv[i];
        if (option == "--json") json = true;
        else if (option == "--no-ratio") use_ratio = false;
        else if (option == "--ambient" && i + 1 < argc) ambient = std::stod(argv[++i]);
        else if (option == "--data" && i + 1 < argc) data_dir = argv[++i];
        else {
            std::cerr << kUsage << "\n";
            return 2;
        }
    }
    try {
        auto bikes = load_bikes(data_dir + "/bikes.txt");
        auto found = bikes.find(bike_name);
        if (found == bikes.end()) {
            std::cerr << "unknown bike " << bike_name << "\n";
            return 2;
        }
        AirConstants air = load_air(data_dir + "/air.txt");
        Diagnosis d = diagnose(found->second, air, read_record_file(record_path), ambient, use_ratio);
        if (json) print_json(d);
        else print_text(d);
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
    return 0;
}