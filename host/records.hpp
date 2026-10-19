#pragma once

#include <cstdint>
#include <string>
#include <vector>

const double kSampleRate = 100e6;
const double kVoltsPerCount = 2.0 / 4096;
const int kLadderStep = 16;
const int kTriggerLevel = 24;

struct SparkRecord {
    std::uint32_t start_time = 0;
    int peak_height = 0;
    int peak_offset = 0;
    int burn_length = 0;
    bool sparked = false;
    std::vector<int> ladder;

    double seconds() const { return start_time / kSampleRate; }
};

std::vector<SparkRecord> parse_bytes(const std::vector<std::uint8_t>& bytes);
std::vector<SparkRecord> read_record_file(const std::string& path);