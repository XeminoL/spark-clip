#include "records.hpp"

#include <fstream>
#include <stdexcept>

namespace {

const std::uint32_t kMarker = 0xA5;

std::uint32_t word_at(const std::vector<std::uint8_t>& bytes, std::size_t word) {
    std::size_t at = word * 4;
    if (at + 3 >= bytes.size()) throw std::runtime_error("record cut short");
    return bytes[at] | bytes[at + 1] << 8 | bytes[at + 2] << 16 | static_cast<std::uint32_t>(bytes[at + 3]) << 24;
}

}

std::vector<SparkRecord> parse_bytes(const std::vector<std::uint8_t>& bytes) {
    std::vector<SparkRecord> records;
    std::size_t word = 0;
    while (word * 4 < bytes.size()) {
        std::uint32_t header = word_at(bytes, word);
        if (header >> 24 != kMarker) throw std::runtime_error("lost record alignment");
        SparkRecord record;
        record.peak_height = header & 0xFFF;
        record.sparked = (header >> 23) & 1;
        int rungs = (header >> 16) & 0x7F;
        record.start_time = word_at(bytes, word + 1);
        std::uint32_t timing = word_at(bytes, word + 2);
        record.peak_offset = timing >> 18;
        record.burn_length = timing & 0x3FFFF;
        for (int rung = 0; rung < rungs; ++rung) {
            std::uint32_t pair = word_at(bytes, word + 3 + rung / 2);
            record.ladder.push_back((pair >> (16 * (rung % 2))) & 0xFFFF);
        }
        records.push_back(record);
        word += 3 + (rungs + 1) / 2;
    }
    return records;
}

std::vector<SparkRecord> read_record_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open " + path);
    std::vector<std::uint8_t> bytes;
    std::string token;
    while (file >> token) bytes.push_back(static_cast<std::uint8_t>(std::stoul(token, nullptr, 16)));
    return parse_bytes(bytes);
}