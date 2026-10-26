#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>

// Minimal CSV helpers — just enough to read measurement files and write
// result files for this project. Not a general-purpose CSV library on
// purpose, to keep things simple.
namespace CsvUtils {

// Reads a CSV file with a header row into rows of doubles.
// Skips the first line (assumed to be the header).
inline std::vector<std::vector<double>> readNumericCsv(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + path);
    }

    std::vector<std::vector<double>> rows;
    std::string line;
    bool first_line = true;

    while (std::getline(file, line)) {
        if (first_line) { first_line = false; continue; } // skip header
        if (line.empty()) continue;

        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stod(cell));
        }
        rows.push_back(row);
    }
    return rows;
}

// Writes a header line followed by one comma-joined line per row.
inline void writeNumericCsv(const std::string& path, const std::vector<std::string>& header,
                             const std::vector<std::vector<double>>& rows) {
    std::ofstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file for writing: " + path);
    }

    for (size_t i = 0; i < header.size(); ++i) {
        file << header[i] << (i + 1 < header.size() ? "," : "\n");
    }
    for (const auto& row : rows) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i] << (i + 1 < row.size() ? "," : "\n");
        }
    }
}

} // namespace CsvUtils
