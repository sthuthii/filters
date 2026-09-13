#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../include/KalmanFilter.hpp"
#include "../include/MotionModel.hpp"
#include "../include/MeasurementModel.hpp"

namespace {

struct Sample {
    double t;
    std::vector<double> z;  // one noisy position reading per spatial dimension
};

// Reads a CSV with header "t,z1,z2,...,zN" (N = number of spatial dimensions).
// Returns the parsed samples and sets `dim` to N (inferred from the header).
std::vector<Sample> readMeasurements(const std::string& path, int& dim) {
    std::vector<Sample> samples;
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open measurements file: " + path);
    }

    std::string line;
    if (!std::getline(file, line) || line.empty()) {
        throw std::runtime_error("Measurements file is empty: " + path);
    }

    // Header determines dimensionality: "t,z1,z2,...,zN" -> dim = N.
    {
        std::stringstream header(line);
        std::string token;
        int col_count = 0;
        while (std::getline(header, token, ',')) ++col_count;
        dim = col_count - 1;  // subtract the "t" column
        if (dim < 1) {
            throw std::runtime_error("Expected at least one z column in header, got: " + line);
        }
    }

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;

        std::getline(ss, token, ',');
        Sample sample;
        sample.t = std::stod(token);

        for (int i = 0; i < dim; ++i) {
            if (!std::getline(ss, token, ',')) {
                throw std::runtime_error("Row has fewer than " + std::to_string(dim) +
                                          " z columns: " + line);
            }
            sample.z.push_back(std::stod(token));
        }
        samples.push_back(std::move(sample));
    }
    return samples;
}

std::string outputHeader(int dim) {
    std::string header = "t";
    for (int i = 0; i < dim; ++i) header += ",z" + std::to_string(i + 1);
    for (int i = 0; i < dim; ++i) header += ",pos_est_" + std::to_string(i + 1);
    for (int i = 0; i < dim; ++i) header += ",vel_est_" + std::to_string(i + 1);
    return header;
}

void writeRow(std::ofstream& out, double t, const std::vector<double>& z,
              const Eigen::VectorXd& x, int dim) {
    out << t;
    for (int i = 0; i < dim; ++i) out << "," << z[i];
    for (int i = 0; i < dim; ++i) out << "," << x(i);         // positions: x(0..dim-1)
    for (int i = 0; i < dim; ++i) out << "," << x(dim + i);   // velocities: x(dim..2*dim-1)
    out << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    // Default paths, can be overridden via command line args.
    std::string input_path = "scenarios/measurements.csv";
    std::string output_path = "results/output.csv";
    if (argc > 1) input_path = argv[1];
    if (argc > 2) output_path = argv[2];

    // Filter tuning parameters.
    const double sigma_a = 0.2;    // assumed process (acceleration) noise std-dev
    const double r_variance = 4.0; // sensor position measurement noise variance (per axis)

    std::vector<Sample> samples;
    int dim = 0;
    try {
        samples = readMeasurements(input_path, dim);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    if (samples.empty()) {
        std::cerr << "No measurements found in " << input_path << std::endl;
        return 1;
    }

    // dim is inferred from the CSV header, so this works unmodified for
    // dim=1 (the original 1D CV case), dim=2 (x/y), dim=3 (x/y/z), etc. —
    // as long as MotionModel / MeasurementModel are built to accept `dim`.
    ConstantVelocityModel motion_model(dim, sigma_a);
    PositionMeasurementModel measurement_model(dim, r_variance);
    KalmanFilter kf(2 * dim);

    std::ofstream out(output_path);
    if (!out.is_open()) {
        std::cerr << "Could not open output file: " << output_path << std::endl;
        return 1;
    }
    out << outputHeader(dim) << "\n";

    // Initialize state from the first measurement: position = first reading,
    // velocity = 0 for every axis.
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2 * dim);
    for (int i = 0; i < dim; ++i) x0(i) = samples[0].z[i];
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(2 * dim, 2 * dim) * 10.0;
    kf.init(x0, P0);
    writeRow(out, samples[0].t, samples[0].z, x0, dim);

    double prev_t = samples[0].t;
    for (size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].t - prev_t;
        prev_t = samples[i].t;

        kf.predict(motion_model.F(dt), motion_model.Q(dt));

        Eigen::VectorXd z(dim);
        for (int d = 0; d < dim; ++d) z(d) = samples[i].z[d];
        kf.update(z, measurement_model.H(), measurement_model.R());

        writeRow(out, samples[i].t, samples[i].z, kf.state(), dim);
    }

    out.close();
    std::cout << "Done (dim=" << dim << "). Wrote " << samples.size()
              << " estimates to " << output_path << std::endl;
    return 0;
}