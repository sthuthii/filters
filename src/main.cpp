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
    double z;  // noisy position measurement
};

// Reads a CSV file with header "t,z" and one (time, measurement) pair per line.
std::vector<Sample> readMeasurements(const std::string& path) {
    std::vector<Sample> samples;
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open measurements file: " + path);
    }

    std::string line;
    bool first_line = true;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        if (first_line) {
            // Skip header if present (non-numeric first token).
            first_line = false;
            if (!std::isdigit(line[0]) && line[0] != '-') {
                continue;
            }
        }
        std::stringstream ss(line);
        std::string t_str, z_str;
        std::getline(ss, t_str, ',');
        std::getline(ss, z_str, ',');
        samples.push_back({std::stod(t_str), std::stod(z_str)});
    }
    return samples;
}

}  // namespace

int main(int argc, char** argv) {
    // Default paths, can be overridden via command line args.
    std::string input_path = "scenarios/measurements.csv";
    std::string output_path = "results/output.csv";
    if (argc > 1) input_path = argv[1];
    if (argc > 2) output_path = argv[2];

    // Filter tuning parameters (Task 1: CV model).
    const double sigma_a = 0.2;    // assumed process (acceleration) noise std-dev
    const double r_variance = 4.0; // sensor position measurement noise variance

    ConstantVelocityModel motion_model(sigma_a);
    PositionMeasurementModel measurement_model(r_variance);
    KalmanFilter kf(2);

    std::vector<Sample> samples;
    try {
        samples = readMeasurements(input_path);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    if (samples.empty()) {
        std::cerr << "No measurements found in " << input_path << std::endl;
        return 1;
    }

    std::ofstream out(output_path);
    if (!out.is_open()) {
        std::cerr << "Could not open output file: " << output_path << std::endl;
        return 1;
    }
    out << "t,z,pos_est,vel_est\n";

    // Initialize state from the first measurement, zero initial velocity.
    Eigen::VectorXd x0(2);
    x0 << samples[0].z, 0.0;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(2, 2) * 10.0;
    kf.init(x0, P0);
    out << samples[0].t << "," << samples[0].z << "," << x0(0) << "," << x0(1) << "\n";

    double prev_t = samples[0].t;
    for (size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].t - prev_t;
        prev_t = samples[i].t;

        kf.predict(motion_model.F(dt), motion_model.Q(dt));

        Eigen::VectorXd z(1);
        z << samples[i].z;
        kf.update(z, measurement_model.H(), measurement_model.R());

        const Eigen::VectorXd& x = kf.state();
        out << samples[i].t << "," << samples[i].z << "," << x(0) << "," << x(1) << "\n";
    }

    out.close();
    std::cout << "Done. Wrote " << samples.size() << " estimates to " << output_path << std::endl;
    return 0;
}
