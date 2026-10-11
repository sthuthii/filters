#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "../include/CoordinateConverter.hpp"
#include "../include/ExtendedKalmanFilter.hpp"
#include "../include/MeasurementModel.hpp"
#include "../include/MotionModel.hpp"

namespace {

struct RadarSample {
    double t;
    double range;
    double bearing;
};

// Reads a CSV file with header "t,range,bearing".
std::vector<RadarSample> readRadarMeasurements(const std::string& path) {
    std::vector<RadarSample> samples;
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open measurements file: " + path);
    }

    std::string line;
    bool first_line = true;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        if (first_line) {
            first_line = false;
            if (!std::isdigit(line[0]) && line[0] != '-') continue;  // skip header
        }
        std::stringstream ss(line);
        std::string t_str, r_str, b_str;
        std::getline(ss, t_str, ',');
        std::getline(ss, r_str, ',');
        std::getline(ss, b_str, ',');
        samples.push_back({std::stod(t_str), std::stod(r_str), std::stod(b_str)});
    }
    return samples;
}

}  // namespace

int main(int argc, char** argv) {
    std::string input_path = "scenarios/radar_measurements.csv";
    std::string output_path = "results/ekf_output.csv";
    if (argc > 1) input_path = argv[1];
    if (argc > 2) output_path = argv[2];

    // Filter tuning parameters. State tracked is [x, y, vx, vy].
    const int dim = 2;
    const double sigma_a = 0.3;         // assumed process (acceleration) noise std-dev
    const double sigma_range = 5.0;     // radar range measurement noise std-dev
    const double sigma_bearing = 0.02;  // radar bearing measurement noise std-dev (radians)

    // Held as MotionModel*/MeasurementModel* — same interface types
    // main.cpp uses, just constructed with different concrete classes.
    // Everything below this line is written against the interface only.
    std::unique_ptr<MotionModel> motion_model =
        std::make_unique<ConstantVelocityModel>(dim, sigma_a);
    std::unique_ptr<MeasurementModel> radar_model =
        std::make_unique<RadarMeasurementModel>(sigma_range, sigma_bearing);
    ExtendedKalmanFilter ekf(2 * dim);

    std::vector<RadarSample> samples;
    try {
        samples = readRadarMeasurements(input_path);
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
    out << "t,range,bearing,x_est,y_est,vx_est,vy_est\n";

    // Initialize state by converting the first radar reading to Cartesian,
    // with zero initial velocity.
    CoordinateConverter::Cartesian c0 =
        CoordinateConverter::polarToCartesian(samples[0].range, samples[0].bearing);
    Eigen::VectorXd x0(4);
    x0 << c0.x, c0.y, 0.0, 0.0;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4) * 50.0;
    ekf.init(x0, P0);
    out << samples[0].t << "," << samples[0].range << "," << samples[0].bearing << ","
        << x0(0) << "," << x0(1) << "," << x0(2) << "," << x0(3) << "\n";

    double prev_t = samples[0].t;
    for (size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].t - prev_t;
        prev_t = samples[i].t;

        // Predict: CV motion is linear, so x_pred = F*x exactly (no
        // approximation here — the nonlinearity in this example is all
        // in the measurement, not the motion).
        Eigen::MatrixXd F = motion_model->F(dt);
        Eigen::VectorXd x_pred = motion_model->predict(ekf.state(), dt);
        ekf.predict(x_pred, F, motion_model->Q(dt));

        // Update: range/bearing is nonlinear in (x, y), so linearize
        // around the predicted state via the Jacobian.
        Eigen::VectorXd z(2);
        z << samples[i].range, samples[i].bearing;
        Eigen::VectorXd z_pred = radar_model->predict(ekf.state());
        Eigen::MatrixXd H = radar_model->jacobian(ekf.state());

        // Bearing wraps around +-pi; normalize the innovation so a true
        // crossing of that boundary isn't seen as a huge jump.
        const double dtheta = CoordinateConverter::normalizeAngle(z(1) - z_pred(1));
        Eigen::VectorXd z_adj = z;
        z_adj(1) = z_pred(1) + dtheta;

        ekf.update(z_adj, z_pred, H, radar_model->R());

        const Eigen::VectorXd& x = ekf.state();
        out << samples[i].t << "," << samples[i].range << "," << samples[i].bearing << ","
            << x(0) << "," << x(1) << "," << x(2) << "," << x(3) << "\n";
    }

    out.close();
    std::cout << "Done. Wrote " << samples.size() << " estimates to " << output_path << std::endl;
    return 0;
}