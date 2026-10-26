// CV-KF vs EKF comparison on a turning/maneuvering target.
//
// A straight-line Kalman Filter (constant velocity) assumes the target
// never turns, so it will lag behind and overshoot during a turn. The
// EKF uses a CTRV (Constant Turn Rate and Velocity) motion model that
// linearizes around the current estimate every step, so it tracks the
// turn correctly. This app runs both filters on the same noisy position
// measurements and writes out both estimates (plus, if ground truth is
// given, prints RMSE for each) so they can be plotted side by side.
//
// Usage:
//   compare_filters <measurements.csv> <cv_output.csv> <ekf_output.csv> [truth.csv]
//
// measurements.csv must have a header row: time,px,py
// truth.csv (optional) must have a header row: time,px,py

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "KalmanFilter.hpp"
#include "ExtendedKalmanFilter.hpp"
#include "MotionModel.hpp"
#include "MeasurementModel.hpp"
#include "TrackingMetrics.hpp"
#include "../lib/Utils/CsvUtils.hpp"

namespace {

void ensureParentDirectory(const std::string& path) {
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
}

// Tunable noise parameters. Kept as simple constants rather than more
// command-line flags, to keep the app easy to run out of the box.
constexpr double kMeasurementNoisePx = 3.0;
constexpr double kMeasurementNoisePy = 3.0;
constexpr double kCvAccelNoise = 2.0;      // CV-KF: assumed accel noise
constexpr double kEkfAccelNoise = 1.0;     // EKF: longitudinal accel noise
constexpr double kEkfYawAccelNoise = 0.3;  // EKF: yaw (turn-rate) accel noise

} // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <measurements.csv> <cv_output.csv> <ekf_output.csv> [truth.csv]\n";
        return 1;
    }

    const std::string measurements_path = argv[1];
    const std::string cv_output_path = argv[2];
    const std::string ekf_output_path = argv[3];
    const bool have_truth = argc > 4;
    const std::string truth_path = have_truth ? argv[4] : "";

    std::vector<std::vector<double>> rows;
    try {
        rows = CsvUtils::readNumericCsv(measurements_path);
    } catch (const std::exception& e) {
        std::cerr << "Error reading measurements: " << e.what() << "\n";
        return 1;
    }
    if (rows.size() < 2) {
        std::cerr << "Need at least 2 measurement rows in " << measurements_path << "\n";
        return 1;
    }
    for (size_t i = 0; i < rows.size(); ++i) {
        if (rows[i].size() < 3) {
            std::cerr << "Measurement row " << (i + 2)
                      << " must contain time, px, and py\n";
            return 1;
        }
        if (i > 0 && rows[i][0] <= rows[i - 1][0]) {
            std::cerr << "Measurement timestamps must be strictly increasing\n";
            return 1;
        }
    }

    // ---- Set up CV-KF (state = [px, py, vx, vy]) ----
    KalmanFilter cv_kf(4);
    {
        Eigen::VectorXd x0(4);
        x0 << rows[0][1], rows[0][2], 0.0, 0.0;
        Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4);
        P0(0, 0) = kMeasurementNoisePx * kMeasurementNoisePx;
        P0(1, 1) = kMeasurementNoisePy * kMeasurementNoisePy;
        P0(2, 2) = 100.0;
        P0(3, 3) = 100.0;
        cv_kf.init(x0, P0);
    }
    PositionMeasurementModelND cv_meas_model(4, kMeasurementNoisePx, kMeasurementNoisePy);

    // ---- Set up EKF (state = [px, py, v, theta, omega]), CTRV model ----
    ExtendedKalmanFilter ekf(5);
    {
        // Estimate an initial heading from the first two measurements so
        // the filter doesn't start out completely blind on theta.
        double theta0 = 0.0;
        if (rows.size() > 1) {
            theta0 = std::atan2(rows[1][2] - rows[0][2], rows[1][1] - rows[0][1]);
        }
        Eigen::VectorXd x0(5);
        x0 << rows[0][1], rows[0][2], 0.0, theta0, 0.0;
        Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(5, 5);
        P0(0, 0) = kMeasurementNoisePx * kMeasurementNoisePx;
        P0(1, 1) = kMeasurementNoisePy * kMeasurementNoisePy;
        P0(2, 2) = 25.0;
        P0(3, 3) = 4.0;
        P0(4, 4) = 4.0;
        ekf.init(x0, P0);
    }
    PositionMeasurementModelND ekf_meas_model(5, kMeasurementNoisePx, kMeasurementNoisePy);
    ConstantTurnModel ctrv(kEkfAccelNoise, kEkfYawAccelNoise);

    // h/Hj are linear here (position is just the first two state entries),
    // but passed as functions since that's the interface ExtendedKalmanFilter expects.
    Eigen::MatrixXd ekf_H = ekf_meas_model.H();
    auto h_func = [ekf_H](const Eigen::VectorXd& x) -> Eigen::VectorXd { return ekf_H * x; };
    auto Hj_func = [ekf_H](const Eigen::VectorXd&) -> Eigen::MatrixXd { return ekf_H; };

    std::vector<std::string> cv_header = {"time", "px", "py", "vx", "vy"};
    std::vector<std::string> ekf_header = {"time", "px", "py", "v", "theta", "omega"};
    std::vector<std::vector<double>> cv_rows, ekf_rows;
    cv_rows.reserve(rows.size());
    ekf_rows.reserve(rows.size());

    std::vector<Eigen::Vector2d> cv_estimates, ekf_estimates;
    cv_estimates.reserve(rows.size());
    ekf_estimates.reserve(rows.size());

    double prev_t = rows[0][0];
    for (size_t i = 0; i < rows.size(); ++i) {
        const double t = rows[i][0];
        const double dt = (i == 0) ? 0.0 : (t - prev_t);
        prev_t = t;

        Eigen::VectorXd z(2);
        z << rows[i][1], rows[i][2];

        if (i > 0) {
            ConstantVelocity2DModel cv_motion(dt, kCvAccelNoise);
            cv_kf.predict(cv_motion.F(), cv_motion.Q());

            Eigen::MatrixXd Q_ekf = ctrv.processNoise(ekf.state(), dt);
            const Eigen::VectorXd x_pred = ctrv.stateTransition(ekf.state(), dt);
            const Eigen::MatrixXd f_jacobian = ctrv.jacobian(ekf.state(), dt);
            ekf.predict(x_pred, f_jacobian, Q_ekf);
        }

        cv_kf.update(z, cv_meas_model.H(), cv_meas_model.R());
        const Eigen::VectorXd z_pred = h_func(ekf.state());
        const Eigen::MatrixXd h_jacobian = Hj_func(ekf.state());
        ekf.update(z, z_pred, h_jacobian, ekf_meas_model.R());

        const Eigen::VectorXd& xc = cv_kf.state();
        const Eigen::VectorXd& xe = ekf.state();
        cv_rows.push_back({t, xc(0), xc(1), xc(2), xc(3)});
        ekf_rows.push_back({t, xe(0), xe(1), xe(2), xe(3), xe(4)});

        cv_estimates.emplace_back(xc(0), xc(1));
        ekf_estimates.emplace_back(xe(0), xe(1));
    }

    try {
        ensureParentDirectory(cv_output_path);
        ensureParentDirectory(ekf_output_path);
        CsvUtils::writeNumericCsv(cv_output_path, cv_header, cv_rows);
        CsvUtils::writeNumericCsv(ekf_output_path, ekf_header, ekf_rows);
    } catch (const std::exception& e) {
        std::cerr << "Error writing output: " << e.what() << "\n";
        return 1;
    }

    std::cout << "Processed " << rows.size() << " measurements.\n";
    std::cout << "CV-KF results written to  " << cv_output_path << "\n";
    std::cout << "EKF results written to    " << ekf_output_path << "\n";

    if (have_truth) {
        std::vector<std::vector<double>> truth_rows;
        try {
            truth_rows = CsvUtils::readNumericCsv(truth_path);
        } catch (const std::exception& e) {
            std::cerr << "Warning: could not read truth file: " << e.what() << "\n";
            return 0;
        }
        if (truth_rows.size() != rows.size()) {
            std::cerr << "Warning: truth file row count (" << truth_rows.size()
                      << ") does not match measurement count (" << rows.size()
                      << "); skipping RMSE computation.\n";
            return 0;
        }
        std::vector<Eigen::Vector2d> truth;
        truth.reserve(truth_rows.size());
        for (const auto& r : truth_rows) truth.emplace_back(r[1], r[2]);

        const double cv_rmse = TrackingMetrics::positionRMSE(truth, cv_estimates);
        const double ekf_rmse = TrackingMetrics::positionRMSE(truth, ekf_estimates);
        std::cout << "\nPosition RMSE vs ground truth:\n";
        std::cout << "  CV-KF: " << cv_rmse << "\n";
        std::cout << "  EKF:   " << ekf_rmse << "\n";
    }

    return 0;
}
