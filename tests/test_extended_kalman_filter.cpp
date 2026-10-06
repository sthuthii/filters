#include <cassert>
#include <cmath>
#include <iostream>

#include "../include/CoordinateConverter.hpp"
#include "../include/ExtendedKalmanFilter.hpp"
#include "../include/MeasurementModel.hpp"
#include "../include/MotionModel.hpp"

// Sanity check: track a 2D constant-velocity target using noise-free
// range/bearing measurements, and verify the EKF estimate converges to
// the true Cartesian state despite the nonlinear measurement model.
int main() {
    const double dt = 1.0;
    const double true_vx = 3.0, true_vy = 1.0;
    const double sigma_a = 0.1;
    const double sigma_range = 0.1, sigma_bearing = 0.001;

    ConstantVelocityModel motion_model(2, sigma_a);
    RadarMeasurementModel radar_model(sigma_range, sigma_bearing);
    ExtendedKalmanFilter ekf(4);

    double true_x = 100.0, true_y = 50.0;  // start away from the origin

    Eigen::VectorXd x0(4);
    x0 << true_x, true_y, 0.0, 0.0;  // deliberately wrong initial velocity guess
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4) * 50.0;
    ekf.init(x0, P0);

    for (int step = 0; step < 60; ++step) {
        true_x += true_vx * dt;
        true_y += true_vy * dt;

        Eigen::MatrixXd F = motion_model.F(dt);
        Eigen::VectorXd x_pred = motion_model.predict(ekf.state(), dt);
        ekf.predict(x_pred, F, motion_model.Q(dt));

        CoordinateConverter::Polar polar = CoordinateConverter::cartesianToPolar(true_x, true_y);  // noise-free
        Eigen::VectorXd z(2);
        z << polar.range, polar.bearing;

        Eigen::VectorXd z_pred = radar_model.predict(ekf.state());
        Eigen::MatrixXd H = radar_model.jacobian(ekf.state());
        ekf.update(z, z_pred, H, radar_model.R());
    }

    const Eigen::VectorXd& x = ekf.state();
    std::cout << "Final position estimate: (" << x(0) << ", " << x(1)
              << ") true: (" << true_x << ", " << true_y << ")\n";
    std::cout << "Final velocity estimate: (" << x(2) << ", " << x(3)
              << ") true: (" << true_vx << ", " << true_vy << ")\n";

    assert(std::abs(x(0) - true_x) < 1.0);
    assert(std::abs(x(1) - true_y) < 1.0);
    assert(std::abs(x(2) - true_vx) < 0.3);
    assert(std::abs(x(3) - true_vy) < 0.3);

    std::cout << "All tests passed.\n";
    return 0;
}