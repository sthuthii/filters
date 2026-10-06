#include <cassert>
#include <cmath>
#include <iostream>

#include "../include/CoordinateConverter.hpp"
#include "../include/MeasurementModel.hpp"
#include "../include/MotionModel.hpp"
#include "../include/UnscentedKalmanFilter.hpp"

// Sanity check: track a 2D constant-velocity target using noise-free
// range/bearing measurements, and verify the UKF estimate converges to
// the true Cartesian state — mirrors test_extended_kalman_filter.cpp so
// the two filters can be compared on the same scenario.
int main() {
    const double dt = 1.0;
    const double true_vx = 3.0, true_vy = 1.0;
    const double sigma_a = 0.1;
    const double sigma_range = 0.1, sigma_bearing = 0.001;

    ConstantVelocityModel motion_model(2, sigma_a);
    RadarMeasurementModel radar_model(sigma_range, sigma_bearing);
    UnscentedKalmanFilter ukf(4);

    double true_x = 100.0, true_y = 50.0;  // start away from the origin

    Eigen::VectorXd x0(4);
    x0 << true_x, true_y, 0.0, 0.0;  // deliberately wrong initial velocity guess
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4) * 50.0;
    ukf.init(x0, P0);

    for (int step = 0; step < 60; ++step) {
        true_x += true_vx * dt;
        true_y += true_vy * dt;

        ukf.predict(motion_model, dt);

        CoordinateConverter::Polar polar = CoordinateConverter::cartesianToPolar(true_x, true_y);  // noise-free
        Eigen::VectorXd z(2);
        z << polar.range, polar.bearing;

        ukf.update(z, radar_model);
    }

    const Eigen::VectorXd& x = ukf.state();
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