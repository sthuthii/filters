#include <cassert>
#include <cmath>
#include <iostream>

#include "../include/KalmanFilter.hpp"
#include "../include/MotionModel.hpp"
#include "../include/MeasurementModel.hpp"

// Sanity checks for the 1D CV Kalman filter:
// 1. With perfect (noise-free) measurements of a true constant-velocity
//    target, the filter's estimate should converge closely to the truth.
// 2. The covariance trace should shrink after each update compared to
//    right after predict (the filter should become more confident).
int main() {
    const int dim = 1;
    const double dt = 1.0;
    const double true_velocity = 2.0;
    const double sigma_a = 0.1;
    const double r_variance = 0.5;

    ConstantVelocityModel motion_model(dim, sigma_a);
    PositionMeasurementModel measurement_model(dim, r_variance);
    KalmanFilter kf(2 * dim);

    Eigen::VectorXd x0(2);
    x0 << 0.0, 0.0;  // deliberately wrong initial velocity guess
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(2, 2) * 10.0;
    kf.init(x0, P0);

    double true_pos = 0.0;
    for (int step = 0; step < 50; ++step) {
        true_pos += true_velocity * dt;

        kf.predict(motion_model.F(dt), motion_model.Q(dt));
        double trace_before_update = kf.covariance().trace();

        Eigen::VectorXd z(1);
        z << true_pos;  // noise-free measurement
        kf.update(z, measurement_model.H(), measurement_model.R());
        double trace_after_update = kf.covariance().trace();

        assert(trace_after_update <= trace_before_update + 1e-9);
    }

    const Eigen::VectorXd& x = kf.state();
    std::cout << "Final position estimate: " << x(0) << " (true: " << true_pos << ")\n";
    std::cout << "Final velocity estimate: " << x(1) << " (true: " << true_velocity << ")\n";

    assert(std::abs(x(0) - true_pos) < 0.5);
    assert(std::abs(x(1) - true_velocity) < 0.2);

    std::cout << "All tests passed.\n";
    return 0;
}
