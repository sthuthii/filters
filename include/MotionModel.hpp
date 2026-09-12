#pragma once

#include "../lib/Eigen/Dense"

// 1D Constant Velocity (CV) motion model.
//
// State vector: x = [position, velocity]^T
// Builds the state transition matrix F and the process noise
// covariance Q (discrete white noise acceleration model) for a
// given time step dt.
class ConstantVelocityModel {
public:
    // sigma_a: standard deviation of the assumed (unmodeled) acceleration.
    explicit ConstantVelocityModel(double sigma_a) : sigma_a_(sigma_a) {}

    Eigen::MatrixXd F(double dt) const {
        Eigen::MatrixXd F(2, 2);
        F << 1.0, dt,
             0.0, 1.0;
        return F;
    }

    Eigen::MatrixXd Q(double dt) const {
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        const double var = sigma_a_ * sigma_a_;

        Eigen::MatrixXd Q(2, 2);
        Q << dt4 / 4.0, dt3 / 2.0,
             dt3 / 2.0, dt2;
        Q *= var;
        return Q;
    }

private:
    double sigma_a_;
};
