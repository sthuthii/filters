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
    ConstantVelocityModel(int dim, double sigma_a) : dim_(dim), sigma_a_(sigma_a) {}

    Eigen::MatrixXd F(double dt) const {
        int n = 2 * dim_;
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n, n);
        F.block(0, dim_, dim_, dim_) = Eigen::MatrixXd::Identity(dim_, dim_) * dt;
        return F;
    }

    Eigen::MatrixXd Q(double dt) const {
        double dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt3*dt, var = sigma_a_*sigma_a_;
        Eigen::Matrix2d q2; q2 << dt4/4.0, dt3/2.0, dt3/2.0, dt2;
        q2 *= var;
        int n = 2 * dim_;
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
        for (int i = 0; i < dim_; ++i) {
            Q(i, i)             = q2(0,0);
            Q(i, dim_ + i)      = q2(0,1);
            Q(dim_ + i, i)      = q2(1,0);
            Q(dim_ + i, dim_+i) = q2(1,1);
        }
        return Q;
    }
private:
    int dim_; double sigma_a_;
};
