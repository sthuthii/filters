#pragma once

#include "../lib/Eigen/Dense"

// Abstract interface for a motion model: describes how the state evolves
// on its own, with no measurement (the "process" side of the filter).
//
// Implementing this lets a filter loop hold a MotionModel& / pointer and
// swap in a different motion assumption (constant velocity today, a
// future coordinated-turn model, etc.) without changing the filter or
// the calling code at all — this is the Strategy pattern applied to
// motion.
class MotionModel {
public:
    virtual ~MotionModel() = default;

    // State transition matrix (or its Jacobian, for a nonlinear model)
    // for a given time step dt.
    virtual Eigen::MatrixXd F(double dt) const = 0;

    // Process noise covariance for a given time step dt.
    virtual Eigen::MatrixXd Q(double dt) const = 0;

    // Predicted state after advancing by dt. For a linear model this is
    // just F(dt) * x; a nonlinear model would evaluate its own f(x, dt)
    // here instead.
    virtual Eigen::VectorXd predict(const Eigen::VectorXd& x, double dt) const = 0;

    // Full state dimension (2 * spatial dim for a CV-style model).
    virtual int stateDim() const = 0;
};

// N-dimensional Constant Velocity (CV) motion model.
//
// State vector: x = [p_1..p_dim, v_1..v_dim]^T. dim=1 reproduces the
// original 1D Task 1 model exactly; dim=2/3 generalizes it to tracking
// x/y or x/y/z position + velocity.
class ConstantVelocityModel : public MotionModel {
public:
    // sigma_a: standard deviation of the assumed (unmodeled) acceleration,
    // applied independently per axis.
    ConstantVelocityModel(int dim, double sigma_a) : dim_(dim), sigma_a_(sigma_a) {}

    int dim() const { return dim_; }
    int stateDim() const override { return 2 * dim_; }

    Eigen::MatrixXd F(double dt) const override {
        const int n = 2 * dim_;
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n, n);
        F.block(0, dim_, dim_, dim_) = Eigen::MatrixXd::Identity(dim_, dim_) * dt;
        return F;
    }

    Eigen::MatrixXd Q(double dt) const override {
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        const double var = sigma_a_ * sigma_a_;

        Eigen::Matrix2d q2;
        q2 << dt4 / 4.0, dt3 / 2.0,
              dt3 / 2.0, dt2;
        q2 *= var;

        const int n = 2 * dim_;
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
        for (int i = 0; i < dim_; ++i) {
            Q(i, i)               = q2(0, 0);
            Q(i, dim_ + i)        = q2(0, 1);
            Q(dim_ + i, i)        = q2(1, 0);
            Q(dim_ + i, dim_ + i) = q2(1, 1);
        }
        return Q;
    }

    // CV motion is linear, so this is exact (not an approximation) — the
    // interface method is still virtual since a future nonlinear motion
    // model (e.g. coordinated turn) would need to override it with its
    // own f(x).
    Eigen::VectorXd predict(const Eigen::VectorXd& x, double dt) const override {
        return F(dt) * x;
    }

private:
    int dim_;
    double sigma_a_;
};