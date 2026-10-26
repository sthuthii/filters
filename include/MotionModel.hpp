#pragma once

#include <cmath>

#include "../lib/Eigen/Dense"

class MotionModel {
public:
    virtual ~MotionModel() = default;
    virtual Eigen::MatrixXd F(double dt) const = 0;
    virtual Eigen::MatrixXd Q(double dt) const = 0;
    virtual Eigen::VectorXd predict(const Eigen::VectorXd& x, double dt) const = 0;
    virtual int stateDim() const = 0;
};

class ConstantVelocityModel : public MotionModel {
public:
    ConstantVelocityModel(int dim, double sigma_a) : dim_(dim), sigma_a_(sigma_a) {}

    int dim() const { return dim_; }
    int stateDim() const override { return 2 * dim_; }

    Eigen::MatrixXd F(double dt) const override {
        const int n = 2 * dim_;
        Eigen::MatrixXd f = Eigen::MatrixXd::Identity(n, n);
        f.block(0, dim_, dim_, dim_) = Eigen::MatrixXd::Identity(dim_, dim_) * dt;
        return f;
    }

    Eigen::MatrixXd Q(double dt) const override {
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        const double variance = sigma_a_ * sigma_a_;
        Eigen::Matrix2d q2;
        q2 << dt4 / 4.0, dt3 / 2.0, dt3 / 2.0, dt2;
        q2 *= variance;

        Eigen::MatrixXd q = Eigen::MatrixXd::Zero(2 * dim_, 2 * dim_);
        for (int i = 0; i < dim_; ++i) {
            q(i, i) = q2(0, 0);
            q(i, dim_ + i) = q2(0, 1);
            q(dim_ + i, i) = q2(1, 0);
            q(dim_ + i, dim_ + i) = q2(1, 1);
        }
        return q;
    }

    Eigen::VectorXd predict(const Eigen::VectorXd& x, double dt) const override {
        return F(dt) * x;
    }

private:
    int dim_;
    double sigma_a_;
};

class ConstantVelocity2DModel {
public:
    ConstantVelocity2DModel(double dt, double accel_noise)
        : dt_(dt), sigma_a_(accel_noise) {}
    Eigen::MatrixXd F() const;
    Eigen::MatrixXd Q() const;

private:
    double dt_;
    double sigma_a_;
};

class ConstantTurnModel {
public:
    ConstantTurnModel(double sigma_a, double sigma_yawdd)
        : sigma_a_(sigma_a), sigma_yawdd_(sigma_yawdd) {}

    Eigen::VectorXd stateTransition(const Eigen::VectorXd& x, double dt) const;
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& x, double dt) const;
    Eigen::MatrixXd processNoise(const Eigen::VectorXd& x, double dt) const;

private:
    static constexpr int kStateDim = 5;
    double sigma_a_;
    double sigma_yawdd_;
    static constexpr double kOmegaEpsilon = 1e-4;
};
