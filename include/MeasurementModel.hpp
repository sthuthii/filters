#pragma once

#include <algorithm>
#include <cmath>

#include "../lib/Eigen/Dense"
#include "CoordinateConverter.hpp"

class MeasurementModel {
public:
    virtual ~MeasurementModel() = default;
    virtual Eigen::VectorXd predict(const Eigen::VectorXd& x) const = 0;
    virtual Eigen::MatrixXd jacobian(const Eigen::VectorXd& x) const = 0;
    virtual Eigen::MatrixXd R() const = 0;
};

class PositionMeasurementModel : public MeasurementModel {
public:
    PositionMeasurementModel(int dim, double r_variance)
        : dim_(dim), r_variance_(r_variance) {}

    Eigen::VectorXd predict(const Eigen::VectorXd& x) const override {
        return H() * x;
    }

    Eigen::MatrixXd jacobian(const Eigen::VectorXd&) const override {
        return H();
    }

    Eigen::MatrixXd R() const override {
        return Eigen::MatrixXd::Identity(dim_, dim_) * r_variance_;
    }

    Eigen::MatrixXd H() const {
        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(dim_, 2 * dim_);
        h.block(0, 0, dim_, dim_) = Eigen::MatrixXd::Identity(dim_, dim_);
        return h;
    }

private:
    int dim_;
    double r_variance_;
};

class PositionMeasurementModelND {
public:
    PositionMeasurementModelND(int state_dim, double sigma_px, double sigma_py)
        : state_dim_(state_dim), sigma_px_(sigma_px), sigma_py_(sigma_py) {}

    Eigen::MatrixXd H() const {
        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(2, state_dim_);
        h(0, 0) = 1.0;
        h(1, 1) = 1.0;
        return h;
    }

    Eigen::MatrixXd R() const {
        Eigen::MatrixXd r = Eigen::MatrixXd::Zero(2, 2);
        r(0, 0) = sigma_px_ * sigma_px_;
        r(1, 1) = sigma_py_ * sigma_py_;
        return r;
    }

private:
    int state_dim_;
    double sigma_px_;
    double sigma_py_;
};

class RadarMeasurementModel : public MeasurementModel {
public:
    RadarMeasurementModel(double sigma_range, double sigma_bearing)
        : sigma_range_(sigma_range), sigma_bearing_(sigma_bearing) {}

    Eigen::VectorXd predict(const Eigen::VectorXd& x) const override {
        const auto polar = CoordinateConverter::cartesianToPolar(x(0), x(1));
        Eigen::VectorXd z(2);
        z << polar.range, polar.bearing;
        return z;
    }

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& x) const override {
        const double px = x(0);
        const double py = x(1);
        const double r2 = std::max(px * px + py * py, 1e-9);
        const double r = std::sqrt(r2);
        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(2, 4);
        h(0, 0) = px / r;
        h(0, 1) = py / r;
        h(1, 0) = -py / r2;
        h(1, 1) = px / r2;
        return h;
    }

    Eigen::MatrixXd R() const override {
        Eigen::MatrixXd r = Eigen::MatrixXd::Zero(2, 2);
        r(0, 0) = sigma_range_ * sigma_range_;
        r(1, 1) = sigma_bearing_ * sigma_bearing_;
        return r;
    }

private:
    double sigma_range_;
    double sigma_bearing_;
};
