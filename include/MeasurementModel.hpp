#pragma once

#include <algorithm>
#include <cmath>

#include "../lib/Eigen/Dense"
#include "CoordinateConverter.hpp"

// Abstract interface for a measurement model: how the sensor's reading
// relates to the state, h(x), plus its Jacobian dh/dx and its noise R.
//
// A linear sensor (PositionMeasurementModel) still fits this exactly:
// h(x) = H*x and jacobian(x) = H for every x, since H doesn't depend on
// x. A nonlinear sensor (RadarMeasurementModel) evaluates the true
// nonlinear relationship and its *local* linearization instead. Because
// both cases fit the same three methods, KalmanFilter and
// ExtendedKalmanFilter loops can both be written against a
// MeasurementModel& / pointer — this is the Strategy pattern applied to
// measurements, and it's what lets main.cpp and main_ekf.cpp share the
// same shape of loop while using different concrete sensors.
class MeasurementModel {
public:
    virtual ~MeasurementModel() = default;

    // h(x): predicted measurement for a given state.
    virtual Eigen::VectorXd predict(const Eigen::VectorXd& x) const = 0;

    // Jacobian of h wrt state, evaluated at x (constant, equal to H, for
    // a linear model).
    virtual Eigen::MatrixXd jacobian(const Eigen::VectorXd& x) const = 0;

    // Measurement noise covariance.
    virtual Eigen::MatrixXd R() const = 0;
};

// N-dimensional position-only measurement model (linear): the sensor
// observes position directly on each axis, not velocity. State is
// [p_1..p_dim, v_1..v_dim]; this pairs with ConstantVelocityModel.
class PositionMeasurementModel : public MeasurementModel {
public:
    // r_variance: variance of the sensor's position measurement noise,
    // applied independently per axis.
    PositionMeasurementModel(int dim, double r_variance) : dim_(dim), r_variance_(r_variance) {}

    Eigen::VectorXd predict(const Eigen::VectorXd& x) const override {
        return H() * x;
    }

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& /*x*/) const override {
        return H();  // constant: doesn't depend on the state it's evaluated at
    }

    Eigen::MatrixXd R() const override {
        return Eigen::MatrixXd::Identity(dim_, dim_) * r_variance_;
    }

    // Kept public (not just an internal helper) since callers that
    // already know they have a linear sensor may want H directly rather
    // than going through predict()/jacobian().
    Eigen::MatrixXd H() const {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim_, 2 * dim_);
        H.block(0, 0, dim_, dim_) = Eigen::MatrixXd::Identity(dim_, dim_);
        return H;
    }

private:
    int dim_;
    double r_variance_;
};

// 2D radar measurement model (nonlinear): the sensor observes range and
// bearing, not Cartesian position/velocity directly. State is fixed as
// [x, y, vx, vy] (a dim=2 CV state). Because range/bearing is a nonlinear
// function of (x, y), the filter must use jacobian(x) — the *local*
// linearization at the predicted state — rather than a single constant H.
class RadarMeasurementModel : public MeasurementModel {
public:
    // sigma_range / sigma_bearing: measurement noise std-dev for range
    // (same units as position) and bearing (radians).
    RadarMeasurementModel(double sigma_range, double sigma_bearing)
        : sigma_range_(sigma_range), sigma_bearing_(sigma_bearing) {}

    // h(x): predicted (range, bearing) for a given Cartesian state.
    Eigen::VectorXd predict(const Eigen::VectorXd& x) const override {
        CoordinateConverter::Polar p = CoordinateConverter::cartesianToPolar(x(0), x(1));
        Eigen::VectorXd z(2);
        z << p.range, p.bearing;
        return z;
    }

    // 2x4: d(range,bearing)/d(x,y,vx,vy). Velocity columns are zero since
    // range/bearing don't depend on velocity.
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& x) const override {
        const double px = x(0), py = x(1);
        const double r2 = px * px + py * py;
        const double r = std::sqrt(std::max(r2, 1e-9));  // avoid divide-by-zero at the origin

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
        H(0, 0) = px / r;
        H(0, 1) = py / r;
        H(1, 0) = -py / r2;
        H(1, 1) = px / r2;
        return H;
    }

    Eigen::MatrixXd R() const override {
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
        R(0, 0) = sigma_range_ * sigma_range_;
        R(1, 1) = sigma_bearing_ * sigma_bearing_;
        return R;
    }

private:
    double sigma_range_;
    double sigma_bearing_;
};  