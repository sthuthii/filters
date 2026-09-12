#pragma once

#include "../lib/Eigen/Dense"

// Position-only measurement model for the 1D CV state [position, velocity].
// The sensor observes position but not velocity, so H extracts position.
class PositionMeasurementModel {
public:
    // r_variance: variance of the sensor's position measurement noise.
    explicit PositionMeasurementModel(double r_variance) : r_variance_(r_variance) {}

    Eigen::MatrixXd H() const {
        Eigen::MatrixXd H(1, 2);
        H << 1.0, 0.0;
        return H;
    }

    Eigen::MatrixXd R() const {
        Eigen::MatrixXd R(1, 1);
        R << r_variance_;
        return R;
    }

private:
    double r_variance_;
};
