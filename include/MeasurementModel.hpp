#pragma once

#include "../lib/Eigen/Dense"

// Position-only measurement model for the 1D CV state [position, velocity].
// The sensor observes position but not velocity, so H extracts position.
class PositionMeasurementModel {
public:
    PositionMeasurementModel(int dim, double r_variance) : dim_(dim), r_variance_(r_variance) {}
    Eigen::MatrixXd H() const {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim_, 2*dim_);
        H.block(0, 0, dim_, dim_) = Eigen::MatrixXd::Identity(dim_, dim_);
        return H;
    }
    Eigen::MatrixXd R() const {
        return Eigen::MatrixXd::Identity(dim_, dim_) * r_variance_;
    }
private:
    int dim_; double r_variance_;
};