#pragma once

#include "../lib/Eigen/Dense"

// Generic linear Kalman Filter.
//
// Works for any state dimension / measurement dimension. The caller is
// responsible for providing the model matrices (F, Q, H, R) each step,
// which keeps this class independent of any specific motion or
// measurement model (see MotionModel.hpp / MeasurementModel.hpp).
class KalmanFilter {
public:
    explicit KalmanFilter(int state_dim);

    // Set the initial state estimate and covariance.
    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

    // Predict step: x = F*x, P = F*P*F^T + Q
    void predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

    // Update step (correction) using measurement z:
    // y = z - H*x
    // S = H*P*H^T + R
    // K = P*H^T*S^-1
    // x = x + K*y
    // P = (I - K*H)*P
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

    const Eigen::VectorXd& state() const { return x_; }
    const Eigen::MatrixXd& covariance() const { return P_; }

private:
    int n_;              // state dimension
    Eigen::VectorXd x_;  // state estimate
    Eigen::MatrixXd P_;  // state covariance
};
