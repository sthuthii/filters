#pragma once

#include "KalmanFilterBase.hpp"

// Linear Kalman Filter.
//
// A thin wrapper over KalmanFilterBase: it computes the linear
// predictions itself (x_pred = F*x, z_pred = H*x — exact for a linear
// model, no approximation) and then delegates the shared covariance
// algebra to the base class. See KalmanFilterBase.hpp for why the math
// lives there instead of here.
class KalmanFilter : public KalmanFilterBase {
public:
    explicit KalmanFilter(int state_dim) : KalmanFilterBase(state_dim) {}

    // Predict step: x = F*x, P = F*P*F^T + Q
    void predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q) {
        predictImpl(F * x_, F, Q);
    }

    // Update step: y = z - H*x, S = H*P*H^T + R, K = P*H^T*S^-1,
    // x = x + K*y, P = (I - K*H)*P
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
        updateImpl(z, H * x_, H, R);
    }
};