#pragma once

#include "KalmanFilterBase.hpp"

// Extended Kalman Filter.
//
// Unlike KalmanFilter, x_pred = f(x) and z_pred = h(x) are nonlinear,
// and this class deliberately doesn't know either function — the caller
// (see MotionModel::predict / MeasurementModel::predict) evaluates them,
// and their Jacobians, each step and passes the results straight
// through to the shared base-class algebra. See KalmanFilterBase.hpp for
// why the underlying math lives there instead of here.
class ExtendedKalmanFilter : public KalmanFilterBase {
public:
    explicit ExtendedKalmanFilter(int state_dim) : KalmanFilterBase(state_dim) {}

    // Predict step. x_pred = f(x_prev), already evaluated by the caller.
    // F is the Jacobian df/dx evaluated at x_prev. P = F*P*F^T + Q.
    void predict(const Eigen::VectorXd& x_pred, const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q) {
        predictImpl(x_pred, F, Q);
    }

    // Update step. z_pred = h(x_predicted), already evaluated by the
    // caller. H is the Jacobian dh/dx evaluated at the predicted state.
    void update(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred,
                const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
        updateImpl(z, z_pred, H, R);
    }
};