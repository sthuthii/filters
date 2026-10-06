#pragma once

#include "../lib/Eigen/Dense"

// Shared state, covariance, and predict/update algebra for KalmanFilter
// and ExtendedKalmanFilter.
//
// Both filters run the exact same covariance math on predict
// (P = F*P*F^T + Q) and the exact same gain/innovation/correction math
// on update — the only real difference between a linear and an extended
// filter is *how* the predicted state and predicted measurement get
// computed (x_pred = F*x / z_pred = H*x for a linear model, vs. an
// arbitrary f(x) / h(x) for a nonlinear one). That varying step is left
// to each subclass's own public predict()/update(), which computes
// whatever it needs and then calls into the protected helpers here.
//
// This is a Template Method-style split: the invariant skeleton (the
// actual filter math) lives once in this base class; the part that
// varies between filter types lives in each subclass. It replaces what
// used to be two near-identical .cpp files.
class KalmanFilterBase {
public:
    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

    const Eigen::VectorXd& state() const { return x_; }
    const Eigen::MatrixXd& covariance() const { return P_; }

protected:
    explicit KalmanFilterBase(int state_dim);

    // Shared predict step. x_pred is whatever the subclass already
    // computed (F*x for a linear model, f(x) for a nonlinear one).
    // P = F*P*F^T + Q.
    void predictImpl(const Eigen::VectorXd& x_pred, const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

    // Shared update step. z_pred is whatever the subclass already
    // computed (H*x for a linear model, h(x) for a nonlinear one).
    // y = z - z_pred; S = H*P*H^T + R; K = P*H^T*S^-1;
    // x = x + K*y; P = (I - K*H)*P.
    void updateImpl(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred,
                     const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

    int n_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
};

