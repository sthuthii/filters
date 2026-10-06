#include "../include/KalmanFilterBase.hpp"

KalmanFilterBase::KalmanFilterBase(int state_dim) : n_(state_dim) {
    x_ = Eigen::VectorXd::Zero(n_);
    P_ = Eigen::MatrixXd::Identity(n_, n_);
}

void KalmanFilterBase::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

void KalmanFilterBase::predictImpl(const Eigen::VectorXd& x_pred, const Eigen::MatrixXd& F,
                                    const Eigen::MatrixXd& Q) {
    x_ = x_pred;
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilterBase::updateImpl(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred,
                                   const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
    Eigen::VectorXd y = z - z_pred;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_, n_);
    P_ = (I - K * H) * P_;
}