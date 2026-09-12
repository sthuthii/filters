#include "../include/KalmanFilter.hpp"

KalmanFilter::KalmanFilter(int state_dim) : n_(state_dim) {
    x_ = Eigen::VectorXd::Zero(n_);
    P_ = Eigen::MatrixXd::Identity(n_, n_);
}

void KalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

void KalmanFilter::predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q) {
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
    Eigen::VectorXd y = z - H * x_;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_, n_);
    P_ = (I - K * H) * P_;
}
