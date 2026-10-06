#include "../include/UnscentedKalmanFilter.hpp"

UnscentedKalmanFilter::UnscentedKalmanFilter(int state_dim, double alpha, double beta, double kappa)
    : KalmanFilterBase(state_dim) {
    const int n = state_dim;
    num_sigma_points_ = 2 * n + 1;
    lambda_ = alpha * alpha * (n + kappa) - n;

    weights_mean_ = Eigen::VectorXd::Zero(num_sigma_points_);
    weights_cov_ = Eigen::VectorXd::Zero(num_sigma_points_);

    weights_mean_(0) = lambda_ / (n + lambda_);
    weights_cov_(0) = lambda_ / (n + lambda_) + (1.0 - alpha * alpha + beta);
    for (int i = 1; i < num_sigma_points_; ++i) {
        weights_mean_(i) = 1.0 / (2.0 * (n + lambda_));
        weights_cov_(i) = weights_mean_(i);
    }

    sigma_points_ = Eigen::MatrixXd::Zero(n, num_sigma_points_);
}

Eigen::MatrixXd UnscentedKalmanFilter::generateSigmaPoints() const {
    const int n = n_;
    Eigen::MatrixXd sigma = Eigen::MatrixXd::Zero(n, num_sigma_points_);

    // Matrix square root of the scaled covariance, via Cholesky: if
    // L*L^T = (n+lambda)*P, then columns of L are the spread directions.
    Eigen::LLT<Eigen::MatrixXd> llt((n + lambda_) * P_);
    Eigen::MatrixXd sqrt_P = llt.matrixL();

    sigma.col(0) = x_;
    for (int i = 0; i < n; ++i) {
        sigma.col(i + 1) = x_ + sqrt_P.col(i);
        sigma.col(n + i + 1) = x_ - sqrt_P.col(i);
    }
    return sigma;
}

void UnscentedKalmanFilter::predict(const MotionModel& motion, double dt) {
    Eigen::MatrixXd sigma = generateSigmaPoints();

    // Propagate every sigma point through the true nonlinear motion
    // model (exact for CV motion, since it's linear; would matter more
    // for a genuinely nonlinear motion model like coordinated turn).
    Eigen::MatrixXd sigma_pred(n_, num_sigma_points_);
    for (int i = 0; i < num_sigma_points_; ++i) {
        sigma_pred.col(i) = motion.predict(sigma.col(i), dt);
    }

    // Recover the predicted mean and covariance from the propagated
    // points (the unscented transform).
    Eigen::VectorXd x_pred = Eigen::VectorXd::Zero(n_);
    for (int i = 0; i < num_sigma_points_; ++i) {
        x_pred += weights_mean_(i) * sigma_pred.col(i);
    }

    Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(n_, n_);
    for (int i = 0; i < num_sigma_points_; ++i) {
        Eigen::VectorXd diff = sigma_pred.col(i) - x_pred;
        P_pred += weights_cov_(i) * diff * diff.transpose();
    }
    P_pred += motion.Q(dt);

    x_ = x_pred;
    P_ = P_pred;
    sigma_points_ = sigma_pred;  // reused by update() to avoid re-sampling
}

void UnscentedKalmanFilter::update(const Eigen::VectorXd& z, const MeasurementModel& measurement) {
    const int m = static_cast<int>(z.size());

    // Propagate the already-predicted sigma points through the true
    // nonlinear measurement model.
    Eigen::MatrixXd z_sigma(m, num_sigma_points_);
    for (int i = 0; i < num_sigma_points_; ++i) {
        z_sigma.col(i) = measurement.predict(sigma_points_.col(i));
    }

    Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(m);
    for (int i = 0; i < num_sigma_points_; ++i) {
        z_pred += weights_mean_(i) * z_sigma.col(i);
    }

    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(m, m);
    Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(n_, m);
    for (int i = 0; i < num_sigma_points_; ++i) {
        Eigen::VectorXd dz = z_sigma.col(i) - z_pred;
        Eigen::VectorXd dx = sigma_points_.col(i) - x_;
        S += weights_cov_(i) * dz * dz.transpose();
        Pxz += weights_cov_(i) * dx * dz.transpose();
    }
    S += measurement.R();

    Eigen::MatrixXd K = Pxz * S.inverse();
    Eigen::VectorXd y = z - z_pred;

    x_ = x_ + K * y;
    P_ = P_ - K * S * K.transpose();
}