#include "MotionModel.hpp"
#include <cmath>

Eigen::MatrixXd ConstantVelocity2DModel::F() const {
    Eigen::MatrixXd f = Eigen::MatrixXd::Identity(4, 4);
    f(0, 2) = dt_;
    f(1, 3) = dt_;
    return f;
}

Eigen::MatrixXd ConstantVelocity2DModel::Q() const {
    const double dt2 = dt_ * dt_;
    const double dt3 = dt2 * dt_;
    const double dt4 = dt3 * dt_;
    const double q = sigma_a_ * sigma_a_;

    Eigen::Matrix2d block;
    block << dt4 / 4.0, dt3 / 2.0,
             dt3 / 2.0, dt2;
    block *= q;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
    // position/velocity block for x: rows/cols (0, 2)
    Q(0, 0) = block(0, 0); Q(0, 2) = block(0, 1);
    Q(2, 0) = block(1, 0); Q(2, 2) = block(1, 1);
    // position/velocity block for y: rows/cols (1, 3)
    Q(1, 1) = block(0, 0); Q(1, 3) = block(0, 1);
    Q(3, 1) = block(1, 0); Q(3, 3) = block(1, 1);
    return Q;
}

// ===================== ConstantTurnModel (CTRV) =====================

Eigen::VectorXd ConstantTurnModel::stateTransition(const Eigen::VectorXd& x, double dt) const {
    const double px = x(0), py = x(1), v = x(2), theta = x(3), omega = x(4);
    Eigen::VectorXd x_next(kStateDim);

    if (std::fabs(omega) < kOmegaEpsilon) {
        // Straight-line special case (turn rate ~ 0): avoid dividing by omega.
        x_next(0) = px + v * std::cos(theta) * dt;
        x_next(1) = py + v * std::sin(theta) * dt;
    } else {
        x_next(0) = px + (v / omega) * (std::sin(theta + omega * dt) - std::sin(theta));
        x_next(1) = py + (v / omega) * (-std::cos(theta + omega * dt) + std::cos(theta));
    }
    x_next(2) = v;
    x_next(3) = theta + omega * dt;
    x_next(4) = omega;
    return x_next;
}

Eigen::MatrixXd ConstantTurnModel::jacobian(const Eigen::VectorXd& x, double dt) const {
    const double v = x(2), theta = x(3), omega = x(4);
    Eigen::MatrixXd Fj = Eigen::MatrixXd::Identity(kStateDim, kStateDim);

    if (std::fabs(omega) < kOmegaEpsilon) {
        // d(px')/dtheta, d(py')/dtheta for the straight-line case
        Fj(0, 2) = std::cos(theta) * dt;
        Fj(0, 3) = -v * std::sin(theta) * dt;
        Fj(1, 2) = std::sin(theta) * dt;
        Fj(1, 3) = v * std::cos(theta) * dt;
        // d/domega is technically a removable singularity at omega=0;
        // leaving it at 0 (from Identity) is the standard practical choice.
    } else {
        const double theta_next = theta + omega * dt;
        const double s1 = std::sin(theta), c1 = std::cos(theta);
        const double s2 = std::sin(theta_next), c2 = std::cos(theta_next);

        Fj(0, 2) = (s2 - s1) / omega;
        Fj(0, 3) = (v / omega) * (c2 - c1);
        Fj(0, 4) = (v * dt * c2) / omega - (v * (s2 - s1)) / (omega * omega);

        Fj(1, 2) = (-c2 + c1) / omega;
        Fj(1, 3) = (v / omega) * (s2 - s1);
        Fj(1, 4) = (v * dt * s2) / omega - (v * (-c2 + c1)) / (omega * omega);
    }
    Fj(3, 4) = dt;
    return Fj;
}

Eigen::MatrixXd ConstantTurnModel::processNoise(const Eigen::VectorXd& x, double dt) const {
    const double theta = x(3);
    const double dt2 = dt * dt;

    // G maps the 2 noise sources (longitudinal accel, yaw accel) onto
    // the 5-dim state. This is the standard CTRV noise injection model.
    Eigen::MatrixXd G(kStateDim, 2);
    G << 0.5 * dt2 * std::cos(theta), 0.0,
         0.5 * dt2 * std::sin(theta), 0.0,
         dt,                          0.0,
         0.0,                         0.5 * dt2,
         0.0,                         dt;

    Eigen::Matrix2d noise_cov;
    noise_cov << sigma_a_ * sigma_a_, 0.0,
                 0.0, sigma_yawdd_ * sigma_yawdd_;

    return G * noise_cov * G.transpose();
}
