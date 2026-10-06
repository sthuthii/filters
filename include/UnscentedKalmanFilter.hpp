#pragma once

#include "KalmanFilterBase.hpp"
#include "MeasurementModel.hpp"
#include "MotionModel.hpp"

// Unscented Kalman Filter.
//
// Unlike KalmanFilter/ExtendedKalmanFilter, the UKF never linearizes
// anything — no F or H Jacobian is used for covariance propagation.
// Instead it propagates a small deterministic set of "sigma points"
// through the true nonlinear f(x)/h(x) directly, then recovers the
// predicted mean/covariance from the spread of those propagated points
// (the "unscented transform"). Because of that, it needs the model's
// predict() function itself at several points, not a Jacobian — so
// predict()/update() take the MotionModel/MeasurementModel directly,
// unlike KalmanFilter's/ExtendedKalmanFilter's matrix-based signatures.
//
// It still derives from KalmanFilterBase to reuse the shared x_/P_
// storage and init()/state()/covariance() accessors, but supplies its
// own predict/update algorithm entirely — the sigma-point math doesn't
// fit the linear-covariance skeleton predictImpl/updateImpl implement
// (see KalmanFilterBase.hpp), so this class writes to the protected
// x_/P_ directly instead of calling those helpers.
class UnscentedKalmanFilter : public KalmanFilterBase {
public:
    // alpha/beta/kappa: standard UKF scaling parameters (Van der Merwe
    // formulation). Defaults are the commonly used values — alpha
    // controls sigma point spread (small and positive), beta
    // incorporates prior knowledge of the distribution (2 is optimal
    // for a Gaussian), kappa is a secondary scaling term (0 is a
    // common default).
    explicit UnscentedKalmanFilter(int state_dim, double alpha = 1e-3, double beta = 2.0,
                                    double kappa = 0.0);

    // Predict step: propagates 2n+1 sigma points through
    // motion.predict(x, dt), then recovers the predicted mean/covariance
    // via the unscented transform. Adds motion.Q(dt) as process noise.
    // The propagated sigma points are kept internally for update().
    void predict(const MotionModel& motion, double dt);

    // Update step: propagates the sigma points from the last predict()
    // through measurement.predict(x) to get predicted measurement sigma
    // points, then applies the standard cross-covariance/gain
    // correction. Must be called after predict().
    //
    // Note: this averages sigma points directly, which is exact for
    // ordinary (non-angular) measurements. For an angular measurement
    // component (e.g. RadarMeasurementModel's bearing), a naive average
    // can be wrong very close to the +-pi wraparound boundary — the same
    // caveat main_ekf.cpp handles explicitly for the EKF. Kept simple
    // here since it only matters in that edge case.
    void update(const Eigen::VectorXd& z, const MeasurementModel& measurement);

private:
    int num_sigma_points_;
    double lambda_;
    Eigen::VectorXd weights_mean_;
    Eigen::VectorXd weights_cov_;
    Eigen::MatrixXd sigma_points_;  // n x (2n+1); the predicted sigma points from the last predict()

    // Builds 2n+1 sigma points around the current x_/P_.
    Eigen::MatrixXd generateSigmaPoints() const;
};