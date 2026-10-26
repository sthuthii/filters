#pragma once

#include <vector>
#include <Eigen/Dense>

// Small set of accuracy metrics for comparing a filter's position
// estimates against ground truth. Used by compare_filters.cpp to
// quantify "how much better is the EKF than the CV-KF on a turn?"
//
// Kept intentionally minimal — just position RMSE (overall and
// per-timestep) — rather than a full tracking-metrics suite, since
// that's all the CV-vs-EKF comparison needs right now.
class TrackingMetrics {
public:
    // Euclidean position error at each timestep: ||estimate_i - truth_i||
    // truth and estimate must be the same length.
    static std::vector<double> positionErrorOverTime(
        const std::vector<Eigen::Vector2d>& truth,
        const std::vector<Eigen::Vector2d>& estimate);

    // Root-mean-square position error over the whole run.
    static double positionRMSE(
        const std::vector<Eigen::Vector2d>& truth,
        const std::vector<Eigen::Vector2d>& estimate);
};
