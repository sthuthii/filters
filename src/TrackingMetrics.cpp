#include "TrackingMetrics.hpp"
#include <cmath>
#include <numeric>
#include <stdexcept>

std::vector<double> TrackingMetrics::positionErrorOverTime(
    const std::vector<Eigen::Vector2d>& truth,
    const std::vector<Eigen::Vector2d>& estimate) {
    if (truth.size() != estimate.size()) {
        throw std::invalid_argument("truth and estimate must have the same length");
    }
    std::vector<double> errors;
    errors.reserve(truth.size());
    for (size_t i = 0; i < truth.size(); ++i) {
        errors.push_back((estimate[i] - truth[i]).norm());
    }
    return errors;
}

double TrackingMetrics::positionRMSE(const std::vector<Eigen::Vector2d>& truth,
                                      const std::vector<Eigen::Vector2d>& estimate) {
    std::vector<double> errors = positionErrorOverTime(truth, estimate);
    if (errors.empty()) return 0.0;
    double sum_sq = std::accumulate(errors.begin(), errors.end(), 0.0,
                                     [](double acc, double e) { return acc + e * e; });
    return std::sqrt(sum_sq / static_cast<double>(errors.size()));
}
