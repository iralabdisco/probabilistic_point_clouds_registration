#include <limits>
#include <cmath>
#include <memory>
#include <vector>

#include <sm/assert_macros.hpp>

#include "point_cloud_registration/probabilisticWeights.h"

namespace point_cloud_registration {

SM_DEFINE_EXCEPTION(Exception, std::runtime_error);

constexpr double pi() { return std::atan(1) * 4; }

void ProbabilisticWeights::updateWeights(
    const std::vector<std::vector<double>>& residuals,
    std::shared_ptr<std::vector<std::vector<double>>> weights) {
  for (std::size_t j = 0; j < residuals.size(); j++) {
    const std::vector<double>* residuals_group = &(residuals[j]);
    double max_log_prob = -std::numeric_limits<double>::infinity();
    std::vector<double> log_probs;
    std::vector<double> expected_weights;
    log_probs.reserve(residuals_group->size());
    expected_weights.reserve(residuals_group->size());
    double marginal_log_likelihood = 0;
    for (double squared_error : *residuals_group) {
      // Update log_probs
      double log_prob;
      if (is_normal_) {
        log_prob = -squared_error / 2 + log_norm_constant_;
      } else {
        log_prob =
            (t_exponent_) * std::log1p(squared_error / v_) - log_norm_constant_;
        const double expected_weight = (v_ + dimension_) / (v_ + squared_error);
        expected_weights.push_back(expected_weight);
      }

      if (log_prob > max_log_prob) {
        max_log_prob = log_prob;
      }
      log_probs.push_back(log_prob);
    }
    for (double log_p : log_probs) {
      marginal_log_likelihood += std::exp(log_p - max_log_prob);
    }
    marginal_log_likelihood = std::log(marginal_log_likelihood) + max_log_prob;
    for (std::size_t i = 0; i < residuals_group->size(); i++) {
      if (is_normal_) {
        weights->at(j)[i] = std::exp(log_probs[i] - marginal_log_likelihood);
      } else {
        weights->at(j)[i] = std::exp(log_probs[i] - marginal_log_likelihood) *
                     expected_weights[i];
      }
    }
  }
}

  ProbabilisticWeights::ProbabilisticWeights(double v, int dimension)
      : dimension_(dimension) {
    SM_ASSERT_TRUE(
        Exception, dimension > 0,
        "The dimension of the error terms must be greater than zero");
    SM_ASSERT_TRUE(Exception, v > 0.0,
                   "The dof of the t-distribution must be greater than zero");
    if (v < std::numeric_limits<double>::infinity()) {
      v_ = v;
      t_exponent_ = -(v + dimension_) / 2.0;
      is_normal_ = false;
      log_norm_constant_ = std::lgamma(v_ / 2) -
                           std::lgamma((v_ + dimension_) / 2) +
                           (v_ / 2) * std::log(pi() * v_);
    } else {
      is_normal_ = true;
      log_norm_constant_ = (dimension_ / 2.0) * std::log(2 * pi());
    }
  }
}  // namespace point_cloud_registration
