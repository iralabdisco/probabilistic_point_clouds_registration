#include <vector>

#include "point_cloud_registration/weightUpdater.h"

namespace point_cloud_registration {

WeightUpdater::WeightUpdater(
    std::vector<WeightedErrorTermGroup>* weighted_error_terms)
    : weighted_error_terms_(weighted_error_terms) {}

ceres::CallbackReturnType WeightUpdater::operator()(
    const ceres::IterationSummary& summary) {
  vector<vector<double>> residuals(weighted_error_terms_.size(),
                                   vector<double>());
  for (std::size_t i = 0; i < weighted_error_terms_.size(); i++) {
    for (WeightedErrorTerm& error_term : weighted_error_terms_[i]) {
      residuals[i].push_back(error_term.squaredError());
    }
  }
  vector<vector<double>> weights(residuals.size());
  for (std::size_t i = 0; i < residuals.size(); i++) {
    weights[i] = vector<double>(residuals[i].size());
  }
  weightCalculator_.updateWeights(residuals, weights);
  for (std::size_t i = 0; i < weighted_error_terms_.size(); i++) {
    for (std::size_t j = 0; j < weighted_error_terms_[i].size(); j++) {
      weighted_error_terms_[i][j]->updateWeight(weights[i][j]);
    }
  }
  return ceres::SOLVER_CONTINUE;
}
}  // namespace point_cloud_registration
