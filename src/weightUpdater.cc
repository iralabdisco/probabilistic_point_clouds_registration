#include <vector>

#include "point_cloud_registration/reprojectionError.h"
#include "point_cloud_registration/weightUpdater.h"

namespace point_cloud_registration {

WeightUpdater::WeightUpdater(
    std::vector<WeightedErrorTermGroup>* weighted_error_terms,
    double dof)
    : weighted_error_terms_(weighted_error_terms),
      weightCalculator_(dof, ReprojectionError::kResiduals) {}

ceres::CallbackReturnType WeightUpdater::operator()(
    const ceres::IterationSummary& summary) {
  std::vector<std::vector<double>> residuals(weighted_error_terms_->size(),
                                             std::vector<double>());
  for (std::size_t i = 0; i < weighted_error_terms_->size(); i++) {
    for (auto error_term : weighted_error_terms_->at(i)) {
      residuals[i].push_back(error_term->squaredError());
    }
  }
  std::vector<std::vector<double>> weights(residuals.size());
  for (std::size_t i = 0; i < residuals.size(); i++) {
    weights[i] = std::vector<double>(residuals[i].size());
  }
  weightCalculator_.updateWeights(residuals, &weights);
  for (std::size_t i = 0; i < weighted_error_terms_->size(); i++) {
    for (std::size_t j = 0; j < weighted_error_terms_->at(i).size(); j++) {
      (*weighted_error_terms_)[i][j]->updateWeight(weights[i][j]);
    }
  }
  return ceres::SOLVER_CONTINUE;
}
}  // namespace point_cloud_registration
