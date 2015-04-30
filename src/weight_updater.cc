#include "point_cloud_registration/weight_updater.h"

#include <vector>

#include "point_cloud_registration/reprojection_error.h"

namespace point_cloud_registration {

WeightUpdater::WeightUpdater(
    std::vector<WeightedErrorTermGroup>* weighted_error_terms, double dof,
    double* rotation, double* translation)
    : weighted_error_terms_(weighted_error_terms),
      weightCalculator_(dof, ReprojectionError::kResiduals),
      rotation_history_(),
      translation_history_(),
      rotation_(rotation),
      translation_(translation) {}

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
  std::shared_ptr<Eigen::Quaternion<double>> estimated_rot(
      new Eigen::Quaternion<double>(rotation_[0], rotation_[1], rotation_[2],
                                    rotation_[3]));
  estimated_rot->normalize();
  rotation_history_.push_back(std::move(estimated_rot));
  std::shared_ptr<Eigen::Vector3d> estimated_translation(
      new Eigen::Vector3d(translation_));
  translation_history_.push_back(std::move(estimated_translation));
  return ceres::SOLVER_CONTINUE;
}
}  // namespace point_cloud_registration
