#include "point_cloud_registration/weight_updater.h"

#include <fstream>
#include <vector>

#include "point_cloud_registration/reprojection_error.h"

namespace point_cloud_registration {

WeightUpdater::WeightUpdater(int num_groups, double dof, double* rotation,
                             double* translation, std::vector<std::vector<int>> data_association, int dense_size)
    : weighted_error_terms_(num_groups),
      weightCalculator_(dof, ReprojectionError::kResiduals),
      rotation_(rotation),
      translation_(translation),
      data_association_(data_association),
      dense_size_(dense_size) {}

ceres::CallbackReturnType WeightUpdater::operator()(
    const ceres::IterationSummary& summary) {
  std::vector<std::vector<double>> residuals(weighted_error_terms_.size(),
                                             std::vector<double>());
  for (std::size_t i = 0; i < weighted_error_terms_.size(); i++) {
    for (auto& error_term : weighted_error_terms_.at(i)) {
      residuals[i].push_back(error_term->squaredError());
    }
  }
  std::vector<std::vector<double>> weights(residuals.size());
  for (std::size_t i = 0; i < residuals.size(); i++) {
    weights[i] = std::vector<double>(residuals[i].size());
  }
  weightCalculator_.updateWeights(residuals, &weights);
  for (std::size_t i = 0; i < weighted_error_terms_.size(); i++) {
    for (std::size_t j = 0; j < weighted_error_terms_.at(i).size(); j++) {
      (weighted_error_terms_)[i][j]->updateWeight(weights[i][j]);
    }
  }

  std::vector<std::vector<double>> weight_matrix(data_association_.size(), std::vector<double>(dense_size_, 0));
  for (size_t i = 0; i < data_association_.size(); i++) {
    for (size_t j = 0; j < data_association_[i].size(); j++) {
      weight_matrix[i][data_association_[i][j]] = weights[i][j];
    }
  }

  Eigen::Quaternion<double> estimated_rot(rotation_[0], rotation_[1],
                                          rotation_[2], rotation_[3]);
  estimated_rot.normalize();
  Eigen::Vector3d estimated_translation(translation_);
  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
  transformation.rotate(estimated_rot);
  transformation.pretranslate(estimated_translation);
  transformation_history_.push_back(transformation);
  return ceres::SOLVER_CONTINUE;
}

void WeightUpdater::addErrorTerm(
    int index, std::unique_ptr<WeightedErrorTerm> error_term) {
  weighted_error_terms_[index].push_back(std::move(error_term));
}

}  // namespace point_cloud_registration
