#include "point_cloud_registration/point_cloud_registration.h"

#include <memory>
#include <string>
#include <vector>

using std::size_t;

namespace point_cloud_registration {

PointCloudRegistration::PointCloudRegistration(
    const pcl::PointCloud<pcl::PointXYZ> &source_cloud,
    const pcl::PointCloud<pcl::PointXYZ> &target_cloud,
    const Eigen::SparseMatrix<int, Eigen::RowMajor> &data_association,
    PointCloudRegistrationParams parameters)
    : error_terms_(),
      data_association_(data_association),
      parameters_(parameters),
      weight_updater_(parameters.dof, parameters.dimension,
                      parameters.max_neighbours) {
  std::copy(std::begin(parameters_.initial_rotation),
            std::end(parameters_.initial_rotation), std::begin(rotation_));
  std::copy(std::begin(parameters_.initial_translation),
            std::end(parameters_.initial_translation),
            std::begin(translation_));
  error_terms_.reserve(source_cloud.size());
  problem_.reset(new ceres::Problem());
  for (size_t i = 0; i < data_association.outerSize(); i++) {
    for (Eigen::SparseMatrix<int, Eigen::RowMajor>::InnerIterator it(
             data_association, i);
         it; ++it) {
      ErrorTerm error_term(source_cloud[i], target_cloud[it.index()]);
      error_terms_.push_back(error_term);
      ErrorTerm *error_term_ptr = &(error_terms_.back());
      problem_->AddResidualBlock(
          new ceres::AutoDiffCostFunction<ErrorTerm, ErrorTerm::kResiduals, 4,
                                          3>(error_term_ptr),
          error_term_ptr->weight(), rotation_, translation_);
    }
  }
  callback(ceres::IterationSummary());
}

void PointCloudRegistration::solve(ceres::Solver::Options options,
                                   ceres::Solver::Summary *summary) {
  options.callbacks.push_back(this);
  options.update_state_every_iteration = true;
  ceres::Solve(options, problem_.get(), summary);
}

Eigen::Affine3d PointCloudRegistration::transformation() {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  Eigen::Quaternion<double> estimated_rot(rotation_[0], rotation_[1],
                                          rotation_[2], rotation_[3]);
  estimated_rot.normalize();
  affine.rotate(estimated_rot);
  affine.pretranslate(Eigen::Vector3d(translation_));
  return affine;
}

ceres::CallbackReturnType PointCloudRegistration::operator()(
    const ceres::IterationSummary &summary) {
  return callback(summary);
}
ceres::CallbackReturnType PointCloudRegistration::callback(
    const ceres::IterationSummary &summary) {
  Eigen::SparseMatrix<double, Eigen::RowMajor> squared_errors(
      data_association_.rows(), data_association_.cols());
  squared_errors.reserve(Eigen::VectorXi::Constant(data_association_.cols(),
                                                   parameters_.max_neighbours));
  int k = 0;
  for (int i = 0; i < data_association_.outerSize(); ++i) {
    for (Eigen::SparseMatrix<int, Eigen::RowMajor>::InnerIterator it(
             data_association_, i);
         it; ++it) {
      double residual[3];
      (error_terms_[k])(rotation_, translation_, residual);
      squared_errors.insert(it.row(), it.col()) = residual[0] * residual[0] +
                                                  residual[1] * residual[1] +
                                                  residual[2] * residual[2];
      k++;
    }
  }
  squared_errors.makeCompressed();
  Eigen::SparseMatrix<double, Eigen::RowMajor> weights_ =
      weight_updater_.updateWeights(squared_errors);
  k = 0;
  for (int i = 0; i < weights_.outerSize(); ++i) {
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
             weights_, i);
         it; ++it) {
      error_terms_[k].updateWeight(it.value());
      k++;
    }
  }
  return ceres::SOLVER_CONTINUE;
}

}  // namespace point_cloud_registration
